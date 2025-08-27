"""
object_matcher_with_pose.py
Self-contained 1-to-1 matcher + pose refinement using alternating optimization.

Dependencies: numpy
Optional: scipy (for optimal Hungarian assignment). If not present, uses greedy fallback.

Author: ChatGPT (adapt to your needs)
"""

import numpy as np
import scipy
from dataclasses import dataclass, field
from typing import List, Tuple, Callable, Any, Optional

# ----------------------------
# Utilities: quaternion and rotation
# ----------------------------
def quat_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion (w, x, y, z) into 3x3 rotation matrix."""
    w, x, y, z = q
    # normalize for safety
    n = np.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=float)
    return R

def rot_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to quaternion (w, x, y, z)."""
    # Use stable algorithm
    m = R
    tr = m[0,0] + m[1,1] + m[2,2]
    if tr > 0:
        S = np.sqrt(tr+1.0) * 2
        w = 0.25 * S
        x = (m[2,1] - m[1,2]) / S
        y = (m[0,2] - m[2,0]) / S
        z = (m[1,0] - m[0,1]) / S
    elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
        S = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
        w = (m[2,1] - m[1,2]) / S
        x = 0.25 * S
        y = (m[0,1] + m[1,0]) / S
        z = (m[0,2] + m[2,0]) / S
    elif m[1,1] > m[2,2]:
        S = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
        w = (m[0,2] - m[2,0]) / S
        x = (m[0,1] + m[1,0]) / S
        y = 0.25 * S
        z = (m[1,2] + m[2,1]) / S
    else:
        S = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
        w = (m[1,0] - m[0,1]) / S
        x = (m[0,2] + m[2,0]) / S
        y = (m[1,2] + m[2,1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z], dtype=float)

def apply_r_t(R: np.ndarray, T: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Apply rotation R and translation T to point(s) p. p can be (3,) or (N,3)."""
    p = np.asarray(p)
    if p.ndim == 1:
        return R @ p + T
    else:
        return (R @ p.T).T + T

# ----------------------------
# Data model
# ----------------------------
@dataclass
class Obj:
    id: int
    pos: np.ndarray            # 3D position (3,)
    conf_dirs: List[Tuple[np.ndarray, float]] = field(default_factory=list)
        # Each element: (unit_vector (3,), cost_multiplier)
        # Lower cost_multiplier -> easier to move along that direction.
    last_seen: Optional[float] = None
    seen_count: int = 0
    missed_count: int = 0
    extras: dict = field(default_factory=dict)  # any other heuristics

# ----------------------------
# Cost function
# ----------------------------
def compute_cost_with_conf_dirs(
    obj: Obj,
    det_pos: np.ndarray,
    R: np.ndarray,
    T: np.ndarray,
    base_pos_weight: float = 1.0,
    eps: float = 1e-8
) -> float:
    """
    Compute a scalar cost of assigning `obj` (old, with pos in world frame) to a detection at det_pos.
    R,T map old positions into new predicted positions: pred = R @ obj.pos + T.
    Movement along obj.conf_dirs is penalized with their provided multipliers.
    Lower multiplier -> cheaper along that direction (e.g. multiplier=0 => free).
    The produced cost is roughly: ||residual_orth||^2 + sum( lambda_v * proj_v^2 )
    scaled by base_pos_weight.
    """
    pred = apply_r_t(R, T, obj.pos)
    delta = det_pos - pred   # vector (3,)
    # Start with delta squared norm
    # Let P = sum_proj where sum over conf_dirs of (proj * v)
    sum_proj = np.zeros(3, dtype=float)
    proj_penalty = 0.0
    for v, mult in obj.conf_dirs:
        v = np.asarray(v, dtype=float)
        if np.linalg.norm(v) < eps:
            continue
        v = v / (np.linalg.norm(v) + eps)
        proj = np.dot(delta, v)
        sum_proj += proj * v
        # movement along v is penalized by 'mult' (smaller => easier)
        proj_penalty += mult * (proj ** 2)

    orth_component = delta - sum_proj
    cost = base_pos_weight * (np.dot(orth_component, orth_component) + proj_penalty)
    return float(cost)

# ----------------------------
# Assignment solver (Hungarian or greedy fallback)
# ----------------------------
def linear_assignment(cost_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Return row_inds, col_inds arrays such that rows[i] matched to cols[i].
    Uses SciPy's linear_sum_assignment when available; otherwise greedy approximate.
    """
    try:
        from scipy.optimize import linear_sum_assignment
        r, c = linear_sum_assignment(cost_matrix)
        return np.array(r, dtype=int), np.array(c, dtype=int)
    except Exception:
        # Greedy fallback: choose smallest cost edges until rows/cols exhausted.
        m, n = cost_matrix.shape
        used_r, used_c = set(), set()
        edges = [(cost_matrix[i, j], i, j) for i in range(m) for j in range(n)]
        edges.sort(key=lambda x: x[0])
        rows, cols = [], []
        for cost, i, j in edges:
            if i in used_r or j in used_c:
                continue
            used_r.add(i); used_c.add(j)
            rows.append(i); cols.append(j)
            if len(used_r) == m or len(used_c) == n:
                break
        return np.array(rows, dtype=int), np.array(cols, dtype=int)

# ----------------------------
# Pose estimation (Kabsch / weighted)
# ----------------------------
def estimate_r_t_weighted(source_pts: np.ndarray, target_pts: np.ndarray, weights: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
    """
    Estimate rotation R and translation T that minimizes sum w_i || R*s_i + T - t_i ||^2.
    source_pts, target_pts: (N,3)
    weights: (N,) or None -> if None, uniform weights.
    Returns (R, T)
    """
    assert source_pts.shape == target_pts.shape
    N = source_pts.shape[0]
    if N == 0:
        return np.eye(3), np.zeros(3)
    if weights is None:
        weights = np.ones(N, dtype=float)
    weights = weights.astype(float)
    W = weights / (np.sum(weights) + 1e-12)

    # Weighted centroids
    cs = np.sum(source_pts * weights[:, None], axis=0) / (np.sum(weights) + 1e-12)
    ct = np.sum(target_pts * weights[:, None], axis=0) / (np.sum(weights) + 1e-12)

    # Center
    S = (source_pts - cs) * np.sqrt(weights[:, None])
    Tt = (target_pts - ct) * np.sqrt(weights[:, None])

    # Covariance
    H = S.T @ Tt
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    # fix reflection
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    T = ct - R @ cs
    return R, T

# ----------------------------
# Main algorithm: alternate optimization + matching
# ----------------------------
def match_with_pose_refinement(
    tracks: List[Obj],
    detections_pos: List[np.ndarray],    # list of (3,) arrays
    R_init: Optional[np.ndarray] = None,
    T_init: Optional[np.ndarray] = None,
    miss_cost_fn: Optional[Callable[[Obj], float]] = None,
    birth_cost_fn: Optional[Callable[[np.ndarray], float]] = None,
    max_iters: int = 3,
    gate_cost: float = 1e8
) -> Tuple[List[Tuple[int, Optional[int]]], np.ndarray, np.ndarray, float]:
    """
    Perform alternating optimization to find assignment and pose (R,T).
    Returns:
      - matches: list of tuples (track_idx, det_idx or None if unmatched)
      - R, T : final estimated transform mapping old track.pos -> detection pos (for matched pairs)
      - total_cost: value of objective
    Notes:
      - unmatched tracks correspond to assigned det_idx = None
      - unmatched detections are those not present in matches as any second element
    """
    m = len(tracks)
    k = len(detections_pos)
    if R_init is None:
        R = np.eye(3)
    else:
        R = R_init.copy()
    if T_init is None:
        T = np.zeros(3)
    else:
        T = T_init.copy()

    # default simple constant miss/birth costs if none provided
    if miss_cost_fn is None:
        miss_cost_fn = lambda tr: 100.0 + 10.0 * tr.missed_count
    if birth_cost_fn is None:
        birth_cost_fn = lambda det: 100.0

    total_cost = 0.0
    # alternating optimization
    for it in range(max_iters):
        # 1) Build pairwise cost matrix (m x k)
        if m == 0 or k == 0:
            C_pair = np.zeros((m, k))
        else:
            C_pair = np.zeros((m, k), dtype=float)
            for i, tr in enumerate(tracks):
                for j, dpos in enumerate(detections_pos):
                    C_pair[i, j] = compute_cost_with_conf_dirs(tr, dpos, R, T)

        # 2) Pad cost matrix with dummy rows/cols to allow misses/births
        n = max(m, k)
        C = np.full((n, n), fill_value=1e9, dtype=float)  # large default
        if m > 0 and k > 0:
            C[:m, :k] = C_pair
        # If m > k: add dummy columns for misses
        if m > k:
            miss_vec = np.array([miss_cost_fn(tr) for tr in tracks], dtype=float)
            C[:m, k:k + (m - k)] = miss_vec[:, None]
        # If k > m: add dummy rows for births
        if k > m:
            birth_vec = np.array([birth_cost_fn(dpos) for dpos in detections_pos], dtype=float)
            C[m:m + (k - m), :k] = birth_vec[None, :]

        # 3) Solve assignment on padded matrix
        row_inds, col_inds = linear_assignment(C)

        # interpret assignment: collect matched pairs only where both indices in bounds
        matched_pairs = []   # list of (track_idx, det_idx)
        unmatched_tracks = set(range(m))
        unmatched_dets = set(range(k))
        for r, c in zip(row_inds, col_inds):
            if r < m and c < k:
                if C[r, c] < gate_cost:
                    matched_pairs.append((r, c))
                    unmatched_tracks.discard(r)
                    unmatched_dets.discard(c)
                else:
                    # treated as no-match => prefer mark as miss (r < m, c >= k) but here c<k so
                    # put back into unmatched
                    pass
            elif r < m and c >= k:
                unmatched_tracks.discard(r)  # assigned to a dummy column => miss
            elif r >= m and c < k:
                unmatched_dets.discard(c)    # assigned to dummy row => birth
            else:
                # dummy-dummy
                pass

        # 4) Pose refinement using matched pairs
        if len(matched_pairs) >= 1:
            src = np.vstack([tracks[i].pos for (i, _) in matched_pairs])
            tgt = np.vstack([detections_pos[j] for (_, j) in matched_pairs])
            # weights: use inverse of current cost + epsilon so well-matched pairs influence more
            costs = np.array([C[pair[0], pair[1]] if pair[0] < C.shape[0] and pair[1] < C.shape[1] else 1e-3 for pair in matched_pairs])
            weights = 1.0 / (costs + 1e-6)
            # clamp weights to reasonable range
            weights = np.clip(weights, 1e-3, 1e3)
            R_new, T_new = estimate_r_t_weighted(src, tgt, weights)
            R, T = R_new, T_new
        else:
            # no matches -> keep previous R,T
            pass

    # After iterations, compute final matching & total cost
    # recompute final costs & assignment to report final result
    if m == 0 or k == 0:
        C_pair = np.zeros((m, k))
    else:
        C_pair = np.zeros((m, k), dtype=float)
        for i, tr in enumerate(tracks):
            for j, dpos in enumerate(detections_pos):
                C_pair[i, j] = compute_cost_with_conf_dirs(tr, dpos, R, T)

    n = max(m, k)
    C = np.full((n, n), fill_value=1e9, dtype=float)
    if m > 0 and k > 0:
        C[:m, :k] = C_pair
    if m > k:
        miss_vec = np.array([miss_cost_fn(tr) for tr in tracks], dtype=float)
        C[:m, k:k + (m - k)] = miss_vec[:, None]
    if k > m:
        birth_vec = np.array([birth_cost_fn(dpos) for dpos in detections_pos], dtype=float)
        C[m:m + (k - m), :k] = birth_vec[None, :]

    row_inds, col_inds = linear_assignment(C)

    # Interpret final assignment into matches list
    matches = []  # list of tuples (track_idx, det_idx or None)
    assigned_dets = set()
    assigned_tracks = set()
    for r, c in zip(row_inds, col_inds):
        if r < m and c < k and C[r, c] < gate_cost:
            matches.append((r, c))
            assigned_dets.add(c); assigned_tracks.add(r)
        elif r < m and c >= k:
            matches.append((r, None))
            assigned_tracks.add(r)
        elif r >= m and c < k:
            # born detection; we'll add births in returns but here represent as (None, det)
            # not adding here to matches for track->det
            assigned_dets.add(c)
        else:
            pass

    # include any unassigned tracks (if Hungarian gave smaller set)
    for i in range(m):
        if i not in assigned_tracks:
            # mark as missed
            matches.append((i, None))

    # optionally include unmatched detections as births (returned separately if needed)
    # compute total cost:
    total_cost = 0.0
    for t_idx, d_idx in matches:
        if d_idx is None:
            total_cost += miss_cost_fn(tracks[t_idx])
        else:
            total_cost += C_pair[t_idx, d_idx]
    # births (detections not matched to any track)
    for j in range(k):
        if j not in assigned_dets:
            total_cost += birth_cost_fn(detections_pos[j])

    return matches, R, T, total_cost

# ----------------------------
# Example: usage with synthetic data
# ----------------------------
def example():
    # create some tracks
    tracks = []
    # track 0: uncertain along Z axis (can move in depth cheaply)
    tracks.append(Obj(
        id=1,
        pos=np.array([0.0, 0.0, 0.0]),
        conf_dirs=[(np.array([0.0, 0.0, 1.0]), 0.1)],  # cheap to move along +Z
        last_seen=0.0,
        seen_count=5
    ))
    # track 1: stable (no cheap directions)
    tracks.append(Obj(
        id=2,
        pos=np.array([1.0, 0.0, 0.0]),
        conf_dirs=[],
        last_seen=0.0,
        seen_count=3
    ))

    # Suppose camera moved slightly so actual detections are old positions transformed + noise.
    # We'll pick a small rotation about Y and small translation
    angle = 0.1  # radians
    qw = np.cos(angle/2)
    qy = np.sin(angle/2)
    q = np.array([qw, 0.0, qy, 0.0])  # rotation about Y
    R_true = quat_to_rot_matrix(q)
    T_true = np.array([0.1, -0.05, 0.2])

    # Create detections by applying true transform to tracks and adding small noise
    detections_pos = []
    rng = np.random.default_rng(1)
    for tr in tracks:
        det = apply_r_t(R_true, T_true, tr.pos)
        det = det + 0.01 * rng.normal(size=3)  # tiny noise
        detections_pos.append(det)

    # Add an extra spurious detection (new object)
    detections_pos.append(np.array([2.0, 2.0, 2.0]))

    # Run matcher
    matches, R_est, T_est, total_cost = match_with_pose_refinement(
        tracks=tracks,
        detections_pos=detections_pos,
        max_iters=4
    )

    print("Matches (track_idx -> det_idx|None):", matches)
    print("Estimated R:", R_est)
    print("Estimated T:", T_est)
    print("True T:", T_true)
    print("Total cost:", total_cost)

    # Interpret matches and update tracks (simple on_match/on_miss/on_birth logic)
    next_tracks = []
    used_dets = set()
    next_id = max([tr.id for tr in tracks]) + 1
    for tr_idx, det_idx in matches:
        if det_idx is None:
            tr = tracks[tr_idx]
            tr.missed_count += 1
            next_tracks.append(tr)
        else:
            tr = tracks[tr_idx]
            detp = detections_pos[det_idx]
            tr.pos = detp.copy()   # naive overwrite; you might instead fuse
            tr.last_seen = 1.0
            tr.seen_count += 1
            tr.missed_count = 0
            next_tracks.append(tr)
            used_dets.add(det_idx)

    # births
    for j in range(len(detections_pos)):
        if j not in used_dets:
            next_tracks.append(Obj(id=next_id, pos=detections_pos[j].copy(), last_seen=1.0, seen_count=1))
            next_id += 1

    print("Next tracks:")
    for t in next_tracks:
        print(f"  id={t.id}, pos={t.pos}, seen={t.seen_count}, missed={t.missed_count}")

if __name__ == "__main__":
    example()

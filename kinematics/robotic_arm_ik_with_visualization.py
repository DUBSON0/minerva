"""
Robust 6-DOF inverse kinematics (IK) module with clean API.

Features
- Forward kinematics via recursive point rotation from fixed initial joint positions `p_home[j]`.
- Accepts joint inputs as angles (radians) or per-joint quaternions; quaternions are projected
  onto the joint's axis for initialization.
- Damped Least-Squares (DLS) Jacobian IK with adaptive damping for singularity robustness.
- Orientation error uses quaternion logarithm map (axis-angle minimal 3D vector).
- Easily extensible and self-contained (depends only on numpy).

Definitions
- Let N = 6 (can work with N>=2 in general).
- `p_home[j]` (shape (N,3)) are joint pivot positions in the base/world frame at the initial pose.
- `a_home[j]` (shape (N,3)) are joint rotation axes at the initial pose, unit vectors in base/world.
- The chain applies joint i rotation about axis `a_i` through point `p_i` and rotates all downstream
  points recursively: translate to `p_i`, rotate, translate back.
- End-effector (TCP) is at `p_N = p_last + R_last * tool_offset` with `tool_offset` in the last
  joint's local frame at home pose.
"""

from dataclasses import dataclass
from typing import Optional, Tuple, List

import numpy as np


# -----------------------------
# Quaternion / SE(3) utilities
# -----------------------------

def _safe_norm(v: np.ndarray, eps: float = 1e-12) -> float:
    n = float(np.linalg.norm(v))
    return n if n > eps else 0.0


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion [w, x, y, z]."""
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n <= 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product, both [w,x,y,z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by unit quaternion q (w, x, y, z)."""
    q = quat_normalize(q)
    w, x, y, z = q
    # Equivalent to R(q) @ v using quaternion math
    vx, vy, vz = v
    # From optimized formula
    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z
    wx = w * x
    wy = w * y
    wz = w * z
    xy = x * y
    xz = x * z
    yz = y * z
    rx = (ww + xx - yy - zz) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz
    ry = 2.0 * (xy + wz) * vx + (ww - xx + yy - zz) * vy + 2.0 * (yz - wx) * vz
    rz = 2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (ww - xx - yy + zz) * vz
    return np.array([rx, ry, rz])


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = _safe_norm(axis)
    if n == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    half = 0.5 * float(angle)
    s = np.sin(half)
    return quat_normalize(np.array([np.cos(half), axis[0] * s, axis[1] * s, axis[2] * s]))


def quat_log(q: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Quaternion logarithm map to R^3 (axis * angle). q must be unit and represent rotation.

    Returns minimal 3D orientation error vector.
    """
    q = quat_normalize(q)
    w, x, y, z = q
    v = np.array([x, y, z])
    sin_half = _safe_norm(v)
    if sin_half < eps:
        # very small rotation; use first-order approximation: 2*v
        return 2.0 * v
    angle = 2.0 * np.arctan2(sin_half, max(w, -w))
    axis = v / sin_half
    return axis * angle


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX convention: yaw (Z), pitch (Y), roll (X). Returns [w,x,y,z]."""
    cz = np.cos(yaw * 0.5)
    sz = np.sin(yaw * 0.5)
    cy = np.cos(pitch * 0.5)
    sy = np.sin(pitch * 0.5)
    cx = np.cos(roll * 0.5)
    sx = np.sin(roll * 0.5)
    w = cz * cy * cx + sz * sy * sx
    x = cz * cy * sx - sz * sy * cx
    y = cz * sy * cx + sz * cy * sx
    z = sz * cy * cx - cz * sy * sx
    return quat_normalize(np.array([w, x, y, z]))


def project_quat_angle_onto_axis(q: np.ndarray, axis: np.ndarray) -> float:
    """Return the signed rotation angle (radians) of quaternion q about unit axis.

    For a unit quaternion q = [w, v], the component about axis a is phi where
    sin(phi/2) = dot(v, a) and cos(phi/2) = w. This extracts the joint's angle
    even if q includes small off-axis components (they are ignored).
    """
    q = quat_normalize(q)
    a = np.asarray(axis, dtype=float)
    a_n = _safe_norm(a)
    if a_n == 0.0:
        return 0.0
    a = a / a_n
    w, x, y, z = q
    s = x * a[0] + y * a[1] + z * a[2]
    return 2.0 * np.arctan2(s, w)


# -----------------------------
# Data structures
# -----------------------------


@dataclass
class FKResult:
    joint_positions: np.ndarray  # (N,3)
    joint_axes_world: np.ndarray  # (N,3)
    end_effector_position: np.ndarray  # (3,)
    end_effector_orientation: np.ndarray  # quaternion [w,x,y,z]


@dataclass
class IKSolution:
    success: bool
    angles: np.ndarray
    fk: FKResult
    iters: int
    message: str
    history: Optional["IKHistory"] = None


@dataclass
class IKHistory:
    iterations: List[int]
    angle_history: List[np.ndarray]           # list of (N,) arrays
    eef_positions: List[np.ndarray]           # list of (3,)
    pos_err_norms: List[float]
    ori_err_norms: List[float]
    total_err_norms: List[float]
    step_norms: List[float]
    lambdas: List[float]


class SixDOFArmIK:
    """Inverse kinematics for an N-DOF serial arm (default N=6).

    Parameters
    - p_home: (N,3) home joint pivot positions in base/world frame.
    - a_home: (N,3) unit joint rotation axes in base/world frame at home.
    - tool_offset: (3,) TCP offset in the last joint's local frame (home orientation).

    Notes
    - Joints are revolute about given axes; prismatic is not modeled.
    - Forward kinematics matches the recursive rotation spec.
    """

    def __init__(
        self,
        p_home: np.ndarray,
        a_home: np.ndarray,
        tool_offset: Optional[np.ndarray] = None,
        lower_limits: Optional[np.ndarray] = None,
        upper_limits: Optional[np.ndarray] = None,
    ) -> None:
        p_home = np.asarray(p_home, dtype=float)
        a_home = np.asarray(a_home, dtype=float)
        if p_home.ndim != 2 or p_home.shape[1] != 3:
            raise ValueError("p_home must have shape (N,3)")
        if a_home.shape != p_home.shape:
            raise ValueError("a_home must have shape (N,3) and match p_home")
        self.N: int = int(p_home.shape[0])
        if self.N < 2:
            raise ValueError("Arm must have at least 2 joints")
        # Normalize axes
        a_norms = np.linalg.norm(a_home, axis=1)
        if np.any(a_norms <= 0.0):
            raise ValueError("All joint axes must be non-zero vectors")
        self.p_home: np.ndarray = p_home.copy()
        self.a_home: np.ndarray = (a_home / a_norms[:, None]).copy()
        self.tool_offset: np.ndarray = (
            np.zeros(3, dtype=float) if tool_offset is None else np.asarray(tool_offset, dtype=float)
        )
        # Internal state: angles (radians)
        self.angles: np.ndarray = np.zeros(self.N, dtype=float)
        # Joint limits (default: [-90°, +90°])
        if lower_limits is None:
            self.lower_limits = -0.5 * np.pi * np.ones(self.N, dtype=float)
        else:
            ll = np.asarray(lower_limits, dtype=float)
            if ll.shape != (self.N,):
                raise ValueError(f"lower_limits must have shape ({self.N},)")
            self.lower_limits = ll
        if upper_limits is None:
            self.upper_limits = 0.5 * np.pi * np.ones(self.N, dtype=float)
        else:
            ul = np.asarray(upper_limits, dtype=float)
            if ul.shape != (self.N,):
                raise ValueError(f"upper_limits must have shape ({self.N},)")
            self.upper_limits = ul
        if np.any(self.lower_limits >= self.upper_limits):
            raise ValueError("Each lower limit must be strictly less than upper limit")

    # -----------------------------
    # State setting
    # -----------------------------
    def set_state_from_angles(self, angles: np.ndarray) -> None:
        angles = np.asarray(angles, dtype=float)
        if angles.shape != (self.N,):
            raise ValueError(f"angles must have shape ({self.N},)")
        self.angles = np.clip(angles, self.lower_limits, self.upper_limits)

    def set_state_from_quaternions(self, quats: np.ndarray) -> None:
        quats = np.asarray(quats, dtype=float)
        if quats.shape != (self.N, 4):
            raise ValueError(f"quaternions must have shape ({self.N}, 4)")
        angles = np.zeros(self.N, dtype=float)
        for i in range(self.N):
            angles[i] = project_quat_angle_onto_axis(quat_normalize(quats[i]), self.a_home[i])
        self.angles = np.clip(angles, self.lower_limits, self.upper_limits)

    def set_joint_limits(self, lower_limits: np.ndarray, upper_limits: np.ndarray) -> None:
        ll = np.asarray(lower_limits, dtype=float)
        ul = np.asarray(upper_limits, dtype=float)
        if ll.shape != (self.N,) or ul.shape != (self.N,):
            raise ValueError(f"limits must each have shape ({self.N},)")
        if np.any(ll >= ul):
            raise ValueError("Each lower limit must be strictly less than upper limit")
        self.lower_limits = ll
        self.upper_limits = ul
        # Clamp current state to the new limits
        self.angles = np.clip(self.angles, self.lower_limits, self.upper_limits)

    # -----------------------------
    # Forward kinematics (recursive)
    # -----------------------------
    def forward_kinematics(self, angles: Optional[np.ndarray] = None) -> FKResult:
        if angles is None:
            angles = self.angles
        else:
            angles = np.asarray(angles, dtype=float)
            if angles.shape != (self.N,):
                raise ValueError(f"angles must have shape ({self.N},)")

        # Initialize joint positions and cumulative orientation
        P = self.p_home.copy()  # (N,3)
        joint_axes_world = np.zeros_like(self.a_home)
        q_cum = np.array([1.0, 0.0, 0.0, 0.0])  # world orientation of downstream frame

        for i in range(self.N):
            # Axis in world after previous joints
            a_world = quat_rotate_vector(q_cum, self.a_home[i])
            joint_axes_world[i] = a_world

            # Rotate all downstream joints about point P[i] and axis a_world by angle[i]
            q_i = quat_from_axis_angle(a_world, angles[i])
            for j in range(i + 1, self.N):
                rel = P[j] - P[i]
                P[j] = P[i] + quat_rotate_vector(q_i, rel)

            # Update downstream orientation
            q_cum = quat_normalize(quat_multiply(q_i, q_cum))

        # End-effector pose
        p_end = P[-1] + quat_rotate_vector(q_cum, self.tool_offset)
        q_end = quat_normalize(q_cum)

        return FKResult(
            joint_positions=P,
            joint_axes_world=joint_axes_world,
            end_effector_position=p_end,
            end_effector_orientation=q_end,
        )

    # -----------------------------
    # Jacobian
    # -----------------------------
    def compute_jacobian(self, angles: Optional[np.ndarray] = None) -> Tuple[np.ndarray, FKResult]:
        fk = self.forward_kinematics(angles)
        p_e = fk.end_effector_position
        P = fk.joint_positions
        A = fk.joint_axes_world
        J = np.zeros((6, self.N), dtype=float)
        for i in range(self.N):
            ai = A[i]
            pi = P[i]
            J[0:3, i] = np.cross(ai, (p_e - pi))  # linear part
            J[3:6, i] = ai  # angular part
        return J, fk

    # -----------------------------
    # IK (Damped Least Squares)
    # -----------------------------
    def solve_ik(
        self,
        target_position: np.ndarray,
        target_orientation_quat: np.ndarray,
        initial_angles: Optional[np.ndarray] = None,
        max_iters: int = 200,
        position_weight: float = 1.0,
        orientation_weight: float = 0.5,
        damping: float = 1e-3,
        min_damping: float = 1e-6,
        step_limit: float = 0.5,
        pos_tol: float = 1e-2,
        ori_tol: float = 1e-2,
        record_history: bool = False,
    ) -> IKSolution:
        """Solve IK to reach target pose.

        target_orientation_quat: Quaternion [w,x,y,z].
        step_limit: Max norm of delta-theta per iteration (radians).
        Weights balance position/orientation error in the least-squares.
        """
        target_p = np.asarray(target_position, dtype=float).reshape(3)
        target_q = quat_normalize(np.asarray(target_orientation_quat, dtype=float).reshape(4))
        if initial_angles is None:
            theta = self.angles.copy()
        else:
            theta = np.asarray(initial_angles, dtype=float)
            if theta.shape != (self.N,):
                raise ValueError(f"initial_angles must have shape ({self.N},)")
        # Respect limits from the start
        theta = np.clip(theta, self.lower_limits, self.upper_limits)

        lam = float(damping)
        prev_err = None

        history: Optional[IKHistory] = None
        if record_history:
            history = IKHistory(
                iterations=[],
                angle_history=[],
                eef_positions=[],
                pos_err_norms=[],
                ori_err_norms=[],
                total_err_norms=[],
                step_norms=[],
                lambdas=[],
            )

        for it in range(1, max_iters + 1):
            J, fk = self.compute_jacobian(theta)
            p = fk.end_effector_position
            q = fk.end_effector_orientation

            e_pos = target_p - p
            # Orientation error: q_err rotates current -> target
            q_err = quat_multiply(target_q, quat_conjugate(q))
            e_rot = quat_log(q_err)

            # Stack error with weights
            e = np.hstack((position_weight * e_pos, orientation_weight * e_rot))

            # Weighted DLS: (J^T W J + lam^2 I) delta = J^T W e
            W = np.diag([
                position_weight,
                position_weight,
                position_weight,
                orientation_weight,
                orientation_weight,
                orientation_weight,
            ])
            JT_W = J.T @ W
            A = JT_W @ J + (lam * lam) * np.eye(self.N)
            b = JT_W @ e

            try:
                delta = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                # Heavier damping fallback
                A = JT_W @ J + (max(lam * 10.0, 1e-2) ** 2) * np.eye(self.N)
                delta = np.linalg.lstsq(A, b, rcond=None)[0]

            # Limit step size
            d_norm = float(np.linalg.norm(delta))
            if d_norm > step_limit and d_norm > 0.0:
                delta = (delta / d_norm) * step_limit

            theta = theta + delta
            # Enforce joint limits after each update
            theta = np.clip(theta, self.lower_limits, self.upper_limits)

            err_pos_norm = float(np.linalg.norm(e_pos))
            err_ori_norm = float(np.linalg.norm(e_rot))
            err_total = err_pos_norm + err_ori_norm

            if history is not None:
                history.iterations.append(it)
                history.angle_history.append(theta.copy())
                history.eef_positions.append(p.copy())
                history.pos_err_norms.append(err_pos_norm)
                history.ori_err_norms.append(err_ori_norm)
                history.total_err_norms.append(err_total)
                history.step_norms.append(d_norm)
                history.lambdas.append(lam)

            # Check convergence
            if err_pos_norm <= pos_tol and err_ori_norm <= ori_tol:
                self.angles = np.clip(theta, self.lower_limits, self.upper_limits)
                final_fk = self.forward_kinematics(theta)
                return IKSolution(
                    success=True,
                    angles=theta,
                    fk=final_fk,
                    iters=it,
                    message="Converged",
                    history=history,
                )

            # Adaptive damping: increase if error grew, otherwise decrease softly
            if prev_err is not None:
                if err_total > prev_err * 1.001:  # grew slightly or worse
                    lam = min(1e1, lam * 5.0)
                else:
                    lam = max(min_damping, lam / 1.2)
            prev_err = err_total

        # Max iters reached
        self.angles = theta.copy()
        final_fk = self.forward_kinematics(theta)
        return IKSolution(
            success=False,
            angles=theta,
            fk=final_fk,
            iters=max_iters,
            message="Max iterations reached without convergence",
            history=history,
        )


def make_ik_plots(history: IKHistory, angle_labels: Optional[List[str]] = None, show: bool = True, save_prefix: Optional[str] = None):
    """Plot IK convergence diagnostics from IKHistory.

    Plots:
    - Error norms (position, orientation, total) and step norms.
    - Joint angles vs iteration.
    - 3D end-effector trajectory.
    """
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception as exc:
        raise RuntimeError("matplotlib is required for visualization. Install it via pip.") from exc

    iters = history.iterations

    # Figure 1: Errors and step norm
    fig1, axs1 = plt.subplots(2, 2, figsize=(10, 6))
    axs1 = axs1.ravel()
    axs1[0].plot(iters, history.pos_err_norms, label="pos err [m]")
    axs1[0].set_title("Position error norm")
    axs1[0].set_xlabel("iter")
    axs1[0].set_ylabel("meters")
    axs1[0].grid(True)

    axs1[1].plot(iters, history.ori_err_norms, color="orange", label="ori err [rad]")
    axs1[1].set_title("Orientation error norm")
    axs1[1].set_xlabel("iter")
    axs1[1].set_ylabel("radians")
    axs1[1].grid(True)

    axs1[2].plot(iters, history.total_err_norms, color="green", label="total err")
    axs1[2].set_title("Total error (pos+ori)")
    axs1[2].set_xlabel("iter")
    axs1[2].grid(True)

    axs1[3].plot(iters, history.step_norms, color="red", label="step norm [rad]")
    axs1[3].set_title("Step norm (||Δθ||)")
    axs1[3].set_xlabel("iter")
    axs1[3].set_ylabel("radians")
    axs1[3].grid(True)

    fig1.tight_layout()

    if save_prefix is not None:
        fig1.savefig(f"{save_prefix}_errors.png", dpi=160)

    # Figure 2: Angles
    angles_mat = np.vstack(history.angle_history)  # (T, N)
    T, N = angles_mat.shape
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    for j in range(N):
        label = angle_labels[j] if angle_labels and j < len(angle_labels) else f"θ{j+1}"
        ax2.plot(iters, angles_mat[:, j], label=label)
    ax2.set_title("Joint angles vs iteration")
    ax2.set_xlabel("iter")
    ax2.set_ylabel("radians")
    ax2.grid(True)
    ax2.legend(ncol=min(N, 3))
    fig2.tight_layout()

    if save_prefix is not None:
        fig2.savefig(f"{save_prefix}_angles.png", dpi=160)

    # Figure 3: 3D trajectory of end-effector
    fig3 = plt.figure(figsize=(6, 6))
    ax3 = fig3.add_subplot(111, projection="3d")
    P = np.vstack(history.eef_positions)
    ax3.plot(P[:, 0], P[:, 1], P[:, 2], "-b", label="EEF path")
    ax3.scatter(P[0, 0], P[0, 1], P[0, 2], color="green", s=40, label="start")
    ax3.scatter(P[-1, 0], P[-1, 1], P[-1, 2], color="red", s=40, label="end")
    ax3.set_title("End-effector trajectory")
    ax3.set_xlabel("X [m]")
    ax3.set_ylabel("Y [m]")
    ax3.set_zlabel("Z [m]")
    ax3.legend(loc="best")
    fig3.tight_layout()

    if save_prefix is not None:
        fig3.savefig(f"{save_prefix}_trajectory.png", dpi=160)

    if show:
        plt.show()
    else:
        plt.close(fig1)
        plt.close(fig2)
        plt.close(fig3)


__all__ = [
    "SixDOFArmIK",
    "IKSolution",
    "FKResult",
    "IKHistory",
    "quat_from_axis_angle",
    "quat_from_euler",
    "quat_multiply",
    "quat_conjugate",
    "quat_normalize",
    "make_ik_plots",
]



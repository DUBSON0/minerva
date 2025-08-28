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

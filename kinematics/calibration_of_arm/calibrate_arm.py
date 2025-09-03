"""
Calibration utility for estimating home joint pivots/axes from measurements.

You provide multiple measurements. Each measurement contains:
- joint_positions_world: (N,3) measured joint pivot positions in world/base frame
- joint_axes_world: (N,3) measured joint axes in world/base frame (unit vectors)
- angles: (N,) joint angles (radians) at the time of measurement

This script inverts the rotations to estimate the home geometry (p_home, a_home)
for each measurement, then averages across measurements for robustness.

How it works (brief):
- Positions: reverse the forward-kinematics rotation sequence in reverse order,
  rotating downstream pivots back by the inverse rotation around each measured
  joint axis by the measured joint angle.
- Axes: for joint i, rotate the measured world axis back by the cumulative
  inverse rotation of upstream joints (computed from measured axes and angles).

Outputs:
- Averaged p_home (N,3) and a_home (N,3) suitable for use with SixDOFArmIK.

Note: Axes are averaged with sign disambiguation and re-normalized.
"""

from dataclasses import dataclass
from typing import List, Tuple, Dict, Any
import json
from pathlib import Path

import numpy as np

# Use local calibration helpers (keeps this script self-contained in this folder)
from some_functions_for_calibration import (
    quat_from_axis_angle,
    quat_rotate_vector,
    quat_conjugate,
    quat_normalize,
)


@dataclass
class CalibrationMeasurement:
    joint_positions_world: np.ndarray  # (N,3)
    joint_axes_world: np.ndarray       # (N,3), unit
    angles: np.ndarray                 # (N,), radians


def _validate_meas(m: CalibrationMeasurement) -> None:
    P = np.asarray(m.joint_positions_world, dtype=float)
    A = np.asarray(m.joint_axes_world, dtype=float)
    theta = np.asarray(m.angles, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3:
        raise ValueError("joint_positions_world must be (N,3)")
    if A.shape != P.shape:
        raise ValueError("joint_axes_world must be same shape as joint_positions_world (N,3)")
    if theta.shape != (P.shape[0],):
        raise ValueError("angles must be shape (N,)")
    # Normalize axes
    norms = np.linalg.norm(A, axis=1)
    if np.any(norms <= 0):
        raise ValueError("All joint axes must be non-zero vectors")
    m.joint_axes_world = (A / norms[:, None]).astype(float)
    m.joint_positions_world = P.astype(float)
    m.angles = theta.astype(float)


def estimate_home_from_measurement(m: CalibrationMeasurement) -> Tuple[np.ndarray, np.ndarray]:
    """Invert measured configuration back to the home geometry.

    Returns (p_home_est, a_home_est) both shaped (N,3).
    """
    _validate_meas(m)
    P = m.joint_positions_world.copy()
    A_world = m.joint_axes_world.copy()
    theta = m.angles.copy()
    N = P.shape[0]

    # 1) Reverse positions by undoing rotations in reverse order
    for i in range(N - 1, -1, -1):
        q_i = quat_from_axis_angle(A_world[i], theta[i])
        q_inv = quat_conjugate(quat_normalize(q_i))
        for j in range(i + 1, N):
            rel = P[j] - P[i]
            P[j] = P[i] + quat_rotate_vector(q_inv, rel)

    p_home_est = P  # after undoing all rotations

    # 2) Recover home axes by rotating measured axes back by cumulative inverse of upstream joints
    a_home_est = np.zeros_like(A_world)
    q_cum = np.array([1.0, 0.0, 0.0, 0.0])  # cumulative rotation of upstream joints (world)
    for i in range(N):
        # a_home[i] = R(q_cum)^{-1} * A_world[i]
        a_home_est[i] = quat_rotate_vector(quat_conjugate(q_cum), A_world[i])
        # Update cumulative using measured world axis and angle for this joint
        q_i = quat_from_axis_angle(A_world[i], theta[i])
        q_cum = quat_normalize(np.array([
            q_i[0] * q_cum[0] - q_i[1] * q_cum[1] - q_i[2] * q_cum[2] - q_i[3] * q_cum[3],
            q_i[0] * q_cum[1] + q_i[1] * q_cum[0] + q_i[2] * q_cum[3] - q_i[3] * q_cum[2],
            q_i[0] * q_cum[2] - q_i[1] * q_cum[3] + q_i[2] * q_cum[0] + q_i[3] * q_cum[1],
            q_i[0] * q_cum[3] + q_i[1] * q_cum[2] - q_i[2] * q_cum[1] + q_i[3] * q_cum[0],
        ]))

    # Normalize axes
    a_home_est /= np.linalg.norm(a_home_est, axis=1, keepdims=True)
    return p_home_est, a_home_est


def average_home_estimates(estimates: List[Tuple[np.ndarray, np.ndarray]]) -> Tuple[np.ndarray, np.ndarray]:
    """Average multiple (p_home, a_home) estimates.

    - Positions: simple arithmetic mean.
    - Axes: mean with sign disambiguation (align to the first estimate per joint), then renormalize.
    """
    if len(estimates) == 0:
        raise ValueError("No estimates provided")
    p_list = [e[0] for e in estimates]
    a_list = [e[1] for e in estimates]
    N = p_list[0].shape[0]

    # Positions: average
    p_home = np.mean(np.stack(p_list, axis=0), axis=0)

    # Axes: sign-aware average per joint
    a_home = np.zeros((N, 3), dtype=float)
    ref = a_list[0]
    for i in range(N):
        acc = np.zeros(3, dtype=float)
        for a in a_list:
            v = a[i]
            if np.dot(v, ref[i]) < 0.0:
                v = -v
            acc += v
        n = np.linalg.norm(acc)
        if n == 0.0:
            a_home[i] = ref[i]
        else:
            a_home[i] = acc / n
    return p_home, a_home


def calibrate_from_measurements(measurements: List[CalibrationMeasurement]) -> Tuple[np.ndarray, np.ndarray]:
    """High-level helper: estimate home geometry from multiple measurements."""
    estimates = [estimate_home_from_measurement(m) for m in measurements]
    return average_home_estimates(estimates)


def load_measurements_from_json(path: str) -> List[CalibrationMeasurement]:
    """Load measurements from a JSON file.

    Accepted formats:
    1) {"measurements": [{"joint_positions_world": [[...],[...]], "joint_axes_world": [[...],[...]], "angles": [...]}, ...]}
    2) [{...}, {...}, ...]  (list directly)
    """
    p = Path(path)
    data = json.loads(p.read_text())
    if isinstance(data, dict) and "measurements" in data:
        entries = data["measurements"]
    elif isinstance(data, list):
        entries = data
    else:
        raise ValueError("Invalid JSON structure for measurements.")

    result: List[CalibrationMeasurement] = []
    for idx, e in enumerate(entries):
        try:
            P = np.asarray(e["joint_positions_world"], dtype=float)
            A = np.asarray(e["joint_axes_world"], dtype=float)
            angles = np.asarray(e["angles"], dtype=float)
        except KeyError as ex:
            raise ValueError(f"Entry {idx} missing key: {ex}")
        result.append(CalibrationMeasurement(P, A, angles))
    return result


def pretty_print_params(p_home: np.ndarray, a_home: np.ndarray) -> None:
    np.set_printoptions(precision=6, suppress=True)
    print("p_home (meters):")
    print(p_home)
    print()
    print("a_home (unit axes in base frame):")
    print(a_home)


def main() -> None:
    # Edit these paths as needed. Keep it simple: just run this file after editing measurements.json
    base_dir = Path(__file__).resolve().parent
    MEASUREMENTS_JSON = base_dir / "measurements.json"
    SAVE_JSON = base_dir / "calibrated_home.json"  # or set to None

    measurements = load_measurements_from_json(MEASUREMENTS_JSON)

    # Validate consistent N
    Ns = {m.joint_positions_world.shape[0] for m in measurements}
    if len(Ns) != 1:
        raise ValueError(f"All measurements must have same N. Found: {Ns}")

    p_home, a_home = calibrate_from_measurements(measurements)
    pretty_print_params(p_home, a_home)

    if SAVE_JSON:
        out = {
            "p_home": p_home.tolist(),
            "a_home": a_home.tolist(),
        }
        SAVE_JSON.write_text(json.dumps(out, indent=2))
        print(f"\nSaved to {str(SAVE_JSON)}")


if __name__ == "__main__":
    main()



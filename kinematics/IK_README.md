## 6‑DOF Arm IK – Practical User Guide

This is a short, practical guide for using `minerva/kinematics/robotic_arm_ik_with_visualization.py`.
No quaternion background required – treat it like a black box for: set geometry → solve IK → (optionally) plot.

### What it does (in plain terms)
- Computes joint angles to reach a target hand position and orientation.
- Uses a robust “Jacobian-based” gradient method with damping to handle singularities.
- Can record per‑iteration data and generate simple plots to help you tune.

### Key things you provide
- `p_home`: shape (N,3). Joint pivot positions at the home pose (base/world frame). Units: meters.
- `a_home`: shape (N,3). Unit rotation axes for each joint at home, expressed in the base/world frame.
- `tool_offset`: shape (3,). End‑effector offset from the last joint’s pivot (in last joint’s home frame).

Tip: Angles default to zeros. If that puts your arm near a straight line, IK can struggle (singularity). Set a reasonable initial angle vector.

### Important functions (day‑to‑day)
- `SixDOFArmIK(p_home, a_home, tool_offset=None)`: create the solver object.
- `set_state_from_angles(angles)`: set current joint angles (radians) before solving.
- `forward_kinematics(angles=None)`: get current end‑effector pose.
- `solve_ik(target_position, target_orientation_quat, ..., record_history=False)`: solve for angles.
- `make_ik_plots(history, ...)`: plot errors, joint angles, and trajectory if you recorded history.

### Minimal usage
1) Create the solver
2) Optionally set initial angles
3) Solve IK
4) Plot (optional)

Example (pseudo‑code):
```python
from minerva.kinematics.robotic_arm_ik_with_visualization import SixDOFArmIK, quat_from_axis_angle, make_ik_plots
import numpy as np

p_home = np.array([...], dtype=float)  # (N,3)
a_home = np.array([...], dtype=float)  # (N,3), each row unit length
tool_offset = np.array([0, 0, 0.1])

arm = SixDOFArmIK(p_home, a_home, tool_offset=tool_offset)
arm.set_state_from_angles(np.array([0.3, 0.2, 0.4, 0.3, 0.1, 0.2]))  # optional but recommended

target_p = np.array([0.3, 0.35, 0.1])
target_q = quat_from_axis_angle([0,1,0], np.deg2rad(30))  # simple 30° around Y (or use identity [1,0,0,0])

sol = arm.solve_ik(target_p, target_q, max_iters=200, position_weight=1.0, orientation_weight=0.5,
                   damping=1e-3, step_limit=0.4, pos_tol=1e-3, ori_tol=1e-3, record_history=True)

if sol.history is not None:
    make_ik_plots(sol.history, angle_labels=["θ1","θ2","θ3","θ4","θ5","θ6"])  # optional plots
```

### Recording vs not recording
- Set `record_history=True` in `solve_ik(...)` to collect per‑iteration data and enable plots.
- Set `record_history=False` (default) for fastest solves with no plotting data.

### Common pitfalls and what to try
- Huge errors or no convergence:
  - Check units (meters everywhere) and that `a_home` rows are unit vectors.
  - Ensure target is reachable and expressed in the same base frame as `p_home`.
  - Start from a non‑singular initial pose: call `set_state_from_angles(...)` with a bent configuration.
  - Temporarily set `orientation_weight=0.0` to focus on position first.
- Slow or oscillatory convergence:
  - Increase `damping` slightly (e.g., to 5e-3) and/or reduce `step_limit`.
  - Balance weights so position and orientation improve together (try 1.0 and 0.5).

### Rough idea of the math
The solver moves joint angles in small steps that reduce a combined position+orientation error using the Jacobian (sensitivity of the hand pose to each joint). Damping is used so that near singularities (e.g., a straightened arm) updates remain stable.

That’s all you need to use it effectively. For details, read the docstrings in the code.



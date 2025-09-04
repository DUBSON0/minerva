import runpy
from pathlib import Path
import numpy as np


def main():
    # Load the IK with visualization helpers
    base_dir = Path(__file__).resolve().parent
    ik_path = base_dir / "robotic_arm_ik_with_visualization.py"
    ik = runpy.run_path(str(ik_path))

    # Example geometry (replace with your robot)
    p_home = np.array([
    [
      -0.06,
      -0.031,
      0.085
    ],
    [
      -0.0391348729104512,
      -0.0033584285624536844,
      0.103
    ],
    [
      -0.14410648290877814,
      -0.026063430893342284,
      0.06592906848333195
    ],
    [
      -0.06069237478063112,
      -0.03495558345831055,
      0.1694086132636755
    ],
    [
      0.018218694429760027,
      -0.019865074412758383,
      0.15885779846959616
    ]
  ], dtype=float)

    a_home = np.array([
    [
      0.0,
      0.0,
      1.0
    ],
    [
      0.2588190451025208,
      0.9659258262890683,
      0.0
    ],
    [
      -0.25881904510252074,
      -0.9659258262890683,
      -1.387778780781446e-17
    ],
    [
      -0.25881904510252074,
      -0.9659258262890684,
      -1.3877787807814457e-17
    ],
    [
      0.875426098065593,
      -0.23456971600980447,
      -0.42261826174069955
    ]
  ], dtype=float)

    tool_offset = np.array([0.0, 0.0, 0.1]) #needs to be corrected

    arm = ik["SixDOFArmIK"](p_home, a_home, tool_offset=tool_offset)
    # Set servo limits to [0, pi] for all joints
    N = p_home.shape[0]
    arm.set_joint_limits(np.zeros(N), np.pi * np.ones(N))
    arm.set_state_from_angles(np.array([1.3, 1.2, 1.4, 1.3, 1.1])) #would be better to set current position of the servos.

    target_p = np.array([-0.1, 0.3, 0.1])
    target_q = ik["quat_from_axis_angle"]([0, 1, 0], np.deg2rad(30))

    # Toggle recording here
    record = False  # set to False for speed, no plots

    sol = arm.solve_ik(
        target_p,
        target_q,
        max_iters=200,
        position_weight=1.0,
        orientation_weight=0, #neglect orientation
        damping=1e-3,
        step_limit=0.4,
        pos_tol=1e-3,
        ori_tol=10, #basically, neglect orientation - seems 5dof isnt enough
        record_history=record,
    )

    print("Success:", sol.success, "iters:", sol.iters)
    print("Angles:", sol.angles)
    print("EEF position:", sol.fk.end_effector_position)

    if record and sol.history is not None:
        ik["make_ik_plots"](sol.history, angle_labels=[f"θ{i+1}" for i in range(6)], show=True, save_prefix=None)


if __name__ == "__main__":
    main()



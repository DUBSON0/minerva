import runpy
import numpy as np


def main():
    # Load the IK with visualization helpers
    ik_path = "/Users/irakli/Desktop/Minerva/minerva/kinematics/robotic_arm_ik_with_visualization.py"
    ik = runpy.run_path(ik_path)

    # Example geometry (replace with your robot)
    p_home = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.3],
        [0.0, 0.2, 0.3],
        [0.0, 0.4, 0.3],
        [0.0, 0.6, 0.3],
        [0.0, 0.8, 0.3],
    ], dtype=float)

    a_home = np.array([
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
    ], dtype=float)

    tool_offset = np.array([0.0, 0.0, 0.1])

    arm = ik["SixDOFArmIK"](p_home, a_home, tool_offset=tool_offset)
    arm.set_state_from_angles(np.array([0.3, 0.2, 0.4, 0.3, 0.1, 0.2]))

    target_p = np.array([0.3, 0.35, 0.1])
    target_q = ik["quat_from_axis_angle"]([0, 1, 0], np.deg2rad(30))

    # Toggle recording here
    record = True  # set to False for speed, no plots

    sol = arm.solve_ik(
        target_p,
        target_q,
        max_iters=200,
        position_weight=1.0,
        orientation_weight=0.5,
        damping=1e-3,
        step_limit=0.4,
        pos_tol=1e-3,
        ori_tol=1e-3,
        record_history=record,
    )

    print("Success:", sol.success, "iters:", sol.iters)
    print("Angles:", sol.angles)
    print("EEF position:", sol.fk.end_effector_position)

    if record and sol.history is not None:
        ik["make_ik_plots"](sol.history, angle_labels=[f"θ{i+1}" for i in range(6)], show=True, save_prefix=None)


if __name__ == "__main__":
    main()



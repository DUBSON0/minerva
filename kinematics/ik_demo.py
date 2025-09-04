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
      0.045720000000000004,
      -0.041275000000000006,
      0.0
    ],
    [
      0.06388325564877748,
      -0.04864857235247293,
      0.009525
    ],
    [
      0.08181709474296417,
      -0.06788026024769611,
      0.10766306395096936
    ],
    [
      0.04024687133100862,
      -0.023301653391979503,
      -0.029240858331350802
    ],
    [
      0.0022025553132980075,
      0.024944578962753483,
      -0.004501838902163992
    ]
  ], dtype=float)

    a_home = np.array([
    [
      0.0,
      0.0,
      1.0
    ],
    [
      0.7313537016191705,
      0.6819983600624986,
      0.0
    ],
    [
      0.7313537016191705,
      0.6819983600624986,
      -5.551115123125783e-17
    ],
    [
      -0.7313537016191706,
      -0.6819983600624986,
      -5.551115123125784e-17
    ],
    [
      -0.2554810823782512,
      0.2739699187456749,
      -0.9271838545667874
    ]
  ], dtype=float)

    tool_offset = np.array([0.0, 0.0, 0.1]) #needs to be corrected

    arm = ik["SixDOFArmIK"](p_home, a_home, tool_offset=tool_offset)
    # Set servo limits to [0, pi] for all joints
    N = p_home.shape[0]
    arm.set_joint_limits(np.zeros(N), np.pi * np.ones(N))
    arm.set_state_from_angles(np.array([1.3, 1.2, 1.4, 1.3, 1.1])) #would be better to set current position of the servos.

    target_p = np.array([-0.1, 0.3, 0.15])
    target_q = ik["quat_from_axis_angle"]([0, 1, 0], np.deg2rad(30))

    # Toggle recording here
    record = True  # set to False for speed, no plots

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



from robotic_arm_ik import SixDOFArmIK
import numpy as np

# Define home joint positions (meters) and axes (unit vectors)
p_home = np.array([
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.3],
    [0.0, 0.2, 0.3],
    [0.0, 0.4, 0.3],
    [0.0, 0.6, 0.3],
    [0.0, 0.8, 0.3],
], dtype=float)

a_home = np.array([
    [0.0, 0.0, 1.0],  # base yaw
    [0.0, 1.0, 0.0],  # shoulder pitch
    [1.0, .0, 0.0],  # elbow pitch
    [0.0, 0.0, 1.0],  # wrist roll
    [0.0, 1.0, 0.0],  # wrist pitch
    [1.0, 0.0, 0.0],  # wrist roll
], dtype=float)

tool_offset = np.array([0.0, 0.0, 0.1])

arm = SixDOFArmIK(p_home, a_home, tool_offset=tool_offset)

# Target pose (position + quaternion [w,x,y,z])
target_p = np.array([0.2, 0.4, 0.6])
target_q = np.array([1.0, 0.0, 0.0, 0.0])

sol = arm.solve_ik(target_p, target_q, max_iters=200)
if sol.success:
    print("Angles (rad):", sol.angles)
    print("Reached position:", sol.fk.end_effector_position)
else:
    print("IK failed:", sol.message)
    print("Failed to reach position, but this was the result:", sol.angles)

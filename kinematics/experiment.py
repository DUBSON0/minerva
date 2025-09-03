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

# Test forward kinematics at different joint positions
print("Joint positions:")
print("Joint 0:", p_home[0])
print("Joint 1:", p_home[1])
print("Joint 2:", p_home[2])
print("Joint 3:", p_home[3])
print("Joint 4:", p_home[4])
print("Joint 5:", p_home[5])

print("\nJoint axes:")
print("Axis 0:", a_home[0])
print("Axis 1:", a_home[1])
print("Axis 2:", a_home[2])
print("Axis 3:", a_home[3])
print("Axis 4:", a_home[4])
print("Axis 5:", a_home[5])

# Test forward kinematics first
fk_home = arm.forward_kinematics()
print("\nHome position (should be ~[0, 0.8, 0.4]):", fk_home.end_effector_position)

# Test with small rotations
test_angles = np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0])  # small rotation at joint 1
fk_test = arm.forward_kinematics(test_angles)
print("Test position with small joint 1 rotation:", fk_test.end_effector_position)

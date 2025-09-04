"""
Inverse Kinematics API for Robotic Arm

This module provides a clean API for computing joint angles from target positions
for a 6-DOF robotic arm using inverse kinematics.
"""

import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union
import numpy as np

# Import the IK module
import robotic_arm_ik_with_visualization as ik_module


# Robot geometry constants
CALIBRATION_FILE = Path(__file__).parent / "calibration_of_arm" / "calibrated_home.json"
TOOL_OFFSET = np.array([0.0, 0.0, 0.1])  # Tool offset in last joint's local frame

# Joint limits (can be adjusted based on robot specifications)
# Note: This is a 5-DOF robot based on the calibration file
DEFAULT_LOWER_LIMITS = np.array([0, 0, 0, 0, 0])
DEFAULT_UPPER_LIMITS = np.array([np.pi, np.pi, np.pi, np.pi, np.pi])


class IKAPI:
    """Inverse Kinematics API for robotic arm control."""

    def __init__(self, calibration_file: Optional[str] = None):
        """
        Initialize the IK API with robot geometry.

        Args:
            calibration_file: Path to calibration JSON file. If None, uses default.
        """
        self.calibration_file = calibration_file or CALIBRATION_FILE
        self.arm = None
        self._load_robot_geometry()

    def _load_robot_geometry(self) -> None:
        """Load robot geometry from calibration file."""
        try:
            with open(self.calibration_file, 'r') as f:
                data = json.load(f)

            p_home = np.array(data['p_home'], dtype=float)
            a_home = np.array(data['a_home'], dtype=float)

            # Initialize the IK solver
            self.arm = ik_module.SixDOFArmIK(
                p_home=p_home,
                a_home=a_home,
                tool_offset=TOOL_OFFSET,
                lower_limits=DEFAULT_LOWER_LIMITS,
                upper_limits=DEFAULT_UPPER_LIMITS
            )

        except FileNotFoundError:
            raise FileNotFoundError(f"Calibration file not found: {self.calibration_file}")
        except KeyError as e:
            raise ValueError(f"Invalid calibration file format: missing key {e}")
        except Exception as e:
            raise RuntimeError(f"Error loading robot geometry: {e}")

    def compute_joint_angles(
        self,
        target_position: Union[List[float], np.ndarray],
        target_orientation: Optional[Union[List[float], np.ndarray]] = None,
        initial_angles: Optional[Union[List[float], np.ndarray]] = None,
        max_iterations: int = 200,
        position_tolerance: float = 1e-3,
        orientation_tolerance: float = 10.0,  # degrees, set high to focus on position
    ) -> Dict:
        """
        Compute joint angles for a target position using inverse kinematics.

        Args:
            target_position: Target position [x, y, z] in meters
            target_orientation: Target orientation as quaternion [w, x, y, z].
                              If None, orientation is ignored (position-only IK).
            initial_angles: Initial joint angles for IK solver. If None, uses current arm state.
            max_iterations: Maximum number of IK iterations
            position_tolerance: Position error tolerance in meters
            orientation_tolerance: Orientation error tolerance in degrees

        Returns:
            Dict containing:
            - success: bool
            - joint_angles: list of 6 joint angles in radians, or None if failed
            - end_effector_position: [x, y, z] achieved position
            - position_error: position error norm in meters
            - orientation_error: orientation error norm in degrees (if orientation provided)
            - iterations: number of iterations used
            - message: status message
            - error: error message if failed
        """
        try:
            # Validate inputs
            target_pos = np.array(target_position, dtype=float)
            if target_pos.shape != (3,):
                raise ValueError("target_position must be a list/array of 3 floats [x, y, z]")

            # Set default orientation if not provided (neutral orientation)
            if target_orientation is None:
                target_ori = ik_module.quat_from_axis_angle([0, 1, 0], 0)  # No rotation
                orientation_weight = 0.0  # Ignore orientation
                # When ignoring orientation, use a very large tolerance so it doesn't prevent convergence
                ori_tol_rad = np.pi  # 180 degrees - basically ignore orientation error
            else:
                target_ori = np.array(target_orientation, dtype=float)
                if target_ori.shape != (4,):
                    raise ValueError("target_orientation must be a quaternion [w, x, y, z]")
                orientation_weight = 0  # Default weight for orientation
                ori_tol_rad = np.deg2rad(orientation_tolerance)

            # Set initial angles if provided
            if initial_angles is not None:
                init_angles = np.array(initial_angles, dtype=float)
                if init_angles.shape != (self.arm.N,):
                    raise ValueError(f"initial_angles must be a list/array of {self.arm.N} floats")
                self.arm.set_state_from_angles(init_angles)

            # Solve IK
            solution = self.arm.solve_ik(
                target_position=target_pos,
                target_orientation_quat=target_ori,
                max_iters=max_iterations,
                position_weight=1.0,
                orientation_weight=orientation_weight,
                pos_tol=position_tolerance,
                ori_tol=ori_tol_rad,
                record_history=False  # Don't record history for API calls
            )

            # Calculate errors
            position_error = np.linalg.norm(target_pos - solution.fk.end_effector_position)
            orientation_error_deg = 0.0
            if target_orientation is not None:
                # Calculate orientation error
                target_q = ik_module.quat_normalize(target_ori)
                current_q = solution.fk.end_effector_orientation
                q_err = ik_module.quat_multiply(target_q, ik_module.quat_conjugate(current_q))
                ori_err_vec = ik_module.quat_log(q_err)
                orientation_error_deg = np.rad2deg(np.linalg.norm(ori_err_vec))

            # Prepare response
            response = {
                "success": solution.success,
                "joint_angles": solution.angles.tolist() if solution.success else None,
                "end_effector_position": solution.fk.end_effector_position.tolist(),
                "position_error": float(position_error),
                "orientation_error": float(orientation_error_deg) if target_orientation is not None else None,
                "iterations": solution.iters,
                "message": solution.message,
                "error": None
            }

            return response

        except Exception as e:
            return {
                "success": False,
                "joint_angles": None,
                "end_effector_position": None,
                "position_error": None,
                "orientation_error": None,
                "iterations": 0,
                "message": "Error during IK computation",
                "error": str(e)
            }

    def get_current_state(self) -> Dict:
        """
        Get the current state of the robotic arm.

        Returns:
            Dict containing current joint angles and end-effector pose.
        """
        try:
            fk_result = self.arm.forward_kinematics()
            return {
                "joint_angles": self.arm.angles.tolist(),
                "end_effector_position": fk_result.end_effector_position.tolist(),
                "end_effector_orientation": fk_result.end_effector_orientation.tolist(),
                "joint_positions": fk_result.joint_positions.tolist(),
                "joint_axes": fk_result.joint_axes_world.tolist()
            }
        except Exception as e:
            return {
                "error": str(e),
                "joint_angles": None,
                "end_effector_position": None,
                "end_effector_orientation": None,
                "joint_positions": None,
                "joint_axes": None
            }

    def set_joint_limits(self, lower_limits: List[float], upper_limits: List[float]) -> bool:
        """
        Set joint angle limits.

        Args:
            lower_limits: List of N lower joint limits in radians (where N is number of joints)
            upper_limits: List of N upper joint limits in radians (where N is number of joints)

        Returns:
            True if successful, False otherwise
        """
        try:
            ll = np.array(lower_limits, dtype=float)
            ul = np.array(upper_limits, dtype=float)
            if ll.shape != (self.arm.N,) or ul.shape != (self.arm.N,):
                raise ValueError(f"Both limits must be lists of {self.arm.N} floats")
            self.arm.set_joint_limits(ll, ul)
            return True
        except Exception:
            return False


# Global API instance for convenience
_api_instance = None

def get_ik_api() -> IKAPI:
    """Get the global IK API instance."""
    global _api_instance
    if _api_instance is None:
        _api_instance = IKAPI()
    return _api_instance

def compute_ik(
    target_position: Union[List[float], np.ndarray],
    target_orientation: Optional[Union[List[float], np.ndarray]] = None,
    **kwargs
) -> Dict:
    """
    Convenience function to compute inverse kinematics.

    Args:
        target_position: Target position [x, y, z] in meters
        target_orientation: Optional target orientation as quaternion [w, x, y, z]
        **kwargs: Additional arguments passed to compute_joint_angles

    Returns:
        Dict with IK results (see IKAPI.compute_joint_angles for details)
    """
    api = get_ik_api()
    return api.compute_joint_angles(target_position, target_orientation, **kwargs)


if __name__ == "__main__":
    # Example usage
    print("IK API Example Usage")
    print("=" * 40)

    # Simple example: compute joint angles for a target position
    target_position = [0.0, 0.05, 0.18]  # [x, y, z] in meters

    print(f"Computing joint angles for target position: {target_position}")

    # Use the API
    result = compute_ik(target_position)

    print(f"\nResults:")
    print(f"  Success: {result['success']}")
    print(f"  Joint angles (radians): {result['joint_angles']}")
    print(f"  End effector position: {result['end_effector_position']}")
    print(f"  Position error: {result['position_error']:.6f} meters")
    print(f"  Iterations: {result['iterations']}")
    print(f"  Message: {result['message']}")

    if result['error']:
        print(f"  Error: {result['error']}")

    print("\n" + "=" * 40)
    print("API Usage Examples:")
    print("1. Basic usage: compute_ik([x, y, z])")
    print("2. With orientation: compute_ik([x, y, z], orientation_quaternion)")
    print("3. With custom settings: compute_ik([x, y, z], max_iterations=1000, position_tolerance=1e-4)")
    print("4. Advanced: api = get_ik_api(); api.compute_joint_angles(...)")

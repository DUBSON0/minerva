#!/usr/bin/env python3
"""
Camera Calibration Module
Handles camera calibration and provides calibration data for 3D reconstruction
"""

import cv2
import numpy as np
import json
import os
from typing import Tuple, List, Dict, Optional
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class CalibrationData:
    """Data class for camera calibration parameters"""
    camera_matrix: np.ndarray
    distortion_coeffs: np.ndarray
    rotation_matrix: np.ndarray
    translation_vector: np.ndarray
    image_size: Tuple[int, int]

class CameraCalibrator:
    """Camera calibration class for single and stereo cameras"""
    
    def __init__(self, chessboard_size: Tuple[int, int] = (9, 6), 
                 square_size: float = 0.025):
        """
        Initialize camera calibrator
        
        Args:
            chessboard_size: Number of internal corners (width, height)
            square_size: Size of chessboard squares in meters
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ..., (8,5,0)
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Arrays to store object points and image points
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
        
        logger.info(f"Camera calibrator initialized with chessboard {chessboard_size}")
    
    def add_calibration_image(self, image: np.ndarray) -> bool:
        """
        Add calibration image for single camera calibration
        
        Args:
            image: Calibration image (grayscale)
            
        Returns:
            True if chessboard was found and points were added
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        
        if ret:
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Add points
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners)
            
            logger.info(f"Added calibration image with {len(corners)} corners")
            return True
        else:
            logger.warning("Chessboard not found in calibration image")
            return False
    
    def calibrate_single_camera(self, image_size: Tuple[int, int]) -> Optional[CalibrationData]:
        """
        Calibrate single camera
        
        Args:
            image_size: Size of calibration images (width, height)
            
        Returns:
            CalibrationData if successful, None otherwise
        """
        if len(self.objpoints) < 5:
            logger.error("Need at least 5 calibration images")
            return None
        
        try:
            # Calibrate camera
            ret, camera_matrix, distortion_coeffs, rotation_vectors, translation_vectors = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, image_size, None, None
            )
            
            if ret:
                # Calculate reprojection error
                mean_error = 0
                for i in range(len(self.objpoints)):
                    imgpoints2, _ = cv2.projectPoints(
                        self.objpoints[i], rotation_vectors[i], translation_vectors[i], 
                        camera_matrix, distortion_coeffs
                    )
                    error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                    mean_error += error
                
                logger.info(f"Single camera calibration successful. Mean reprojection error: {mean_error/len(self.objpoints):.4f}")
                
                # Return calibration data (identity rotation and zero translation for single camera)
                return CalibrationData(
                    camera_matrix=camera_matrix,
                    distortion_coeffs=distortion_coeffs,
                    rotation_matrix=np.eye(3),
                    translation_vector=np.zeros(3),
                    image_size=image_size
                )
            else:
                logger.error("Single camera calibration failed")
                return None
                
        except Exception as e:
            logger.error(f"Error during single camera calibration: {e}")
            return None
    
    def calibrate_stereo_cameras(self, image_size: Tuple[int, int], 
                                camera1_data: CalibrationData,
                                camera2_data: CalibrationData) -> Optional[Dict]:
        """
        Calibrate stereo camera pair
        
        Args:
            image_size: Size of calibration images
            camera1_data: Calibration data for camera 1
            camera2_data: Calibration data for camera 2
            
        Returns:
            Stereo calibration data if successful, None otherwise
        """
        if len(self.objpoints) < 10:
            logger.error("Need at least 10 stereo calibration images")
            return None
        
        try:
            # Stereo calibration
            ret, camera1_matrix, camera1_dist, camera2_matrix, camera2_dist, 
            rotation_matrix, translation_vector, essential_matrix, fundamental_matrix = cv2.stereoCalibrate(
                self.objpoints, self.imgpoints, self.imgpoints,  # Using same points for both cameras
                camera1_data.camera_matrix, camera1_data.distortion_coeffs,
                camera2_data.camera_matrix, camera2_data.distortion_coeffs,
                image_size
            )
            
            if ret:
                logger.info("Stereo camera calibration successful")
                
                return {
                    'camera1_matrix': camera1_matrix,
                    'camera1_dist': camera1_dist,
                    'camera2_matrix': camera2_matrix,
                    'camera2_dist': camera2_dist,
                    'rotation_matrix': rotation_matrix,
                    'translation_vector': translation_vector,
                    'essential_matrix': essential_matrix,
                    'fundamental_matrix': fundamental_matrix
                }
            else:
                logger.error("Stereo camera calibration failed")
                return None
                
        except Exception as e:
            logger.error(f"Error during stereo camera calibration: {e}")
            return None
    
    def save_calibration(self, calibration_data: CalibrationData, filename: str):
        """Save calibration data to file"""
        try:
            data = {
                'camera_matrix': calibration_data.camera_matrix.tolist(),
                'distortion_coeffs': calibration_data.distortion_coeffs.tolist(),
                'rotation_matrix': calibration_data.rotation_matrix.tolist(),
                'translation_vector': calibration_data.translation_vector.tolist(),
                'image_size': calibration_data.image_size
            }
            
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            logger.info(f"Calibration data saved to {filename}")
            
        except Exception as e:
            logger.error(f"Error saving calibration data: {e}")
    
    def load_calibration(self, filename: str) -> Optional[CalibrationData]:
        """Load calibration data from file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            calibration_data = CalibrationData(
                camera_matrix=np.array(data['camera_matrix']),
                distortion_coeffs=np.array(data['distortion_coeffs']),
                rotation_matrix=np.array(data['rotation_matrix']),
                translation_vector=np.array(data['translation_vector']),
                image_size=tuple(data['image_size'])
            )
            
            logger.info(f"Calibration data loaded from {filename}")
            return calibration_data
            
        except Exception as e:
            logger.error(f"Error loading calibration data: {e}")
            return None

def get_default_calibration() -> Tuple[CalibrationData, CalibrationData]:
    """
    Get default calibration data for testing purposes
    
    Returns:
        Tuple of (camera1_calibration, camera2_calibration)
    """
    # Default camera intrinsic matrix (640x480 resolution)
    fx, fy = 500, 500  # Focal length in pixels
    cx, cy = 320, 240  # Principal point
    
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float32)
    
    # Default distortion coefficients (no distortion)
    distortion_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
    
    # Default rotation matrix (identity)
    rotation_matrix = np.eye(3, dtype=np.float32)
    
    # Default translation vector (camera 2 is 10cm to the right of camera 1)
    translation_vector = np.array([0.1, 0, 0], dtype=np.float32)
    
    # Image size
    image_size = (640, 480)
    
    # Create calibration data for both cameras
    camera1_data = CalibrationData(
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_coeffs,
        rotation_matrix=rotation_matrix,
        translation_vector=np.zeros(3, dtype=np.float32),
        image_size=image_size
    )
    
    camera2_data = CalibrationData(
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_coeffs,
        rotation_matrix=rotation_matrix,
        translation_vector=translation_vector,
        image_size=image_size
    )
    
    logger.info("Default calibration data created")
    return camera1_data, camera2_data

def create_camera_configs_from_calibration(camera1_cal: CalibrationData, 
                                         camera2_cal: CalibrationData) -> List:
    """
    Create camera configurations from calibration data
    
    Args:
        camera1_cal: Calibration data for camera 1
        camera2_cal: Calibration data for camera 2
        
    Returns:
        List of camera configurations for FoodDetector
    """
    from food_detection_system import CameraConfig
    
    config1 = CameraConfig(
        camera_id=0,
        intrinsic_matrix=camera1_cal.camera_matrix,
        distortion_coeffs=camera1_cal.distortion_coeffs,
        rotation_matrix=camera1_cal.rotation_matrix,
        translation_vector=camera1_cal.translation_vector,
        baseline=0.0  # Camera 1 is reference
    )
    
    config2 = CameraConfig(
        camera_id=1,
        intrinsic_matrix=camera2_cal.camera_matrix,
        distortion_coeffs=camera2_cal.distortion_coeffs,
        rotation_matrix=camera2_cal.rotation_matrix,
        translation_vector=camera2_cal.translation_vector,
        baseline=np.linalg.norm(camera2_cal.translation_vector)  # Distance between cameras
    )
    
    return [config1, config2]

def interactive_calibration(calibrator: CameraCalibrator, camera: 'CameraInterface') -> bool:
    """
    Interactive calibration using live camera feed
    
    Args:
        calibrator: CameraCalibrator instance
        camera: Camera interface
        
    Returns:
        True if calibration was successful
    """
    print("Interactive Camera Calibration")
    print("=" * 40)
    print("1. Hold a chessboard pattern in front of the camera")
    print("2. Press 'c' to capture calibration image")
    print("3. Press 'q' to quit and calibrate")
    print("4. Need at least 5 images for calibration")
    print("=" * 40)
    
    captured_images = 0
    
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to read from camera")
            continue
        
        # Display frame
        cv2.imshow('Calibration', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('c'):
            # Try to capture calibration image
            if calibrator.add_calibration_image(frame):
                captured_images += 1
                print(f"Captured calibration image {captured_images}")
            else:
                print("Chessboard not found. Please adjust position.")
        
        elif key == ord('q'):
            break
    
    cv2.destroyAllWindows()
    
    if captured_images >= 5:
        print(f"\nCaptured {captured_images} images. Calibrating...")
        return True
    else:
        print(f"\nNeed at least 5 images. Only captured {captured_images}")
        return False


#!/usr/bin/env python3
"""
Camera Interface Module
Provides unified interface for different camera types (USB, Pi Camera, etc.)
"""

import cv2
import numpy as np
import time
from typing import Optional, Tuple, List
import logging
from abc import ABC, abstractmethod

logger = logging.getLogger(__name__)

class CameraInterface(ABC):
    """Abstract base class for camera interfaces"""
    
    @abstractmethod
    def open(self) -> bool:
        """Open camera connection"""
        pass
    
    @abstractmethod
    def close(self):
        """Close camera connection"""
        pass
    
    @abstractmethod
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read frame from camera"""
        pass
    
    @abstractmethod
    def is_opened(self) -> bool:
        """Check if camera is opened"""
        pass
    
    @abstractmethod
    def get_resolution(self) -> Tuple[int, int]:
        """Get camera resolution"""
        pass
    
    @abstractmethod
    def set_resolution(self, width: int, height: int) -> bool:
        """Set camera resolution"""
        pass

class USBCamera(CameraInterface):
    """USB camera interface using OpenCV"""
    
    def __init__(self, camera_id: int, resolution: Tuple[int, int] = (640, 480)):
        """
        Initialize USB camera
        
        Args:
            camera_id: Camera device ID
            resolution: Camera resolution (width, height)
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.cap = None
        self.is_open = False
        
        logger.info(f"USB Camera {camera_id} initialized with resolution {resolution}")
    
    def open(self) -> bool:
        """Open USB camera connection"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            
            if not self.cap.isOpened():
                logger.error(f"Failed to open USB camera {self.camera_id}")
                return False
            
            # Try to set resolution, but don't fail if it doesn't work
            try:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
                
                # Verify the resolution was actually set
                actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                
                if abs(actual_width - self.resolution[0]) > 10 or abs(actual_height - self.resolution[1]) > 10:
                    logger.warning(f"Requested resolution {self.resolution} not supported, using {actual_width}x{actual_height}")
                    self.resolution = (actual_width, actual_height)
                else:
                    logger.info(f"Resolution set to {self.resolution}")
                    
            except Exception as e:
                logger.warning(f"Could not set resolution to {self.resolution}: {e}")
                # Get actual resolution
                actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.resolution = (actual_width, actual_height)
                logger.info(f"Using camera's default resolution: {self.resolution}")
            
            # Set other properties for better performance
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            self.is_open = True
            logger.info(f"USB Camera {self.camera_id} opened successfully with resolution {self.resolution}")
            return True
            
        except Exception as e:
            logger.error(f"Error opening USB camera {self.camera_id}: {e}")
            return False
    
    def close(self):
        """Close USB camera connection"""
        if self.cap:
            self.cap.release()
            self.cap = None
        self.is_open = False
        logger.info(f"USB Camera {self.camera_id} closed")
    
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read frame from USB camera"""
        if not self.is_open or not self.cap:
            return False, None
        
        ret, frame = self.cap.read()
        if not ret:
            logger.warning(f"Failed to read frame from USB camera {self.camera_id}")
            return False, None
        
        return True, frame
    
    def is_opened(self) -> bool:
        """Check if USB camera is opened"""
        return self.is_open and self.cap and self.cap.isOpened()
    
    def get_resolution(self) -> Tuple[int, int]:
        """Get USB camera resolution"""
        if not self.is_open or not self.cap:
            return self.resolution
        
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return (width, height)
    
    def set_resolution(self, width: int, height: int) -> bool:
        """Set USB camera resolution"""
        if not self.is_open or not self.cap:
            return False
        
        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.resolution = (width, height)
            logger.info(f"USB Camera {self.camera_id} resolution set to {width}x{height}")
            return True
        except Exception as e:
            logger.error(f"Error setting USB camera {self.camera_id} resolution: {e}")
            return False

class PiCamera(CameraInterface):
    """Raspberry Pi Camera interface using picamera2"""
    
    def __init__(self, camera_id: int = 0, resolution: Tuple[int, int] = (640, 480)):
        """
        Initialize Pi Camera
        
        Args:
            camera_id: Camera device ID (usually 0 for Pi Camera)
            resolution: Camera resolution (width, height)
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.camera = None
        self.is_open = False
        
        logger.info(f"Pi Camera {camera_id} initialized with resolution {resolution}")
    
    def open(self) -> bool:
        """Open Pi Camera connection"""
        try:
            # Try to import picamera2
            from picamera2 import Picamera2
            
            self.camera = Picamera2()
            
            # Configure camera
            config = self.camera.create_preview_configuration(
                main={"size": self.resolution},
                buffer_count=4
            )
            self.camera.configure(config)
            
            # Start camera
            self.camera.start()
            
            # Wait for camera to start
            time.sleep(2)
            
            self.is_open = True
            logger.info(f"Pi Camera {self.camera_id} opened successfully")
            return True
            
        except ImportError:
            logger.error("picamera2 not available. Install with: pip install picamera2")
            return False
        except Exception as e:
            logger.error(f"Error opening Pi Camera {self.camera_id}: {e}")
            return False
    
    def close(self):
        """Close Pi Camera connection"""
        if self.camera:
            self.camera.close()
            self.camera = None
        self.is_open = False
        logger.info(f"Pi Camera {self.camera_id} closed")
    
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read frame from Pi Camera"""
        if not self.is_open or not self.camera:
            return False, None
        
        try:
            frame = self.camera.capture_array()
            # Convert from RGB to BGR for OpenCV compatibility
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return True, frame
        except Exception as e:
            logger.warning(f"Failed to read frame from Pi Camera {self.camera_id}: {e}")
            return False, None
    
    def is_opened(self) -> bool:
        """Check if Pi Camera is opened"""
        return self.is_open and self.camera is not None
    
    def get_resolution(self) -> Tuple[int, int]:
        """Get Pi Camera resolution"""
        return self.resolution
    
    def set_resolution(self, width: int, height: int) -> bool:
        """Set Pi Camera resolution"""
        if not self.is_open or not self.camera:
            return False
        
        try:
            # Stop camera
            self.camera.stop()
            
            # Reconfigure with new resolution
            config = self.camera.create_preview_configuration(
                main={"size": (width, height)},
                buffer_count=4
            )
            self.camera.configure(config)
            
            # Restart camera
            self.camera.start()
            time.sleep(1)
            
            self.resolution = (width, height)
            logger.info(f"Pi Camera {self.camera_id} resolution set to {width}x{height}")
            return True
            
        except Exception as e:
            logger.error(f"Error setting Pi Camera {self.camera_id} resolution: {e}")
            return False

class MockCamera(CameraInterface):
    """Mock camera for testing purposes"""
    
    def __init__(self, camera_id: int, resolution: Tuple[int, int] = (640, 480)):
        """
        Initialize mock camera
        
        Args:
            camera_id: Camera device ID
            resolution: Camera resolution (width, height)
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.is_open = False
        self.frame_count = 0
        
        # Create a mock image with some shapes
        self.mock_image = self._create_mock_image()
        
        logger.info(f"Mock Camera {camera_id} initialized with resolution {resolution}")
    
    def _create_mock_image(self) -> np.ndarray:
        """Create a mock image with geometric shapes"""
        img = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)
        
        # Add some colored shapes to simulate food objects
        # Orange rectangle (carrot-like)
        cv2.rectangle(img, (100, 150), (200, 300), (0, 165, 255), -1)
        
        # White circle (egg-like)
        cv2.circle(img, (400, 200), 50, (255, 255, 255), -1)
        
        # Green rectangle (cucumber-like)
        cv2.rectangle(img, (300, 100), (450, 200), (0, 255, 0), -1)
        
        # Red circle (tomato-like)
        cv2.circle(img, (150, 100), 40, (0, 0, 255), -1)
        
        return img
    
    def open(self) -> bool:
        """Open mock camera connection"""
        self.is_open = True
        logger.info(f"Mock Camera {self.camera_id} opened successfully")
        return True
    
    def close(self):
        """Close mock camera connection"""
        self.is_open = False
        logger.info(f"Mock Camera {self.camera_id} closed")
    
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read frame from mock camera"""
        if not self.is_open:
            return False, None
        
        # Simulate some movement by slightly shifting the image
        self.frame_count += 1
        shift = int(5 * np.sin(self.frame_count * 0.1))
        
        # Create a shifted version of the mock image
        shifted_img = np.roll(self.mock_image, shift, axis=1)
        
        return True, shifted_img
    
    def is_opened(self) -> bool:
        """Check if mock camera is opened"""
        return self.is_open
    
    def get_resolution(self) -> Tuple[int, int]:
        """Get mock camera resolution"""
        return self.resolution
    
    def set_resolution(self, width: int, height: int) -> bool:
        """Set mock camera resolution"""
        self.resolution = (width, height)
        self.mock_image = self._create_mock_image()
        logger.info(f"Mock Camera {self.camera_id} resolution set to {width}x{height}")
        return True

class CameraManager:
    """Manager class for handling multiple cameras"""
    
    def __init__(self):
        """Initialize camera manager"""
        self.cameras: List[CameraInterface] = []
        self.is_initialized = False
        
        logger.info("Camera manager initialized")
    
    def add_camera(self, camera: CameraInterface) -> bool:
        """Add a camera to the manager"""
        try:
            if camera.open():
                self.cameras.append(camera)
                logger.info(f"Camera {camera.camera_id} added successfully")
                return True
            else:
                logger.error(f"Failed to add camera {camera.camera_id}")
                return False
        except Exception as e:
            logger.error(f"Error adding camera {camera.camera_id}: {e}")
            return False
    
    def remove_camera(self, camera_id: int) -> bool:
        """Remove a camera from the manager"""
        for i, camera in enumerate(self.cameras):
            if camera.camera_id == camera_id:
                camera.close()
                self.cameras.pop(i)
                logger.info(f"Camera {camera_id} removed successfully")
                return True
        
        logger.warning(f"Camera {camera_id} not found")
        return False
    
    def get_camera(self, camera_id: int) -> Optional[CameraInterface]:
        """Get camera by ID"""
        for camera in self.cameras:
            if camera.camera_id == camera_id:
                return camera
        return None
    
    def read_all_cameras(self) -> List[Tuple[int, np.ndarray]]:
        """Read frames from all cameras"""
        frames = []
        
        for camera in self.cameras:
            if camera.is_opened():
                ret, frame = camera.read()
                if ret:
                    frames.append((camera.camera_id, frame))
                else:
                    logger.warning(f"Failed to read from camera {camera.camera_id}")
        
        return frames
    
    def close_all_cameras(self):
        """Close all cameras"""
        for camera in self.cameras:
            camera.close()
        self.cameras.clear()
        logger.info("All cameras closed")
    
    def get_camera_count(self) -> int:
        """Get number of cameras"""
        return len(self.cameras)
    
    def is_ready(self) -> bool:
        """Check if camera manager is ready"""
        # For single camera mode, we only need 1 camera
        # For stereo mode, we need at least 2 cameras
        min_cameras = 1  # Allow single camera operation
        return len(self.cameras) >= min_cameras and all(camera.is_opened() for camera in self.cameras)

def create_camera(camera_type: str, camera_id: int, resolution: Tuple[int, int] = (640, 480)) -> CameraInterface:
    """
    Factory function to create camera instances
    
    Args:
        camera_type: Type of camera ('usb', 'pi', 'mock')
        camera_id: Camera device ID
        resolution: Camera resolution
        
    Returns:
        CameraInterface instance
    """
    if camera_type.lower() == 'usb':
        return USBCamera(camera_id, resolution)
    elif camera_type.lower() == 'pi':
        return PiCamera(camera_id, resolution)
    elif camera_type.lower() == 'mock':
        return MockCamera(camera_id, resolution)
    else:
        raise ValueError(f"Unknown camera type: {camera_type}")

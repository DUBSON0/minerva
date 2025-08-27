#!/usr/bin/env python3
"""
Food Detection System for Robotic Arm Control
Detects food objects from two cameras and provides 3D location and shape information
"""

import cv2
import numpy as np
import time
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum
import json
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ShapeType(Enum):
    """Enumeration for detected object shapes"""
    CYLINDER = "cylinder"
    ELLIPSE = "ellipse"
    SPHERE = "sphere"
    UNKNOWN = "unknown"

@dataclass
class ObjectShape:
    """Data class for object shape information"""
    shape_type: ShapeType
    elongation: float  # 0.0 = circle/sphere, 1.0 = very elongated
    aspect_ratio: float  # width/height ratio
    confidence: float  # detection confidence (0.0-1.0)

@dataclass
class DetectedObject:
    """Data class for detected food objects"""
    id: int
    bbox: Tuple[int, int, int, int]  # x, y, width, height
    center_2d: Tuple[float, float]  # 2D center in image coordinates
    center_3d: Tuple[float, float, float]  # 3D center in world coordinates (x, y, z)
    shape: ObjectShape
    color: Tuple[int, int, int]  # BGR color
    area: float  # pixel area

@dataclass
class CameraConfig:
    """Camera configuration and calibration data"""
    camera_id: int
    intrinsic_matrix: np.ndarray  # 3x3 camera intrinsic matrix
    distortion_coeffs: np.ndarray  # distortion coefficients
    rotation_matrix: np.ndarray  # 3x3 rotation matrix
    translation_vector: np.ndarray  # 3x1 translation vector
    baseline: float  # distance between cameras (if stereo)

class FoodDetector:
    """Main class for food object detection and 3D localization"""
    
    def __init__(self, camera_configs: List[CameraConfig], 
                 detection_config: Dict = None):
        """
        Initialize the food detection system
        
        Args:
            camera_configs: List of camera configurations
            detection_config: Detection parameters
        """
        self.camera_configs = camera_configs
        self.detection_config = detection_config or self._get_default_config()
        
        # Initialize object tracking
        self.next_object_id = 0
        self.tracked_objects: Dict[int, DetectedObject] = {}
        
        # Color ranges for food detection (HSV)
        self.food_colors = {
            'carrot': ([0, 50, 50], [20, 255, 255]),      # Orange
            'egg': ([0, 0, 200], [180, 30, 255]),         # White
            'tomato': ([0, 100, 100], [10, 255, 255]),    # Red
            'cucumber': ([35, 50, 50], [85, 255, 255]),   # Green
        }
        
        logger.info("Food detection system initialized")
    
    def _get_default_config(self) -> Dict:
        """Get default detection configuration"""
        return {
            'min_area': 1000,           # Minimum object area in pixels
            'max_area': 50000,          # Maximum object area in pixels
            'confidence_threshold': 0.5, # Minimum detection confidence
            'nms_threshold': 0.3,       # Non-maximum suppression threshold
            'max_objects': 10,          # Maximum objects to detect
            'color_similarity_threshold': 0.8,
            'shape_similarity_threshold': 0.7,
        }
    
    def detect_objects_in_image(self, image: np.ndarray, camera_id: int) -> List[DetectedObject]:
        """
        Detect food objects in a single camera image
        
        Args:
            image: Input image (BGR format)
            camera_id: ID of the camera that captured the image
            
        Returns:
            List of detected objects
        """
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        detected_objects = []
        
        # Detect objects for each food color
        for food_name, (lower_hsv, upper_hsv) in self.food_colors.items():
            # Create mask for current color
            mask = cv2.inRange(hsv, np.array(lower_hsv), np.array(upper_hsv))
            
            # Apply morphological operations to clean up the mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter by area
                if area < self.detection_config['min_area'] or area > self.detection_config['max_area']:
                    continue
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate center
                center_2d = (x + w/2, y + h/2)
                
                # Analyze shape
                shape = self._analyze_shape(contour, w, h)
                
                # Calculate average color
                color = self._get_average_color(image, contour)
                
                # Create detected object
                obj = DetectedObject(
                    id=self.next_object_id,
                    bbox=(x, y, w, h),
                    center_2d=center_2d,
                    center_3d=(0, 0, 0),  # Will be calculated later
                    shape=shape,
                    color=color,
                    area=area
                )
                
                detected_objects.append(obj)
                self.next_object_id += 1
        
        # Apply non-maximum suppression
        detected_objects = self._apply_nms(detected_objects)
        
        logger.info(f"Camera {camera_id}: Detected {len(detected_objects)} objects")
        return detected_objects
    
    def _analyze_shape(self, contour: np.ndarray, width: int, height: int) -> ObjectShape:
        """
        Analyze the shape of a detected object
        
        Args:
            contour: Object contour
            width: Bounding box width
            height: Bounding box height
            
        Returns:
            ObjectShape with shape information
        """
        # Calculate aspect ratio
        aspect_ratio = width / height if height > 0 else 1.0
        
        # Calculate elongation using contour analysis
        if len(contour) >= 5:
            # Fit ellipse to contour
            try:
                ellipse = cv2.fitEllipse(contour)
                major_axis = max(ellipse[1])
                minor_axis = min(ellipse[1])
                elongation = 1.0 - (minor_axis / major_axis) if major_axis > 0 else 0.0
            except:
                elongation = 0.0
        else:
            elongation = 0.0
        
        # Determine shape type
        if elongation > 0.7:
            shape_type = ShapeType.CYLINDER
        elif elongation > 0.3:
            shape_type = ShapeType.ELLIPSE
        elif aspect_ratio < 1.5:
            shape_type = ShapeType.SPHERE
        else:
            shape_type = ShapeType.UNKNOWN
        
        # Calculate confidence based on contour regularity
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        
        confidence = min(1.0, circularity * 2)  # Scale circularity to confidence
        
        return ObjectShape(
            shape_type=shape_type,
            elongation=elongation,
            aspect_ratio=aspect_ratio,
            confidence=confidence
        )
    
    def _get_average_color(self, image: np.ndarray, contour: np.ndarray) -> Tuple[int, int, int]:
        """Get average color of an object"""
        # Create mask for the contour
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [contour], 255)
        
        # Calculate mean color
        mean_color = cv2.mean(image, mask=mask)
        return (int(mean_color[0]), int(mean_color[1]), int(mean_color[2]))
    
    def _apply_nms(self, objects: List[DetectedObject]) -> List[DetectedObject]:
        """Apply non-maximum suppression to remove overlapping detections"""
        if not objects:
            return []
        
        # Sort by confidence
        objects.sort(key=lambda x: x.shape.confidence, reverse=True)
        
        filtered_objects = []
        
        for obj in objects:
            should_add = True
            
            for existing_obj in filtered_objects:
                # Calculate overlap
                overlap = self._calculate_overlap(obj.bbox, existing_obj.bbox)
                
                if overlap > self.detection_config['nms_threshold']:
                    should_add = False
                    break
            
            if should_add:
                filtered_objects.append(obj)
                
                # Limit number of objects
                if len(filtered_objects) >= self.detection_config['max_objects']:
                    break
        
        return filtered_objects
    
    def _calculate_overlap(self, bbox1: Tuple[int, int, int, int], 
                          bbox2: Tuple[int, int, int, int]) -> float:
        """Calculate overlap ratio between two bounding boxes"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # Calculate intersection
        x_left = max(x1, x2)
        y_top = max(y1, y2)
        x_right = min(x1 + w1, x2 + w2)
        y_bottom = min(y1 + h1, y2 + h2)
        
        if x_right < x_left or y_bottom < y_top:
            return 0.0
        
        intersection = (x_right - x_left) * (y_bottom - y_top)
        area1 = w1 * h1
        area2 = w2 * h2
        
        return intersection / min(area1, area2)
    
    def match_objects_between_cameras(self, objects_cam1: List[DetectedObject], 
                                    objects_cam2: List[DetectedObject]) -> List[Tuple[DetectedObject, DetectedObject]]:
        """
        Match objects between two camera views
        
        Args:
            objects_cam1: Objects detected in camera 1
            objects_cam2: Objects detected in camera 2
            
        Returns:
            List of matched object pairs
        """
        matches = []
        
        for obj1 in objects_cam1:
            best_match = None
            best_score = 0
            
            for obj2 in objects_cam2:
                # Calculate similarity score
                score = self._calculate_object_similarity(obj1, obj2)
                
                if score > best_score and score > self.detection_config['shape_similarity_threshold']:
                    best_score = score
                    best_match = obj2
            
            if best_match:
                matches.append((obj1, best_match))
        
        logger.info(f"Matched {len(matches)} objects between cameras")
        return matches
    
    def _calculate_object_similarity(self, obj1: DetectedObject, obj2: DetectedObject) -> float:
        """Calculate similarity between two objects"""
        # Shape similarity
        shape_similarity = 1.0 - abs(obj1.shape.elongation - obj2.shape.elongation)
        
        # Color similarity (normalized Euclidean distance in RGB)
        color_diff = np.linalg.norm(np.array(obj1.color) - np.array(obj2.color))
        color_similarity = max(0, 1.0 - color_diff / 441.67)  # 441.67 = sqrt(255^2 + 255^2 + 255^2)
        
        # Area similarity
        area_ratio = min(obj1.area, obj2.area) / max(obj1.area, obj2.area)
        
        # Combined similarity score
        similarity = (shape_similarity * 0.4 + color_similarity * 0.4 + area_ratio * 0.2)
        
        return similarity
    
    def calculate_3d_positions(self, matched_objects: List[Tuple[DetectedObject, DetectedObject]]) -> List[DetectedObject]:
        """
        Calculate 3D positions of matched objects
        
        Args:
            matched_objects: List of matched object pairs
            
        Returns:
            List of objects with 3D positions
        """
        objects_3d = []
        
        for obj1, obj2 in matched_objects:
            # Get camera configurations
            cam1_config = self.camera_configs[0]
            cam2_config = self.camera_configs[1]
            
            # Calculate 3D position using triangulation
            point_3d = self._triangulate_point(
                obj1.center_2d, obj2.center_2d,
                cam1_config, cam2_config
            )
            
            # Create 3D object
            obj_3d = DetectedObject(
                id=obj1.id,
                bbox=obj1.bbox,
                center_2d=obj1.center_2d,
                center_3d=point_3d,
                shape=obj1.shape,
                color=obj1.color,
                area=obj1.area
            )
            
            objects_3d.append(obj_3d)
        
        logger.info(f"Calculated 3D positions for {len(objects_3d)} objects")
        return objects_3d
    
    def _triangulate_point(self, point1: Tuple[float, float], point2: Tuple[float, float],
                           cam1_config: CameraConfig, cam2_config: CameraConfig) -> Tuple[float, float, float]:
        """
        Triangulate 3D point from two camera views
        
        Args:
            point1: 2D point in camera 1
            point2: 2D point in camera 2
            cam1_config: Camera 1 configuration
            cam2_config: Camera 2 configuration
            
        Returns:
            3D point coordinates (x, y, z)
        """
        # Convert to homogeneous coordinates
        point1_homog = np.array([point1[0], point1[1], 1.0])
        point2_homog = np.array([point2[0], point2[1], 1.0])
        
        # Undistort points - use only first 2 coordinates
        point1_undist = cv2.undistortPoints(
            np.array([[[point1[0], point1[1]]]], dtype=np.float32), 
            cam1_config.intrinsic_matrix, 
            cam1_config.distortion_coeffs
        ).reshape(2)
        
        point2_undist = cv2.undistortPoints(
            np.array([[[point2[0], point2[1]]]], dtype=np.float32), 
            cam2_config.intrinsic_matrix, 
            cam2_config.distortion_coeffs
        ).reshape(2)
        
        # Convert to normalized coordinates
        point1_norm = np.array([point1_undist[0], point1_undist[1], 1.0])
        point2_norm = np.array([point2_undist[0], point2_undist[1], 1.0])
        
        # Create projection matrices
        # Ensure translation vector is a column vector
        t1 = cam1_config.translation_vector.reshape(3, 1) if cam1_config.translation_vector.ndim == 1 else cam1_config.translation_vector
        t2 = cam2_config.translation_vector.reshape(3, 1) if cam2_config.translation_vector.ndim == 1 else cam2_config.translation_vector
        
        P1 = cam1_config.intrinsic_matrix @ np.hstack([cam1_config.rotation_matrix, t1])
        P2 = cam2_config.intrinsic_matrix @ np.hstack([cam2_config.rotation_matrix, t2])
        
        # Triangulate using DLT method
        point_3d = cv2.triangulatePoints(P1, P2, point1_norm[:2], point2_norm[:2])
        
        # Convert from homogeneous to 3D coordinates
        point_3d = point_3d[:3] / point_3d[3]
        
        return tuple(point_3d.flatten())
    
    def process_frame_pair(self, frame1: np.ndarray, frame2: np.ndarray) -> List[DetectedObject]:
        """
        Process a pair of frames from two cameras
        
        Args:
            frame1: Frame from camera 1
            frame2: Frame from camera 2
            
        Returns:
            List of detected objects with 3D positions
        """
        # Detect objects in each frame
        objects_cam1 = self.detect_objects_in_image(frame1, 0)
        objects_cam2 = self.detect_objects_in_image(frame2, 1)
        
        # Match objects between cameras
        matched_objects = self.match_objects_between_cameras(objects_cam1, objects_cam2)
        
        # Calculate 3D positions
        objects_3d = self.calculate_3d_positions(matched_objects)
        
        return objects_3d
    
    def process_single_frame(self, frame: np.ndarray, camera_id: int = 0) -> List[DetectedObject]:
        """
        Process a single frame from one camera (for webcam testing)
        
        Args:
            frame: Frame from camera
            camera_id: Camera ID
            
        Returns:
            List of detected objects (2D only, no 3D positions)
        """
        # Detect objects in the frame
        objects = self.detect_objects_in_image(frame, camera_id)
        
        # Return objects with 2D information only
        return objects
    
    def get_robotic_arm_targets(self, objects_3d: List[DetectedObject]) -> List[Dict]:
        """
        Convert detected objects to robotic arm target format
        
        Args:
            objects_3d: List of objects with 3D positions
            
        Returns:
            List of targets for robotic arm control
        """
        targets = []
        
        for obj in objects_3d:
            target = {
                'id': obj.id,
                'position': {
                    'x': obj.center_3d[0],
                    'y': obj.center_3d[1],
                    'z': obj.center_3d[2]
                },
                'shape': {
                    'type': obj.shape.shape_type.value,
                    'elongation': obj.shape.elongation,
                    'aspect_ratio': obj.shape.aspect_ratio,
                    'confidence': obj.shape.confidence
                },
                'color': obj.color,
                'area': obj.area,
                'bbox': obj.bbox
            }
            
            targets.append(target)
        
        return targets
    
    def visualize_detections(self, frame: np.ndarray, objects: List[DetectedObject], 
                           camera_id: int) -> np.ndarray:
        """
        Visualize detected objects on frame
        
        Args:
            frame: Input frame
            objects: Detected objects
            camera_id: Camera ID
            
        Returns:
            Frame with visualizations
        """
        vis_frame = frame.copy()
        
        for obj in objects:
            # Draw bounding box
            x, y, w, h = obj.bbox
            cv2.rectangle(vis_frame, (x, y), (x + w, y + h), obj.color, 2)
            
            # Draw object ID and shape
            label = f"ID:{obj.id} {obj.shape.shape_type.value}"
            cv2.putText(vis_frame, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 2)
            
            # Draw center point
            center_x, center_y = int(obj.center_2d[0]), int(obj.center_2d[1])
            cv2.circle(vis_frame, (center_x, center_y), 3, (0, 255, 0), -1)
            
            # Draw 3D coordinates if available
            if obj.center_3d != (0, 0, 0):
                coords_text = f"({obj.center_3d[0]:.2f}, {obj.center_3d[1]:.2f}, {obj.center_3d[2]:.2f})"
                cv2.putText(vis_frame, coords_text, (x, y + h + 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return vis_frame

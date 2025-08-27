#!/usr/bin/env python3
"""
Main Application for Food Detection and Robotic Arm Control
Robust version with simplified threading and resilient visualization.
"""

import os
import cv2
import time
import json
import argparse
import logging
import threading
from typing import List, Dict, Optional
from queue import Queue

# Our modules
from food_detection_system import FoodDetector
from camera_interface import CameraManager, create_camera
from camera_calibration import get_default_calibration, create_camera_configs_from_calibration

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class FoodDetectionApp:
    def __init__(self, config_file: str = None, webcam_mode: bool = False, mock_mode: bool = False):
        self.webcam_mode = webcam_mode
        self.mock_mode = mock_mode
        self.config = self._get_default_config()

        # Components
        self.camera_manager = CameraManager()
        self.food_detector: Optional[FoodDetector] = None

        # Threading
        self.is_running = False
        self.detection_thread: Optional[threading.Thread] = None
        self.detection_queue: Queue = Queue(maxsize=5)

        # Shared latest frame for display (read-only for UI)
        self._frame_lock = threading.Lock()
        self.latest_frame = None

        # Stats
        self.fps_counter = 0
        self.fps_start_time = time.time()

        logger.info("Food detection application initialized")

    def _get_default_config(self) -> Dict:
        if self.webcam_mode:
            return {
                'cameras': {
                    'webcam': {'type': 'usb', 'id': 0, 'resolution': [640, 480]}
                },
                'detection': {
                    'min_area': 1000,
                    'max_area': 50000,
                    'confidence_threshold': 0.5,
                    'nms_threshold': 0.3,
                    'max_objects': 10
                }
            }
        else:
            return {
                'cameras': {
                    'camera1': {'type': 'mock' if self.mock_mode else 'usb', 'id': 0, 'resolution': [640, 480]},
                    'camera2': {'type': 'mock' if self.mock_mode else 'usb', 'id': 1, 'resolution': [640, 480]}
                },
                'detection': {
                    'min_area': 1000,
                    'max_area': 50000,
                    'confidence_threshold': 0.5,
                    'nms_threshold': 0.3,
                    'max_objects': 10
                }
            }

    def setup_cameras(self) -> bool:
        try:
            cams = self.config['cameras']
            for name, cfg in cams.items():
                cam = create_camera(cfg['type'], cfg['id'], tuple(cfg['resolution']))
                if not self.camera_manager.add_camera(cam):
                    logger.error(f"Failed to add {name}")
                    return False
            # Require 1 in webcam mode, 2 in stereo
            need = 1 if self.webcam_mode else 2
            if self.camera_manager.get_camera_count() < need:
                logger.error(f"Need at least {need} camera(s)")
                return False
            logger.info(f"Setup {self.camera_manager.get_camera_count()} camera(s)")
            return True
        except Exception as e:
            logger.error(f"setup_cameras error: {e}")
            return False

    def setup_food_detector(self) -> bool:
        try:
            cam1_cal, cam2_cal = get_default_calibration()
            camera_configs = create_camera_configs_from_calibration(cam1_cal, cam2_cal)
            self.food_detector = FoodDetector(camera_configs, self.config['detection'])
            logger.info("Food detector ready")
            return True
        except Exception as e:
            logger.error(f"setup_food_detector error: {e}")
            return False

    def start_detection(self):
        if not self.camera_manager.get_camera_count() or not self.food_detector:
            logger.error("Not ready to start detection")
            return
        self.is_running = True
        self.detection_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.detection_thread.start()
        logger.info("Detection started")

    def stop_detection(self):
        self.is_running = False
        if self.detection_thread:
            self.detection_thread.join(timeout=2)
        logger.info("Detection stopped")

    def _detection_loop(self):
        consecutive_failures = 0
        max_failures = 3
        while self.is_running:
            try:
                frames = self.camera_manager.read_all_cameras()
                if len(frames) < (1 if self.webcam_mode else 2):
                    time.sleep(0.05)
                    continue

                # Prepare frames
                if self.webcam_mode:
                    frame1 = frames[0][1]
                    if frame1 is None or frame1.size == 0:
                        time.sleep(0.05)
                        continue
                    with self._frame_lock:
                        self.latest_frame = frame1.copy()
                    start = time.time()
                    detected = self.food_detector.process_single_frame(frame1, 0)
                else:
                    frames.sort(key=lambda x: x[0])
                    frame1 = frames[0][1]
                    frame2 = frames[1][1]
                    if any(f is None or f.size == 0 for f in (frame1, frame2)):
                        time.sleep(0.05)
                        continue
                    with self._frame_lock:
                        self.latest_frame = frame1.copy()
                    start = time.time()
                    detected = self.food_detector.process_frame_pair(frame1, frame2)

                processing_time = time.time() - start
                consecutive_failures = 0

                # Stats
                self.fps_counter += 1
                if time.time() - self.fps_start_time >= 1.0:
                    fps = self.fps_counter / (time.time() - self.fps_start_time)
                    logger.info(f"Detection FPS: {fps:.1f}, Processing: {processing_time*1000:.1f}ms")
                    self.fps_counter = 0
                    self.fps_start_time = time.time()

                # Push results (non-blocking)
                if detected:
                    try:
                        self.detection_queue.put({'objects': detected, 'timestamp': time.time()}, timeout=0.05)
                    except:
                        # If full, skip this frame
                        pass

                # Pace
                sleep_time = max(0.01, 0.033 - processing_time)
                time.sleep(sleep_time)

            except Exception as e:
                consecutive_failures += 1
                logger.error(f"Detection loop error: {e}")
                if consecutive_failures >= max_failures:
                    time.sleep(2.0)
                    consecutive_failures = 0
                else:
                    time.sleep(0.2)
        logger.info("Detection loop ended")

    def get_latest_detections(self) -> Optional[Dict]:
        if self.detection_queue.empty():
            return None
        try:
            return self.detection_queue.get_nowait()
        except:
            return None

    def run_interactive(self):
        # OpenCV compat tweaks (sometimes helps on macOS)
        os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_MSMF", "0")
        cv2.setUseOptimized(False)

        if not self.setup_cameras():
            return
        if not self.setup_food_detector():
            return
        self.start_detection()

        window_name = "Food Detection - Webcam" if self.webcam_mode else "Food Detection - Stereo"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        try:
            while True:
                data = self.get_latest_detections()

                # Get display frame (do not access camera here)
                display_frame = None
                with self._frame_lock:
                    if self.latest_frame is not None:
                        display_frame = self.latest_frame.copy()

                if display_frame is not None and display_frame.size != 0:
                    if data and data.get('objects'):
                        for obj in data['objects']:
                            try:
                                # Draw 2D bbox if available
                                if hasattr(obj, 'bbox') and obj.bbox is not None:
                                    x, y, w, h = obj.bbox
                                    cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                                    if hasattr(obj, 'shape') and obj.shape:
                                        label = f"{obj.shape.shape_type.value} ({obj.shape.confidence:.2f})"
                                        cv2.putText(display_frame, label, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                                # If 3D center available
                                if hasattr(obj, 'center_3d') and obj.center_3d:
                                    cx, cy, cz = obj.center_3d
                                    cv2.putText(display_frame, f"3D: ({cx:.2f},{cy:.2f},{cz:.2f})", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            except Exception:
                                continue
                    try:
                        cv2.imshow(window_name, display_frame)
                    except Exception as e:
                        logger.error(f"Display error: {e}")
                        break

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self._save_detection_results()
                elif key == ord('t'):
                    targets = self.get_robotic_arm_targets()
                    print(json.dumps(targets, indent=2))

        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.stop_detection()
            self.camera_manager.close_all_cameras()
            cv2.destroyAllWindows()
            logger.info("Application stopped")

    def run_headless(self):
        if not self.setup_cameras():
            return
        if not self.setup_food_detector():
            return
        self.start_detection()
        try:
            while True:
                data = self.get_latest_detections()
                if data and data.get('objects'):
                    targets = self.get_robotic_arm_targets()
                    print(json.dumps(targets, indent=2))
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.stop_detection()
            self.camera_manager.close_all_cameras()
            logger.info("Application stopped")

    def _save_detection_results(self):
        try:
            data = self.get_latest_detections()
            if not data:
                logger.warning("No detections to save")
                return
            fname = f"detection_results_{int(time.time())}.json"
            with open(fname, 'w') as f:
                json.dump([self._obj_to_target(obj) for obj in data['objects']], f, indent=2)
            logger.info(f"Saved {fname}")
        except Exception as e:
            logger.error(f"Save error: {e}")

    def _obj_to_target(self, obj) -> Dict:
        try:
            target = {
                'id': getattr(obj, 'id', -1),
                'bbox': getattr(obj, 'bbox', None),
                'color': getattr(obj, 'color', None),
                'area': getattr(obj, 'area', None),
                'shape': None,
                'position': None
            }
            if hasattr(obj, 'shape') and obj.shape:
                target['shape'] = {
                    'type': obj.shape.shape_type.value,
                    'elongation': obj.shape.elongation,
                    'aspect_ratio': obj.shape.aspect_ratio,
                    'confidence': obj.shape.confidence
                }
            if hasattr(obj, 'center_3d') and obj.center_3d:
                target['position'] = {'x': obj.center_3d[0], 'y': obj.center_3d[1], 'z': obj.center_3d[2]}
            return target
        except Exception:
            return {}

    def get_robotic_arm_targets(self) -> List[Dict]:
        data = self.get_latest_detections()
        if not data:
            return []
        return [self._obj_to_target(obj) for obj in data['objects']]


def main():
    parser = argparse.ArgumentParser(description='Food Detection System')
    parser.add_argument('--config', '-c', type=str, help='Optional config path (unused in this simplified version)')
    parser.add_argument('--headless', action='store_true', help='Run without UI')
    parser.add_argument('--mock', action='store_true', help='Use mock cameras for stereo')
    parser.add_argument('--webcam', action='store_true', help='Use single webcam')
    args = parser.parse_args()

    app = FoodDetectionApp(webcam_mode=args.webcam, mock_mode=args.mock)
    if args.headless:
        app.run_headless()
    else:
        app.run_interactive()

if __name__ == '__main__':
    main()

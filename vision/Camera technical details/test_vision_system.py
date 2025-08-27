#!/usr/bin/env python3
"""
Test Script for Food Detection Vision System
Tests the system with mock cameras to verify functionality
"""

import sys
import time
import json
from food_detection_system import FoodDetector, CameraConfig
from camera_interface import create_camera, CameraManager
from camera_calibration import get_default_calibration, create_camera_configs_from_calibration

def test_camera_interface():
    """Test camera interface functionality"""
    print("Testing Camera Interface")
    print("=" * 40)
    
    try:
        # Create mock cameras
        camera1 = create_camera('mock', 0, (640, 480))
        camera2 = create_camera('mock', 1, (640, 480))
        
        # Test camera operations
        assert camera1.open(), "Failed to open camera 1"
        assert camera2.open(), "Failed to open camera 2"
        
        # Test frame reading
        ret1, frame1 = camera1.read()
        ret2, frame2 = camera2.read()
        
        assert ret1 and ret2, "Failed to read frames"
        assert frame1 is not None and frame2 is not None, "Frames are None"
        assert frame1.shape == (480, 640, 3), f"Unexpected frame1 shape: {frame1.shape}"
        assert frame2.shape == (480, 640, 3), f"Unexpected frame2 shape: {frame2.shape}"
        
        print("✓ Camera interface test passed")
        
        # Cleanup
        camera1.close()
        camera2.close()
        
        return True
        
    except Exception as e:
        print(f"✗ Camera interface test failed: {e}")
        return False

def test_food_detection():
    """Test food detection functionality"""
    print("\nTesting Food Detection System")
    print("=" * 40)
    
    try:
        # Get default calibration
        camera1_cal, camera2_cal = get_default_calibration()
        
        # Create camera configurations
        camera_configs = create_camera_configs_from_calibration(camera1_cal, camera2_cal)
        
        # Create food detector
        detector = FoodDetector(camera_configs)
        
        # Create mock cameras
        camera1 = create_camera('mock', 0, (640, 480))
        camera2 = create_camera('mock', 1, (640, 480))
        
        camera1.open()
        camera2.open()
        
        # Read frames
        ret1, frame1 = camera1.read()
        ret2, frame2 = camera2.read()
        
        if not ret1 or not ret2:
            raise Exception("Failed to read camera frames")
        
        # Test object detection in single image
        objects_cam1 = detector.detect_objects_in_image(frame1, 0)
        objects_cam2 = detector.detect_objects_in_image(frame2, 1)
        
        print(f"Camera 1 detected {len(objects_cam1)} objects")
        print(f"Camera 2 detected {len(objects_cam2)} objects")
        
        # Test object matching
        matched_objects = detector.match_objects_between_cameras(objects_cam1, objects_cam2)
        print(f"Matched {len(matched_objects)} objects between cameras")
        
        # Test 3D position calculation
        objects_3d = detector.calculate_3d_positions(matched_objects)
        print(f"Calculated 3D positions for {len(objects_3d)} objects")
        
        # Test robotic arm targets
        targets = detector.get_robotic_arm_targets(objects_3d)
        print(f"Generated {len(targets)} robotic arm targets")
        
        if targets:
            print("\nSample target:")
            print(json.dumps(targets[0], indent=2))
        
        # Cleanup
        camera1.close()
        camera2.close()
        
        print("✓ Food detection test passed")
        return True
        
    except Exception as e:
        print(f"✗ Food detection test failed: {e}")
        return False

def test_integration():
    """Test full system integration"""
    print("\nTesting System Integration")
    print("=" * 40)
    
    try:
        # Create camera manager
        camera_manager = CameraManager()
        
        # Add mock cameras
        camera1 = create_camera('mock', 0, (640, 480))
        camera2 = create_camera('mock', 1, (640, 480))
        
        assert camera_manager.add_camera(camera1), "Failed to add camera 1"
        assert camera_manager.add_camera(camera2), "Failed to add camera 2"
        
        # Test camera manager
        assert camera_manager.get_camera_count() == 2, "Wrong camera count"
        assert camera_manager.is_ready(), "Camera manager not ready"
        
        # Test frame reading
        frames = camera_manager.read_all_cameras()
        assert len(frames) == 2, f"Expected 2 frames, got {len(frames)}"
        
        # Cleanup
        camera_manager.close_all_cameras()
        
        print("✓ System integration test passed")
        return True
        
    except Exception as e:
        print(f"✗ System integration test failed: {e}")
        return False

def test_performance():
    """Test system performance"""
    print("\nTesting System Performance")
    print("=" * 40)
    
    try:
        # Setup system
        camera1_cal, camera2_cal = get_default_calibration()
        camera_configs = create_camera_configs_from_calibration(camera1_cal, camera2_cal)
        detector = FoodDetector(camera_configs)
        
        camera1 = create_camera('mock', 0, (640, 480))
        camera2 = create_camera('mock', 1, (640, 480))
        
        camera1.open()
        camera2.open()
        
        # Performance test
        num_frames = 10
        total_time = 0
        
        for i in range(num_frames):
            ret1, frame1 = camera1.read()
            ret2, frame2 = camera2.read()
            
            if ret1 and ret2:
                start_time = time.time()
                objects = detector.process_frame_pair(frame1, frame2)
                processing_time = time.time() - start_time
                
                total_time += processing_time
                print(f"Frame {i+1}: {len(objects)} objects, {processing_time*1000:.1f}ms")
        
        avg_time = total_time / num_frames
        fps = 1.0 / avg_time
        
        print(f"\nPerformance Results:")
        print(f"  Average processing time: {avg_time*1000:.1f}ms")
        print(f"  Theoretical FPS: {fps:.1f}")
        
        # Cleanup
        camera1.close()
        camera2.close()
        
        print("✓ Performance test passed")
        return True
        
    except Exception as e:
        print(f"✗ Performance test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("Food Detection Vision System - Test Suite")
    print("=" * 60)
    
    tests = [
        ("Camera Interface", test_camera_interface),
        ("Food Detection", test_food_detection),
        ("System Integration", test_integration),
        ("Performance", test_performance)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        try:
            if test_func():
                passed += 1
        except Exception as e:
            print(f"✗ {test_name} test crashed: {e}")
    
    print("\n" + "=" * 60)
    print(f"Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! The vision system is working correctly.")
        print("\nTo run the full application:")
        print("  python main_application.py --mock")
        print("\nTo run in headless mode:")
        print("  python main_application.py --mock --headless")
    else:
        print("❌ Some tests failed. Please check the errors above.")
        sys.exit(1)

if __name__ == "__main__":
    main()


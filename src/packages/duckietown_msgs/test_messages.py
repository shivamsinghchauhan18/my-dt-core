#!/usr/bin/env python3
"""
Test script to verify message definitions are correct.
This script can be run to validate the message structure.
"""

import sys
import os

# Add the src directory to Python path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def test_message_imports():
    """Test that all message types can be imported (when ROS is available)."""
    try:
        # Test basic ROS imports
        from std_msgs.msg import Header
        from geometry_msgs.msg import Point32, Vector3
        print("✓ Basic ROS message imports successful")
        
        # Test our utility classes
        from packages.image_processing.src.advanced_vision_utils import (
            RiskLevel, SafetyLevel, HealthStatus, BoundingBox, 
            VehicleState, ObjectDetection, SafetyStatus,
            AdaptiveThresholdDetector, TemporalConsistencyFilter,
            PolynomialCurveFitter, ROIManager, PerformanceMonitor
        )
        print("✓ Advanced vision utilities import successful")
        
        # Test utility functions
        from duckietown_msgs.msg_utils import (
            create_header, bounding_box_to_points, vector3d_to_ros
        )
        print("✓ Message utilities import successful")
        
        return True
        
    except ImportError as e:
        print(f"✗ Import failed: {e}")
        print("Note: ROS message imports will fail outside of ROS environment")
        return False


def test_data_structures():
    """Test that our data structures work correctly."""
    try:
        from packages.image_processing.src.advanced_vision_utils import (
            BoundingBox, VehicleState, ObjectDetection, SafetyStatus,
            Point3D, Vector3D, RiskLevel, SafetyLevel, HealthStatus
        )
        
        # Test BoundingBox
        bbox = BoundingBox(10, 20, 100, 50)
        assert bbox.center == (60, 45), f"Expected (60, 45), got {bbox.center}"
        assert bbox.area == 5000, f"Expected 5000, got {bbox.area}"
        print("✓ BoundingBox tests passed")
        
        # Test VehicleState
        pos = Point3D(1.0, 2.0, 0.0)
        vel = Vector3D(0.5, 0.0, 0.0)
        acc = Vector3D(0.0, 0.0, 0.0)
        vehicle_state = VehicleState(pos, vel, acc, 0.1, 1234567890.0)
        print("✓ VehicleState creation successful")
        
        # Test ObjectDetection
        detection = ObjectDetection(
            class_name="duckiebot",
            confidence=0.85,
            bounding_box=bbox,
            distance=2.5,
            relative_velocity=vel,
            risk_level=RiskLevel.MEDIUM,
            processing_time=0.02
        )
        assert detection.is_high_risk() == False, "Medium risk should not be high risk"
        print("✓ ObjectDetection tests passed")
        
        # Test SafetyStatus
        safety_status = SafetyStatus.create_safe_status()
        assert safety_status.overall_level == SafetyLevel.SAFE
        assert safety_status.system_health_score == 100.0
        print("✓ SafetyStatus tests passed")
        
        return True
        
    except Exception as e:
        print(f"✗ Data structure test failed: {e}")
        return False


def test_algorithms():
    """Test that algorithm classes work correctly."""
    try:
        import numpy as np
        from packages.image_processing.src.advanced_vision_utils import (
            AdaptiveThresholdDetector, PolynomialCurveFitter, 
            ROIManager, PerformanceMonitor
        )
        
        # Test AdaptiveThresholdDetector
        detector = AdaptiveThresholdDetector()
        test_image = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
        edges = detector.detect_edges(test_image)
        assert edges.shape == test_image.shape, "Edge detection should preserve image shape"
        print("✓ AdaptiveThresholdDetector tests passed")
        
        # Test PolynomialCurveFitter
        fitter = PolynomialCurveFitter(degree=2)
        test_points = [(i, i**2 + np.random.normal(0, 0.1)) for i in range(20)]
        coeffs = fitter.fit_curve(test_points)
        assert coeffs is not None, "Curve fitting should succeed with sufficient points"
        print("✓ PolynomialCurveFitter tests passed")
        
        # Test ROIManager
        roi_manager = ROIManager(640, 480)
        from packages.image_processing.src.advanced_vision_utils import VehicleState, Point3D, Vector3D
        vehicle_state = VehicleState(
            Point3D(0, 0, 0), Vector3D(1.0, 0, 0), Vector3D(0, 0, 0), 0, 0
        )
        roi = roi_manager.update_roi(vehicle_state)
        assert len(roi) == 4, "ROI should have 4 components (x, y, width, height)"
        print("✓ ROIManager tests passed")
        
        # Test PerformanceMonitor
        monitor = PerformanceMonitor()
        start_time = monitor.start_timing()
        import time
        time.sleep(0.001)  # 1ms
        processing_time = monitor.end_timing(start_time)
        assert processing_time > 0, "Processing time should be positive"
        print("✓ PerformanceMonitor tests passed")
        
        return True
        
    except Exception as e:
        print(f"✗ Algorithm test failed: {e}")
        return False


def main():
    """Run all tests."""
    print("Testing Duckietown Messages and Advanced Vision Utils...")
    print("=" * 60)
    
    tests_passed = 0
    total_tests = 3
    
    if test_message_imports():
        tests_passed += 1
    
    if test_data_structures():
        tests_passed += 1
        
    if test_algorithms():
        tests_passed += 1
    
    print("=" * 60)
    print(f"Tests completed: {tests_passed}/{total_tests} passed")
    
    if tests_passed == total_tests:
        print("✓ All tests passed!")
        return 0
    else:
        print("✗ Some tests failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
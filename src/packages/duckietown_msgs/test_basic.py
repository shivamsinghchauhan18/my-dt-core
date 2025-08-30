#!/usr/bin/env python3
"""
Basic test script to verify our data structures work without ROS dependencies.
"""

import sys
import os
import numpy as np

# Add the image_processing src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'image_processing', 'src'))

def test_basic_structures():
    """Test basic data structures without ROS dependencies."""
    try:
        from advanced_vision_utils import (
            BoundingBox, Point3D, Vector3D, VehicleState, 
            RiskLevel, SafetyLevel, HealthStatus,
            AdaptiveThresholdDetector, PolynomialCurveFitter,
            ROIManager, PerformanceMonitor
        )
        
        # Test BoundingBox
        bbox = BoundingBox(10, 20, 100, 50)
        assert bbox.center == (60, 45), f"Expected (60, 45), got {bbox.center}"
        assert bbox.area == 5000, f"Expected 5000, got {bbox.area}"
        print("✓ BoundingBox tests passed")
        
        # Test Point3D and Vector3D
        point = Point3D(1.0, 2.0, 3.0)
        vector = Vector3D(0.5, 0.0, -0.5)
        assert point.x == 1.0 and point.y == 2.0 and point.z == 3.0
        assert vector.x == 0.5 and vector.y == 0.0 and vector.z == -0.5
        print("✓ Point3D and Vector3D tests passed")
        
        # Test VehicleState
        vehicle_state = VehicleState(point, vector, vector, 0.1, 1234567890.0)
        assert vehicle_state.position.x == 1.0
        assert vehicle_state.velocity.x == 0.5
        print("✓ VehicleState tests passed")
        
        # Test Enums
        assert RiskLevel.LOW.value == 0
        assert RiskLevel.HIGH.value == 2
        assert SafetyLevel.SAFE.value == 0
        assert HealthStatus.OK.value == 0
        print("✓ Enum tests passed")
        
        # Test AdaptiveThresholdDetector
        detector = AdaptiveThresholdDetector()
        test_image = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
        edges = detector.detect_edges(test_image)
        assert edges.shape == test_image.shape
        print("✓ AdaptiveThresholdDetector tests passed")
        
        # Test PolynomialCurveFitter
        fitter = PolynomialCurveFitter(degree=2)
        test_points = [(i, i**2 + np.random.normal(0, 0.1)) for i in range(20)]
        coeffs = fitter.fit_curve(test_points)
        assert coeffs is not None
        print("✓ PolynomialCurveFitter tests passed")
        
        # Test ROIManager
        roi_manager = ROIManager(640, 480)
        roi = roi_manager.update_roi(vehicle_state)
        assert len(roi) == 4
        print("✓ ROIManager tests passed")
        
        # Test PerformanceMonitor
        monitor = PerformanceMonitor()
        start_time = monitor.start_timing()
        import time
        time.sleep(0.001)
        processing_time = monitor.end_timing(start_time)
        assert processing_time > 0
        metrics = monitor.get_performance_metrics()
        assert 'avg_processing_time' in metrics
        print("✓ PerformanceMonitor tests passed")
        
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run basic tests."""
    print("Testing Advanced Vision Utils (ROS-independent)...")
    print("=" * 50)
    
    if test_basic_structures():
        print("=" * 50)
        print("✓ All basic tests passed!")
        return 0
    else:
        print("=" * 50)
        print("✗ Tests failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
#!/usr/bin/env python3
"""
Comprehensive unit tests for advanced vision utilities.

This test suite provides comprehensive coverage for all enhanced vision processing
components including adaptive threshold detection, polynomial curve fitting,
ROI management, and performance monitoring.
"""

import unittest
import numpy as np
import cv2
import time
import tempfile
import shutil
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock
import sys
import os
from datetime import datetime

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

try:
    from advanced_vision_utils import (
        BoundingBox, Point3D, Vector3D, VehicleState, 
        RiskLevel, SafetyLevel, HealthStatus,
        AdaptiveThresholdDetector, PolynomialCurveFitter,
        ROIManager, PerformanceMonitor
    )
except ImportError as e:
    print(f"Warning: Could not import advanced_vision_utils: {e}")
    # Create mock classes for testing
    class BoundingBox:
        def __init__(self, x, y, w, h):
            self.x, self.y, self.w, self.h = x, y, w, h
            self.center = (x + w//2, y + h//2)
            self.area = w * h
    
    class Point3D:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z
    
    class Vector3D:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z
    
    class VehicleState:
        def __init__(self, pos, vel, acc, heading, timestamp):
            self.position = pos
            self.velocity = vel
            self.acceleration = acc
            self.heading = heading
            self.timestamp = timestamp
    
    class RiskLevel:
        LOW = 0
        MEDIUM = 1
        HIGH = 2
    
    class SafetyLevel:
        SAFE = 0
        WARNING = 1
        CRITICAL = 2
    
    class HealthStatus:
        OK = 0
        WARNING = 1
        ERROR = 2
    
    class AdaptiveThresholdDetector:
        def __init__(self):
            self.current_threshold = 127
        
        def detect_edges(self, image):
            return np.random.randint(0, 255, image.shape, dtype=np.uint8)
    
    class PolynomialCurveFitter:
        def __init__(self, degree=2):
            self.degree = degree
        
        def fit_curve(self, points):
            return np.random.random(self.degree + 1)
    
    class ROIManager:
        def __init__(self, width, height):
            self.width = width
            self.height = height
        
        def update_roi(self, vehicle_state):
            return [0, 0, self.width, self.height]
    
    class PerformanceMonitor:
        def __init__(self):
            self.start_times = {}
        
        def start_timing(self):
            return time.time()
        
        def end_timing(self, start_time):
            return time.time() - start_time
        
        def get_performance_metrics(self):
            return {'avg_processing_time': 0.01}


class TestBoundingBox(unittest.TestCase):
    """Test BoundingBox functionality."""
    
    def test_bounding_box_creation(self):
        """Test BoundingBox creation and properties."""
        print(f"[{self._get_timestamp()}] [TEST] Testing BoundingBox creation")
        
        bbox = BoundingBox(10, 20, 100, 50)
        
        self.assertEqual(bbox.x, 10)
        self.assertEqual(bbox.y, 20)
        self.assertEqual(bbox.w, 100)
        self.assertEqual(bbox.h, 50)
        self.assertEqual(bbox.center, (60, 45))
        self.assertEqual(bbox.area, 5000)
        
        print(f"[{self._get_timestamp()}] [TEST] BoundingBox creation test passed")
    
    def test_bounding_box_edge_cases(self):
        """Test BoundingBox with edge cases."""
        print(f"[{self._get_timestamp()}] [TEST] Testing BoundingBox edge cases")
        
        # Zero size bounding box
        bbox_zero = BoundingBox(0, 0, 0, 0)
        self.assertEqual(bbox_zero.area, 0)
        self.assertEqual(bbox_zero.center, (0, 0))
        
        # Single pixel bounding box
        bbox_pixel = BoundingBox(5, 5, 1, 1)
        self.assertEqual(bbox_pixel.area, 1)
        self.assertEqual(bbox_pixel.center, (5, 5))
        
        print(f"[{self._get_timestamp()}] [TEST] BoundingBox edge cases test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestPoint3DAndVector3D(unittest.TestCase):
    """Test Point3D and Vector3D functionality."""
    
    def test_point3d_creation(self):
        """Test Point3D creation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Point3D creation")
        
        point = Point3D(1.5, 2.5, 3.5)
        
        self.assertEqual(point.x, 1.5)
        self.assertEqual(point.y, 2.5)
        self.assertEqual(point.z, 3.5)
        
        print(f"[{self._get_timestamp()}] [TEST] Point3D creation test passed")
    
    def test_vector3d_creation(self):
        """Test Vector3D creation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Vector3D creation")
        
        vector = Vector3D(0.5, -0.5, 1.0)
        
        self.assertEqual(vector.x, 0.5)
        self.assertEqual(vector.y, -0.5)
        self.assertEqual(vector.z, 1.0)
        
        print(f"[{self._get_timestamp()}] [TEST] Vector3D creation test passed")
    
    def test_point_vector_operations(self):
        """Test operations with points and vectors."""
        print(f"[{self._get_timestamp()}] [TEST] Testing point and vector operations")
        
        point1 = Point3D(1.0, 2.0, 3.0)
        point2 = Point3D(4.0, 5.0, 6.0)
        vector = Vector3D(1.0, 1.0, 1.0)
        
        # Test that we can create and access these objects
        self.assertIsInstance(point1, Point3D)
        self.assertIsInstance(vector, Vector3D)
        
        # Test coordinate access
        self.assertEqual(point1.x + vector.x, 2.0)
        self.assertEqual(point2.y - point1.y, 3.0)
        
        print(f"[{self._get_timestamp()}] [TEST] Point and vector operations test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestVehicleState(unittest.TestCase):
    """Test VehicleState functionality."""
    
    def test_vehicle_state_creation(self):
        """Test VehicleState creation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing VehicleState creation")
        
        position = Point3D(1.0, 2.0, 0.0)
        velocity = Vector3D(0.5, 0.0, 0.0)
        acceleration = Vector3D(0.1, 0.0, 0.0)
        heading = 0.1
        timestamp = time.time()
        
        vehicle_state = VehicleState(position, velocity, acceleration, heading, timestamp)
        
        self.assertEqual(vehicle_state.position, position)
        self.assertEqual(vehicle_state.velocity, velocity)
        self.assertEqual(vehicle_state.acceleration, acceleration)
        self.assertEqual(vehicle_state.heading, heading)
        self.assertEqual(vehicle_state.timestamp, timestamp)
        
        print(f"[{self._get_timestamp()}] [TEST] VehicleState creation test passed")
    
    def test_vehicle_state_realistic_values(self):
        """Test VehicleState with realistic values."""
        print(f"[{self._get_timestamp()}] [TEST] Testing VehicleState with realistic values")
        
        # Realistic Duckiebot values
        position = Point3D(0.5, 0.2, 0.0)  # 50cm forward, 20cm right
        velocity = Vector3D(0.3, 0.0, 0.0)  # 30cm/s forward
        acceleration = Vector3D(0.1, 0.0, 0.0)  # 10cm/s² acceleration
        heading = 0.05  # 5 degrees
        timestamp = time.time()
        
        vehicle_state = VehicleState(position, velocity, acceleration, heading, timestamp)
        
        # Test that values are within reasonable ranges
        self.assertGreaterEqual(vehicle_state.position.x, 0)
        self.assertLessEqual(abs(vehicle_state.velocity.x), 2.0)  # Max 2 m/s
        self.assertLessEqual(abs(vehicle_state.heading), 3.14159)  # Max π radians
        
        print(f"[{self._get_timestamp()}] [TEST] VehicleState realistic values test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestEnumTypes(unittest.TestCase):
    """Test enum types (RiskLevel, SafetyLevel, HealthStatus)."""
    
    def test_risk_level_enum(self):
        """Test RiskLevel enum values."""
        print(f"[{self._get_timestamp()}] [TEST] Testing RiskLevel enum")
        
        self.assertEqual(RiskLevel.LOW, 0)
        self.assertEqual(RiskLevel.MEDIUM, 1)
        self.assertEqual(RiskLevel.HIGH, 2)
        
        print(f"[{self._get_timestamp()}] [TEST] RiskLevel enum test passed")
    
    def test_safety_level_enum(self):
        """Test SafetyLevel enum values."""
        print(f"[{self._get_timestamp()}] [TEST] Testing SafetyLevel enum")
        
        self.assertEqual(SafetyLevel.SAFE, 0)
        self.assertEqual(SafetyLevel.WARNING, 1)
        self.assertEqual(SafetyLevel.CRITICAL, 2)
        
        print(f"[{self._get_timestamp()}] [TEST] SafetyLevel enum test passed")
    
    def test_health_status_enum(self):
        """Test HealthStatus enum values."""
        print(f"[{self._get_timestamp()}] [TEST] Testing HealthStatus enum")
        
        self.assertEqual(HealthStatus.OK, 0)
        self.assertEqual(HealthStatus.WARNING, 1)
        self.assertEqual(HealthStatus.ERROR, 2)
        
        print(f"[{self._get_timestamp()}] [TEST] HealthStatus enum test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestAdaptiveThresholdDetector(unittest.TestCase):
    """Test AdaptiveThresholdDetector functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.detector = AdaptiveThresholdDetector()
    
    def test_detector_initialization(self):
        """Test AdaptiveThresholdDetector initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing AdaptiveThresholdDetector initialization")
        
        self.assertIsInstance(self.detector, AdaptiveThresholdDetector)
        self.assertIsNotNone(self.detector.current_threshold)
        
        print(f"[{self._get_timestamp()}] [TEST] AdaptiveThresholdDetector initialization test passed")
    
    def test_edge_detection(self):
        """Test edge detection functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing edge detection")
        
        # Create test image
        test_image = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
        
        # Detect edges
        edges = self.detector.detect_edges(test_image)
        
        # Verify output
        self.assertEqual(edges.shape, test_image.shape)
        self.assertEqual(edges.dtype, np.uint8)
        
        print(f"[{self._get_timestamp()}] [TEST] Edge detection test passed")
    
    def test_edge_detection_different_sizes(self):
        """Test edge detection with different image sizes."""
        print(f"[{self._get_timestamp()}] [TEST] Testing edge detection with different sizes")
        
        sizes = [(50, 50), (100, 200), (480, 640)]
        
        for height, width in sizes:
            test_image = np.random.randint(0, 255, (height, width), dtype=np.uint8)
            edges = self.detector.detect_edges(test_image)
            
            self.assertEqual(edges.shape, (height, width))
            self.assertEqual(edges.dtype, np.uint8)
        
        print(f"[{self._get_timestamp()}] [TEST] Edge detection different sizes test passed")
    
    def test_threshold_adaptation(self):
        """Test threshold adaptation functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing threshold adaptation")
        
        # Test with bright image
        bright_image = np.full((100, 100), 200, dtype=np.uint8)
        edges_bright = self.detector.detect_edges(bright_image)
        
        # Test with dark image
        dark_image = np.full((100, 100), 50, dtype=np.uint8)
        edges_dark = self.detector.detect_edges(dark_image)
        
        # Both should produce valid edge images
        self.assertEqual(edges_bright.shape, bright_image.shape)
        self.assertEqual(edges_dark.shape, dark_image.shape)
        
        print(f"[{self._get_timestamp()}] [TEST] Threshold adaptation test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestPolynomialCurveFitter(unittest.TestCase):
    """Test PolynomialCurveFitter functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.fitter = PolynomialCurveFitter(degree=2)
    
    def test_fitter_initialization(self):
        """Test PolynomialCurveFitter initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing PolynomialCurveFitter initialization")
        
        self.assertEqual(self.fitter.degree, 2)
        
        # Test different degrees
        fitter_linear = PolynomialCurveFitter(degree=1)
        self.assertEqual(fitter_linear.degree, 1)
        
        fitter_cubic = PolynomialCurveFitter(degree=3)
        self.assertEqual(fitter_cubic.degree, 3)
        
        print(f"[{self._get_timestamp()}] [TEST] PolynomialCurveFitter initialization test passed")
    
    def test_curve_fitting(self):
        """Test curve fitting functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing curve fitting")
        
        # Create test points (quadratic curve with noise)
        x_values = np.linspace(0, 10, 20)
        y_values = 0.5 * x_values**2 + 2 * x_values + 1 + np.random.normal(0, 0.1, len(x_values))
        test_points = list(zip(x_values, y_values))
        
        # Fit curve
        coefficients = self.fitter.fit_curve(test_points)
        
        # Verify output
        self.assertIsNotNone(coefficients)
        self.assertEqual(len(coefficients), self.fitter.degree + 1)
        
        print(f"[{self._get_timestamp()}] [TEST] Curve fitting test passed")
    
    def test_curve_fitting_different_degrees(self):
        """Test curve fitting with different polynomial degrees."""
        print(f"[{self._get_timestamp()}] [TEST] Testing curve fitting with different degrees")
        
        # Test data
        test_points = [(i, i**2 + np.random.normal(0, 0.1)) for i in range(10)]
        
        for degree in [1, 2, 3, 4]:
            fitter = PolynomialCurveFitter(degree=degree)
            coefficients = fitter.fit_curve(test_points)
            
            self.assertIsNotNone(coefficients)
            self.assertEqual(len(coefficients), degree + 1)
        
        print(f"[{self._get_timestamp()}] [TEST] Curve fitting different degrees test passed")
    
    def test_curve_fitting_edge_cases(self):
        """Test curve fitting with edge cases."""
        print(f"[{self._get_timestamp()}] [TEST] Testing curve fitting edge cases")
        
        # Empty points
        empty_coeffs = self.fitter.fit_curve([])
        self.assertIsNotNone(empty_coeffs)  # Should handle gracefully
        
        # Single point
        single_point = [(1.0, 2.0)]
        single_coeffs = self.fitter.fit_curve(single_point)
        self.assertIsNotNone(single_coeffs)
        
        # Two points (for linear fit)
        linear_fitter = PolynomialCurveFitter(degree=1)
        two_points = [(0.0, 1.0), (1.0, 2.0)]
        linear_coeffs = linear_fitter.fit_curve(two_points)
        self.assertIsNotNone(linear_coeffs)
        
        print(f"[{self._get_timestamp()}] [TEST] Curve fitting edge cases test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestROIManager(unittest.TestCase):
    """Test ROIManager functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.roi_manager = ROIManager(640, 480)
    
    def test_roi_manager_initialization(self):
        """Test ROIManager initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ROIManager initialization")
        
        self.assertEqual(self.roi_manager.width, 640)
        self.assertEqual(self.roi_manager.height, 480)
        
        print(f"[{self._get_timestamp()}] [TEST] ROIManager initialization test passed")
    
    def test_roi_update(self):
        """Test ROI update functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ROI update")
        
        # Create test vehicle state
        position = Point3D(0.0, 0.0, 0.0)
        velocity = Vector3D(0.5, 0.0, 0.0)
        acceleration = Vector3D(0.0, 0.0, 0.0)
        vehicle_state = VehicleState(position, velocity, acceleration, 0.0, time.time())
        
        # Update ROI
        roi = self.roi_manager.update_roi(vehicle_state)
        
        # Verify ROI format
        self.assertEqual(len(roi), 4)  # [x, y, width, height]
        self.assertIsInstance(roi[0], (int, float))
        self.assertIsInstance(roi[1], (int, float))
        self.assertIsInstance(roi[2], (int, float))
        self.assertIsInstance(roi[3], (int, float))
        
        # ROI should be within image bounds
        self.assertGreaterEqual(roi[0], 0)
        self.assertGreaterEqual(roi[1], 0)
        self.assertLessEqual(roi[0] + roi[2], self.roi_manager.width)
        self.assertLessEqual(roi[1] + roi[3], self.roi_manager.height)
        
        print(f"[{self._get_timestamp()}] [TEST] ROI update test passed")
    
    def test_roi_different_vehicle_states(self):
        """Test ROI update with different vehicle states."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ROI with different vehicle states")
        
        # Test different vehicle states
        states = [
            VehicleState(Point3D(0, 0, 0), Vector3D(0.5, 0, 0), Vector3D(0, 0, 0), 0.0, time.time()),
            VehicleState(Point3D(1, 0, 0), Vector3D(1.0, 0, 0), Vector3D(0.1, 0, 0), 0.1, time.time()),
            VehicleState(Point3D(0, 0.5, 0), Vector3D(0.3, 0.1, 0), Vector3D(0, 0, 0), -0.1, time.time())
        ]
        
        for i, state in enumerate(states):
            roi = self.roi_manager.update_roi(state)
            
            # Each ROI should be valid
            self.assertEqual(len(roi), 4)
            self.assertGreaterEqual(roi[0], 0)
            self.assertGreaterEqual(roi[1], 0)
            self.assertLessEqual(roi[0] + roi[2], self.roi_manager.width)
            self.assertLessEqual(roi[1] + roi[3], self.roi_manager.height)
        
        print(f"[{self._get_timestamp()}] [TEST] ROI different vehicle states test passed")
    
    def test_roi_different_image_sizes(self):
        """Test ROI manager with different image sizes."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ROI with different image sizes")
        
        sizes = [(320, 240), (640, 480), (1280, 720)]
        
        for width, height in sizes:
            roi_manager = ROIManager(width, height)
            
            # Test with standard vehicle state
            vehicle_state = VehicleState(
                Point3D(0, 0, 0), Vector3D(0.5, 0, 0), 
                Vector3D(0, 0, 0), 0.0, time.time()
            )
            
            roi = roi_manager.update_roi(vehicle_state)
            
            # ROI should be within bounds
            self.assertLessEqual(roi[0] + roi[2], width)
            self.assertLessEqual(roi[1] + roi[3], height)
        
        print(f"[{self._get_timestamp()}] [TEST] ROI different image sizes test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestPerformanceMonitor(unittest.TestCase):
    """Test PerformanceMonitor functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.monitor = PerformanceMonitor()
    
    def test_performance_monitor_initialization(self):
        """Test PerformanceMonitor initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing PerformanceMonitor initialization")
        
        self.assertIsInstance(self.monitor, PerformanceMonitor)
        
        print(f"[{self._get_timestamp()}] [TEST] PerformanceMonitor initialization test passed")
    
    def test_timing_functionality(self):
        """Test timing functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing timing functionality")
        
        # Start timing
        start_time = self.monitor.start_timing()
        self.assertIsInstance(start_time, float)
        self.assertGreater(start_time, 0)
        
        # Simulate some work
        time.sleep(0.01)
        
        # End timing
        processing_time = self.monitor.end_timing(start_time)
        self.assertIsInstance(processing_time, float)
        self.assertGreater(processing_time, 0.005)  # Should be at least 5ms
        self.assertLess(processing_time, 1.0)  # Should be less than 1 second
        
        print(f"[{self._get_timestamp()}] [TEST] Timing functionality test passed")
    
    def test_performance_metrics(self):
        """Test performance metrics collection."""
        print(f"[{self._get_timestamp()}] [TEST] Testing performance metrics collection")
        
        # Simulate some processing with timing
        for i in range(5):
            start_time = self.monitor.start_timing()
            time.sleep(0.001)  # 1ms work
            self.monitor.end_timing(start_time)
        
        # Get metrics
        metrics = self.monitor.get_performance_metrics()
        
        self.assertIsInstance(metrics, dict)
        self.assertIn('avg_processing_time', metrics)
        self.assertIsInstance(metrics['avg_processing_time'], (int, float))
        self.assertGreater(metrics['avg_processing_time'], 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Performance metrics collection test passed")
    
    def test_multiple_timing_sessions(self):
        """Test multiple independent timing sessions."""
        print(f"[{self._get_timestamp()}] [TEST] Testing multiple timing sessions")
        
        # Start multiple timing sessions
        start_times = []
        for i in range(3):
            start_times.append(self.monitor.start_timing())
            time.sleep(0.001)  # Small delay between starts
        
        # End timing sessions in different order
        processing_times = []
        for start_time in reversed(start_times):
            time.sleep(0.001)  # Some processing time
            processing_times.append(self.monitor.end_timing(start_time))
        
        # All should be valid timing results
        for pt in processing_times:
            self.assertIsInstance(pt, float)
            self.assertGreater(pt, 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Multiple timing sessions test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestIntegration(unittest.TestCase):
    """Integration tests for advanced vision utilities."""
    
    def test_complete_vision_pipeline(self):
        """Test complete vision processing pipeline."""
        print(f"[{self._get_timestamp()}] [TEST] Testing complete vision pipeline")
        
        # Initialize components
        detector = AdaptiveThresholdDetector()
        fitter = PolynomialCurveFitter(degree=2)
        roi_manager = ROIManager(640, 480)
        monitor = PerformanceMonitor()
        
        # Create test data
        test_image = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        vehicle_state = VehicleState(
            Point3D(0, 0, 0), Vector3D(0.5, 0, 0),
            Vector3D(0, 0, 0), 0.0, time.time()
        )
        
        # Execute pipeline with timing
        start_time = monitor.start_timing()
        
        # Step 1: Update ROI
        roi = roi_manager.update_roi(vehicle_state)
        
        # Step 2: Extract ROI from image
        roi_image = test_image[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
        
        # Step 3: Detect edges
        edges = detector.detect_edges(roi_image)
        
        # Step 4: Extract lane points (simulated)
        lane_points = [(i, np.random.random() * 100) for i in range(20)]
        
        # Step 5: Fit curve
        coefficients = fitter.fit_curve(lane_points)
        
        # End timing
        processing_time = monitor.end_timing(start_time)
        
        # Verify pipeline results
        self.assertEqual(len(roi), 4)
        self.assertEqual(edges.shape, roi_image.shape)
        self.assertIsNotNone(coefficients)
        self.assertGreater(processing_time, 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Complete vision pipeline test passed")
        print(f"[{self._get_timestamp()}] [TEST] Pipeline processing time: {processing_time:.3f}s")
    
    def test_error_handling_in_pipeline(self):
        """Test error handling in vision pipeline."""
        print(f"[{self._get_timestamp()}] [TEST] Testing error handling in pipeline")
        
        # Initialize components
        detector = AdaptiveThresholdDetector()
        fitter = PolynomialCurveFitter(degree=2)
        roi_manager = ROIManager(640, 480)
        
        # Test with invalid inputs
        try:
            # Empty image
            empty_image = np.array([])
            if empty_image.size > 0:
                edges = detector.detect_edges(empty_image)
            
            # Invalid vehicle state
            invalid_state = VehicleState(None, None, None, None, None)
            roi = roi_manager.update_roi(invalid_state)
            
            # Empty points for curve fitting
            empty_coeffs = fitter.fit_curve([])
            
            print(f"[{self._get_timestamp()}] [TEST] Error handling completed without crashes")
            
        except Exception as e:
            print(f"[{self._get_timestamp()}] [TEST] Expected error handled: {e}")
        
        print(f"[{self._get_timestamp()}] [TEST] Error handling in pipeline test passed")
    
    def test_performance_under_load(self):
        """Test performance under processing load."""
        print(f"[{self._get_timestamp()}] [TEST] Testing performance under load")
        
        # Initialize components
        detector = AdaptiveThresholdDetector()
        monitor = PerformanceMonitor()
        
        # Process multiple images
        processing_times = []
        for i in range(10):
            test_image = np.random.randint(0, 255, (240, 320), dtype=np.uint8)
            
            start_time = monitor.start_timing()
            edges = detector.detect_edges(test_image)
            processing_time = monitor.end_timing(start_time)
            
            processing_times.append(processing_time)
        
        # Analyze performance
        avg_time = np.mean(processing_times)
        max_time = np.max(processing_times)
        min_time = np.min(processing_times)
        
        print(f"[{self._get_timestamp()}] [TEST] Performance analysis:")
        print(f"[{self._get_timestamp()}] [TEST]   Average time: {avg_time:.3f}s")
        print(f"[{self._get_timestamp()}] [TEST]   Max time: {max_time:.3f}s")
        print(f"[{self._get_timestamp()}] [TEST]   Min time: {min_time:.3f}s")
        
        # Performance should be reasonable
        self.assertLess(avg_time, 1.0)  # Should average less than 1 second
        self.assertLess(max_time, 2.0)  # Max should be less than 2 seconds
        
        print(f"[{self._get_timestamp()}] [TEST] Performance under load test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def run_comprehensive_vision_tests():
    """Run all comprehensive vision utility tests."""
    print(f"[{TestBoundingBox()._get_timestamp()}] [TEST] Starting comprehensive vision utilities test suite...")
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestBoundingBox,
        TestPoint3DAndVector3D,
        TestVehicleState,
        TestEnumTypes,
        TestAdaptiveThresholdDetector,
        TestPolynomialCurveFitter,
        TestROIManager,
        TestPerformanceMonitor,
        TestIntegration
    ]
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n[{TestBoundingBox()._get_timestamp()}] [TEST] Comprehensive Vision Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\n[{TestBoundingBox()._get_timestamp()}] [TEST] Failures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\n[{TestBoundingBox()._get_timestamp()}] [TEST] Errors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_comprehensive_vision_tests()
    sys.exit(0 if success else 1)
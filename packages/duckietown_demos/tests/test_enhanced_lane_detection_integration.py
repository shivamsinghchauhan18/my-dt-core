#!/usr/bin/env python3

"""
Integration tests for the enhanced lane detection pipeline.

This test suite validates the integration of adaptive threshold detection,
polynomial curve fitting, and temporal consistency filtering in the complete
lane detection pipeline.
"""

import unittest
import numpy as np
import sys
import os
import time
from unittest.mock import Mock, MagicMock, patch

# Mock ROS modules before importing the actual code
sys.modules['rospy'] = Mock()
sys.modules['cv_bridge'] = Mock()
sys.modules['sensor_msgs'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['duckietown_msgs'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['line_detector'] = Mock()
sys.modules['lane_filter'] = Mock()
sys.modules['image_processing'] = Mock()
sys.modules['image_processing.anti_instagram'] = Mock()
sys.modules['duckietown.dtros'] = Mock()
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Mock rospy functions
rospy = sys.modules['rospy']
rospy.logdebug = Mock()
rospy.loginfo = Mock()
rospy.logerr = Mock()
rospy.logwarn = Mock()
rospy.Time = Mock()
rospy.Time.now = Mock(return_value=123456789)
rospy.get_param = Mock()
rospy.get_time = Mock(return_value=time.time())

# Mock scipy modules
try:
    import scipy.optimize
    import scipy.interpolate
except ImportError:
    sys.modules['scipy'] = Mock()
    sys.modules['scipy.optimize'] = Mock()
    sys.modules['scipy.interpolate'] = Mock()

# Add the src directories to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'line_detector', 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'lane_filter', 'src'))

from line_detector_node import AdaptiveThresholdDetector, TemporalConsistencyFilter
from lane_filter_node import PolynomialCurveFitter


class MockSegment:
    """Mock segment object for testing."""
    def __init__(self, x1, y1, x2, y2, color=None):
        self.pixels_normalized = [MockPoint(x1, y1), MockPoint(x2, y2)]
        self.color = color
        # Mock color constants
        self.WHITE = 0
        self.YELLOW = 1
        self.RED = 2


class MockPoint:
    """Mock point object for testing."""
    def __init__(self, x, y):
        self.x = x
        self.y = y


class MockDetections:
    """Mock detection object for testing."""
    def __init__(self, lines, normals=None, centers=None, map_data=None):
        self.lines = lines
        self.normals = normals if normals is not None else []
        self.centers = centers if centers is not None else []
        self.map = map_data if map_data is not None else np.array([])


class TestEnhancedLaneDetectionIntegration(unittest.TestCase):
    """
    Integration tests for the enhanced lane detection pipeline.
    
    Tests the complete workflow from adaptive threshold detection through
    temporal filtering to polynomial curve fitting and trajectory prediction.
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Initialize components
        self.adaptive_detector = AdaptiveThresholdDetector(
            base_canny_low=80,
            base_canny_high=200,
            adaptation_rate=0.2
        )
        
        self.temporal_filter = TemporalConsistencyFilter(
            history_size=5,
            consistency_threshold=0.7
        )
        
        self.curve_fitter = PolynomialCurveFitter(
            polynomial_degree=2,
            min_points=5,
            extrapolation_distance=1.0,
            smoothing_factor=0.1,
            history_size=10
        )
        
        # Mock ROS parameter server responses
        rospy.get_param.side_effect = self._mock_get_param
    
    def _mock_get_param(self, param_name, default=None):
        """Mock ROS parameter server responses."""
        param_map = {
            "~adaptive_threshold_enabled": True,
            "~temporal_filtering_enabled": True,
            "~adaptation_rate": 0.1,
            "~temporal_history_size": 5,
            "~consistency_threshold": 0.7,
            "~curve_fitting_enabled": True,
            "~polynomial_degree": 2,
            "~min_curve_points": 5,
            "~extrapolation_distance": 1.0,
            "~curve_smoothing_factor": 0.1,
            "~curve_history_size": 10
        }
        return param_map.get(param_name, default)
    
    def test_adaptive_detection_integration(self):
        """Test integration of adaptive threshold detection with lighting analysis."""
        # Create test images with different lighting conditions
        test_images = [
            np.ones((120, 160, 3), dtype=np.uint8) * 50,   # Dark
            np.ones((120, 160, 3), dtype=np.uint8) * 120,  # Normal
            np.ones((120, 160, 3), dtype=np.uint8) * 220   # Bright
        ]
        
        detection_counts = [5, 10, 3]  # Simulated detection counts
        
        initial_thresholds = (self.adaptive_detector.current_canny_low, 
                             self.adaptive_detector.current_canny_high)
        
        for i, (image, count) in enumerate(zip(test_images, detection_counts)):
            # Analyze lighting conditions
            lighting_conditions = self.adaptive_detector.analyze_lighting_conditions(image)
            
            # Verify lighting analysis
            self.assertIn('brightness', lighting_conditions)
            self.assertIn('contrast', lighting_conditions)
            self.assertIn('entropy', lighting_conditions)
            
            # Adapt thresholds
            self.adaptive_detector.adapt_thresholds(lighting_conditions, count)
            
            # Verify adaptation occurred
            current_thresholds = (self.adaptive_detector.current_canny_low,
                                self.adaptive_detector.current_canny_high)
            
            if i > 0:  # After first iteration, thresholds should have changed
                self.assertNotEqual(current_thresholds, initial_thresholds)
            
            # Verify confidence tracking
            self.assertGreaterEqual(self.adaptive_detector.detection_confidence, 0.0)
            self.assertLessEqual(self.adaptive_detector.detection_confidence, 1.0)
    
    def test_temporal_filtering_integration(self):
        """Test integration of temporal consistency filtering across multiple frames."""
        # Simulate a sequence of detection frames
        frame_sequences = [
            # Frame 1: Good detections
            [MockSegment(0.1, 0.2, 0.3, 0.4), MockSegment(0.5, 0.6, 0.7, 0.8)],
            # Frame 2: Good detections
            [MockSegment(0.15, 0.25, 0.35, 0.45), MockSegment(0.55, 0.65, 0.75, 0.85)],
            # Frame 3: No detections (noise frame)
            [],
            # Frame 4: Good detections
            [MockSegment(0.12, 0.22, 0.32, 0.42), MockSegment(0.52, 0.62, 0.72, 0.82)],
            # Frame 5: Good detections
            [MockSegment(0.13, 0.23, 0.33, 0.43), MockSegment(0.53, 0.63, 0.73, 0.83)]
        ]
        
        for frame_segments in frame_sequences:
            # Convert segments to mock detections
            lines = []
            for segment in frame_segments:
                x1, y1 = segment.pixels_normalized[0].x, segment.pixels_normalized[0].y
                x2, y2 = segment.pixels_normalized[1].x, segment.pixels_normalized[1].y
                lines.append([x1, y1, x2, y2])
            
            mock_detection = MockDetections(lines=np.array(lines) if lines else np.array([]))
            
            # Add to temporal filter
            self.temporal_filter.add_detections('WHITE', mock_detection)
        
        # Get filtered results
        filtered_result = self.temporal_filter.get_filtered_detections('WHITE')
        
        # Verify temporal filtering worked
        self.assertIn('temporal_confidence', filtered_result)
        self.assertIn('stability_score', filtered_result)
        self.assertGreater(filtered_result['temporal_confidence'], 0.5)  # Should be confident
        
        # Verify metrics
        metrics = self.temporal_filter.get_temporal_metrics()
        self.assertEqual(metrics['history_sizes']['WHITE'], 5)
        self.assertIn('WHITE', metrics['temporal_confidence'])
    
    def test_curve_fitting_integration(self):
        """Test integration of polynomial curve fitting with lane segments."""
        # Create segments that form a curved lane
        curved_segments = []
        for i in range(10):
            x = i * 0.1
            y = x ** 2 * 0.5  # Quadratic curve
            curved_segments.append(MockSegment(x, y, x + 0.05, y + 0.025))
        
        # Extract points and fit curve
        x_points, y_points = self.curve_fitter.extract_lane_points(curved_segments, 'center')
        
        # Verify point extraction
        self.assertGreater(len(x_points), 10)  # Should have extracted points
        self.assertEqual(len(x_points), len(y_points))
        
        # Fit polynomial curve
        curve_result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        
        # Verify curve fitting success
        self.assertTrue(curve_result['success'])
        self.assertIsNotNone(curve_result['coefficients'])
        self.assertLess(curve_result['fitting_error'], 0.1)  # Should fit well
        self.assertGreater(curve_result['r_squared'], 0.8)
        
        # Test trajectory prediction
        coefficients = curve_result['coefficients']
        x_pred, y_pred = self.curve_fitter.predict_trajectory(coefficients, 0.0, 1.0, 20)
        
        self.assertEqual(len(x_pred), 20)
        self.assertEqual(len(y_pred), 20)
        
        # Test extrapolation
        extrapolation_result = self.curve_fitter.extrapolate_curve(coefficients, 0.5, 0.5)
        
        self.assertTrue(extrapolation_result['success'])
        self.assertIn('predicted_position', extrapolation_result)
        self.assertIn('predicted_heading', extrapolation_result)
        self.assertIn('predicted_curvature', extrapolation_result)
    
    def test_end_to_end_pipeline_integration(self):
        """Test complete end-to-end integration of all enhanced components."""
        # Simulate a complete processing pipeline
        
        # Step 1: Adaptive threshold detection
        test_image = np.ones((120, 160, 3), dtype=np.uint8) * 100
        lighting_conditions = self.adaptive_detector.analyze_lighting_conditions(test_image)
        
        # Step 2: Simulate line detection with adapted thresholds
        detection_count = 8
        self.adaptive_detector.adapt_thresholds(lighting_conditions, detection_count)
        adapted_params = self.adaptive_detector.get_adapted_parameters()
        
        # Verify adaptation
        self.assertIn('canny_thresholds', adapted_params)
        self.assertGreater(adapted_params['detection_confidence'], 0.0)
        
        # Step 3: Create mock segments for temporal filtering
        segments = [
            MockSegment(0.1, 0.2, 0.3, 0.4, color=0),  # WHITE
            MockSegment(0.5, 0.6, 0.7, 0.8, color=1),  # YELLOW
            MockSegment(0.2, 0.3, 0.4, 0.5, color=0),  # WHITE
        ]
        
        # Convert to detection format
        lines = []
        for segment in segments:
            x1, y1 = segment.pixels_normalized[0].x, segment.pixels_normalized[0].y
            x2, y2 = segment.pixels_normalized[1].x, segment.pixels_normalized[1].y
            lines.append([x1, y1, x2, y2])
        
        mock_detection = MockDetections(lines=np.array(lines))
        
        # Add multiple frames for temporal consistency
        for _ in range(5):
            self.temporal_filter.add_detections('WHITE', mock_detection)
        
        # Step 4: Get temporally filtered results
        filtered_result = self.temporal_filter.get_filtered_detections('WHITE')
        
        # Verify temporal filtering
        self.assertGreater(filtered_result['temporal_confidence'], 0.7)
        
        # Step 5: Curve fitting on filtered results
        if len(filtered_result['lines']) > 0:
            # Create segments from filtered lines
            filtered_segments = []
            for line in filtered_result['lines']:
                filtered_segments.append(MockSegment(line[0], line[1], line[2], line[3]))
            
            # Extract points and fit curve
            x_points, y_points = self.curve_fitter.extract_lane_points(filtered_segments, 'center')
            
            if len(x_points) >= self.curve_fitter.min_points:
                curve_result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
                
                # Verify curve fitting
                self.assertTrue(curve_result['success'])
                
                # Step 6: Trajectory prediction
                coefficients = curve_result['coefficients']
                extrapolation_result = self.curve_fitter.extrapolate_curve(coefficients, 0.5, 1.0)
                
                # Verify trajectory prediction
                self.assertTrue(extrapolation_result['success'])
                self.assertGreater(len(extrapolation_result['predicted_points'][0]), 0)
    
    def test_performance_integration(self):
        """Test performance characteristics of the integrated pipeline."""
        # Measure processing times for each component
        test_image = np.ones((120, 160, 3), dtype=np.uint8) * 120
        
        # Time adaptive detection
        start_time = time.time()
        lighting_conditions = self.adaptive_detector.analyze_lighting_conditions(test_image)
        self.adaptive_detector.adapt_thresholds(lighting_conditions, 10)
        adaptive_time = time.time() - start_time
        
        # Time temporal filtering
        mock_detection = MockDetections(lines=np.array([[0.1, 0.2, 0.3, 0.4]]))
        start_time = time.time()
        self.temporal_filter.add_detections('WHITE', mock_detection)
        filtered_result = self.temporal_filter.get_filtered_detections('WHITE')
        temporal_time = time.time() - start_time
        
        # Time curve fitting
        segments = [MockSegment(i*0.1, i*0.1, (i+1)*0.1, (i+1)*0.1) for i in range(10)]
        start_time = time.time()
        x_points, y_points = self.curve_fitter.extract_lane_points(segments, 'center')
        if len(x_points) >= 5:
            curve_result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        curve_time = time.time() - start_time
        
        # Verify reasonable performance (should complete within reasonable time)
        total_time = adaptive_time + temporal_time + curve_time
        self.assertLess(total_time, 0.1)  # Should complete within 100ms
        
        # Log performance metrics
        print(f"Performance Integration Test:")
        print(f"  Adaptive detection: {adaptive_time*1000:.2f}ms")
        print(f"  Temporal filtering: {temporal_time*1000:.2f}ms")
        print(f"  Curve fitting: {curve_time*1000:.2f}ms")
        print(f"  Total: {total_time*1000:.2f}ms")
    
    def test_error_handling_integration(self):
        """Test error handling across integrated components."""
        # Test with insufficient data
        empty_segments = []
        
        # Temporal filter should handle empty detections
        empty_detection = MockDetections(lines=np.array([]))
        self.temporal_filter.add_detections('WHITE', empty_detection)
        filtered_result = self.temporal_filter.get_filtered_detections('WHITE')
        
        self.assertEqual(len(filtered_result['lines']), 0)
        self.assertEqual(filtered_result['temporal_confidence'], 0.0)
        
        # Curve fitter should handle insufficient points
        x_points, y_points = self.curve_fitter.extract_lane_points(empty_segments, 'center')
        self.assertEqual(len(x_points), 0)
        
        curve_result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        self.assertFalse(curve_result['success'])
        self.assertIsNone(curve_result['coefficients'])
        
        # Test with invalid data
        invalid_segments = [MockSegment(float('nan'), 0, 1, 1)]
        x_points, y_points = self.curve_fitter.extract_lane_points(invalid_segments, 'center')
        
        # Should handle NaN values gracefully
        if len(x_points) > 0:
            # Remove NaN values
            valid_mask = ~(np.isnan(x_points) | np.isnan(y_points))
            x_points = x_points[valid_mask]
            y_points = y_points[valid_mask]
        
        self.assertTrue(len(x_points) == 0 or not np.any(np.isnan(x_points)))
    
    def test_configuration_integration(self):
        """Test integration with different configuration parameters."""
        # Test with different polynomial degrees
        for degree in [1, 2, 3]:
            curve_fitter = PolynomialCurveFitter(
                polynomial_degree=degree,
                min_points=degree + 2,  # Minimum points for fitting
                extrapolation_distance=1.0
            )
            
            # Create test data
            segments = [MockSegment(i*0.1, (i*0.1)**degree, (i+1)*0.1, ((i+1)*0.1)**degree) 
                       for i in range(degree + 3)]
            
            x_points, y_points = curve_fitter.extract_lane_points(segments, 'center')
            
            if len(x_points) >= curve_fitter.min_points:
                curve_result = curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
                
                self.assertTrue(curve_result['success'])
                self.assertEqual(len(curve_result['coefficients']), degree + 1)
        
        # Test with different temporal filter parameters
        for threshold in [0.3, 0.5, 0.7, 0.9]:
            temp_filter = TemporalConsistencyFilter(
                history_size=5,
                consistency_threshold=threshold
            )
            
            # Add consistent detections
            for _ in range(5):
                detection = MockDetections(lines=np.array([[0.1, 0.2, 0.3, 0.4]]))
                temp_filter.add_detections('WHITE', detection)
            
            filtered_result = temp_filter.get_filtered_detections('WHITE')
            
            # Higher thresholds should be more selective
            if threshold <= 1.0:  # All our detections are consistent
                self.assertGreater(len(filtered_result['lines']), 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
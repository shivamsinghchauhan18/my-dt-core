#!/usr/bin/env python3

import unittest
import numpy as np
import sys
import os

# Add the src directory to the path to import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from lane_filter_node import PolynomialCurveFitter


class MockSegment:
    """Mock segment object for testing."""
    def __init__(self, x1, y1, x2, y2):
        self.pixels_normalized = [MockPoint(x1, y1), MockPoint(x2, y2)]


class MockPoint:
    """Mock point object for testing."""
    def __init__(self, x, y):
        self.x = x
        self.y = y


class TestPolynomialCurveFitter(unittest.TestCase):
    """
    Unit tests for the PolynomialCurveFitter class.
    
    Tests polynomial curve fitting, trajectory prediction, and extrapolation
    capabilities for enhanced lane following.
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.curve_fitter = PolynomialCurveFitter(
            polynomial_degree=2,
            min_points=5,
            extrapolation_distance=1.0,
            smoothing_factor=0.1,
            history_size=10
        )
    
    def test_initialization(self):
        """Test proper initialization of PolynomialCurveFitter."""
        self.assertEqual(self.curve_fitter.polynomial_degree, 2)
        self.assertEqual(self.curve_fitter.min_points, 5)
        self.assertEqual(self.curve_fitter.extrapolation_distance, 1.0)
        self.assertEqual(self.curve_fitter.smoothing_factor, 0.1)
        self.assertEqual(self.curve_fitter.history_size, 10)
        
        # Check that curve history is initialized for all lane types
        expected_lane_types = ['left', 'right', 'center']
        for lane_type in expected_lane_types:
            self.assertIn(lane_type, self.curve_fitter.curve_history)
            self.assertEqual(len(self.curve_fitter.curve_history[lane_type]), 0)
    
    def test_extract_lane_points_sufficient_segments(self):
        """Test lane point extraction with sufficient segments."""
        # Create mock segments forming a line
        segments = [
            MockSegment(0.1, 0.2, 0.2, 0.3),
            MockSegment(0.3, 0.4, 0.4, 0.5),
            MockSegment(0.5, 0.6, 0.6, 0.7)
        ]
        
        x_points, y_points = self.curve_fitter.extract_lane_points(segments, 'center')
        
        # Should have 6 points (2 per segment)
        self.assertEqual(len(x_points), 6)
        self.assertEqual(len(y_points), 6)
        
        # Points should be sorted by x coordinate
        self.assertTrue(np.all(x_points[:-1] <= x_points[1:]))
    
    def test_extract_lane_points_insufficient_segments(self):
        """Test lane point extraction with insufficient segments."""
        # Create only 2 segments (4 points, less than min_points=5)
        segments = [
            MockSegment(0.1, 0.2, 0.2, 0.3),
            MockSegment(0.3, 0.4, 0.4, 0.5)
        ]
        
        x_points, y_points = self.curve_fitter.extract_lane_points(segments, 'center')
        
        # Should return empty arrays
        self.assertEqual(len(x_points), 0)
        self.assertEqual(len(y_points), 0)
    
    def test_fit_polynomial_curve_linear_data(self):
        """Test polynomial curve fitting with linear data."""
        # Create perfect linear data
        x_points = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        y_points = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])  # y = x
        
        result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        
        self.assertTrue(result['success'])
        self.assertIsNotNone(result['coefficients'])
        self.assertLess(result['fitting_error'], 0.01)  # Should be very small for perfect fit
        self.assertGreater(result['r_squared'], 0.99)  # Should be close to 1
        self.assertEqual(len(result['coefficients']), 3)  # Degree 2 polynomial has 3 coefficients
    
    def test_fit_polynomial_curve_quadratic_data(self):
        """Test polynomial curve fitting with quadratic data."""
        # Create perfect quadratic data: y = x^2
        x_points = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        y_points = x_points ** 2
        
        result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        
        self.assertTrue(result['success'])
        self.assertLess(result['fitting_error'], 0.01)
        self.assertGreater(result['r_squared'], 0.99)
        
        # For y = x^2, coefficients should be approximately [1, 0, 0]
        coeffs = result['coefficients']
        self.assertAlmostEqual(coeffs[0], 1.0, places=1)  # x^2 coefficient
        self.assertAlmostEqual(coeffs[1], 0.0, places=1)  # x coefficient
        self.assertAlmostEqual(coeffs[2], 0.0, places=1)  # constant term
    
    def test_fit_polynomial_curve_insufficient_points(self):
        """Test polynomial curve fitting with insufficient points."""
        # Only 3 points (less than min_points=5)
        x_points = np.array([0.0, 0.5, 1.0])
        y_points = np.array([0.0, 0.5, 1.0])
        
        result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        
        self.assertFalse(result['success'])
        self.assertIsNone(result['coefficients'])
        self.assertEqual(result['fitting_error'], float('inf'))
        self.assertEqual(result['r_squared'], 0.0)
    
    def test_fit_polynomial_curve_noisy_data(self):
        """Test polynomial curve fitting with noisy data."""
        # Create noisy quadratic data
        x_points = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        y_points = x_points ** 2 + np.random.normal(0, 0.05, len(x_points))  # Add noise
        
        result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        
        self.assertTrue(result['success'])
        self.assertIsNotNone(result['coefficients'])
        self.assertGreater(result['fitting_error'], 0.0)  # Should have some error due to noise
        self.assertLess(result['fitting_error'], 0.2)  # But not too much
        self.assertGreater(result['r_squared'], 0.8)  # Should still be a good fit
    
    def test_get_smoothed_coefficients_empty_history(self):
        """Test getting smoothed coefficients with empty history."""
        result = self.curve_fitter.get_smoothed_coefficients('center')
        self.assertIsNone(result)
    
    def test_get_smoothed_coefficients_with_history(self):
        """Test getting smoothed coefficients with history."""
        # Add some curve fitting results to history
        x_points = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        y_points = x_points ** 2
        
        # Fit multiple times with slight variations
        for i in range(3):
            y_noisy = y_points + np.random.normal(0, 0.01, len(y_points))
            self.curve_fitter.fit_polynomial_curve(x_points, y_noisy, 'center')
        
        smoothed_coeffs = self.curve_fitter.get_smoothed_coefficients('center')
        
        self.assertIsNotNone(smoothed_coeffs)
        self.assertEqual(len(smoothed_coeffs), 3)
        # Should be close to [1, 0, 0] for y = x^2
        self.assertAlmostEqual(smoothed_coeffs[0], 1.0, places=0)
    
    def test_predict_trajectory(self):
        """Test trajectory prediction using fitted curve."""
        # Use simple linear coefficients: y = x (coefficients [0, 1, 0])
        coefficients = np.array([0.0, 1.0, 0.0])
        
        x_pred, y_pred = self.curve_fitter.predict_trajectory(
            coefficients, x_start=0.0, x_end=1.0, num_points=11
        )
        
        self.assertEqual(len(x_pred), 11)
        self.assertEqual(len(y_pred), 11)
        
        # For y = x, predicted y should equal x
        np.testing.assert_array_almost_equal(x_pred, y_pred, decimal=10)
        
        # Check range
        self.assertAlmostEqual(x_pred[0], 0.0)
        self.assertAlmostEqual(x_pred[-1], 1.0)
    
    def test_predict_trajectory_none_coefficients(self):
        """Test trajectory prediction with None coefficients."""
        x_pred, y_pred = self.curve_fitter.predict_trajectory(
            None, x_start=0.0, x_end=1.0, num_points=10
        )
        
        self.assertEqual(len(x_pred), 0)
        self.assertEqual(len(y_pred), 0)
    
    def test_extrapolate_curve_linear(self):
        """Test curve extrapolation with linear curve."""
        # Linear coefficients: y = x (coefficients [0, 1, 0])
        coefficients = np.array([0.0, 1.0, 0.0])
        current_x = 0.5
        extrapolation_distance = 0.5
        
        result = self.curve_fitter.extrapolate_curve(coefficients, current_x, extrapolation_distance)
        
        self.assertTrue(result['success'])
        
        # For y = x, future point should be (1.0, 1.0)
        expected_x = current_x + extrapolation_distance
        expected_y = expected_x  # Since y = x
        
        self.assertAlmostEqual(result['predicted_position'][0], expected_x)
        self.assertAlmostEqual(result['predicted_position'][1], expected_y)
        
        # For linear curve, heading should be arctan(1) = π/4
        expected_heading = np.arctan(1.0)
        self.assertAlmostEqual(result['predicted_heading'], expected_heading, places=5)
        
        # Linear curve has zero curvature
        self.assertAlmostEqual(result['predicted_curvature'], 0.0, places=5)
    
    def test_extrapolate_curve_quadratic(self):
        """Test curve extrapolation with quadratic curve."""
        # Quadratic coefficients: y = x^2 (coefficients [1, 0, 0])
        coefficients = np.array([1.0, 0.0, 0.0])
        current_x = 0.5
        extrapolation_distance = 0.5
        
        result = self.curve_fitter.extrapolate_curve(coefficients, current_x, extrapolation_distance)
        
        self.assertTrue(result['success'])
        
        # For y = x^2, future point should be (1.0, 1.0)
        expected_x = current_x + extrapolation_distance
        expected_y = expected_x ** 2
        
        self.assertAlmostEqual(result['predicted_position'][0], expected_x)
        self.assertAlmostEqual(result['predicted_position'][1], expected_y)
        
        # For y = x^2, derivative at x=1 is 2x = 2
        expected_heading = np.arctan(2.0)
        self.assertAlmostEqual(result['predicted_heading'], expected_heading, places=5)
        
        # Curvature should be non-zero for quadratic
        self.assertGreater(result['predicted_curvature'], 0.0)
    
    def test_extrapolate_curve_none_coefficients(self):
        """Test curve extrapolation with None coefficients."""
        result = self.curve_fitter.extrapolate_curve(None, 0.5, 0.5)
        
        self.assertFalse(result['success'])
        self.assertEqual(len(result['predicted_points'][0]), 0)
        self.assertEqual(len(result['predicted_points'][1]), 0)
    
    def test_get_curve_metrics_empty(self):
        """Test getting curve metrics with no history."""
        metrics = self.curve_fitter.get_curve_metrics()
        
        self.assertIn('fitting_errors', metrics)
        self.assertIn('prediction_accuracy', metrics)
        self.assertIn('history_sizes', metrics)
        self.assertIn('last_coefficients', metrics)
        
        # All should be empty
        self.assertEqual(len(metrics['fitting_errors']), 0)
        self.assertEqual(len(metrics['prediction_accuracy']), 0)
        self.assertEqual(len(metrics['last_coefficients']), 0)
        
        # History sizes should be 0
        for lane_type in ['left', 'right', 'center']:
            self.assertEqual(metrics['history_sizes'][lane_type], 0)
    
    def test_get_curve_metrics_with_data(self):
        """Test getting curve metrics with fitting history."""
        # Add some curve fitting results
        x_points = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        y_points = x_points ** 2
        
        # Fit curves for different lane types
        for lane_type in ['left', 'right', 'center']:
            self.curve_fitter.fit_polynomial_curve(x_points, y_points, lane_type)
        
        metrics = self.curve_fitter.get_curve_metrics()
        
        # Should have data for all lane types
        self.assertEqual(len(metrics['fitting_errors']), 3)
        self.assertEqual(len(metrics['last_coefficients']), 3)
        
        # Should have average metrics
        for lane_type in ['left', 'right', 'center']:
            self.assertIn(f'{lane_type}_avg_error', metrics)
            self.assertIn(f'{lane_type}_avg_r_squared', metrics)
            self.assertIn(f'{lane_type}_avg_curvature', metrics)
            
            # Values should be reasonable
            self.assertGreaterEqual(metrics[f'{lane_type}_avg_error'], 0.0)
            self.assertGreaterEqual(metrics[f'{lane_type}_avg_r_squared'], 0.0)
            self.assertLessEqual(metrics[f'{lane_type}_avg_r_squared'], 1.0)
            self.assertGreaterEqual(metrics[f'{lane_type}_avg_curvature'], 0.0)
    
    def test_history_size_limit(self):
        """Test that curve history is properly limited."""
        x_points = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        y_points = x_points ** 2
        
        # Add more fits than history_size (10)
        for i in range(15):
            y_noisy = y_points + np.random.normal(0, 0.01, len(y_points))
            self.curve_fitter.fit_polynomial_curve(x_points, y_noisy, 'center')
        
        # History should be limited to history_size
        self.assertEqual(len(self.curve_fitter.curve_history['center']), 10)
        
        # Should contain the most recent fits
        timestamps = [h['timestamp'] for h in self.curve_fitter.curve_history['center']]
        self.assertEqual(timestamps, sorted(timestamps))  # Should be in chronological order
    
    def test_different_polynomial_degrees(self):
        """Test curve fitting with different polynomial degrees."""
        # Test with degree 1 (linear)
        linear_fitter = PolynomialCurveFitter(polynomial_degree=1, min_points=3)
        
        x_points = np.array([0.0, 0.5, 1.0])
        y_points = np.array([0.0, 0.5, 1.0])  # Linear data
        
        result = linear_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        
        self.assertTrue(result['success'])
        self.assertEqual(len(result['coefficients']), 2)  # Linear has 2 coefficients
        self.assertLess(result['fitting_error'], 0.01)
        
        # Test with degree 3 (cubic)
        cubic_fitter = PolynomialCurveFitter(polynomial_degree=3, min_points=5)
        
        x_points = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
        y_points = x_points ** 3  # Cubic data
        
        result = cubic_fitter.fit_polynomial_curve(x_points, y_points, 'center')
        
        self.assertTrue(result['success'])
        self.assertEqual(len(result['coefficients']), 4)  # Cubic has 4 coefficients
        self.assertLess(result['fitting_error'], 0.01)


if __name__ == '__main__':
    unittest.main()
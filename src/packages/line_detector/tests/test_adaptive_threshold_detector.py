#!/usr/bin/env python3

import unittest
import numpy as np
import cv2
import sys
import os

# Add the src directory to the path to import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from line_detector_node import AdaptiveThresholdDetector


class TestAdaptiveThresholdDetector(unittest.TestCase):
    """
    Unit tests for the AdaptiveThresholdDetector class.
    
    Tests adaptive threshold adjustment based on lighting conditions,
    parameter adaptation, and detection confidence tracking.
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.detector = AdaptiveThresholdDetector(
            base_canny_low=80,
            base_canny_high=200,
            adaptation_rate=0.2
        )
    
    def test_initialization(self):
        """Test proper initialization of AdaptiveThresholdDetector."""
        self.assertEqual(self.detector.base_canny_low, 80)
        self.assertEqual(self.detector.base_canny_high, 200)
        self.assertEqual(self.detector.adaptation_rate, 0.2)
        self.assertEqual(self.detector.current_canny_low, 80)
        self.assertEqual(self.detector.current_canny_high, 200)
        self.assertEqual(len(self.detector.brightness_history), 0)
        self.assertEqual(len(self.detector.contrast_history), 0)
    
    def test_lighting_analysis_bright_image(self):
        """Test lighting condition analysis for bright images."""
        # Create a bright test image
        bright_image = np.ones((120, 160, 3), dtype=np.uint8) * 220
        
        conditions = self.detector.analyze_lighting_conditions(bright_image)
        
        self.assertIn('brightness', conditions)
        self.assertIn('contrast', conditions)
        self.assertIn('entropy', conditions)
        self.assertGreater(conditions['brightness'], 200)
        self.assertLess(conditions['contrast'], 50)  # Low contrast for uniform image
        self.assertGreater(conditions['processing_time'], 0)
    
    def test_lighting_analysis_dark_image(self):
        """Test lighting condition analysis for dark images."""
        # Create a dark test image
        dark_image = np.ones((120, 160, 3), dtype=np.uint8) * 30
        
        conditions = self.detector.analyze_lighting_conditions(dark_image)
        
        self.assertLess(conditions['brightness'], 50)
        self.assertLess(conditions['contrast'], 50)  # Low contrast for uniform image
        self.assertGreater(conditions['processing_time'], 0)
    
    def test_lighting_analysis_high_contrast_image(self):
        """Test lighting condition analysis for high contrast images."""
        # Create a high contrast test image (checkerboard pattern)
        high_contrast_image = np.zeros((120, 160, 3), dtype=np.uint8)
        high_contrast_image[::2, ::2] = 255  # White squares
        high_contrast_image[1::2, 1::2] = 255  # White squares
        
        conditions = self.detector.analyze_lighting_conditions(high_contrast_image)
        
        self.assertGreater(conditions['contrast'], 100)  # High contrast
        self.assertGreater(conditions['entropy'], 0.5)  # High information content
    
    def test_threshold_adaptation_dark_conditions(self):
        """Test threshold adaptation for dark lighting conditions."""
        # Simulate dark lighting conditions
        dark_conditions = {
            'brightness': 50,
            'contrast': 30,
            'entropy': 2.0,
            'brightness_stability': 5.0,
            'contrast_stability': 3.0,
            'processing_time': 0.001
        }
        
        initial_low = self.detector.current_canny_low
        initial_high = self.detector.current_canny_high
        
        self.detector.adapt_thresholds(dark_conditions, 5)
        
        # Thresholds should be lowered for dark conditions
        self.assertLess(self.detector.current_canny_low, initial_low)
        self.assertLess(self.detector.current_canny_high, initial_high)
        self.assertGreater(self.detector.detection_confidence, 0)
    
    def test_threshold_adaptation_bright_conditions(self):
        """Test threshold adaptation for bright lighting conditions."""
        # Simulate bright lighting conditions
        bright_conditions = {
            'brightness': 200,
            'contrast': 40,
            'entropy': 3.0,
            'brightness_stability': 2.0,
            'contrast_stability': 1.0,
            'processing_time': 0.001
        }
        
        initial_low = self.detector.current_canny_low
        initial_high = self.detector.current_canny_high
        
        self.detector.adapt_thresholds(bright_conditions, 8)
        
        # Thresholds should be raised for bright conditions
        self.assertGreater(self.detector.current_canny_low, initial_low)
        self.assertGreater(self.detector.current_canny_high, initial_high)
        self.assertGreater(self.detector.detection_confidence, 0)
    
    def test_threshold_adaptation_normal_conditions(self):
        """Test threshold adaptation for normal lighting conditions."""
        # Simulate normal lighting conditions
        normal_conditions = {
            'brightness': 120,
            'contrast': 50,
            'entropy': 4.0,
            'brightness_stability': 3.0,
            'contrast_stability': 2.0,
            'processing_time': 0.001
        }
        
        self.detector.adapt_thresholds(normal_conditions, 10)
        
        # Thresholds should be close to base values for normal conditions
        self.assertAlmostEqual(self.detector.current_canny_low, 80, delta=20)
        self.assertAlmostEqual(self.detector.current_canny_high, 200, delta=50)
    
    def test_detection_confidence_tracking(self):
        """Test detection confidence tracking based on detection count."""
        conditions = {
            'brightness': 120,
            'contrast': 50,
            'entropy': 4.0,
            'brightness_stability': 3.0,
            'contrast_stability': 2.0,
            'processing_time': 0.001
        }
        
        # Test with good detection count
        self.detector.adapt_thresholds(conditions, 15)
        confidence_with_detections = self.detector.detection_confidence
        
        # Test with no detections
        self.detector.adapt_thresholds(conditions, 0)
        confidence_without_detections = self.detector.detection_confidence
        
        # Confidence should decrease when no detections
        self.assertLess(confidence_without_detections, confidence_with_detections)
    
    def test_get_adapted_parameters(self):
        """Test retrieval of adapted parameters."""
        # Add some history
        test_image = np.ones((120, 160, 3), dtype=np.uint8) * 100
        self.detector.analyze_lighting_conditions(test_image)
        
        params = self.detector.get_adapted_parameters()
        
        self.assertIn('canny_thresholds', params)
        self.assertIn('detection_confidence', params)
        self.assertIn('brightness_avg', params)
        self.assertIn('contrast_avg', params)
        
        self.assertEqual(len(params['canny_thresholds']), 2)
        self.assertIsInstance(params['canny_thresholds'][0], int)
        self.assertIsInstance(params['canny_thresholds'][1], int)
        self.assertGreaterEqual(params['detection_confidence'], 0.0)
        self.assertLessEqual(params['detection_confidence'], 1.0)
    
    def test_brightness_history_management(self):
        """Test brightness history management with maximum size."""
        # Add more than the maximum history size
        for i in range(15):
            test_image = np.ones((120, 160, 3), dtype=np.uint8) * (50 + i * 10)
            self.detector.analyze_lighting_conditions(test_image)
        
        # History should be limited to maximum size (10)
        self.assertEqual(len(self.detector.brightness_history), 10)
        self.assertEqual(len(self.detector.contrast_history), 10)
        
        # Should contain the most recent values
        self.assertGreater(self.detector.brightness_history[-1], 
                          self.detector.brightness_history[0])
    
    def test_adaptation_rate_effect(self):
        """Test the effect of adaptation rate on threshold changes."""
        # Create detector with high adaptation rate
        fast_detector = AdaptiveThresholdDetector(adaptation_rate=0.8)
        
        # Create detector with low adaptation rate  
        slow_detector = AdaptiveThresholdDetector(adaptation_rate=0.1)
        
        dark_conditions = {
            'brightness': 40,
            'contrast': 25,
            'entropy': 2.0,
            'brightness_stability': 5.0,
            'contrast_stability': 3.0,
            'processing_time': 0.001
        }
        
        initial_low = 80
        
        fast_detector.adapt_thresholds(dark_conditions, 5)
        slow_detector.adapt_thresholds(dark_conditions, 5)
        
        # Fast detector should adapt more quickly
        fast_change = abs(fast_detector.current_canny_low - initial_low)
        slow_change = abs(slow_detector.current_canny_low - initial_low)
        
        self.assertGreater(fast_change, slow_change)


if __name__ == '__main__':
    unittest.main()
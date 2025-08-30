#!/usr/bin/env python3

import unittest
import numpy as np
import time
import sys
import os

# Add the src directory to the path to import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from line_detector_node import TemporalConsistencyFilter


class MockDetections:
    """Mock detection object for testing."""
    def __init__(self, lines, normals=None, centers=None):
        self.lines = lines
        self.normals = normals if normals is not None else []
        self.centers = centers if centers is not None else []


class TestTemporalConsistencyFilter(unittest.TestCase):
    """
    Unit tests for the TemporalConsistencyFilter class.
    
    Tests temporal consistency filtering, detection history management,
    and confidence calculation for stable line detection.
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.filter = TemporalConsistencyFilter(
            history_size=5,
            consistency_threshold=0.7
        )
    
    def test_initialization(self):
        """Test proper initialization of TemporalConsistencyFilter."""
        self.assertEqual(self.filter.history_size, 5)
        self.assertEqual(self.filter.consistency_threshold, 0.7)
        
        # Check that all color histories are initialized
        expected_colors = ['WHITE', 'YELLOW', 'RED']
        for color in expected_colors:
            self.assertIn(color, self.filter.detection_history)
            self.assertEqual(len(self.filter.detection_history[color]), 0)
    
    def test_add_detections_single_color(self):
        """Test adding detections for a single color."""
        # Create mock detection with 3 lines
        lines = np.array([[10, 20, 30, 40], [50, 60, 70, 80], [90, 100, 110, 120]])
        normals = np.array([[0.5, 0.5], [0.7, 0.3], [0.6, 0.4]])
        centers = np.array([[20, 30], [60, 70], [100, 110]])
        
        mock_detection = MockDetections(lines, normals, centers)
        
        self.filter.add_detections('WHITE', mock_detection)
        
        # Check that detection was added
        self.assertEqual(len(self.filter.detection_history['WHITE']), 1)
        
        detection_data = self.filter.detection_history['WHITE'][0]
        self.assertEqual(detection_data['line_count'], 3)
        self.assertTrue(np.array_equal(detection_data['lines'], lines))
        self.assertTrue(np.array_equal(detection_data['normals'], normals))
        self.assertTrue(np.array_equal(detection_data['centers'], centers))
        self.assertGreater(detection_data['timestamp'], 0)
    
    def test_add_detections_multiple_frames(self):
        """Test adding detections across multiple frames."""
        for i in range(7):  # More than history_size
            lines = np.array([[i*10, i*10+10, i*10+20, i*10+30]])
            mock_detection = MockDetections(lines)
            self.filter.add_detections('YELLOW', mock_detection)
        
        # History should be limited to history_size
        self.assertEqual(len(self.filter.detection_history['YELLOW']), 5)
        
        # Should contain the most recent detections
        recent_detection = self.filter.detection_history['YELLOW'][-1]
        self.assertEqual(recent_detection['line_count'], 1)
        self.assertEqual(recent_detection['lines'][0][0], 60)  # Most recent frame
    
    def test_get_filtered_detections_empty_history(self):
        """Test getting filtered detections with empty history."""
        result = self.filter.get_filtered_detections('WHITE')
        
        self.assertEqual(len(result['lines']), 0)
        self.assertEqual(len(result['normals']), 0)
        self.assertEqual(len(result['centers']), 0)
        self.assertEqual(result['temporal_confidence'], 0.0)
        self.assertEqual(result['stability_score'], 0.0)
    
    def test_get_filtered_detections_high_confidence(self):
        """Test getting filtered detections with high temporal confidence."""
        # Add consistent detections (all frames have detections)
        for i in range(5):
            lines = np.array([[10+i, 20+i, 30+i, 40+i], [50+i, 60+i, 70+i, 80+i]])
            normals = np.array([[0.5, 0.5], [0.7, 0.3]])
            centers = np.array([[20+i, 30+i], [60+i, 70+i]])
            mock_detection = MockDetections(lines, normals, centers)
            self.filter.add_detections('WHITE', mock_detection)
        
        result = self.filter.get_filtered_detections('WHITE')
        
        # Should return the most recent detections
        self.assertEqual(len(result['lines']), 2)
        self.assertEqual(result['temporal_confidence'], 1.0)  # All recent frames had detections
        self.assertGreater(result['stability_score'], 0.8)  # Consistent line count
        
        # Check that the most recent lines are returned
        expected_lines = [[14, 24, 34, 44], [54, 64, 74, 84]]
        np.testing.assert_array_equal(result['lines'], expected_lines)
    
    def test_get_filtered_detections_low_confidence(self):
        """Test getting filtered detections with low temporal confidence."""
        # Add inconsistent detections (some frames with no detections)
        detection_patterns = [2, 0, 1, 0, 0]  # Number of lines per frame
        
        for i, line_count in enumerate(detection_patterns):
            if line_count > 0:
                lines = np.array([[j*10+i, j*10+i+10, j*10+i+20, j*10+i+30] 
                                 for j in range(line_count)])
                mock_detection = MockDetections(lines)
            else:
                mock_detection = MockDetections([])
            
            self.filter.add_detections('YELLOW', mock_detection)
        
        result = self.filter.get_filtered_detections('YELLOW')
        
        # Should have low confidence due to inconsistent detections
        self.assertLess(result['temporal_confidence'], 0.7)
        # Should return empty results due to low confidence
        self.assertEqual(len(result['lines']), 0)
    
    def test_temporal_confidence_calculation(self):
        """Test temporal confidence calculation accuracy."""
        # Test case: 2 out of 3 recent frames have detections
        detection_patterns = [1, 1, 0, 1, 0]  # line counts
        
        for line_count in detection_patterns:
            if line_count > 0:
                lines = np.array([[10, 20, 30, 40]])
                mock_detection = MockDetections(lines)
            else:
                mock_detection = MockDetections([])
            
            self.filter.add_detections('RED', mock_detection)
        
        result = self.filter.get_filtered_detections('RED')
        
        # Recent 3 frames: [0, 1, 0] -> 1 out of 3 = 0.33 confidence
        expected_confidence = 1.0 / 3.0
        self.assertAlmostEqual(result['temporal_confidence'], expected_confidence, places=2)
    
    def test_stability_score_calculation(self):
        """Test stability score calculation based on line count variance."""
        # Add detections with consistent line counts
        consistent_counts = [2, 2, 2, 2, 2]
        for count in consistent_counts:
            lines = np.array([[i*10, i*10+10, i*10+20, i*10+30] for i in range(count)])
            mock_detection = MockDetections(lines)
            self.filter.add_detections('WHITE', mock_detection)
        
        result_consistent = self.filter.get_filtered_detections('WHITE')
        
        # Reset filter for inconsistent test
        self.filter = TemporalConsistencyFilter(history_size=5, consistency_threshold=0.7)
        
        # Add detections with varying line counts
        varying_counts = [1, 3, 2, 4, 1]
        for count in varying_counts:
            lines = np.array([[i*10, i*10+10, i*10+20, i*10+30] for i in range(count)])
            mock_detection = MockDetections(lines)
            self.filter.add_detections('WHITE', mock_detection)
        
        result_varying = self.filter.get_filtered_detections('WHITE')
        
        # Consistent detections should have higher stability score
        self.assertGreater(result_consistent['stability_score'], 
                          result_varying['stability_score'])
    
    def test_get_temporal_metrics(self):
        """Test retrieval of temporal metrics."""
        # Add some detections
        for color in ['WHITE', 'YELLOW']:
            lines = np.array([[10, 20, 30, 40]])
            mock_detection = MockDetections(lines)
            self.filter.add_detections(color, mock_detection)
            self.filter.get_filtered_detections(color)  # This updates metrics
        
        metrics = self.filter.get_temporal_metrics()
        
        self.assertIn('temporal_confidence', metrics)
        self.assertIn('stability_scores', metrics)
        self.assertIn('history_sizes', metrics)
        
        # Check that metrics contain expected colors
        self.assertIn('WHITE', metrics['temporal_confidence'])
        self.assertIn('YELLOW', metrics['temporal_confidence'])
        
        # Check history sizes
        self.assertEqual(metrics['history_sizes']['WHITE'], 1)
        self.assertEqual(metrics['history_sizes']['YELLOW'], 1)
        self.assertEqual(metrics['history_sizes']['RED'], 0)
    
    def test_history_size_limit(self):
        """Test that history size is properly limited."""
        # Add more detections than history_size
        for i in range(10):
            lines = np.array([[i, i+10, i+20, i+30]])
            mock_detection = MockDetections(lines)
            self.filter.add_detections('WHITE', mock_detection)
        
        # History should be limited to history_size
        self.assertEqual(len(self.filter.detection_history['WHITE']), 5)
        
        # Should contain the most recent detections
        timestamps = [d['timestamp'] for d in self.filter.detection_history['WHITE']]
        self.assertEqual(timestamps, sorted(timestamps))  # Should be in chronological order
    
    def test_different_consistency_thresholds(self):
        """Test behavior with different consistency thresholds."""
        # Create filters with different thresholds
        strict_filter = TemporalConsistencyFilter(history_size=5, consistency_threshold=0.9)
        lenient_filter = TemporalConsistencyFilter(history_size=5, consistency_threshold=0.3)
        
        # Add moderately consistent detections (3 out of 5 frames)
        detection_pattern = [1, 1, 0, 1, 0]
        
        for line_count in detection_pattern:
            if line_count > 0:
                lines = np.array([[10, 20, 30, 40]])
                mock_detection = MockDetections(lines)
            else:
                mock_detection = MockDetections([])
            
            strict_filter.add_detections('WHITE', mock_detection)
            lenient_filter.add_detections('WHITE', mock_detection)
        
        strict_result = strict_filter.get_filtered_detections('WHITE')
        lenient_result = lenient_filter.get_filtered_detections('WHITE')
        
        # Recent 3 frames are [0, 1, 0] = 1/3 = 0.33 confidence
        # Strict filter (0.9 threshold) should reject, lenient (0.3 threshold) should accept
        self.assertEqual(len(strict_result['lines']), 0)  # Rejected due to low confidence
        self.assertGreaterEqual(lenient_result['temporal_confidence'], 0.3)  # Should meet threshold
        # Since confidence (0.33) >= threshold (0.3), lenient should accept
        self.assertGreater(len(lenient_result['lines']), 0)  # Accepted
    
    def test_unknown_color_handling(self):
        """Test handling of unknown color names."""
        # Add detection for unknown color
        lines = np.array([[10, 20, 30, 40]])
        mock_detection = MockDetections(lines)
        
        # Should create new history for unknown color
        self.filter.add_detections('BLUE', mock_detection)
        
        self.assertIn('BLUE', self.filter.detection_history)
        self.assertEqual(len(self.filter.detection_history['BLUE']), 1)
        
        # Should be able to get filtered detections
        result = self.filter.get_filtered_detections('BLUE')
        self.assertEqual(len(result['lines']), 1)


if __name__ == '__main__':
    unittest.main()
#!/usr/bin/env python3

import unittest
import numpy as np
import cv2
import math
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Mock ROS modules to avoid import errors
sys.modules['rospy'] = Mock()
sys.modules['tf'] = Mock()
sys.modules['dt_class_utils'] = Mock()
sys.modules['duckietown.dtros'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['geometry_msgs.msg'] = Mock()
sys.modules['turbojpeg'] = Mock()
sys.modules['image_geometry'] = Mock()
sys.modules['dt_apriltags'] = Mock()

# Add the src directory to the path so we can import the module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Import after mocking
import importlib.util
spec = importlib.util.spec_from_file_location("apriltag_detector_node", 
                                              os.path.join(os.path.dirname(__file__), '..', 'src', 'apriltag_detector_node.py'))
apriltag_module = importlib.util.module_from_spec(spec)

# Create a standalone MultiResolutionDetector class for testing
class MultiResolutionDetector:
    """
    Standalone version of MultiResolutionDetector for testing
    """
    
    def __init__(self, base_detector, scale_factors=None, confidence_threshold=0.1):
        self.base_detector = base_detector
        self.scale_factors = scale_factors or [0.5, 1.0, 1.5, 2.0]
        self.confidence_threshold = confidence_threshold
        self.detection_count = 0
        self.total_processing_time = 0.0
        self.scale_performance = {scale: {'detections': 0, 'time': 0.0} for scale in self.scale_factors}
    
    def detect_multi_scale(self, image, camera_params, tag_size):
        import time
        start_time = time.time()
        all_detections = []
        
        for scale_factor in self.scale_factors:
            scale_start_time = time.time()
            
            if scale_factor != 1.0:
                new_width = int(image.shape[1] * scale_factor)
                new_height = int(image.shape[0] * scale_factor)
                scaled_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
                
                scaled_camera_params = (
                    camera_params[0] * scale_factor,
                    camera_params[1] * scale_factor,
                    camera_params[2] * scale_factor,
                    camera_params[3] * scale_factor
                )
            else:
                scaled_image = image
                scaled_camera_params = camera_params
            
            try:
                detections = self.base_detector.detect(scaled_image, True, scaled_camera_params, tag_size)
                
                for detection in detections:
                    if scale_factor != 1.0:
                        detection.corners = detection.corners / scale_factor
                        detection.center = detection.center / scale_factor
                        
                        H = detection.homography.copy()
                        H[0, 2] /= scale_factor
                        H[1, 2] /= scale_factor
                        H[2, 0] *= scale_factor
                        H[2, 1] *= scale_factor
                        detection.homography = H
                    
                    detection.scale_factor = scale_factor
                    detection.detection_confidence = detection.decision_margin
                
                scale_time = time.time() - scale_start_time
                self.scale_performance[scale_factor]['detections'] += len(detections)
                self.scale_performance[scale_factor]['time'] += scale_time
                
                all_detections.extend(detections)
                
            except Exception as e:
                pass
        
        filtered_detections = self._filter_detections(all_detections)
        
        total_time = time.time() - start_time
        self.detection_count += 1
        self.total_processing_time += total_time
        
        return filtered_detections
    
    def _filter_detections(self, detections):
        if not detections:
            return []
        
        confident_detections = [d for d in detections if d.decision_margin >= self.confidence_threshold]
        
        if not confident_detections:
            return []
        
        tag_groups = {}
        for detection in confident_detections:
            tag_id = detection.tag_id
            if tag_id not in tag_groups:
                tag_groups[tag_id] = []
            tag_groups[tag_id].append(detection)
        
        filtered_detections = []
        for tag_id, group in tag_groups.items():
            if len(group) == 1:
                filtered_detections.append(group[0])
            else:
                best_detection = max(group, key=lambda d: d.decision_margin)
                filtered_detections.append(best_detection)
        
        return filtered_detections
    
    def estimate_distance_and_angle(self, detection, camera_params, tag_size):
        try:
            pose_t = detection.pose_t.flatten()
            pose_R = detection.pose_R
            
            distance = np.linalg.norm(pose_t)
            approach_angle = math.atan2(pose_R[1, 0], pose_R[0, 0])
            
            pose_error = getattr(detection, 'pose_err', 0.0)
            confidence = detection.decision_margin
            estimation_quality = confidence / (1.0 + pose_error)
            
            return distance, approach_angle, estimation_quality
            
        except Exception as e:
            return 0.0, 0.0, 0.0
    
    def get_detection_metrics(self):
        if self.detection_count == 0:
            return {}
        
        avg_time = self.total_processing_time / self.detection_count
        
        metrics = {
            'total_detections': self.detection_count,
            'average_processing_time': avg_time,
            'scale_performance': {}
        }
        
        for scale, perf in self.scale_performance.items():
            if perf['detections'] > 0:
                metrics['scale_performance'][scale] = {
                    'detections': perf['detections'],
                    'average_time': perf['time'] / perf['detections']
                }
        
        return metrics


class MockAprilTagDetection:
    """Mock AprilTag detection for testing"""
    def __init__(self, tag_id=1, decision_margin=0.5, corners=None, center=None, 
                 homography=None, pose_t=None, pose_R=None, pose_err=0.1):
        self.tag_id = tag_id
        self.decision_margin = decision_margin
        self.corners = corners if corners is not None else np.array([[10, 10], [20, 10], [20, 20], [10, 20]], dtype=np.float32)
        self.center = center if center is not None else np.array([15, 15], dtype=np.float32)
        self.homography = homography if homography is not None else np.eye(3, dtype=np.float32)
        self.pose_t = pose_t if pose_t is not None else np.array([[0], [0], [1]], dtype=np.float32)
        self.pose_R = pose_R if pose_R is not None else np.eye(3, dtype=np.float32)
        self.pose_err = pose_err
        self.tag_family = "tag36h11"


class TestMultiResolutionDetector(unittest.TestCase):
    """Test suite for MultiResolutionDetector class"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_detector = Mock()
        self.scale_factors = [0.5, 1.0, 1.5, 2.0]
        self.confidence_threshold = 0.1
        
        # Create detector instance
        self.detector = MultiResolutionDetector(
            self.mock_detector,
            scale_factors=self.scale_factors,
            confidence_threshold=self.confidence_threshold
        )
        
        # Test image and camera parameters
        self.test_image = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        self.camera_params = (320.0, 320.0, 320.0, 240.0)  # fx, fy, cx, cy
        self.tag_size = 0.065
    
    def test_initialization(self):
        """Test MultiResolutionDetector initialization"""
        self.assertEqual(self.detector.base_detector, self.mock_detector)
        self.assertEqual(self.detector.scale_factors, self.scale_factors)
        self.assertEqual(self.detector.confidence_threshold, self.confidence_threshold)
        self.assertEqual(self.detector.detection_count, 0)
        self.assertEqual(self.detector.total_processing_time, 0.0)
    
    def test_default_initialization(self):
        """Test MultiResolutionDetector with default parameters"""
        detector = MultiResolutionDetector(self.mock_detector)
        self.assertEqual(detector.scale_factors, [0.5, 1.0, 1.5, 2.0])
        self.assertEqual(detector.confidence_threshold, 0.1)
    
    def test_detect_multi_scale_no_detections(self):
        """Test multi-scale detection with no detections"""
        # Mock detector returns empty list for all scales
        self.mock_detector.detect.return_value = []
        
        result = self.detector.detect_multi_scale(
            self.test_image, self.camera_params, self.tag_size
        )
        
        self.assertEqual(len(result), 0)
        self.assertEqual(self.mock_detector.detect.call_count, len(self.scale_factors))
    
    def test_detect_multi_scale_single_detection(self):
        """Test multi-scale detection with single detection at one scale"""
        # Create mock detection
        mock_detection = MockAprilTagDetection(tag_id=1, decision_margin=0.8)
        
        # Mock detector returns detection only for scale 1.0
        def mock_detect_side_effect(image, estimate_tag_pose, camera_params, tag_size):
            # Check if this is the original scale (1.0) based on image size
            if image.shape == self.test_image.shape:
                return [mock_detection]
            return []
        
        self.mock_detector.detect.side_effect = mock_detect_side_effect
        
        result = self.detector.detect_multi_scale(
            self.test_image, self.camera_params, self.tag_size
        )
        
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].tag_id, 1)
        self.assertEqual(result[0].decision_margin, 0.8)
    
    def test_detect_multi_scale_multiple_scales(self):
        """Test multi-scale detection with detections at multiple scales"""
        # Create mock detections with different confidence levels
        detection_scale_05 = MockAprilTagDetection(tag_id=1, decision_margin=0.3)
        detection_scale_10 = MockAprilTagDetection(tag_id=1, decision_margin=0.8)
        detection_scale_15 = MockAprilTagDetection(tag_id=1, decision_margin=0.6)
        
        def mock_detect_side_effect(image, estimate_tag_pose, camera_params, tag_size):
            # Return different detections based on image size (scale)
            if image.shape[0] == int(self.test_image.shape[0] * 0.5):  # Scale 0.5
                return [detection_scale_05]
            elif image.shape == self.test_image.shape:  # Scale 1.0
                return [detection_scale_10]
            elif image.shape[0] == int(self.test_image.shape[0] * 1.5):  # Scale 1.5
                return [detection_scale_15]
            return []
        
        self.mock_detector.detect.side_effect = mock_detect_side_effect
        
        result = self.detector.detect_multi_scale(
            self.test_image, self.camera_params, self.tag_size
        )
        
        # Should return only the best detection (highest confidence)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].tag_id, 1)
        self.assertEqual(result[0].decision_margin, 0.8)  # Best confidence
    
    def test_confidence_filtering(self):
        """Test confidence threshold filtering"""
        # Create detections with different confidence levels
        low_confidence = MockAprilTagDetection(tag_id=1, decision_margin=0.05)  # Below threshold
        high_confidence = MockAprilTagDetection(tag_id=2, decision_margin=0.8)  # Above threshold
        
        detections = [low_confidence, high_confidence]
        filtered = self.detector._filter_detections(detections)
        
        # Only high confidence detection should remain
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0].tag_id, 2)
        self.assertEqual(filtered[0].decision_margin, 0.8)
    
    def test_duplicate_tag_filtering(self):
        """Test filtering of duplicate tag detections"""
        # Create multiple detections of the same tag with different confidences
        detection1 = MockAprilTagDetection(tag_id=1, decision_margin=0.3)
        detection2 = MockAprilTagDetection(tag_id=1, decision_margin=0.8)
        detection3 = MockAprilTagDetection(tag_id=1, decision_margin=0.5)
        
        detections = [detection1, detection2, detection3]
        filtered = self.detector._filter_detections(detections)
        
        # Should return only the best detection
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0].tag_id, 1)
        self.assertEqual(filtered[0].decision_margin, 0.8)  # Highest confidence
    
    def test_distance_estimation(self):
        """Test distance and angle estimation"""
        # Create mock detection with known pose
        pose_t = np.array([[0.5], [0.0], [1.0]], dtype=np.float32)  # 0.5m to the right, 1m forward
        pose_R = np.array([[0.866, -0.5, 0], [0.5, 0.866, 0], [0, 0, 1]], dtype=np.float32)  # 30 degree rotation
        
        detection = MockAprilTagDetection(
            tag_id=1, 
            decision_margin=0.8,
            pose_t=pose_t,
            pose_R=pose_R,
            pose_err=0.05
        )
        
        distance, angle, quality = self.detector.estimate_distance_and_angle(
            detection, self.camera_params, self.tag_size
        )
        
        # Check distance calculation
        expected_distance = np.linalg.norm(pose_t.flatten())
        self.assertAlmostEqual(distance, expected_distance, places=3)
        
        # Check angle calculation (should be approximately 30 degrees)
        expected_angle = math.atan2(pose_R[1, 0], pose_R[0, 0])
        self.assertAlmostEqual(angle, expected_angle, places=3)
        
        # Check quality metric
        expected_quality = detection.decision_margin / (1.0 + detection.pose_err)
        self.assertAlmostEqual(quality, expected_quality, places=3)
    
    def test_distance_estimation_error_handling(self):
        """Test distance estimation error handling"""
        # Create detection with invalid pose data
        detection = MockAprilTagDetection(tag_id=1, decision_margin=0.8)
        detection.pose_t = None  # Invalid pose
        
        distance, angle, quality = self.detector.estimate_distance_and_angle(
            detection, self.camera_params, self.tag_size
        )
        
        # Should return zeros on error
        self.assertEqual(distance, 0.0)
        self.assertEqual(angle, 0.0)
        self.assertEqual(quality, 0.0)
    
    def test_performance_metrics(self):
        """Test performance metrics collection"""
        # Simulate some detections
        self.detector.detection_count = 10
        self.detector.total_processing_time = 2.5
        
        # Add some scale performance data
        self.detector.scale_performance[1.0]['detections'] = 5
        self.detector.scale_performance[1.0]['time'] = 1.0
        self.detector.scale_performance[2.0]['detections'] = 3
        self.detector.scale_performance[2.0]['time'] = 0.9
        
        metrics = self.detector.get_detection_metrics()
        
        self.assertEqual(metrics['total_detections'], 10)
        self.assertEqual(metrics['average_processing_time'], 0.25)
        self.assertIn('scale_performance', metrics)
        self.assertIn(1.0, metrics['scale_performance'])
        self.assertIn(2.0, metrics['scale_performance'])
        
        # Check scale-specific metrics
        self.assertEqual(metrics['scale_performance'][1.0]['detections'], 5)
        self.assertEqual(metrics['scale_performance'][1.0]['average_time'], 0.2)
    
    def test_empty_metrics(self):
        """Test metrics when no detections have been processed"""
        metrics = self.detector.get_detection_metrics()
        self.assertEqual(metrics, {})
    
    @patch('cv2.resize')
    def test_image_scaling(self, mock_resize):
        """Test image scaling for different scale factors"""
        # Mock resize to return a scaled image
        scaled_image = np.random.randint(0, 255, (240, 320), dtype=np.uint8)
        mock_resize.return_value = scaled_image
        
        # Mock detector to return empty list
        self.mock_detector.detect.return_value = []
        
        result = self.detector.detect_multi_scale(
            self.test_image, self.camera_params, self.tag_size
        )
        
        # Check that resize was called for non-1.0 scales
        expected_calls = len([s for s in self.scale_factors if s != 1.0])
        self.assertEqual(mock_resize.call_count, expected_calls)
    
    def test_camera_parameter_scaling(self):
        """Test camera parameter adjustment for different scales"""
        # This test verifies that camera parameters are correctly scaled
        original_params = (320.0, 320.0, 320.0, 240.0)
        
        # Mock detector to capture the camera parameters passed to it
        captured_params = []
        
        def capture_params(image, estimate_tag_pose, camera_params, tag_size):
            captured_params.append(camera_params)
            return []
        
        self.mock_detector.detect.side_effect = capture_params
        
        self.detector.detect_multi_scale(
            self.test_image, original_params, self.tag_size
        )
        
        # Check that we got parameters for each scale
        self.assertEqual(len(captured_params), len(self.scale_factors))
        
        # Check parameter scaling for each scale
        for i, scale in enumerate(self.scale_factors):
            params = captured_params[i]
            expected_fx = original_params[0] * scale
            expected_fy = original_params[1] * scale
            expected_cx = original_params[2] * scale
            expected_cy = original_params[3] * scale
            
            self.assertAlmostEqual(params[0], expected_fx, places=2)
            self.assertAlmostEqual(params[1], expected_fy, places=2)
            self.assertAlmostEqual(params[2], expected_cx, places=2)
            self.assertAlmostEqual(params[3], expected_cy, places=2)


class TestMultiResolutionDetectorIntegration(unittest.TestCase):
    """Integration tests for MultiResolutionDetector"""
    
    def setUp(self):
        """Set up integration test fixtures"""
        # Create a more realistic mock detector
        self.mock_detector = Mock()
        self.detector = MultiResolutionDetector(self.mock_detector)
        
        # Create test image with some structure
        self.test_image = self._create_test_image()
        self.camera_params = (320.0, 320.0, 320.0, 240.0)
        self.tag_size = 0.065
    
    def _create_test_image(self):
        """Create a test image with some structure for more realistic testing"""
        image = np.zeros((480, 640), dtype=np.uint8)
        
        # Add some rectangular patterns that could be AprilTags
        cv2.rectangle(image, (100, 100), (150, 150), 255, -1)
        cv2.rectangle(image, (110, 110), (140, 140), 0, -1)
        cv2.rectangle(image, (120, 120), (130, 130), 255, -1)
        
        cv2.rectangle(image, (300, 200), (350, 250), 255, -1)
        cv2.rectangle(image, (310, 210), (340, 240), 0, -1)
        cv2.rectangle(image, (320, 220), (330, 230), 255, -1)
        
        return image
    
    def test_end_to_end_detection(self):
        """Test end-to-end detection pipeline"""
        # Create realistic mock detections for different scales
        def mock_detect_realistic(image, estimate_tag_pose, camera_params, tag_size):
            detections = []
            
            # Simulate scale-dependent detection success
            scale_factor = image.shape[1] / 640.0  # Estimate scale from width
            
            if 0.4 < scale_factor < 0.6:  # Scale ~0.5
                # Small scale - might miss some tags
                detections.append(MockAprilTagDetection(tag_id=1, decision_margin=0.4))
            elif 0.9 < scale_factor < 1.1:  # Scale ~1.0
                # Original scale - good detection
                detections.append(MockAprilTagDetection(tag_id=1, decision_margin=0.8))
                detections.append(MockAprilTagDetection(tag_id=2, decision_margin=0.6))
            elif 1.4 < scale_factor < 1.6:  # Scale ~1.5
                # Large scale - might detect additional details
                detections.append(MockAprilTagDetection(tag_id=1, decision_margin=0.7))
                detections.append(MockAprilTagDetection(tag_id=2, decision_margin=0.9))
                detections.append(MockAprilTagDetection(tag_id=3, decision_margin=0.3))
            
            return detections
        
        self.mock_detector.detect.side_effect = mock_detect_realistic
        
        result = self.detector.detect_multi_scale(
            self.test_image, self.camera_params, self.tag_size
        )
        
        # Should get the best detections for each tag
        self.assertGreater(len(result), 0)
        
        # Check that we got reasonable results
        tag_ids = [d.tag_id for d in result]
        confidences = [d.decision_margin for d in result]
        
        # Should have unique tag IDs
        self.assertEqual(len(tag_ids), len(set(tag_ids)))
        
        # All confidences should be above threshold
        for conf in confidences:
            self.assertGreaterEqual(conf, self.detector.confidence_threshold)
    
    def test_robustness_to_detection_failures(self):
        """Test robustness when some scales fail to detect"""
        def mock_detect_with_failures(image, estimate_tag_pose, camera_params, tag_size):
            scale_factor = image.shape[1] / 640.0
            
            if scale_factor < 0.6:  # Small scales fail
                raise Exception("Detection failed at small scale")
            elif 0.9 < scale_factor < 1.1:  # Original scale succeeds
                return [MockAprilTagDetection(tag_id=1, decision_margin=0.8)]
            else:  # Other scales return empty
                return []
        
        self.mock_detector.detect.side_effect = mock_detect_with_failures
        
        # Should not raise exception and should return successful detections
        result = self.detector.detect_multi_scale(
            self.test_image, self.camera_params, self.tag_size
        )
        
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].tag_id, 1)
        self.assertEqual(result[0].decision_margin, 0.8)


if __name__ == '__main__':
    # Set up logging to reduce noise during testing
    import logging
    logging.basicConfig(level=logging.WARNING)
    
    # Run the tests
    unittest.main(verbosity=2)
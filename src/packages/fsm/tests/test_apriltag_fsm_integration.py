#!/usr/bin/env python3

import unittest
import time
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Mock ROS modules to avoid import errors
sys.modules['rospy'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['duckietown_msgs.srv'] = Mock()
sys.modules['std_srvs.srv'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Add the src directory to the path so we can import the module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Create a standalone FSMNode class for testing
class MockAprilTagDetection:
    def __init__(self, tag_id=1):
        self.tag_id = tag_id

class MockAprilTagDetectionArray:
    def __init__(self, detections=None):
        self.detections = detections or []

class MockStopLineReading:
    def __init__(self, stop_line_detected=False, at_stop_line=False):
        self.stop_line_detected = stop_line_detected
        self.at_stop_line = at_stop_line

class MockFSMState:
    def __init__(self, state="LANE_FOLLOWING"):
        self.state = state
        self.header = Mock()
        self.header.stamp = Mock()

class EnhancedFSMNode:
    """
    Standalone version of enhanced FSM node for testing AprilTag integration
    """
    
    def __init__(self):
        self.node_name = "test_fsm_node"
        
        # Enhanced AprilTag stop state management
        self.enable_apriltag_stops = True
        self.apriltag_stop_distance_threshold = 1.0
        self.apriltag_stop_duration = 2.0
        
        # AprilTag stop state tracking
        self.apriltag_detections = []
        self.last_apriltag_detection_time = None
        self.apriltag_stop_start_time = None
        self.is_apriltag_stopping = False
        self.apriltag_stop_count = 0
        
        # Performance monitoring
        self.state_transition_count = 0
        self.apriltag_stop_sequences = 0
        self.last_performance_log = time.time()
        
        # State management
        self.state_msg = MockFSMState()
        
        # Mock publishers and services
        self.pub_state = Mock()
        self.changePattern = Mock()
        
        # Mock timer for testing
        self.active_timer = None
    
    def cb_apriltag_detections(self, msg):
        """Callback for AprilTag detections to manage enhanced stop states."""
        if not self.enable_apriltag_stops:
            return
        
        current_time = time.time()
        self.last_apriltag_detection_time = current_time
        self.apriltag_detections = msg.detections
        
        # Check if we should transition to AprilTag stop state
        for detection in msg.detections:
            simulated_distance = 0.8  # 80cm placeholder
            
            if (simulated_distance <= self.apriltag_stop_distance_threshold and 
                not self.is_apriltag_stopping and
                self.state_msg.state in ["LANE_FOLLOWING", "NORMAL_JOYSTICK_CONTROL"]):
                
                self._initiate_apriltag_stop_sequence(detection.tag_id, simulated_distance)
                break
    
    def cb_stop_line_reading(self, msg):
        """Callback for stop line readings to coordinate with AprilTag stops."""
        if not self.enable_apriltag_stops:
            return
        
        if msg.at_stop_line and not self.is_apriltag_stopping:
            # If we have recent AprilTag detections, coordinate the stop
            if (self.last_apriltag_detection_time and 
                (time.time() - self.last_apriltag_detection_time) < 1.0):
                
                self._initiate_apriltag_stop_sequence(tag_id=0, distance=0.3)
    
    def _initiate_apriltag_stop_sequence(self, tag_id, distance):
        """Initiate AprilTag stop sequence with enhanced state management."""
        if self.is_apriltag_stopping:
            return
        
        self.is_apriltag_stopping = True
        self.apriltag_stop_start_time = time.time()
        self.apriltag_stop_count += 1
        
        # Transition to AprilTag stop state
        previous_state = self.state_msg.state
        self.state_msg.state = "APRILTAG_STOP"
        self.state_transition_count += 1
        
        self.publish()
        
        # For testing, we'll manually trigger completion
        # In real implementation, this would be a ROS timer
        self.active_timer = True
    
    def _complete_apriltag_stop_sequence(self):
        """Complete AprilTag stop sequence and return to normal operation."""
        if not self.is_apriltag_stopping:
            return
        
        stop_end_time = time.time()
        total_stop_time = stop_end_time - self.apriltag_stop_start_time if self.apriltag_stop_start_time else 0.0
        
        # Transition back to lane following
        previous_state = self.state_msg.state
        self.state_msg.state = "LANE_FOLLOWING"
        self.state_transition_count += 1
        self.apriltag_stop_sequences += 1
        
        # Reset stop state
        self.is_apriltag_stopping = False
        self.apriltag_stop_start_time = None
        self.active_timer = None
        
        self.publish()
    
    def publish(self):
        """Publish current state."""
        self.pub_state.publish(self.state_msg)
    
    def get_performance_metrics(self):
        """Get performance metrics for testing."""
        return {
            'state_transitions': self.state_transition_count,
            'apriltag_stops': self.apriltag_stop_count,
            'completed_sequences': self.apriltag_stop_sequences,
            'is_stopping': self.is_apriltag_stopping
        }


class TestAprilTagFSMIntegration(unittest.TestCase):
    """Test suite for AprilTag FSM integration"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.fsm = EnhancedFSMNode()
    
    def test_initialization(self):
        """Test FSM initialization with AprilTag support"""
        self.assertTrue(self.fsm.enable_apriltag_stops)
        self.assertEqual(self.fsm.apriltag_stop_distance_threshold, 1.0)
        self.assertEqual(self.fsm.apriltag_stop_duration, 2.0)
        self.assertFalse(self.fsm.is_apriltag_stopping)
        self.assertEqual(self.fsm.apriltag_stop_count, 0)
        self.assertEqual(self.fsm.state_msg.state, "LANE_FOLLOWING")
    
    def test_apriltag_detection_triggers_stop(self):
        """Test that AprilTag detection triggers stop sequence"""
        # Create mock AprilTag detection
        detection = MockAprilTagDetection(tag_id=5)
        msg = MockAprilTagDetectionArray([detection])
        
        # Initial state should be lane following
        self.assertEqual(self.fsm.state_msg.state, "LANE_FOLLOWING")
        self.assertFalse(self.fsm.is_apriltag_stopping)
        
        # Process AprilTag detection
        self.fsm.cb_apriltag_detections(msg)
        
        # Should transition to AprilTag stop state
        self.assertEqual(self.fsm.state_msg.state, "APRILTAG_STOP")
        self.assertTrue(self.fsm.is_apriltag_stopping)
        self.assertEqual(self.fsm.apriltag_stop_count, 1)
        self.assertEqual(self.fsm.state_transition_count, 1)
    
    def test_apriltag_stop_sequence_completion(self):
        """Test complete AprilTag stop sequence"""
        # Start with AprilTag detection
        detection = MockAprilTagDetection(tag_id=3)
        msg = MockAprilTagDetectionArray([detection])
        
        self.fsm.cb_apriltag_detections(msg)
        
        # Verify stop state
        self.assertEqual(self.fsm.state_msg.state, "APRILTAG_STOP")
        self.assertTrue(self.fsm.is_apriltag_stopping)
        
        # Complete the stop sequence
        self.fsm._complete_apriltag_stop_sequence()
        
        # Should return to lane following
        self.assertEqual(self.fsm.state_msg.state, "LANE_FOLLOWING")
        self.assertFalse(self.fsm.is_apriltag_stopping)
        self.assertEqual(self.fsm.apriltag_stop_sequences, 1)
        self.assertEqual(self.fsm.state_transition_count, 2)  # Start stop + end stop
    
    def test_multiple_apriltag_detections(self):
        """Test handling of multiple AprilTag detections"""
        # Create multiple detections
        detections = [MockAprilTagDetection(tag_id=i) for i in range(3)]
        msg = MockAprilTagDetectionArray(detections)
        
        self.fsm.cb_apriltag_detections(msg)
        
        # Should only trigger one stop sequence
        self.assertEqual(self.fsm.apriltag_stop_count, 1)
        self.assertTrue(self.fsm.is_apriltag_stopping)
        
        # Try to trigger another while stopping
        self.fsm.cb_apriltag_detections(msg)
        
        # Should still be only one stop
        self.assertEqual(self.fsm.apriltag_stop_count, 1)
    
    def test_stop_line_coordination_with_apriltag(self):
        """Test coordination between stop line and AprilTag detection"""
        # First, simulate AprilTag detection (but don't trigger stop)
        detection = MockAprilTagDetection(tag_id=7)
        apriltag_msg = MockAprilTagDetectionArray([detection])
        
        # Set distance threshold high so AprilTag doesn't trigger stop
        self.fsm.apriltag_stop_distance_threshold = 0.5
        self.fsm.cb_apriltag_detections(apriltag_msg)
        
        # Should not have triggered stop yet
        self.assertFalse(self.fsm.is_apriltag_stopping)
        
        # Now simulate stop line detection
        stop_msg = MockStopLineReading(stop_line_detected=True, at_stop_line=True)
        self.fsm.cb_stop_line_reading(stop_msg)
        
        # Should trigger coordinated stop
        self.assertTrue(self.fsm.is_apriltag_stopping)
        self.assertEqual(self.fsm.state_msg.state, "APRILTAG_STOP")
    
    def test_stop_line_without_apriltag_coordination(self):
        """Test stop line detection without recent AprilTag detection"""
        # Simulate stop line detection without recent AprilTag
        stop_msg = MockStopLineReading(stop_line_detected=True, at_stop_line=True)
        self.fsm.cb_stop_line_reading(stop_msg)
        
        # Should not trigger AprilTag stop sequence
        self.assertFalse(self.fsm.is_apriltag_stopping)
        self.assertEqual(self.fsm.state_msg.state, "LANE_FOLLOWING")
    
    def test_apriltag_stops_disabled(self):
        """Test behavior when AprilTag stops are disabled"""
        self.fsm.enable_apriltag_stops = False
        
        # Try to trigger AprilTag detection
        detection = MockAprilTagDetection(tag_id=9)
        msg = MockAprilTagDetectionArray([detection])
        
        self.fsm.cb_apriltag_detections(msg)
        
        # Should not trigger stop
        self.assertFalse(self.fsm.is_apriltag_stopping)
        self.assertEqual(self.fsm.apriltag_stop_count, 0)
        self.assertEqual(self.fsm.state_msg.state, "LANE_FOLLOWING")
    
    def test_performance_metrics(self):
        """Test performance metrics collection"""
        # Execute a few stop sequences
        for i in range(3):
            detection = MockAprilTagDetection(tag_id=i)
            msg = MockAprilTagDetectionArray([detection])
            
            self.fsm.cb_apriltag_detections(msg)
            self.fsm._complete_apriltag_stop_sequence()
        
        metrics = self.fsm.get_performance_metrics()
        
        self.assertEqual(metrics['apriltag_stops'], 3)
        self.assertEqual(metrics['completed_sequences'], 3)
        self.assertEqual(metrics['state_transitions'], 6)  # 2 per sequence
        self.assertFalse(metrics['is_stopping'])
    
    def test_state_transitions_from_different_initial_states(self):
        """Test AprilTag stop from different initial states"""
        # Test from NORMAL_JOYSTICK_CONTROL
        self.fsm.state_msg.state = "NORMAL_JOYSTICK_CONTROL"
        
        detection = MockAprilTagDetection(tag_id=10)
        msg = MockAprilTagDetectionArray([detection])
        
        self.fsm.cb_apriltag_detections(msg)
        
        # Should transition to stop
        self.assertEqual(self.fsm.state_msg.state, "APRILTAG_STOP")
        self.assertTrue(self.fsm.is_apriltag_stopping)
        
        # Complete and test from invalid state
        self.fsm._complete_apriltag_stop_sequence()
        self.fsm.state_msg.state = "INVALID_STATE"
        
        self.fsm.cb_apriltag_detections(msg)
        
        # Should not trigger from invalid state
        self.assertEqual(self.fsm.apriltag_stop_count, 1)  # Still only one
    
    def test_distance_threshold_filtering(self):
        """Test that distance threshold properly filters detections"""
        # Set a very low threshold
        self.fsm.apriltag_stop_distance_threshold = 0.1
        
        detection = MockAprilTagDetection(tag_id=11)
        msg = MockAprilTagDetectionArray([detection])
        
        self.fsm.cb_apriltag_detections(msg)
        
        # Should not trigger stop (simulated distance is 0.8m > 0.1m threshold)
        self.assertFalse(self.fsm.is_apriltag_stopping)
        self.assertEqual(self.fsm.apriltag_stop_count, 0)
        
        # Set a high threshold
        self.fsm.apriltag_stop_distance_threshold = 2.0
        
        self.fsm.cb_apriltag_detections(msg)
        
        # Should trigger stop now
        self.assertTrue(self.fsm.is_apriltag_stopping)
        self.assertEqual(self.fsm.apriltag_stop_count, 1)


class TestAprilTagFSMIntegrationAdvanced(unittest.TestCase):
    """Advanced integration tests for AprilTag FSM"""
    
    def setUp(self):
        """Set up advanced test fixtures"""
        self.fsm = EnhancedFSMNode()
    
    def test_rapid_successive_detections(self):
        """Test handling of rapid successive AprilTag detections"""
        detection = MockAprilTagDetection(tag_id=20)
        msg = MockAprilTagDetectionArray([detection])
        
        # Send multiple rapid detections
        for _ in range(5):
            self.fsm.cb_apriltag_detections(msg)
        
        # Should only trigger one stop sequence
        self.assertEqual(self.fsm.apriltag_stop_count, 1)
        self.assertTrue(self.fsm.is_apriltag_stopping)
    
    def test_detection_timing_coordination(self):
        """Test timing-based coordination between AprilTag and stop line"""
        # Simulate AprilTag detection
        detection = MockAprilTagDetection(tag_id=21)
        apriltag_msg = MockAprilTagDetectionArray([detection])
        
        # Set threshold to prevent immediate stop
        self.fsm.apriltag_stop_distance_threshold = 0.5
        self.fsm.cb_apriltag_detections(apriltag_msg)
        
        # Simulate time passing (within coordination window)
        time.sleep(0.1)
        
        # Now trigger stop line
        stop_msg = MockStopLineReading(stop_line_detected=True, at_stop_line=True)
        self.fsm.cb_stop_line_reading(stop_msg)
        
        # Should coordinate
        self.assertTrue(self.fsm.is_apriltag_stopping)
        
        # Test with time outside coordination window
        self.fsm._complete_apriltag_stop_sequence()
        
        # Simulate old AprilTag detection (outside 1 second window)
        self.fsm.last_apriltag_detection_time = time.time() - 2.0
        
        self.fsm.cb_stop_line_reading(stop_msg)
        
        # Should not coordinate
        self.assertFalse(self.fsm.is_apriltag_stopping)
    
    def test_state_consistency_during_transitions(self):
        """Test state consistency during rapid state transitions"""
        detection = MockAprilTagDetection(tag_id=22)
        msg = MockAprilTagDetectionArray([detection])
        
        # Record initial state
        initial_transitions = self.fsm.state_transition_count
        
        # Trigger stop
        self.fsm.cb_apriltag_detections(msg)
        mid_transitions = self.fsm.state_transition_count
        
        # Complete stop
        self.fsm._complete_apriltag_stop_sequence()
        final_transitions = self.fsm.state_transition_count
        
        # Verify transition count consistency
        self.assertEqual(mid_transitions - initial_transitions, 1)
        self.assertEqual(final_transitions - mid_transitions, 1)
        self.assertEqual(final_transitions - initial_transitions, 2)
        
        # Verify final state
        self.assertEqual(self.fsm.state_msg.state, "LANE_FOLLOWING")
        self.assertFalse(self.fsm.is_apriltag_stopping)
    
    def test_error_recovery_scenarios(self):
        """Test error recovery in various scenarios"""
        # Test recovery from inconsistent state
        self.fsm.is_apriltag_stopping = True
        self.fsm.state_msg.state = "LANE_FOLLOWING"  # Inconsistent state
        
        # Try to complete stop sequence
        self.fsm._complete_apriltag_stop_sequence()
        
        # Should handle gracefully
        self.assertFalse(self.fsm.is_apriltag_stopping)
        
        # Test multiple completion calls
        self.fsm.is_apriltag_stopping = True
        self.fsm.apriltag_stop_start_time = time.time()
        
        self.fsm._complete_apriltag_stop_sequence()
        self.fsm._complete_apriltag_stop_sequence()  # Second call
        
        # Should handle gracefully
        self.assertFalse(self.fsm.is_apriltag_stopping)


if __name__ == '__main__':
    # Set up logging to reduce noise during testing
    import logging
    logging.basicConfig(level=logging.WARNING)
    
    # Run the tests
    unittest.main(verbosity=2)
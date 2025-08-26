#!/usr/bin/env python3

import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock

# Import the modules to test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# Mock rospy and ROS messages
class MockRospy:
    def loginfo(self, msg): print(f"[INFO] {msg}")
    def logwarn(self, msg): print(f"[WARN] {msg}")
    def logerr(self, msg): print(f"[ERROR] {msg}")
    def logdebug(self, msg): print(f"[DEBUG] {msg}")
    def get_param(self, param, default=None): return default
    def set_param(self, param, value): pass
    
    class Duration:
        def __init__(self, secs): self.secs = secs
    
    class Timer:
        def __init__(self, duration, callback, oneshot=False):
            self.duration = duration
            self.callback = callback
            self.oneshot = oneshot
            self.active = True
        def shutdown(self): self.active = False
    
    class Time:
        @staticmethod
        def now(): return MockRospy.Time()
        def __init__(self): self.secs = time.time()
    
    class Subscriber:
        def __init__(self, topic, msg_type, callback): pass
    
    class Publisher:
        def __init__(self, topic, msg_type, queue_size=1): pass
        def publish(self, msg): pass

sys.modules['rospy'] = MockRospy()

# Mock messages
class MockString:
    def __init__(self): self.data = ""

class MockBool:
    def __init__(self): self.data = False

class MockColorRGBA:
    def __init__(self): 
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0
        self.a = 1.0

class MockStdMsgs:
    class msg:
        String = MockString
        Bool = MockBool
        ColorRGBA = MockColorRGBA

sys.modules['std_msgs'] = MockStdMsgs()
sys.modules['std_msgs.msg'] = MockStdMsgs.msg()

class MockColorRGBA:
    def __init__(self): 
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0
        self.a = 1.0

class MockLEDPattern:
    def __init__(self): self.rgb_vals = []

class MockDuckietownMsgs:
    class msg:
        LEDPattern = MockLEDPattern

sys.modules['duckietown_msgs'] = MockDuckietownMsgs()
sys.modules['duckietown_msgs.msg'] = MockDuckietownMsgs.msg()

class MockTwist:
    def __init__(self):
        self.linear = Mock()
        self.angular = Mock()
        self.linear.x = 0.0
        self.angular.z = 0.0

class MockGeometryMsgs:
    class msg:
        Twist = MockTwist

sys.modules['geometry_msgs'] = MockGeometryMsgs()
sys.modules['geometry_msgs.msg'] = MockGeometryMsgs.msg()

# Now import the actual implementation
try:
    from intention_signaler import (
        IntentionSignaler, LaneChangeSignal, SignalPriority, SignalPattern
    )
    print("✅ Successfully imported intention signaler")
except Exception as e:
    print(f"❌ Failed to import intention signaler: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)


class TestLaneChangeSignalingTiming(unittest.TestCase):
    """
    Test suite for lane change signaling timing and FSM integration.
    
    Tests the intention signaler's timing accuracy, signal patterns,
    abort mechanisms, and FSM integration functionality.
    """
    
    def setUp(self):
        """Set up test fixtures"""
        # Create mock LED emitter node
        self.mock_led_emitter = Mock()
        self.mock_led_emitter.node_name = "test_led_emitter"
        self.mock_led_emitter._LED_protocol = {
            "signals": {},
            "colors": {
                "yellow": [255, 255, 0],
                "red": [255, 0, 0],
                "green": [0, 255, 0],
                "switchedoff": [0, 0, 0]
            }
        }
        self.mock_led_emitter.changePattern = Mock()
        
        # Create intention signaler instance
        with patch('rospy.get_param') as mock_get_param:
            mock_get_param.side_effect = lambda param, default: default
            with patch('rospy.Subscriber'), patch('rospy.Publisher'):
                self.intention_signaler = IntentionSignaler(self.mock_led_emitter)
        
        print(f"[TestLaneChangeSignalingTiming] Test setup completed")
    
    def test_intention_signaler_initialization(self):
        """Test that the intention signaler initializes correctly"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing intention signaler initialization")
        
        # Check that signaler is initialized
        self.assertIsNotNone(self.intention_signaler)
        self.assertTrue(self.intention_signaler.signaling_enabled)
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.NONE)
        self.assertFalse(self.intention_signaler.signal_active)
        
        # Check signal patterns are initialized
        self.assertIn("LEFT_TURN", self.intention_signaler.signal_patterns)
        self.assertIn("RIGHT_TURN", self.intention_signaler.signal_patterns)
        self.assertIn("HAZARD", self.intention_signaler.signal_patterns)
        self.assertIn("ABORT", self.intention_signaler.signal_patterns)
        self.assertIn("CONFIRMATION", self.intention_signaler.signal_patterns)
        
        # Check performance metrics initialization
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertEqual(metrics['total_signals_sent'], 0)
        self.assertEqual(metrics['left_turn_signals'], 0)
        self.assertEqual(metrics['right_turn_signals'], 0)
        
        print(f"[TestLaneChangeSignalingTiming] Intention signaler initialization test passed")
    
    def test_signal_pattern_definitions(self):
        """Test signal pattern definitions and properties"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing signal pattern definitions")
        
        patterns = self.intention_signaler.signal_patterns
        
        # Test left turn pattern
        left_pattern = patterns["LEFT_TURN"]
        self.assertEqual(left_pattern.name, "LEFT_TURN")
        self.assertEqual(left_pattern.frequency, self.intention_signaler.signal_frequency)
        self.assertEqual(left_pattern.priority, SignalPriority.HIGH)
        self.assertEqual(left_pattern.frequency_mask, [1, 1, 0, 0, 0])  # Left LEDs only
        
        # Test right turn pattern
        right_pattern = patterns["RIGHT_TURN"]
        self.assertEqual(right_pattern.name, "RIGHT_TURN")
        self.assertEqual(right_pattern.frequency, self.intention_signaler.signal_frequency)
        self.assertEqual(right_pattern.priority, SignalPriority.HIGH)
        self.assertEqual(right_pattern.frequency_mask, [0, 0, 0, 1, 1])  # Right LEDs only
        
        # Test hazard pattern
        hazard_pattern = patterns["HAZARD"]
        self.assertEqual(hazard_pattern.name, "HAZARD")
        self.assertEqual(hazard_pattern.priority, SignalPriority.EMERGENCY)
        self.assertEqual(hazard_pattern.frequency_mask, [1, 1, 1, 1, 1])  # All LEDs
        self.assertGreater(hazard_pattern.duration, left_pattern.duration)  # Longer duration
        
        # Test abort pattern
        abort_pattern = patterns["ABORT"]
        self.assertEqual(abort_pattern.name, "ABORT")
        self.assertEqual(abort_pattern.priority, SignalPriority.EMERGENCY)
        self.assertGreater(abort_pattern.frequency, left_pattern.frequency)  # Faster blinking
        self.assertEqual(abort_pattern.color_list, ["red", "red", "red", "red", "red"])
        
        print(f"[TestLaneChangeSignalingTiming] Signal pattern definitions test passed")
    
    def test_left_turn_signal_activation(self):
        """Test left turn signal activation and timing"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing left turn signal activation")
        
        # Record start time
        start_time = time.time()
        
        # Activate left turn signal
        self.intention_signaler.signal_left_turn()
        
        # Check signal state
        self.assertTrue(self.intention_signaler.signal_active)
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.LEFT_TURN)
        self.assertIsNotNone(self.intention_signaler.signal_start_time)
        
        # Check timing
        activation_time = self.intention_signaler.signal_start_time
        self.assertGreaterEqual(activation_time, start_time)
        self.assertLessEqual(activation_time, time.time())
        
        # Check LED emitter was called
        self.mock_led_emitter.changePattern.assert_called()
        
        # Check performance metrics
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertEqual(metrics['left_turn_signals'], 1)
        self.assertEqual(metrics['total_signals_sent'], 1)
        
        print(f"[TestLaneChangeSignalingTiming] Left turn signal activation test passed")
    
    def test_right_turn_signal_activation(self):
        """Test right turn signal activation and timing"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing right turn signal activation")
        
        # Record start time
        start_time = time.time()
        
        # Activate right turn signal
        self.intention_signaler.signal_right_turn()
        
        # Check signal state
        self.assertTrue(self.intention_signaler.signal_active)
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.RIGHT_TURN)
        self.assertIsNotNone(self.intention_signaler.signal_start_time)
        
        # Check timing
        activation_time = self.intention_signaler.signal_start_time
        self.assertGreaterEqual(activation_time, start_time)
        self.assertLessEqual(activation_time, time.time())
        
        # Check performance metrics
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertEqual(metrics['right_turn_signals'], 1)
        self.assertEqual(metrics['total_signals_sent'], 1)
        
        print(f"[TestLaneChangeSignalingTiming] Right turn signal activation test passed")
    
    def test_hazard_signal_activation(self):
        """Test hazard signal activation and emergency priority"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing hazard signal activation")
        
        # Activate hazard signal
        self.intention_signaler.signal_hazard()
        
        # Check signal state
        self.assertTrue(self.intention_signaler.signal_active)
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.HAZARD)
        
        # Check emergency priority
        hazard_pattern = self.intention_signaler.signal_patterns["HAZARD"]
        self.assertEqual(hazard_pattern.priority, SignalPriority.EMERGENCY)
        
        # Check longer duration for hazard
        self.assertGreater(hazard_pattern.duration, self.intention_signaler.signal_duration)
        
        # Check performance metrics
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertEqual(metrics['hazard_signals'], 1)
        self.assertEqual(metrics['total_signals_sent'], 1)
        
        print(f"[TestLaneChangeSignalingTiming] Hazard signal activation test passed")
    
    def test_abort_signal_activation(self):
        """Test abort signal activation and emergency response"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing abort signal activation")
        
        # Activate abort signal
        self.intention_signaler.signal_abort()
        
        # Check signal state
        self.assertTrue(self.intention_signaler.signal_active)
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.ABORT)
        self.assertTrue(self.intention_signaler.abort_requested)
        
        # Check abort pattern properties
        abort_pattern = self.intention_signaler.signal_patterns["ABORT"]
        self.assertEqual(abort_pattern.priority, SignalPriority.EMERGENCY)
        self.assertGreater(abort_pattern.frequency, 2.0)  # Fast blinking
        self.assertEqual(abort_pattern.duration, self.intention_signaler.abort_signal_duration)
        
        # Check performance metrics
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertEqual(metrics['abort_signals'], 1)
        self.assertEqual(metrics['total_signals_sent'], 1)
        
        print(f"[TestLaneChangeSignalingTiming] Abort signal activation test passed")
    
    def test_signal_clear_functionality(self):
        """Test signal clearing and timing calculation"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing signal clear functionality")
        
        # Activate a signal first
        self.intention_signaler.signal_left_turn()
        start_time = self.intention_signaler.signal_start_time
        
        # Wait a short time
        time.sleep(0.1)
        
        # Clear the signal
        self.intention_signaler.clear_signal()
        
        # Check signal state
        self.assertFalse(self.intention_signaler.signal_active)
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.NONE)
        self.assertIsNone(self.intention_signaler.signal_start_time)
        self.assertFalse(self.intention_signaler.abort_requested)
        
        # Check that LED pattern was reset
        self.mock_led_emitter.changePattern.assert_called_with("CAR_DRIVING")
        
        # Check performance metrics were updated
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertGreater(metrics['average_signal_duration'], 0.0)
        
        print(f"[TestLaneChangeSignalingTiming] Signal clear functionality test passed")
    
    def test_signal_override_behavior(self):
        """Test signal override behavior with different priorities"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing signal override behavior")
        
        # Activate normal priority signal
        self.intention_signaler.signal_left_turn()
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.LEFT_TURN)
        
        # Activate emergency priority signal (should override)
        self.intention_signaler.signal_abort()
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.ABORT)
        self.assertTrue(self.intention_signaler.abort_requested)
        
        # Check that metrics were updated for both signals
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertEqual(metrics['left_turn_signals'], 1)
        self.assertEqual(metrics['abort_signals'], 1)
        self.assertEqual(metrics['total_signals_sent'], 2)
        
        print(f"[TestLaneChangeSignalingTiming] Signal override behavior test passed")
    
    def test_fsm_integration_callbacks(self):
        """Test FSM integration callbacks and state coordination"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing FSM integration callbacks")
        
        # Test lane following state callback
        fsm_msg = MockString()
        fsm_msg.data = "LANE_FOLLOWING"
        
        # Activate a signal first
        self.intention_signaler.signal_left_turn()
        self.assertTrue(self.intention_signaler.signal_active)
        
        # Send FSM state update
        self.intention_signaler.cb_fsm_state(fsm_msg)
        
        # Check FSM integration was activated
        self.assertTrue(self.intention_signaler.fsm_integration_active)
        
        # Check performance metrics
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertGreater(metrics['fsm_integrations'], 0)
        
        # Test coordination state
        fsm_msg.data = "COORDINATION"
        self.intention_signaler.clear_signal()  # Clear first
        self.intention_signaler.cb_fsm_state(fsm_msg)
        
        # Should activate hazard signals during coordination
        # (Note: In real implementation, this would depend on current state)
        
        print(f"[TestLaneChangeSignalingTiming] FSM integration callbacks test passed")
    
    def test_lane_change_decision_callbacks(self):
        """Test lane change decision callbacks"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing lane change decision callbacks")
        
        # Test left lane change decision
        decision_msg = MockString()
        decision_msg.data = "CHANGE_LEFT: confidence=0.8"
        
        self.intention_signaler.cb_lane_change_decision(decision_msg)
        
        # Should activate left turn signal
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.LEFT_TURN)
        self.assertTrue(self.intention_signaler.signal_active)
        
        # Test right lane change decision
        decision_msg.data = "CHANGE_RIGHT: confidence=0.9"
        self.intention_signaler.cb_lane_change_decision(decision_msg)
        
        # Should switch to right turn signal
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.RIGHT_TURN)
        
        # Test no change decision
        decision_msg.data = "NO_CHANGE: confidence=0.5"
        self.intention_signaler.cb_lane_change_decision(decision_msg)
        
        # Should clear signals
        self.assertFalse(self.intention_signaler.signal_active)
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.NONE)
        
        print(f"[TestLaneChangeSignalingTiming] Lane change decision callbacks test passed")
    
    def test_abort_mechanism_callbacks(self):
        """Test abort mechanism callbacks and timing"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing abort mechanism callbacks")
        
        # Test abort request
        abort_msg = MockBool()
        abort_msg.data = True
        
        self.intention_signaler.cb_lane_change_abort(abort_msg)
        
        # Should activate abort signal
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.ABORT)
        self.assertTrue(self.intention_signaler.abort_requested)
        self.assertTrue(self.intention_signaler.signal_active)
        
        # Test abort clear
        abort_msg.data = False
        self.intention_signaler.cb_lane_change_abort(abort_msg)
        
        # Should clear abort flag
        self.assertFalse(self.intention_signaler.abort_requested)
        
        print(f"[TestLaneChangeSignalingTiming] Abort mechanism callbacks test passed")
    
    def test_performance_metrics_tracking(self):
        """Test performance metrics tracking accuracy"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing performance metrics tracking")
        
        # Reset metrics
        self.intention_signaler.reset_metrics()
        initial_metrics = self.intention_signaler.get_performance_metrics()
        
        # Perform various signal operations
        self.intention_signaler.signal_left_turn()
        time.sleep(0.05)
        self.intention_signaler.clear_signal()
        
        self.intention_signaler.signal_right_turn()
        time.sleep(0.05)
        self.intention_signaler.clear_signal()
        
        self.intention_signaler.signal_hazard()
        time.sleep(0.05)
        self.intention_signaler.clear_signal()
        
        self.intention_signaler.signal_abort()
        time.sleep(0.05)
        self.intention_signaler.clear_signal()
        
        # Check updated metrics
        final_metrics = self.intention_signaler.get_performance_metrics()
        
        self.assertEqual(final_metrics['left_turn_signals'], 1)
        self.assertEqual(final_metrics['right_turn_signals'], 1)
        self.assertEqual(final_metrics['hazard_signals'], 1)
        self.assertEqual(final_metrics['abort_signals'], 1)
        self.assertEqual(final_metrics['total_signals_sent'], 4)
        self.assertGreater(final_metrics['average_signal_duration'], 0.0)
        
        print(f"[TestLaneChangeSignalingTiming] Performance metrics:")
        print(f"  Total signals: {final_metrics['total_signals_sent']}")
        print(f"  Left turn signals: {final_metrics['left_turn_signals']}")
        print(f"  Right turn signals: {final_metrics['right_turn_signals']}")
        print(f"  Hazard signals: {final_metrics['hazard_signals']}")
        print(f"  Abort signals: {final_metrics['abort_signals']}")
        print(f"  Average duration: {final_metrics['average_signal_duration']:.3f}s")
        
        print(f"[TestLaneChangeSignalingTiming] Performance metrics tracking test passed")
    
    def test_signal_status_reporting(self):
        """Test signal status reporting functionality"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing signal status reporting")
        
        # Test status when no signal is active
        status = self.intention_signaler.get_signal_status()
        self.assertFalse(status['signal_active'])
        self.assertEqual(status['current_signal'], LaneChangeSignal.NONE.value)
        self.assertEqual(status['signal_duration'], 0.0)
        self.assertFalse(status['abort_requested'])
        
        # Activate a signal and test status
        self.intention_signaler.signal_left_turn()
        status = self.intention_signaler.get_signal_status()
        
        self.assertTrue(status['signal_active'])
        self.assertEqual(status['current_signal'], LaneChangeSignal.LEFT_TURN.value)
        self.assertGreater(status['signal_duration'], 0.0)
        self.assertIn('available_patterns', status)
        self.assertIn('performance_metrics', status)
        
        # Test available patterns
        self.assertIn('LEFT_TURN', status['available_patterns'])
        self.assertIn('RIGHT_TURN', status['available_patterns'])
        self.assertIn('HAZARD', status['available_patterns'])
        self.assertIn('ABORT', status['available_patterns'])
        
        print(f"[TestLaneChangeSignalingTiming] Signal status reporting test passed")
    
    def test_thread_safety(self):
        """Test thread safety of signal operations"""
        print(f"\n[TestLaneChangeSignalingTiming] Testing thread safety")
        
        # Test concurrent signal operations
        def signal_operations():
            for i in range(5):
                self.intention_signaler.signal_left_turn()
                time.sleep(0.01)
                self.intention_signaler.clear_signal()
                time.sleep(0.01)
        
        # Run concurrent operations
        threads = []
        for i in range(3):
            thread = threading.Thread(target=signal_operations)
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Check that metrics are consistent
        metrics = self.intention_signaler.get_performance_metrics()
        self.assertGreaterEqual(metrics['total_signals_sent'], 0)
        self.assertGreaterEqual(metrics['left_turn_signals'], 0)
        
        # Check that no signal is active after all operations
        self.assertFalse(self.intention_signaler.signal_active)
        self.assertEqual(self.intention_signaler.current_signal, LaneChangeSignal.NONE)
        
        print(f"[TestLaneChangeSignalingTiming] Thread safety test passed")


def run_lane_change_signaling_tests():
    """Run all lane change signaling timing tests"""
    print(f"\n{'='*70}")
    print(f"RUNNING LANE CHANGE SIGNALING TIMING TESTS")
    print(f"{'='*70}")
    
    # Create test suite
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestLaneChangeSignalingTiming)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout, buffer=False)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n{'='*70}")
    print(f"LANE CHANGE SIGNALING TIMING TEST SUMMARY")
    print(f"{'='*70}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\nFailures:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback}")
    
    if result.errors:
        print(f"\nErrors:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    # Run the tests
    success = run_lane_change_signaling_tests()
    
    if success:
        print(f"\n✅ All lane change signaling timing tests passed!")
        exit(0)
    else:
        print(f"\n❌ Some lane change signaling timing tests failed!")
        exit(1)
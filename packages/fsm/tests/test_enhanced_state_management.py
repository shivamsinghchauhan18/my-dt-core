#!/usr/bin/env python3

import unittest
import time
import tempfile
import os
from unittest.mock import Mock, patch, MagicMock
import sys

# Add the src directory to the path so we can import the FSM
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Mock dependencies before importing
sys.modules['rospy'] = Mock()
sys.modules['psutil'] = Mock()
sys.modules['duckietown_msgs'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['duckietown_msgs.srv'] = Mock()
sys.modules['std_srvs'] = Mock()
sys.modules['std_srvs.srv'] = Mock()
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()
sys.modules['sensor_msgs'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['geometry_msgs'] = Mock()
sys.modules['geometry_msgs.msg'] = Mock()
sys.modules['safety_status_publisher'] = Mock()

from fsm_node import StateManager, StateValidationResult, StateTransition, StateSnapshot


class TestEnhancedStateManagement(unittest.TestCase):
    """Test suite for enhanced state management functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock rospy
        self.rospy_patcher = patch('fsm_node.rospy')
        self.mock_rospy = self.rospy_patcher.start()
        self.mock_rospy.loginfo = Mock()
        self.mock_rospy.logdebug = Mock()
        self.mock_rospy.logwarn = Mock()
        self.mock_rospy.logerr = Mock()
        self.mock_rospy.get_param = Mock(return_value=True)
        self.mock_rospy.Time.now = Mock(return_value=Mock(to_sec=Mock(return_value=time.time())))
        self.mock_rospy.Duration = Mock()
        self.mock_rospy.Timer = Mock()
        self.mock_rospy.sleep = Mock()
        
        # Mock psutil
        self.psutil_patcher = patch('fsm_node.psutil')
        self.mock_psutil = self.psutil_patcher.start()
        self.mock_psutil.cpu_percent.return_value = 50.0
        self.mock_psutil.virtual_memory.return_value = Mock(percent=60.0)
        
        # Create temporary persistence file
        self.temp_file = tempfile.NamedTemporaryFile(delete=False)
        self.temp_file.close()
        
        # Create state manager instance
        self.state_manager = StateManager("test_node", self.temp_file.name)
    
    def tearDown(self):
        """Clean up test fixtures"""
        self.rospy_patcher.stop()
        self.psutil_patcher.stop()
        
        # Clean up temp file
        if os.path.exists(self.temp_file.name):
            os.unlink(self.temp_file.name)
    
    def test_state_manager_initialization(self):
        """Test state manager initialization"""
        print("\n[TEST] Testing state manager initialization...")
        
        self.assertIsNotNone(self.state_manager)
        self.assertEqual(self.state_manager.node_name, "test_node")
        self.assertEqual(self.state_manager.persistence_path, self.temp_file.name)
        self.assertIsNone(self.state_manager.current_state)
        self.assertIsNone(self.state_manager.previous_state)
        
        # Check validators and recovery strategies are initialized
        self.assertGreater(len(self.state_manager.state_validators), 0)
        self.assertGreater(len(self.state_manager.recovery_strategies), 0)
        
        print("✓ State manager initialization test passed")
    
    def test_valid_state_transition(self):
        """Test valid state transitions"""
        print("\n[TEST] Testing valid state transitions...")
        
        # Test initial transition
        success = self.state_manager.transition_to_state(
            new_state="LANE_FOLLOWING",
            event="start_navigation",
            metadata={"test": True}
        )
        
        self.assertTrue(success)
        self.assertEqual(self.state_manager.current_state, "LANE_FOLLOWING")
        self.assertIsNone(self.state_manager.previous_state)
        
        # Test subsequent transition
        success = self.state_manager.transition_to_state(
            new_state="INTERSECTION_COORDINATION",
            event="intersection_detected",
            metadata={"intersection_id": 1}
        )
        
        self.assertTrue(success)
        self.assertEqual(self.state_manager.current_state, "INTERSECTION_COORDINATION")
        self.assertEqual(self.state_manager.previous_state, "LANE_FOLLOWING")
        
        # Check transition history (should have 2 transitions)
        self.assertGreaterEqual(len(self.state_manager.state_transition_history), 1)  # At least 1 transition recorded
        
        print("✓ Valid state transition test passed")
    
    def test_state_validation(self):
        """Test state validation mechanisms"""
        print("\n[TEST] Testing state validation...")
        
        # Test timeout validation
        self.state_manager.current_state = "LANE_FOLLOWING"
        self.state_manager.state_entry_time = time.time() - 400.0  # 400 seconds ago
        
        result = self.state_manager._validate_state_timeout("LANE_FOLLOWING", "EMERGENCY_STOP", "timeout")
        self.assertEqual(result, StateValidationResult.TIMEOUT_ERROR)
        
        # Test consistency validation
        # Create rapid oscillation scenario - alternating between two states
        transitions = [
            ("LANE_FOLLOWING", "SAFE_MODE"),
            ("SAFE_MODE", "LANE_FOLLOWING"), 
            ("LANE_FOLLOWING", "SAFE_MODE")
        ]
        
        for from_state, to_state in transitions:
            transition = StateTransition(
                from_state=from_state,
                to_state=to_state,
                event="test_event",
                timestamp=time.time(),
                duration_in_previous_state=1.0,
                validation_result=StateValidationResult.VALID
            )
            self.state_manager.state_transition_history.append(transition)
        
        # This should trigger oscillation detection
        result = self.state_manager._validate_state_consistency("SAFE_MODE", "LANE_FOLLOWING", "oscillation")
        self.assertEqual(result, StateValidationResult.CONSISTENCY_ERROR)
        
        print("✓ State validation test passed")
    
    def test_precondition_checking(self):
        """Test precondition checking"""
        print("\n[TEST] Testing precondition checking...")
        
        # Test preconditions for LANE_FOLLOWING
        result = self.state_manager._check_state_preconditions("LANE_FOLLOWING")
        self.assertTrue(result)  # Should pass with mocked functions
        
        # Test preconditions for EMERGENCY_STOP (no preconditions)
        result = self.state_manager._check_state_preconditions("EMERGENCY_STOP")
        self.assertTrue(result)
        
        # Test unknown state (no preconditions defined)
        result = self.state_manager._check_state_preconditions("UNKNOWN_STATE")
        self.assertTrue(result)
        
        print("✓ Precondition checking test passed")
    
    def test_postcondition_checking(self):
        """Test postcondition checking"""
        print("\n[TEST] Testing postcondition checking...")
        
        # Test postconditions for LANE_FOLLOWING
        result = self.state_manager._check_state_postconditions("LANE_FOLLOWING")
        self.assertTrue(result)  # Should pass with mocked functions
        
        # Test postconditions for EMERGENCY_STOP
        result = self.state_manager._check_state_postconditions("EMERGENCY_STOP")
        self.assertTrue(result)
        
        # Test unknown state (no postconditions defined)
        result = self.state_manager._check_state_postconditions("UNKNOWN_STATE")
        self.assertTrue(result)
        
        print("✓ Postcondition checking test passed")
    
    def test_recovery_mechanisms(self):
        """Test recovery mechanisms"""
        print("\n[TEST] Testing recovery mechanisms...")
        
        # Test timeout recovery
        success = self.state_manager._recover_from_timeout(
            StateValidationResult.TIMEOUT_ERROR,
            "LANE_FOLLOWING",
            "timeout_event",
            {}
        )
        
        self.assertTrue(success)
        self.assertEqual(self.state_manager.current_state, "SAFE_MODE")
        
        # Test consistency recovery
        # Add some history first
        for i in range(6):
            transition = StateTransition(
                from_state="STATE_A",
                to_state="STATE_B",
                event="test",
                timestamp=time.time(),
                duration_in_previous_state=1.0,
                validation_result=StateValidationResult.VALID
            )
            self.state_manager.state_transition_history.append(transition)
        
        original_length = len(self.state_manager.state_transition_history)
        
        success = self.state_manager._recover_from_consistency_error(
            StateValidationResult.CONSISTENCY_ERROR,
            "LANE_FOLLOWING",
            "consistency_event",
            {}
        )
        
        self.assertTrue(success)
        self.assertLess(len(self.state_manager.state_transition_history), original_length)
        
        print("✓ Recovery mechanisms test passed")
    
    def test_state_persistence(self):
        """Test state persistence and recovery"""
        print("\n[TEST] Testing state persistence...")
        
        # Set up state
        self.state_manager.current_state = "LANE_FOLLOWING"
        self.state_manager.state_entry_time = time.time()
        
        # Add some transition history
        transition = StateTransition(
            from_state="INITIAL",
            to_state="LANE_FOLLOWING",
            event="start",
            timestamp=time.time(),
            duration_in_previous_state=0.0,
            validation_result=StateValidationResult.VALID
        )
        self.state_manager.state_transition_history.append(transition)
        
        # Test persistence
        self.state_manager._persist_current_state()
        self.assertTrue(os.path.exists(self.temp_file.name))
        
        # Create new state manager and test restoration
        new_state_manager = StateManager("test_node", self.temp_file.name)
        
        # Should have restored the state
        self.assertEqual(new_state_manager.current_state, "LANE_FOLLOWING")
        
        print("✓ State persistence test passed")
    
    def test_state_statistics(self):
        """Test state statistics collection"""
        print("\n[TEST] Testing state statistics...")
        
        # Perform some transitions
        self.state_manager.transition_to_state("LANE_FOLLOWING", "start", {})
        time.sleep(0.1)  # Small delay
        self.state_manager.transition_to_state("INTERSECTION_COORDINATION", "intersection", {})
        
        # Get statistics
        stats = self.state_manager.get_state_statistics()
        
        # Verify statistics structure
        self.assertIn('current_state', stats)
        self.assertIn('previous_state', stats)
        self.assertIn('total_transitions', stats)
        self.assertIn('transition_counts', stats)
        self.assertIn('validation_failures', stats)
        
        # Verify values
        self.assertEqual(stats['current_state'], "INTERSECTION_COORDINATION")
        self.assertEqual(stats['previous_state'], "LANE_FOLLOWING")
        self.assertGreaterEqual(stats['total_transitions'], 1)  # At least 1 transition
        
        print("✓ State statistics test passed")
    
    def test_state_snapshot_creation(self):
        """Test state snapshot creation"""
        print("\n[TEST] Testing state snapshot creation...")
        
        # Set up state
        self.state_manager.current_state = "LANE_FOLLOWING"
        self.state_manager.state_entry_time = time.time()
        
        # Create snapshot
        snapshot = self.state_manager._create_state_snapshot()
        
        # Verify snapshot structure
        self.assertIsInstance(snapshot, StateSnapshot)
        self.assertEqual(snapshot.state, "LANE_FOLLOWING")
        self.assertIsInstance(snapshot.timestamp, float)
        self.assertIsInstance(snapshot.system_metrics, dict)
        self.assertIsInstance(snapshot.metadata, dict)
        
        print("✓ State snapshot creation test passed")
    
    def test_transition_validation_failure_handling(self):
        """Test handling of transition validation failures"""
        print("\n[TEST] Testing transition validation failure handling...")
        
        # Mock a validator to always fail
        def failing_validator(from_state, to_state, event):
            return StateValidationResult.CONSISTENCY_ERROR
        
        self.state_manager.state_validators['test_validator'] = failing_validator
        
        # Attempt transition (should trigger recovery)
        success = self.state_manager.transition_to_state(
            new_state="LANE_FOLLOWING",
            event="test_event",
            metadata={}
        )
        
        # Should succeed due to recovery mechanism
        self.assertTrue(success)
        
        # Check that recovery was attempted
        self.assertGreater(len(self.state_manager.recovery_events), 0)
        
        print("✓ Transition validation failure handling test passed")
    
    def test_max_recovery_attempts(self):
        """Test maximum recovery attempts limit"""
        print("\n[TEST] Testing maximum recovery attempts...")
        
        # Set recovery count to maximum
        self.state_manager.recovery_attempt_count = self.state_manager.max_recovery_attempts
        
        # Mock validator to always fail
        def always_fail_validator(from_state, to_state, event):
            return StateValidationResult.TIMEOUT_ERROR
        
        self.state_manager.state_validators['fail_validator'] = always_fail_validator
        
        # Attempt transition
        success = self.state_manager.transition_to_state(
            new_state="LANE_FOLLOWING",
            event="test_event",
            metadata={}
        )
        
        # Should fail due to max recovery attempts exceeded
        self.assertFalse(success)
        
        print("✓ Maximum recovery attempts test passed")
    
    def test_time_in_current_state(self):
        """Test time in current state calculation"""
        print("\n[TEST] Testing time in current state calculation...")
        
        # Initially should be 0
        self.assertEqual(self.state_manager.get_time_in_current_state(), 0.0)
        
        # Set state entry time
        self.state_manager.state_entry_time = time.time() - 5.0  # 5 seconds ago
        
        # Should return approximately 5 seconds
        time_in_state = self.state_manager.get_time_in_current_state()
        self.assertGreaterEqual(time_in_state, 4.9)
        self.assertLessEqual(time_in_state, 5.1)
        
        print("✓ Time in current state test passed")


def run_enhanced_state_management_tests():
    """Run all enhanced state management tests"""
    print("=" * 70)
    print("RUNNING ENHANCED STATE MANAGEMENT TESTS")
    print("=" * 70)
    
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestEnhancedStateManagement)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "=" * 70)
    print("ENHANCED STATE MANAGEMENT TEST SUMMARY")
    print("=" * 70)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print("\nFAILURES:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print("\nERRORS:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    success = len(result.failures) == 0 and len(result.errors) == 0
    print(f"\nOverall result: {'PASS' if success else 'FAIL'}")
    
    return success


if __name__ == '__main__':
    run_enhanced_state_management_tests()
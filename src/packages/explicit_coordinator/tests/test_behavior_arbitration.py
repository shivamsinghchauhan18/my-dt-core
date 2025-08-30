#!/usr/bin/env python3

import unittest
import time
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the src directory to the path so we can import the coordinator
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Mock rospy before importing coordinator_node
sys.modules['rospy'] = Mock()
sys.modules['psutil'] = Mock()
sys.modules['duckietown_msgs'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

from coordinator_node import BehaviorArbitrator, BehaviorRequest, BehaviorPriority, ArbitrationResult


class TestBehaviorArbitration(unittest.TestCase):
    """Test suite for advanced behavior arbitration logic"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock rospy to avoid ROS dependencies in unit tests
        self.rospy_patcher = patch('coordinator_node.rospy')
        self.mock_rospy = self.rospy_patcher.start()
        self.mock_rospy.loginfo = Mock()
        self.mock_rospy.logdebug = Mock()
        self.mock_rospy.logwarn = Mock()
        self.mock_rospy.logerr = Mock()
        
        # Create arbitrator instance
        self.arbitrator = BehaviorArbitrator("test_node")
        
        # Test behavior requests
        self.test_behaviors = [
            {
                'id': 'lane_following',
                'priority': BehaviorPriority.LANE_FOLLOWING.value,
                'confidence': 0.8
            },
            {
                'id': 'obstacle_avoidance',
                'priority': BehaviorPriority.OBSTACLE_AVOIDANCE.value,
                'confidence': 0.9
            },
            {
                'id': 'emergency_stop',
                'priority': BehaviorPriority.EMERGENCY_STOP.value,
                'confidence': 1.0
            },
            {
                'id': 'intersection_coordination',
                'priority': BehaviorPriority.INTERSECTION_COORDINATION.value,
                'confidence': 0.7
            }
        ]
    
    def tearDown(self):
        """Clean up test fixtures"""
        self.rospy_patcher.stop()
    
    def test_behavior_registration(self):
        """Test behavior registration functionality"""
        print("\n[TEST] Testing behavior registration...")
        
        # Test successful registration
        success = self.arbitrator.register_behavior(
            behavior_id="test_behavior",
            priority=500,
            source_node="test_node",
            metadata={'description': 'Test behavior'}
        )
        
        self.assertTrue(success)
        self.assertIn("test_behavior", self.arbitrator.registered_behaviors)
        
        # Verify registration details
        behavior_info = self.arbitrator.registered_behaviors["test_behavior"]
        self.assertEqual(behavior_info['priority'], 500)
        self.assertEqual(behavior_info['source_node'], "test_node")
        self.assertEqual(behavior_info['request_count'], 0)
        
        print("✓ Behavior registration test passed")
    
    def test_behavior_unregistration(self):
        """Test behavior unregistration functionality"""
        print("\n[TEST] Testing behavior unregistration...")
        
        # Register a behavior first
        self.arbitrator.register_behavior("temp_behavior", 400, "test_node")
        self.assertIn("temp_behavior", self.arbitrator.registered_behaviors)
        
        # Test successful unregistration
        success = self.arbitrator.unregister_behavior("temp_behavior")
        self.assertTrue(success)
        self.assertNotIn("temp_behavior", self.arbitrator.registered_behaviors)
        
        # Test unregistering non-existent behavior
        success = self.arbitrator.unregister_behavior("non_existent")
        self.assertFalse(success)
        
        print("✓ Behavior unregistration test passed")
    
    def test_priority_based_arbitration(self):
        """Test priority-based behavior arbitration"""
        print("\n[TEST] Testing priority-based arbitration...")
        
        # Register test behaviors
        for behavior in self.test_behaviors:
            self.arbitrator.register_behavior(
                behavior_id=behavior['id'],
                priority=behavior['priority'],
                source_node="test_node"
            )
        
        # Create behavior requests
        requests = []
        for behavior in self.test_behaviors:
            request = BehaviorRequest(
                behavior_id=behavior['id'],
                priority=behavior['priority'],
                command={'action': 'test'},
                timestamp=time.time(),
                source_node="test_node",
                confidence=behavior['confidence']
            )
            requests.append(request)
            self.arbitrator.submit_behavior_request(request)
        
        # Perform arbitration
        result = self.arbitrator.arbitrate_behaviors(strategy='priority_based')
        
        # Emergency stop should be selected (highest priority)
        self.assertIsNotNone(result.selected_behavior)
        self.assertEqual(result.selected_behavior.behavior_id, 'emergency_stop')
        self.assertEqual(len(result.rejected_behaviors), 3)
        self.assertTrue(result.conflict_resolution_applied)
        
        print("✓ Priority-based arbitration test passed")
    
    def test_confidence_weighted_arbitration(self):
        """Test confidence-weighted behavior arbitration"""
        print("\n[TEST] Testing confidence-weighted arbitration...")
        
        # Register behaviors with same priority but different confidence
        behaviors = [
            {'id': 'behavior_a', 'priority': 500, 'confidence': 0.9},
            {'id': 'behavior_b', 'priority': 500, 'confidence': 0.7},
            {'id': 'behavior_c', 'priority': 500, 'confidence': 0.8}
        ]
        
        for behavior in behaviors:
            self.arbitrator.register_behavior(
                behavior_id=behavior['id'],
                priority=behavior['priority'],
                source_node="test_node"
            )
            
            request = BehaviorRequest(
                behavior_id=behavior['id'],
                priority=behavior['priority'],
                command={'action': 'test'},
                timestamp=time.time(),
                source_node="test_node",
                confidence=behavior['confidence']
            )
            self.arbitrator.submit_behavior_request(request)
        
        # Perform confidence-weighted arbitration
        result = self.arbitrator.arbitrate_behaviors(strategy='confidence_weighted')
        
        # Behavior with highest confidence should be selected
        self.assertIsNotNone(result.selected_behavior)
        self.assertEqual(result.selected_behavior.behavior_id, 'behavior_a')
        self.assertEqual(result.selected_behavior.confidence, 0.9)
        
        print("✓ Confidence-weighted arbitration test passed")
    
    def test_safety_first_arbitration(self):
        """Test safety-first behavior arbitration"""
        print("\n[TEST] Testing safety-first arbitration...")
        
        # Register mixed safety and normal behaviors
        for behavior in self.test_behaviors:
            self.arbitrator.register_behavior(
                behavior_id=behavior['id'],
                priority=behavior['priority'],
                source_node="test_node"
            )
            
            request = BehaviorRequest(
                behavior_id=behavior['id'],
                priority=behavior['priority'],
                command={'action': 'test'},
                timestamp=time.time(),
                source_node="test_node",
                confidence=behavior['confidence']
            )
            self.arbitrator.submit_behavior_request(request)
        
        # Perform safety-first arbitration
        result = self.arbitrator.arbitrate_behaviors(strategy='safety_first')
        
        # Emergency stop (safety-critical) should be selected
        self.assertIsNotNone(result.selected_behavior)
        self.assertEqual(result.selected_behavior.behavior_id, 'emergency_stop')
        self.assertTrue(result.selected_behavior.priority >= BehaviorPriority.SAFETY_CRITICAL.value)
        
        print("✓ Safety-first arbitration test passed")
    
    def test_temporal_priority_arbitration(self):
        """Test temporal priority behavior arbitration"""
        print("\n[TEST] Testing temporal priority arbitration...")
        
        # Register behaviors
        self.arbitrator.register_behavior("recent_emergency", BehaviorPriority.SAFETY_CRITICAL.value, "test_node")
        self.arbitrator.register_behavior("old_normal", BehaviorPriority.LANE_FOLLOWING.value, "test_node")
        
        current_time = time.time()
        
        # Create requests with different timestamps
        recent_request = BehaviorRequest(
            behavior_id="recent_emergency",
            priority=BehaviorPriority.SAFETY_CRITICAL.value,
            command={'action': 'emergency'},
            timestamp=current_time,  # Recent
            source_node="test_node",
            confidence=0.9
        )
        
        old_request = BehaviorRequest(
            behavior_id="old_normal",
            priority=BehaviorPriority.LANE_FOLLOWING.value,
            command={'action': 'follow'},
            timestamp=current_time - 5.0,  # 5 seconds old
            source_node="test_node",
            confidence=1.0
        )
        
        self.arbitrator.submit_behavior_request(recent_request)
        self.arbitrator.submit_behavior_request(old_request)
        
        # Perform temporal priority arbitration
        result = self.arbitrator.arbitrate_behaviors(strategy='temporal_priority')
        
        # Recent emergency should be selected despite lower base confidence
        self.assertIsNotNone(result.selected_behavior)
        self.assertEqual(result.selected_behavior.behavior_id, 'recent_emergency')
        
        print("✓ Temporal priority arbitration test passed")
    
    def test_conflict_resolution(self):
        """Test conflict resolution mechanisms"""
        print("\n[TEST] Testing conflict resolution...")
        
        # Register multiple behaviors with same priority
        conflict_behaviors = [
            {'id': 'conflict_a', 'priority': 500, 'confidence': 0.8},
            {'id': 'conflict_b', 'priority': 500, 'confidence': 0.8},
            {'id': 'conflict_c', 'priority': 500, 'confidence': 0.8}
        ]
        
        for behavior in conflict_behaviors:
            self.arbitrator.register_behavior(
                behavior_id=behavior['id'],
                priority=behavior['priority'],
                source_node="test_node"
            )
            
            request = BehaviorRequest(
                behavior_id=behavior['id'],
                priority=behavior['priority'],
                command={'action': 'test'},
                timestamp=time.time(),
                source_node="test_node",
                confidence=behavior['confidence']
            )
            self.arbitrator.submit_behavior_request(request)
        
        # Perform arbitration
        result = self.arbitrator.arbitrate_behaviors()
        
        # Should detect and resolve conflict
        self.assertTrue(result.conflict_resolution_applied)
        self.assertIsNotNone(result.selected_behavior)
        self.assertEqual(len(result.rejected_behaviors), 2)
        
        print("✓ Conflict resolution test passed")
    
    def test_priority_update(self):
        """Test behavior priority updates"""
        print("\n[TEST] Testing behavior priority updates...")
        
        # Register behavior
        self.arbitrator.register_behavior("updateable_behavior", 400, "test_node")
        
        # Verify initial priority
        initial_priority = self.arbitrator.registered_behaviors["updateable_behavior"]['priority']
        self.assertEqual(initial_priority, 400)
        
        # Update priority
        success = self.arbitrator.update_behavior_priority("updateable_behavior", 600)
        self.assertTrue(success)
        
        # Verify updated priority
        updated_priority = self.arbitrator.registered_behaviors["updateable_behavior"]['priority']
        self.assertEqual(updated_priority, 600)
        
        # Test updating non-existent behavior
        success = self.arbitrator.update_behavior_priority("non_existent", 700)
        self.assertFalse(success)
        
        print("✓ Priority update test passed")
    
    def test_arbitration_statistics(self):
        """Test arbitration statistics collection"""
        print("\n[TEST] Testing arbitration statistics...")
        
        # Register and submit some behaviors
        self.arbitrator.register_behavior("stats_test", 500, "test_node")
        
        request = BehaviorRequest(
            behavior_id="stats_test",
            priority=500,
            command={'action': 'test'},
            timestamp=time.time(),
            source_node="test_node",
            confidence=0.8
        )
        self.arbitrator.submit_behavior_request(request)
        
        # Perform arbitration
        self.arbitrator.arbitrate_behaviors()
        
        # Get statistics
        stats = self.arbitrator.get_arbitration_statistics()
        
        # Verify statistics structure
        self.assertIn('total_arbitrations', stats)
        self.assertIn('total_conflicts', stats)
        self.assertIn('average_arbitration_time', stats)
        self.assertIn('registered_behaviors', stats)
        self.assertIn('behavior_statistics', stats)
        
        # Verify values
        self.assertEqual(stats['total_arbitrations'], 1)
        self.assertEqual(stats['registered_behaviors'], 1)
        self.assertIn('stats_test', stats['behavior_statistics'])
        
        print("✓ Arbitration statistics test passed")
    
    def test_no_requests_arbitration(self):
        """Test arbitration with no active requests"""
        print("\n[TEST] Testing arbitration with no requests...")
        
        # Perform arbitration with no requests
        result = self.arbitrator.arbitrate_behaviors()
        
        # Should return empty result
        self.assertIsNone(result.selected_behavior)
        self.assertEqual(len(result.rejected_behaviors), 0)
        self.assertFalse(result.conflict_resolution_applied)
        self.assertEqual(result.reason, "No active behavior requests")
        
        print("✓ No requests arbitration test passed")
    
    def test_unregistered_behavior_request(self):
        """Test submitting request for unregistered behavior"""
        print("\n[TEST] Testing unregistered behavior request...")
        
        # Create request for unregistered behavior
        request = BehaviorRequest(
            behavior_id="unregistered_behavior",
            priority=500,
            command={'action': 'test'},
            timestamp=time.time(),
            source_node="test_node",
            confidence=0.8
        )
        
        # Should reject the request
        success = self.arbitrator.submit_behavior_request(request)
        self.assertFalse(success)
        
        print("✓ Unregistered behavior request test passed")


def run_arbitration_tests():
    """Run all behavior arbitration tests"""
    print("=" * 60)
    print("RUNNING BEHAVIOR ARBITRATION TESTS")
    print("=" * 60)
    
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestBehaviorArbitration)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "=" * 60)
    print("BEHAVIOR ARBITRATION TEST SUMMARY")
    print("=" * 60)
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
    run_arbitration_tests()
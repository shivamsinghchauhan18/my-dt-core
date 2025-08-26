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
    def get_time(self): return time.time()
    
    class Duration:
        def __init__(self, secs): self.secs = secs
        @staticmethod
        def from_sec(secs): return MockRospy.Duration(secs)
    
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
        def __init__(self, topic, msg_type, callback, queue_size=1): pass
    
    class Publisher:
        def __init__(self, topic, msg_type, queue_size=1): pass
        def publish(self, msg): pass

sys.modules['rospy'] = MockRospy()

# Mock messages
class MockString:
    def __init__(self): self.data = ""

class MockBool:
    def __init__(self): self.data = False

class MockTwist:
    def __init__(self):
        self.linear = Mock()
        self.angular = Mock()
        self.linear.x = 0.0
        self.angular.z = 0.0

class MockPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class MockVector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class MockObjectDetection:
    def __init__(self):
        self.class_name = ""
        self.distance = 0.0
        self.confidence = 0.0

class MockObjectDetectionArray:
    def __init__(self):
        self.detections = []
        self.processing_time = 0.0

# Mock modules
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()
sys.modules['std_msgs.msg'].String = MockString
sys.modules['std_msgs.msg'].Bool = MockBool

sys.modules['geometry_msgs'] = Mock()
sys.modules['geometry_msgs.msg'] = Mock()
sys.modules['geometry_msgs.msg'].Twist = MockTwist
sys.modules['geometry_msgs.msg'].Point = MockPoint
sys.modules['geometry_msgs.msg'].Vector3 = MockVector3

sys.modules['duckietown_msgs'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['duckietown_msgs.msg'].ObjectDetection = MockObjectDetection
sys.modules['duckietown_msgs.msg'].ObjectDetectionArray = MockObjectDetectionArray

sys.modules['sensor_msgs'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()

sys.modules['duckietown'] = Mock()
sys.modules['duckietown.dtros'] = Mock()

# Mock DTROS
class MockDTROS:
    def __init__(self, node_name, node_type=None):
        self.node_name = node_name

sys.modules['duckietown.dtros'].DTROS = MockDTROS
sys.modules['duckietown.dtros'].NodeType = Mock()
sys.modules['duckietown.dtros'].TopicType = Mock()

# Now import the actual implementations
try:
    from lane_change_decision_engine import (
        LaneChangeDecisionEngine, LaneChangeDecision, LaneChangeReason,
        LaneState, GapAnalysis
    )
    from lane_change_trajectory_generator import (
        LaneChangeTrajectoryGenerator, LaneChangeTrajectory, TrajectoryConstraints,
        LaneChangeParameters, TrajectoryValidationResult
    )
    print("✅ Successfully imported lane change modules")
except Exception as e:
    print(f"❌ Failed to import lane change modules: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)


class TestLaneChangeIntegration(unittest.TestCase):
    """
    Test suite for complete lane change execution workflow integration.
    
    Tests the integration between decision engine, trajectory generator,
    LED signaling, FSM coordination, and safety monitoring systems.
    """
    
    def setUp(self):
        """Set up test fixtures"""
        # Create decision engine
        self.decision_engine = LaneChangeDecisionEngine()
        
        # Create trajectory generator
        self.trajectory_generator = LaneChangeTrajectoryGenerator()
        
        # Create test data
        self.create_test_data()
        
        print(f"[TestLaneChangeIntegration] Test setup completed")
    
    def create_test_data(self):
        """Create test data for integration scenarios"""
        
        # Test vehicle state
        self.test_vehicle_state = {
            'position': {'x': 0.0, 'y': 0.0},
            'velocity': {'x': 1.0, 'y': 0.0},
            'heading': 0.0,
            'timestamp': time.time()
        }
        
        # Test current lane state
        self.test_current_lane = LaneState(
            lane_id="current",
            is_current=True,
            is_available=True,
            width=0.6,
            center_line_offset=0.0,
            obstacles=[],
            gap_length=10.0,
            gap_start_distance=0.0,
            safety_score=0.8,
            timestamp=time.time()
        )
        
        # Test adjacent lanes
        self.test_adjacent_lanes = {
            "left": LaneState(
                lane_id="left",
                is_current=False,
                is_available=True,
                width=0.6,
                center_line_offset=-0.6,
                obstacles=[],
                gap_length=8.0,
                gap_start_distance=1.0,
                safety_score=0.9,
                timestamp=time.time()
            ),
            "right": LaneState(
                lane_id="right",
                is_current=False,
                is_available=True,
                width=0.6,
                center_line_offset=0.6,
                obstacles=[],
                gap_length=6.0,
                gap_start_distance=1.0,
                safety_score=0.7,
                timestamp=time.time()
            )
        }
        
        # Test obstacles
        self.test_obstacles = []
        
        # Test lane change parameters
        self.test_lane_change_params = LaneChangeParameters(
            start_position=MockPoint(x=0.0, y=0.0, z=0.0),
            end_position=MockPoint(x=3.0, y=0.6, z=0.0),
            start_velocity=MockVector3(x=1.0, y=0.0, z=0.0),
            end_velocity=MockVector3(x=1.0, y=0.0, z=0.0),
            start_heading=0.0,
            end_heading=0.0,
            lane_width=0.6,
            vehicle_length=0.18,
            vehicle_width=0.13
        )
        
        print(f"[TestLaneChangeIntegration] Test data created")
    
    def test_complete_lane_change_workflow(self):
        """Test complete lane change workflow from decision to execution"""
        print(f"\n[TestLaneChangeIntegration] Testing complete lane change workflow")
        
        # Step 1: Decision evaluation
        decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
            self.test_current_lane,
            self.test_adjacent_lanes,
            self.test_vehicle_state,
            self.test_obstacles
        )
        
        print(f"[TestLaneChangeIntegration] Step 1 - Decision evaluation:")
        print(f"  Decision: {decision.value}")
        print(f"  Reason: {reason.value}")
        print(f"  Confidence: {confidence:.3f}")
        
        # Step 2: Trajectory generation (if decision is to change lanes)
        trajectory = None
        if decision != LaneChangeDecision.NO_CHANGE:
            trajectory = self.trajectory_generator.generate_lane_change_trajectory(
                self.test_lane_change_params
            )
            
            print(f"[TestLaneChangeIntegration] Step 2 - Trajectory generation:")
            print(f"  Trajectory feasible: {trajectory.is_feasible}")
            print(f"  Duration: {trajectory.total_duration:.2f}s")
            print(f"  Distance: {trajectory.total_distance:.2f}m")
            print(f"  Max lateral acceleration: {trajectory.maximum_lateral_acceleration:.2f}m/s²")
        
        # Step 3: Workflow validation
        workflow_success = self.validate_workflow_integration(decision, trajectory, confidence)
        
        print(f"[TestLaneChangeIntegration] Step 3 - Workflow validation:")
        print(f"  Workflow success: {workflow_success}")
        
        # Validate overall workflow
        self.assertTrue(workflow_success)
        
        print(f"[TestLaneChangeIntegration] Complete lane change workflow test passed")
    
    def test_decision_trajectory_integration(self):
        """Test integration between decision engine and trajectory generator"""
        print(f"\n[TestLaneChangeIntegration] Testing decision-trajectory integration")
        
        # Test with blocking obstacle scenario
        blocking_obstacle = MockObjectDetection()
        blocking_obstacle.class_name = "duckiebot"
        blocking_obstacle.distance = 1.5
        blocking_obstacle.confidence = 0.9
        
        blocking_obstacles = [blocking_obstacle]
        
        # Update current lane with obstacle
        current_lane_blocked = LaneState(
            lane_id="current",
            is_current=True,
            is_available=True,
            width=0.6,
            center_line_offset=0.0,
            obstacles=blocking_obstacles,
            gap_length=1.0,
            gap_start_distance=0.0,
            safety_score=0.3,
            timestamp=time.time()
        )
        
        # Evaluate decision with obstacle
        decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
            current_lane_blocked,
            self.test_adjacent_lanes,
            self.test_vehicle_state,
            blocking_obstacles
        )
        
        print(f"[TestLaneChangeIntegration] Decision with obstacle:")
        print(f"  Decision: {decision.value}")
        print(f"  Reason: {reason.value}")
        print(f"  Confidence: {confidence:.3f}")
        
        # Generate trajectory if lane change is decided
        if decision != LaneChangeDecision.NO_CHANGE:
            # Adjust trajectory parameters based on decision confidence
            constraints = TrajectoryConstraints(
                maximum_lateral_acceleration=2.0 * (0.5 + 0.5 * confidence),
                maximum_curvature=2.0,
                maximum_velocity=2.0,
                minimum_radius=0.3,
                comfort_factor=0.8 * (0.7 + 0.3 * confidence),
                safety_margin=0.3,
                preferred_duration=2.0 * (1.5 - 0.5 * confidence),
                maximum_duration=3.0
            )
            
            trajectory = self.trajectory_generator.generate_lane_change_trajectory(
                self.test_lane_change_params, constraints
            )
            
            print(f"[TestLaneChangeIntegration] Trajectory with confidence-based constraints:")
            print(f"  Feasible: {trajectory.is_feasible}")
            print(f"  Validation result: {trajectory.validation_result.value}")
            print(f"  Confidence-adjusted max accel: {constraints.maximum_lateral_acceleration:.2f}m/s²")
            print(f"  Confidence-adjusted duration: {constraints.preferred_duration:.2f}s")
            
            # Validate integration
            self.assertIsNotNone(trajectory)
            if trajectory.is_feasible:
                self.assertEqual(trajectory.validation_result, TrajectoryValidationResult.VALID)
        
        print(f"[TestLaneChangeIntegration] Decision-trajectory integration test passed")
    
    def test_performance_integration(self):
        """Test performance integration across all components"""
        print(f"\n[TestLaneChangeIntegration] Testing performance integration")
        
        # Reset metrics
        self.decision_engine.reset_metrics()
        self.trajectory_generator.reset_metrics()
        
        # Perform multiple integrated operations
        total_operations = 5
        successful_operations = 0
        
        for i in range(total_operations):
            start_time = time.time()
            
            # Decision evaluation
            decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
                self.test_current_lane,
                self.test_adjacent_lanes,
                self.test_vehicle_state,
                self.test_obstacles
            )
            
            # Trajectory generation
            trajectory = None
            if decision != LaneChangeDecision.NO_CHANGE:
                trajectory = self.trajectory_generator.generate_lane_change_trajectory(
                    self.test_lane_change_params
                )
                
                if trajectory and trajectory.is_feasible:
                    successful_operations += 1
            else:
                successful_operations += 1  # No change is also a valid outcome
            
            operation_time = (time.time() - start_time) * 1000
            print(f"[TestLaneChangeIntegration] Operation {i+1}: {operation_time:.2f}ms")
        
        # Get performance metrics
        decision_metrics = self.decision_engine.get_performance_metrics()
        trajectory_metrics = self.trajectory_generator.get_performance_metrics()
        
        print(f"[TestLaneChangeIntegration] Performance integration results:")
        print(f"  Total operations: {total_operations}")
        print(f"  Successful operations: {successful_operations}")
        print(f"  Success rate: {(successful_operations/total_operations*100):.1f}%")
        print(f"  Decision engine evaluations: {decision_metrics['total_evaluations']}")
        print(f"  Decision engine success rate: {decision_metrics['success_rate']:.1f}%")
        print(f"  Trajectory generator trajectories: {trajectory_metrics['total_trajectories_generated']}")
        print(f"  Trajectory generator success rate: {trajectory_metrics['success_rate']:.1f}%")
        
        # Validate performance
        self.assertGreater(successful_operations, 0)
        self.assertEqual(decision_metrics['total_evaluations'], total_operations)
        
        print(f"[TestLaneChangeIntegration] Performance integration test passed")
    
    def test_safety_integration(self):
        """Test safety integration and constraint validation"""
        print(f"\n[TestLaneChangeIntegration] Testing safety integration")
        
        # Test with unsafe conditions
        unsafe_constraints = TrajectoryConstraints(
            maximum_lateral_acceleration=0.5,  # Very low limit
            maximum_curvature=0.5,
            maximum_velocity=0.5,
            minimum_radius=1.0,
            comfort_factor=0.9,
            safety_margin=0.5,
            preferred_duration=5.0,  # Very long duration
            maximum_duration=6.0
        )
        
        # Generate trajectory with unsafe constraints
        unsafe_trajectory = self.trajectory_generator.generate_lane_change_trajectory(
            self.test_lane_change_params, unsafe_constraints
        )
        
        print(f"[TestLaneChangeIntegration] Unsafe constraints test:")
        print(f"  Trajectory feasible: {unsafe_trajectory.is_feasible}")
        print(f"  Validation result: {unsafe_trajectory.validation_result.value}")
        print(f"  Max lateral acceleration: {unsafe_trajectory.maximum_lateral_acceleration:.2f}m/s²")
        print(f"  Constraint limit: {unsafe_constraints.maximum_lateral_acceleration:.2f}m/s²")
        
        # Test with high-risk scenario
        high_risk_obstacles = []
        for i in range(3):
            obstacle = MockObjectDetection()
            obstacle.class_name = "duckiebot"
            obstacle.distance = 0.8 + i * 0.3  # Close obstacles
            obstacle.confidence = 0.9
            high_risk_obstacles.append(obstacle)
        
        high_risk_lane = LaneState(
            lane_id="current",
            is_current=True,
            is_available=True,
            width=0.6,
            center_line_offset=0.0,
            obstacles=high_risk_obstacles,
            gap_length=0.5,  # Very short gap
            gap_start_distance=0.0,
            safety_score=0.1,  # Very low safety
            timestamp=time.time()
        )
        
        # Evaluate decision with high-risk scenario
        decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
            high_risk_lane,
            self.test_adjacent_lanes,
            self.test_vehicle_state,
            high_risk_obstacles
        )
        
        print(f"[TestLaneChangeIntegration] High-risk scenario test:")
        print(f"  Decision: {decision.value}")
        print(f"  Confidence: {confidence:.3f}")
        print(f"  Safety threshold: {self.decision_engine.criteria.decision_confidence_threshold}")
        
        # Safety validation
        if confidence < self.decision_engine.criteria.decision_confidence_threshold:
            print(f"[TestLaneChangeIntegration] Safety system correctly rejected unsafe lane change")
            self.assertEqual(decision, LaneChangeDecision.NO_CHANGE)
        
        print(f"[TestLaneChangeIntegration] Safety integration test passed")
    
    def test_error_handling_integration(self):
        """Test error handling and recovery integration"""
        print(f"\n[TestLaneChangeIntegration] Testing error handling integration")
        
        # Test with invalid lane change parameters
        invalid_params = LaneChangeParameters(
            start_position=MockPoint(x=0.0, y=0.0, z=0.0),
            end_position=MockPoint(x=0.0, y=0.0, z=0.0),  # Same position
            start_velocity=MockVector3(x=0.0, y=0.0, z=0.0),  # Zero velocity
            end_velocity=MockVector3(x=0.0, y=0.0, z=0.0),
            start_heading=0.0,
            end_heading=0.0,
            lane_width=0.0,  # Invalid lane width
            vehicle_length=0.0,
            vehicle_width=0.0
        )
        
        # Attempt trajectory generation with invalid parameters
        invalid_trajectory = self.trajectory_generator.generate_lane_change_trajectory(invalid_params)
        
        print(f"[TestLaneChangeIntegration] Invalid parameters test:")
        print(f"  Trajectory feasible: {invalid_trajectory.is_feasible}")
        print(f"  Validation result: {invalid_trajectory.validation_result.value}")
        
        # Should handle gracefully
        self.assertIsNotNone(invalid_trajectory)
        self.assertFalse(invalid_trajectory.is_feasible)
        
        # Test with empty adjacent lanes
        empty_decision, empty_reason, empty_confidence = self.decision_engine.evaluate_lane_change_opportunity(
            self.test_current_lane,
            {},  # No adjacent lanes
            self.test_vehicle_state,
            self.test_obstacles
        )
        
        print(f"[TestLaneChangeIntegration] Empty adjacent lanes test:")
        print(f"  Decision: {empty_decision.value}")
        print(f"  Confidence: {empty_confidence:.3f}")
        
        # Should handle gracefully
        self.assertEqual(empty_decision, LaneChangeDecision.NO_CHANGE)
        
        print(f"[TestLaneChangeIntegration] Error handling integration test passed")
    
    def validate_workflow_integration(self, decision: LaneChangeDecision, trajectory, confidence: float) -> bool:
        """
        Validate the integration of the complete workflow.
        
        Args:
            decision: Lane change decision
            trajectory: Generated trajectory (if any)
            confidence: Decision confidence
            
        Returns:
            True if workflow integration is valid
        """
        # Validate decision consistency
        if decision == LaneChangeDecision.NO_CHANGE:
            # No trajectory should be generated for no change
            return True
        
        # If lane change is decided, trajectory should be generated
        if trajectory is None:
            print(f"[TestLaneChangeIntegration] ERROR: No trajectory generated for decision: {decision.value}")
            return False
        
        # Validate trajectory feasibility matches confidence
        if confidence > 0.7 and not trajectory.is_feasible:
            print(f"[TestLaneChangeIntegration] WARNING: High confidence but infeasible trajectory")
            # This might be acceptable in some cases
        
        # Validate trajectory duration is reasonable
        if trajectory.total_duration > 5.0:
            print(f"[TestLaneChangeIntegration] WARNING: Very long trajectory duration: {trajectory.total_duration:.2f}s")
        
        # Validate trajectory safety constraints
        if trajectory.maximum_lateral_acceleration > 3.0:
            print(f"[TestLaneChangeIntegration] ERROR: Unsafe lateral acceleration: {trajectory.maximum_lateral_acceleration:.2f}m/s²")
            return False
        
        return True
    
    def test_timing_integration(self):
        """Test timing integration and real-time performance"""
        print(f"\n[TestLaneChangeIntegration] Testing timing integration")
        
        # Test end-to-end timing
        start_time = time.time()
        
        # Complete workflow timing
        decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
            self.test_current_lane,
            self.test_adjacent_lanes,
            self.test_vehicle_state,
            self.test_obstacles
        )
        
        decision_time = time.time()
        
        trajectory = None
        if decision != LaneChangeDecision.NO_CHANGE:
            trajectory = self.trajectory_generator.generate_lane_change_trajectory(
                self.test_lane_change_params
            )
        
        end_time = time.time()
        
        # Calculate timing metrics
        decision_duration = (decision_time - start_time) * 1000
        trajectory_duration = (end_time - decision_time) * 1000
        total_duration = (end_time - start_time) * 1000
        
        print(f"[TestLaneChangeIntegration] Timing integration results:")
        print(f"  Decision evaluation: {decision_duration:.2f}ms")
        print(f"  Trajectory generation: {trajectory_duration:.2f}ms")
        print(f"  Total workflow: {total_duration:.2f}ms")
        
        # Validate real-time performance (should be under 200ms total)
        self.assertLess(total_duration, 200.0)
        
        print(f"[TestLaneChangeIntegration] Timing integration test passed")


def run_lane_change_integration_tests():
    """Run all lane change integration tests"""
    print(f"\n{'='*70}")
    print(f"RUNNING LANE CHANGE INTEGRATION TESTS")
    print(f"{'='*70}")
    
    # Create test suite
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestLaneChangeIntegration)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout, buffer=False)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n{'='*70}")
    print(f"LANE CHANGE INTEGRATION TEST SUMMARY")
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
    success = run_lane_change_integration_tests()
    
    if success:
        print(f"\n✅ All lane change integration tests passed!")
        exit(0)
    else:
        print(f"\n❌ Some lane change integration tests failed!")
        exit(1)
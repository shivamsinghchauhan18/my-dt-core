#!/usr/bin/env python3

import unittest
import time
import math
import numpy as np
from unittest.mock import Mock, patch

import rospy
from geometry_msgs.msg import Point, Vector3, Twist
from duckietown_enhanced_msgs.msg import ObjectDetection

# Import the modules to test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from avoidance_planner import (
    AvoidancePlanner, AvoidanceStrategy, AvoidanceState, 
    AvoidanceTrajectory, AvoidanceMetrics
)
from risk_assessment_engine import RiskLevel, RiskFactors, VehicleState


class TestAvoidanceStrategySelection(unittest.TestCase):
    """
    Test suite for avoidance strategy selection logic.
    
    Tests the core requirement 3.3: "WHEN collision risk is high THEN the system 
    SHALL select an appropriate avoidance strategy (stop, slow, swerve)"
    """
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock rospy to avoid ROS dependencies in unit tests
        rospy.loginfo = Mock()
        rospy.logwarn = Mock()
        rospy.logerr = Mock()
        rospy.logdebug = Mock()
        
        self.planner = AvoidancePlanner()
        
        # Create standard test vehicle state
        self.test_vehicle_state = VehicleState(
            position=Point(x=0.0, y=0.0, z=0.0),
            velocity=Vector3(x=0.8, y=0.0, z=0.0),
            heading=0.0,
            timestamp=time.time()
        )
        
        # Create standard test detection
        self.test_detection = ObjectDetection()
        self.test_detection.class_name = "duckiebot"
        self.test_detection.confidence = 0.8
        self.test_detection.distance = 0.6
        self.test_detection.relative_velocity = Vector3(x=-0.5, y=0.0, z=0.0)
    
    def create_risk_factors(self, distance_risk=0.5, velocity_risk=0.5, 
                          object_type_risk=0.5, trajectory_risk=0.5,
                          ttc=3.0, lateral_clearance=0.5):
        """Helper method to create risk factors for testing."""
        return RiskFactors(
            distance_risk=distance_risk,
            velocity_risk=velocity_risk,
            object_type_risk=object_type_risk,
            trajectory_risk=trajectory_risk,
            time_to_collision=ttc,
            lateral_clearance=lateral_clearance
        )
    
    def test_stop_strategy_selection_critical_distance(self):
        """Test that STOP strategy is selected for critical distances."""
        print("\n=== Testing STOP strategy selection for critical distance ===")
        
        # Create detection at critical distance
        detection = self.test_detection
        detection.distance = 0.2  # Below critical threshold (0.3m)
        
        risk_factors = self.create_risk_factors(
            distance_risk=0.9,
            ttc=2.0,
            lateral_clearance=0.4
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.CRITICAL)]
        
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        self.assertEqual(strategy, AvoidanceStrategy.STOP)
        self.assertIsNotNone(trajectory)
        self.assertEqual(trajectory.strategy, AvoidanceStrategy.STOP)
        
        print(f"✓ STOP strategy correctly selected for distance {detection.distance}m")
        print(f"✓ Trajectory generated with {len(trajectory.waypoints)} waypoints")
        print(f"✓ Trajectory duration: {trajectory.total_duration:.2f}s")
    
    def test_stop_strategy_selection_critical_ttc(self):
        """Test that STOP strategy is selected for critical time-to-collision."""
        print("\n=== Testing STOP strategy selection for critical TTC ===")
        
        detection = self.test_detection
        detection.distance = 0.8  # Above critical distance threshold
        
        risk_factors = self.create_risk_factors(
            distance_risk=0.6,
            ttc=0.8,  # Below critical TTC threshold (1.0s)
            lateral_clearance=0.4
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.CRITICAL)]
        
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        self.assertEqual(strategy, AvoidanceStrategy.STOP)
        self.assertIsNotNone(trajectory)
        
        print(f"✓ STOP strategy correctly selected for TTC {risk_factors.time_to_collision:.2f}s")
        print(f"✓ Trajectory safety margin: {trajectory.safety_margin:.2f}m")
    
    def test_swerve_strategy_selection(self):
        """Test that SWERVE strategy is selected for appropriate conditions."""
        print("\n=== Testing SWERVE strategy selection ===")
        
        detection = self.test_detection
        detection.distance = 0.6  # Safe distance for swerving
        
        risk_factors = self.create_risk_factors(
            distance_risk=0.5,
            ttc=2.0,
            lateral_clearance=0.3  # Below lateral safety margin
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.HIGH)]
        
        # Mock the swerve safety check to return True
        with patch.object(self.planner, '_can_swerve_safely', return_value=True):
            strategy, trajectory = self.planner.plan_avoidance(
                risk_assessments, self.test_vehicle_state
            )
        
        self.assertEqual(strategy, AvoidanceStrategy.SWERVE)
        self.assertIsNotNone(trajectory)
        self.assertEqual(trajectory.strategy, AvoidanceStrategy.SWERVE)
        
        print(f"✓ SWERVE strategy correctly selected")
        print(f"✓ Lateral clearance: {risk_factors.lateral_clearance:.2f}m")
        print(f"✓ Trajectory duration: {trajectory.total_duration:.2f}s")
    
    def test_slow_strategy_selection(self):
        """Test that SLOW strategy is selected for moderate risk conditions."""
        print("\n=== Testing SLOW strategy selection ===")
        
        detection = self.test_detection
        detection.distance = 0.7  # Within warning threshold
        
        risk_factors = self.create_risk_factors(
            distance_risk=0.4,
            ttc=2.5,
            lateral_clearance=0.8  # Good lateral clearance
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.HIGH)]
        
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        self.assertEqual(strategy, AvoidanceStrategy.SLOW)
        self.assertIsNotNone(trajectory)
        self.assertEqual(trajectory.strategy, AvoidanceStrategy.SLOW)
        
        print(f"✓ SLOW strategy correctly selected")
        print(f"✓ Distance: {detection.distance:.2f}m (warning threshold: {self.planner.warning_distance_threshold:.2f}m)")
        print(f"✓ Target speed reduction factor: {self.planner.slow_speed_factor}")
    
    def test_no_avoidance_for_safe_conditions(self):
        """Test that no avoidance is triggered for safe conditions."""
        print("\n=== Testing no avoidance for safe conditions ===")
        
        detection = self.test_detection
        detection.distance = 2.0  # Safe distance
        
        risk_factors = self.create_risk_factors(
            distance_risk=0.1,
            ttc=5.0,
            lateral_clearance=1.0
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.LOW)]
        
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        self.assertEqual(strategy, AvoidanceStrategy.NONE)
        self.assertIsNone(trajectory)
        
        print(f"✓ No avoidance correctly selected for safe conditions")
        print(f"✓ Distance: {detection.distance:.2f}m (safe threshold: {self.planner.safe_distance_threshold:.2f}m)")
    
    def test_multiple_objects_strategy_selection(self):
        """Test strategy selection with multiple high-risk objects."""
        print("\n=== Testing strategy selection with multiple objects ===")
        
        # Create multiple detections
        detection1 = self.test_detection
        detection1.distance = 0.4
        detection1.class_name = "duckiebot"
        
        detection2 = ObjectDetection()
        detection2.class_name = "cone"
        detection2.confidence = 0.7
        detection2.distance = 0.6
        detection2.relative_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        
        risk_factors1 = self.create_risk_factors(
            distance_risk=0.8,
            ttc=1.5,
            lateral_clearance=0.3
        )
        
        risk_factors2 = self.create_risk_factors(
            distance_risk=0.6,
            ttc=3.0,
            lateral_clearance=0.5
        )
        
        risk_assessments = [
            (detection1, risk_factors1, RiskLevel.HIGH),
            (detection2, risk_factors2, RiskLevel.HIGH)
        ]
        
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        # Should select SLOW or STOP based on closest object
        self.assertIn(strategy, [AvoidanceStrategy.SLOW, AvoidanceStrategy.STOP])
        self.assertIsNotNone(trajectory)
        
        print(f"✓ Strategy selected for multiple objects: {strategy.value}")
        print(f"✓ Closest object distance: {min(detection1.distance, detection2.distance):.2f}m")
    
    def test_strategy_priority_critical_overrides_high(self):
        """Test that critical risk objects override high risk objects in strategy selection."""
        print("\n=== Testing strategy priority: critical overrides high ===")
        
        # High risk object at moderate distance
        detection1 = self.test_detection
        detection1.distance = 0.7
        risk_factors1 = self.create_risk_factors(ttc=2.0)
        
        # Critical risk object at closer distance
        detection2 = ObjectDetection()
        detection2.class_name = "barrier"
        detection2.confidence = 0.9
        detection2.distance = 0.25  # Critical distance
        detection2.relative_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        risk_factors2 = self.create_risk_factors(distance_risk=0.95, ttc=0.8)
        
        risk_assessments = [
            (detection1, risk_factors1, RiskLevel.HIGH),
            (detection2, risk_factors2, RiskLevel.CRITICAL)
        ]
        
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        self.assertEqual(strategy, AvoidanceStrategy.STOP)
        print(f"✓ STOP strategy correctly prioritized over other strategies")
        print(f"✓ Critical object distance: {detection2.distance:.2f}m")


class TestAvoidanceTrajectoryGeneration(unittest.TestCase):
    """
    Test suite for avoidance trajectory generation.
    
    Tests trajectory generation for different strategies and validates
    safety requirements.
    """
    
    def setUp(self):
        """Set up test fixtures."""
        rospy.loginfo = Mock()
        rospy.logwarn = Mock()
        rospy.logerr = Mock()
        rospy.logdebug = Mock()
        
        self.planner = AvoidancePlanner()
        
        self.test_vehicle_state = VehicleState(
            position=Point(x=0.0, y=0.0, z=0.0),
            velocity=Vector3(x=0.8, y=0.0, z=0.0),
            heading=0.0,
            timestamp=time.time()
        )
    
    def test_stop_trajectory_generation(self):
        """Test generation of stop trajectory."""
        print("\n=== Testing STOP trajectory generation ===")
        
        trajectory = self.planner._generate_stop_trajectory(
            self.test_vehicle_state, time.time()
        )
        
        self.assertIsNotNone(trajectory)
        self.assertEqual(trajectory.strategy, AvoidanceStrategy.STOP)
        self.assertGreater(len(trajectory.waypoints), 0)
        self.assertGreater(trajectory.total_duration, 0)
        
        # Check that trajectory ends with zero velocity
        self.assertAlmostEqual(trajectory.velocities[-1], 0.0, places=2)
        
        # Check that all velocities are non-negative and decreasing
        for i in range(1, len(trajectory.velocities)):
            self.assertGreaterEqual(trajectory.velocities[i-1], trajectory.velocities[i])
            self.assertGreaterEqual(trajectory.velocities[i], 0.0)
        
        print(f"✓ Stop trajectory generated with {len(trajectory.waypoints)} waypoints")
        print(f"✓ Duration: {trajectory.total_duration:.2f}s")
        print(f"✓ Final velocity: {trajectory.velocities[-1]:.3f} m/s")
    
    def test_slow_trajectory_generation(self):
        """Test generation of slow trajectory."""
        print("\n=== Testing SLOW trajectory generation ===")
        
        high_risk_objects = []  # Empty for this test
        
        trajectory = self.planner._generate_slow_trajectory(
            self.test_vehicle_state, high_risk_objects, time.time()
        )
        
        self.assertIsNotNone(trajectory)
        self.assertEqual(trajectory.strategy, AvoidanceStrategy.SLOW)
        self.assertGreater(len(trajectory.waypoints), 0)
        
        # Check that trajectory reduces speed
        initial_speed = self.test_vehicle_state.velocity.x
        target_speed = initial_speed * self.planner.slow_speed_factor
        
        # Final velocity should be close to target speed
        final_velocity = trajectory.velocities[-1]
        self.assertAlmostEqual(final_velocity, target_speed, delta=0.1)
        
        print(f"✓ Slow trajectory generated with {len(trajectory.waypoints)} waypoints")
        print(f"✓ Initial speed: {initial_speed:.2f} m/s")
        print(f"✓ Target speed: {target_speed:.2f} m/s")
        print(f"✓ Final speed: {final_velocity:.2f} m/s")
    
    def test_swerve_trajectory_generation(self):
        """Test generation of swerve trajectory."""
        print("\n=== Testing SWERVE trajectory generation ===")
        
        high_risk_objects = []  # Empty for this test
        
        trajectory = self.planner._generate_swerve_trajectory(
            self.test_vehicle_state, high_risk_objects, None, time.time()
        )
        
        self.assertIsNotNone(trajectory)
        self.assertEqual(trajectory.strategy, AvoidanceStrategy.SWERVE)
        self.assertGreater(len(trajectory.waypoints), 0)
        
        # Check that trajectory has lateral motion
        y_positions = [wp.y for wp in trajectory.waypoints]
        max_lateral_offset = max(abs(y) for y in y_positions)
        
        self.assertGreater(max_lateral_offset, 0.1)  # Should have significant lateral motion
        
        # Check that trajectory returns to center
        self.assertAlmostEqual(y_positions[-1], 0.0, delta=0.1)
        
        # Check angular velocities are within limits
        for angular_vel in trajectory.angular_velocities:
            self.assertLessEqual(abs(angular_vel), self.planner.max_angular_velocity)
        
        print(f"✓ Swerve trajectory generated with {len(trajectory.waypoints)} waypoints")
        print(f"✓ Max lateral offset: {max_lateral_offset:.2f}m")
        print(f"✓ Final lateral position: {y_positions[-1]:.3f}m")
        print(f"✓ Max angular velocity: {max(abs(av) for av in trajectory.angular_velocities):.2f} rad/s")
    
    def test_trajectory_safety_validation(self):
        """Test trajectory safety validation against obstacles."""
        print("\n=== Testing trajectory safety validation ===")
        
        # Create a simple straight trajectory
        trajectory = AvoidanceTrajectory(
            waypoints=[
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=0.5, y=0.0, z=0.0),
                Point(x=1.0, y=0.0, z=0.0)
            ],
            velocities=[0.8, 0.6, 0.4],
            angular_velocities=[0.0, 0.0, 0.0],
            timestamps=[time.time(), time.time() + 0.5, time.time() + 1.0],
            total_duration=1.0,
            safety_margin=0.5,
            strategy=AvoidanceStrategy.SLOW
        )
        
        # Create obstacle far from trajectory (safe)
        detection = ObjectDetection()
        detection.distance = 2.0
        detection.relative_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        
        risk_factors = RiskFactors(
            distance_risk=0.1, velocity_risk=0.1, object_type_risk=0.5,
            trajectory_risk=0.1, time_to_collision=10.0, lateral_clearance=1.0
        )
        
        high_risk_objects = [(detection, risk_factors, RiskLevel.HIGH)]
        
        is_safe = self.planner._validate_trajectory_safety(
            trajectory, high_risk_objects, time.time()
        )
        
        self.assertTrue(is_safe)
        print(f"✓ Safe trajectory correctly validated as safe")
        
        # Test with obstacle too close (unsafe)
        detection.distance = 0.2  # Very close
        risk_factors.lateral_clearance = 0.1  # Too close
        
        is_safe = self.planner._validate_trajectory_safety(
            trajectory, high_risk_objects, time.time()
        )
        
        self.assertFalse(is_safe)
        print(f"✓ Unsafe trajectory correctly identified as unsafe")


class TestAvoidanceExecution(unittest.TestCase):
    """
    Test suite for avoidance trajectory execution.
    
    Tests the execution of avoidance maneuvers and requirement 3.4:
    "WHEN executing avoidance maneuvers THEN the system SHALL maintain 
    safe distances of at least 50cm from obstacles"
    """
    
    def setUp(self):
        """Set up test fixtures."""
        rospy.loginfo = Mock()
        rospy.logwarn = Mock()
        rospy.logerr = Mock()
        rospy.logdebug = Mock()
        
        self.planner = AvoidancePlanner()
    
    def test_trajectory_execution_timing(self):
        """Test that trajectory execution follows correct timing."""
        print("\n=== Testing trajectory execution timing ===")
        
        # Create test trajectory
        start_time = time.time()
        trajectory = AvoidanceTrajectory(
            waypoints=[
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=0.5, y=0.0, z=0.0),
                Point(x=1.0, y=0.0, z=0.0)
            ],
            velocities=[0.8, 0.6, 0.4],
            angular_velocities=[0.0, 0.1, 0.0],
            timestamps=[start_time, start_time + 0.5, start_time + 1.0],
            total_duration=1.0,
            safety_margin=0.5,
            strategy=AvoidanceStrategy.SLOW
        )
        
        # Test execution at different time points
        cmd1 = self.planner.execute_trajectory(trajectory, start_time)
        self.assertIsNotNone(cmd1)
        self.assertEqual(self.planner.current_state, AvoidanceState.EXECUTING)
        
        cmd2 = self.planner.execute_trajectory(trajectory, start_time + 0.5)
        self.assertIsNotNone(cmd2)
        
        # Test completion
        cmd3 = self.planner.execute_trajectory(trajectory, start_time + 1.1)
        self.assertIsNone(cmd3)  # Should be complete
        
        print(f"✓ Trajectory execution timing correct")
        print(f"✓ Commands generated at appropriate times")
        print(f"✓ Execution completed after duration")
    
    def test_safety_margin_requirement(self):
        """Test that safety margin requirement (50cm) is enforced."""
        print("\n=== Testing safety margin requirement (50cm) ===")
        
        # Verify that minimum safety margin meets requirement 3.4
        required_margin = 0.5  # 50cm as per requirement
        
        self.assertGreaterEqual(
            self.planner.minimum_safety_margin, 
            required_margin,
            f"Minimum safety margin {self.planner.minimum_safety_margin}m "
            f"does not meet requirement of {required_margin}m"
        )
        
        print(f"✓ Minimum safety margin: {self.planner.minimum_safety_margin:.2f}m >= {required_margin:.2f}m")
        print(f"✓ Preferred safety margin: {self.planner.preferred_safety_margin:.2f}m")
        print(f"✓ Lateral safety margin: {self.planner.lateral_safety_margin:.2f}m")
    
    def test_trajectory_abort_mechanism(self):
        """Test trajectory abort mechanism for unsafe conditions."""
        print("\n=== Testing trajectory abort mechanism ===")
        
        # Create and start trajectory execution
        trajectory = AvoidanceTrajectory(
            waypoints=[Point(x=0.0, y=0.0, z=0.0)],
            velocities=[0.5],
            angular_velocities=[0.0],
            timestamps=[time.time()],
            total_duration=2.0,
            safety_margin=0.5,
            strategy=AvoidanceStrategy.SLOW
        )
        
        # Start execution
        cmd = self.planner.execute_trajectory(trajectory, time.time())
        self.assertIsNotNone(cmd)
        self.assertEqual(self.planner.current_state, AvoidanceState.EXECUTING)
        
        # Abort trajectory
        abort_reason = "Obstacle too close"
        self.planner.abort_trajectory(abort_reason, time.time())
        
        self.assertEqual(self.planner.current_state, AvoidanceState.RECOVERING)
        self.assertIsNone(self.planner.current_trajectory)
        
        print(f"✓ Trajectory abort mechanism working correctly")
        print(f"✓ State changed to RECOVERING after abort")


if __name__ == '__main__':
    # Set up test environment
    print("=" * 80)
    print("AVOIDANCE PLANNER UNIT TESTS")
    print("=" * 80)
    print("Testing intelligent avoidance strategies for requirement 3.3, 3.4, 3.5")
    print()
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add strategy selection tests
    test_suite.addTest(unittest.makeSuite(TestAvoidanceStrategySelection))
    
    # Add trajectory generation tests
    test_suite.addTest(unittest.makeSuite(TestAvoidanceTrajectoryGeneration))
    
    # Add execution tests
    test_suite.addTest(unittest.makeSuite(TestAvoidanceExecution))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Print summary
    print("\n" + "=" * 80)
    print("TEST SUMMARY")
    print("=" * 80)
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
    
    if result.wasSuccessful():
        print("\n✓ ALL TESTS PASSED - Avoidance planner meets requirements!")
    else:
        print("\n✗ SOME TESTS FAILED - Review implementation")
    
    print("=" * 80)
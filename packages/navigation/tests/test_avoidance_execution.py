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


class TestAvoidanceExecutionRequirements(unittest.TestCase):
    """
    Test suite for avoidance execution requirements.
    
    Specifically tests:
    - Requirement 3.4: "WHEN executing avoidance maneuvers THEN the system 
      SHALL maintain safe distances of at least 50cm from obstacles"
    - Requirement 3.5: "WHEN the avoidance path is clear THEN the system 
      SHALL return to normal lane following"
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
    
    def test_requirement_3_4_safety_distance_maintenance(self):
        """
        Test Requirement 3.4: System maintains safe distances of at least 50cm.
        """
        print("\n=== Testing Requirement 3.4: Safe Distance Maintenance ===")
        
        # Verify configuration meets requirement
        required_safety_distance = 0.5  # 50cm as specified in requirement
        
        self.assertGreaterEqual(
            self.planner.minimum_safety_margin,
            required_safety_distance,
            f"Minimum safety margin {self.planner.minimum_safety_margin}m "
            f"must be >= {required_safety_distance}m per Requirement 3.4"
        )
        
        print(f"✓ Configuration check: minimum_safety_margin = {self.planner.minimum_safety_margin}m >= 0.5m")
        
        # Test trajectory validation enforces safety distance
        # Create trajectory that violates safety distance
        unsafe_trajectory = AvoidanceTrajectory(
            waypoints=[
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=0.3, y=0.0, z=0.0),  # Close to obstacle
                Point(x=0.6, y=0.0, z=0.0)
            ],
            velocities=[0.5, 0.4, 0.3],
            angular_velocities=[0.0, 0.0, 0.0],
            timestamps=[time.time(), time.time() + 0.5, time.time() + 1.0],
            total_duration=1.0,
            safety_margin=0.3,  # Below required margin
            strategy=AvoidanceStrategy.SLOW
        )
        
        # Create obstacle close to trajectory
        detection = ObjectDetection()
        detection.distance = 0.4  # 40cm - violates 50cm requirement
        detection.relative_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        
        risk_factors = RiskFactors(
            distance_risk=0.8, velocity_risk=0.3, object_type_risk=0.5,
            trajectory_risk=0.6, time_to_collision=2.0, 
            lateral_clearance=0.3  # Below required 50cm
        )
        
        high_risk_objects = [(detection, risk_factors, RiskLevel.HIGH)]
        
        # Trajectory should be rejected as unsafe
        is_safe = self.planner._validate_trajectory_safety(
            unsafe_trajectory, high_risk_objects, time.time()
        )
        
        self.assertFalse(is_safe, "Trajectory with insufficient safety margin should be rejected")
        print(f"✓ Unsafe trajectory correctly rejected (clearance: {risk_factors.lateral_clearance}m < 0.5m)")
        
        # Test with safe trajectory
        safe_trajectory = AvoidanceTrajectory(
            waypoints=[
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=0.5, y=0.8, z=0.0),  # Swerve away from obstacle
                Point(x=1.0, y=0.0, z=0.0)
            ],
            velocities=[0.5, 0.4, 0.3],
            angular_velocities=[0.0, 0.2, -0.2],
            timestamps=[time.time(), time.time() + 0.5, time.time() + 1.0],
            total_duration=1.0,
            safety_margin=0.6,  # Above required margin
            strategy=AvoidanceStrategy.SWERVE
        )
        
        # Create obstacle that maintains safe distance
        detection.distance = 1.0
        risk_factors.lateral_clearance = 0.7  # Above required 50cm
        
        is_safe = self.planner._validate_trajectory_safety(
            safe_trajectory, high_risk_objects, time.time()
        )
        
        self.assertTrue(is_safe, "Trajectory with sufficient safety margin should be accepted")
        print(f"✓ Safe trajectory correctly accepted (clearance: {risk_factors.lateral_clearance}m >= 0.5m)")
    
    def test_requirement_3_5_return_to_normal_following(self):
        """
        Test Requirement 3.5: System returns to normal lane following when path is clear.
        """
        print("\n=== Testing Requirement 3.5: Return to Normal Lane Following ===")
        
        # Start with high-risk situation requiring avoidance
        detection = ObjectDetection()
        detection.class_name = "duckiebot"
        detection.confidence = 0.8
        detection.distance = 0.6
        detection.relative_velocity = Vector3(x=-0.3, y=0.0, z=0.0)
        
        risk_factors = RiskFactors(
            distance_risk=0.7, velocity_risk=0.5, object_type_risk=0.8,
            trajectory_risk=0.6, time_to_collision=2.0, lateral_clearance=0.4
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.HIGH)]
        
        # Plan avoidance - should trigger avoidance strategy
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        self.assertNotEqual(strategy, AvoidanceStrategy.NONE)
        self.assertNotEqual(self.planner.current_state, AvoidanceState.NORMAL)
        
        print(f"✓ Avoidance triggered: strategy = {strategy.value}, state = {self.planner.current_state.value}")
        
        # Simulate path becoming clear (no high-risk objects)
        clear_risk_assessments = []  # Empty - no objects detected
        
        # First call with clear path - should continue current execution if active
        if trajectory:
            # Start execution
            cmd = self.planner.execute_trajectory(trajectory, time.time())
            self.assertIsNotNone(cmd)
            self.assertEqual(self.planner.current_state, AvoidanceState.EXECUTING)
            print(f"✓ Trajectory execution started")
        
        # Simulate time passing and path remaining clear
        time.sleep(0.1)  # Small delay to simulate time passage
        
        # Plan avoidance with clear path
        clear_strategy, clear_trajectory = self.planner.plan_avoidance(
            clear_risk_assessments, self.test_vehicle_state
        )
        
        # After sufficient time with clear path, should return to normal
        # Simulate multiple calls with clear path to trigger return to normal
        for i in range(3):
            time.sleep(0.1)
            clear_strategy, clear_trajectory = self.planner.plan_avoidance(
                clear_risk_assessments, self.test_vehicle_state
            )
        
        # Should eventually return to normal operation
        self.assertEqual(clear_strategy, AvoidanceStrategy.NONE)
        self.assertIsNone(clear_trajectory)
        
        print(f"✓ Returned to normal operation: strategy = {clear_strategy.value}")
        print(f"✓ Final state: {self.planner.current_state.value}")
    
    def test_smooth_trajectory_generation(self):
        """Test that generated trajectories are smooth and executable."""
        print("\n=== Testing Smooth Trajectory Generation ===")
        
        # Test stop trajectory smoothness
        stop_trajectory = self.planner._generate_stop_trajectory(
            self.test_vehicle_state, time.time()
        )
        
        # Check velocity profile is smooth (no sudden jumps)
        for i in range(1, len(stop_trajectory.velocities)):
            velocity_change = abs(stop_trajectory.velocities[i] - stop_trajectory.velocities[i-1])
            max_allowed_change = 0.5  # Maximum velocity change between waypoints
            
            self.assertLessEqual(
                velocity_change, max_allowed_change,
                f"Velocity change too large: {velocity_change:.3f} > {max_allowed_change}"
            )
        
        print(f"✓ Stop trajectory velocity profile is smooth")
        print(f"✓ Max velocity change: {max(abs(stop_trajectory.velocities[i] - stop_trajectory.velocities[i-1]) for i in range(1, len(stop_trajectory.velocities))):.3f} m/s")
        
        # Test swerve trajectory smoothness
        swerve_trajectory = self.planner._generate_swerve_trajectory(
            self.test_vehicle_state, [], None, time.time()
        )
        
        # Check angular velocities are within limits
        for angular_vel in swerve_trajectory.angular_velocities:
            self.assertLessEqual(
                abs(angular_vel), self.planner.max_angular_velocity,
                f"Angular velocity exceeds limit: {abs(angular_vel):.3f} > {self.planner.max_angular_velocity}"
            )
        
        # Check lateral motion returns to center
        y_positions = [wp.y for wp in swerve_trajectory.waypoints]
        self.assertAlmostEqual(y_positions[-1], 0.0, delta=0.1, 
                              msg="Swerve trajectory should return to center")
        
        print(f"✓ Swerve trajectory angular velocities within limits")
        print(f"✓ Swerve trajectory returns to center: final y = {y_positions[-1]:.3f}m")
    
    def test_avoidance_strategy_performance_monitoring(self):
        """Test performance monitoring and metrics collection."""
        print("\n=== Testing Performance Monitoring ===")
        
        # Execute several avoidance scenarios and check metrics
        scenarios = [
            (AvoidanceStrategy.STOP, 0.2, RiskLevel.CRITICAL),
            (AvoidanceStrategy.SLOW, 0.7, RiskLevel.HIGH),
            (AvoidanceStrategy.SWERVE, 0.6, RiskLevel.HIGH)
        ]
        
        for expected_strategy, distance, risk_level in scenarios:
            detection = ObjectDetection()
            detection.class_name = "test_object"
            detection.confidence = 0.8
            detection.distance = distance
            detection.relative_velocity = Vector3(x=-0.2, y=0.0, z=0.0)
            
            risk_factors = RiskFactors(
                distance_risk=0.8 if risk_level == RiskLevel.CRITICAL else 0.6,
                velocity_risk=0.5, object_type_risk=0.5, trajectory_risk=0.5,
                time_to_collision=1.0 if risk_level == RiskLevel.CRITICAL else 2.5,
                lateral_clearance=0.3 if expected_strategy == AvoidanceStrategy.SWERVE else 0.6
            )
            
            risk_assessments = [(detection, risk_factors, risk_level)]
            
            # Mock swerve safety check for swerve scenarios
            with patch.object(self.planner, '_can_swerve_safely', 
                            return_value=(expected_strategy == AvoidanceStrategy.SWERVE)):
                strategy, trajectory = self.planner.plan_avoidance(
                    risk_assessments, self.test_vehicle_state
                )
            
            if expected_strategy == AvoidanceStrategy.SWERVE and not self.planner._can_swerve_safely([], self.test_vehicle_state):
                # If swerve is not safe, expect SLOW instead
                expected_strategy = AvoidanceStrategy.SLOW
            
            print(f"✓ Scenario: distance={distance}m, expected={expected_strategy.value}, actual={strategy.value}")
        
        # Check performance metrics
        metrics = self.planner.get_performance_metrics()
        
        self.assertIsInstance(metrics, dict)
        self.assertIn('total_executions', metrics)
        self.assertIn('success_rate', metrics)
        self.assertIn('strategy_distribution', metrics)
        self.assertIn('current_state', metrics)
        
        print(f"✓ Performance metrics collected:")
        print(f"  Total executions: {metrics['total_executions']}")
        print(f"  Success rate: {metrics['success_rate']:.1f}%")
        print(f"  Current state: {metrics['current_state']}")
    
    def test_integration_with_existing_navigation(self):
        """Test integration with existing navigation utilities."""
        print("\n=== Testing Integration with Existing Navigation ===")
        
        # Test that avoidance planner can work with existing navigation data structures
        from duckietown_msgs.msg import LanePose
        
        # Create mock lane pose
        lane_pose = LanePose()
        lane_pose.d = 0.1  # 10cm offset from center
        lane_pose.phi = 0.05  # Small heading error
        
        # Test planning with lane pose information
        detection = ObjectDetection()
        detection.class_name = "cone"
        detection.confidence = 0.7
        detection.distance = 0.8
        detection.relative_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        
        risk_factors = RiskFactors(
            distance_risk=0.5, velocity_risk=0.2, object_type_risk=0.6,
            trajectory_risk=0.4, time_to_collision=3.0, lateral_clearance=0.5
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.HIGH)]
        
        # Plan avoidance with lane pose
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state, lane_pose
        )
        
        self.assertIsNotNone(strategy)
        print(f"✓ Planning with lane pose successful: strategy = {strategy.value}")
        
        if trajectory:
            print(f"✓ Trajectory generated with lane pose context")
            print(f"  Duration: {trajectory.total_duration:.2f}s")
            print(f"  Waypoints: {len(trajectory.waypoints)}")


class TestAvoidanceRobustness(unittest.TestCase):
    """
    Test suite for avoidance system robustness and edge cases.
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
    
    def test_edge_case_zero_velocity(self):
        """Test avoidance planning when vehicle has zero velocity."""
        print("\n=== Testing Edge Case: Zero Velocity ===")
        
        # Create vehicle state with zero velocity
        stationary_vehicle = VehicleState(
            position=Point(x=0.0, y=0.0, z=0.0),
            velocity=Vector3(x=0.0, y=0.0, z=0.0),
            heading=0.0,
            timestamp=time.time()
        )
        
        detection = ObjectDetection()
        detection.class_name = "duckiebot"
        detection.confidence = 0.8
        detection.distance = 0.4
        detection.relative_velocity = Vector3(x=-0.5, y=0.0, z=0.0)
        
        risk_factors = RiskFactors(
            distance_risk=0.8, velocity_risk=0.6, object_type_risk=0.8,
            trajectory_risk=0.7, time_to_collision=0.8, lateral_clearance=0.3
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.CRITICAL)]
        
        # Should still be able to plan (likely STOP strategy)
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, stationary_vehicle
        )
        
        self.assertIsNotNone(strategy)
        print(f"✓ Zero velocity handled: strategy = {strategy.value}")
        
        if trajectory:
            # All velocities should remain zero or very low
            max_velocity = max(trajectory.velocities)
            self.assertLessEqual(max_velocity, 0.1)
            print(f"✓ Trajectory velocities appropriate for stationary vehicle: max = {max_velocity:.3f} m/s")
    
    def test_edge_case_very_close_obstacle(self):
        """Test avoidance planning with extremely close obstacles."""
        print("\n=== Testing Edge Case: Very Close Obstacle ===")
        
        detection = ObjectDetection()
        detection.class_name = "barrier"
        detection.confidence = 0.9
        detection.distance = 0.1  # 10cm - extremely close
        detection.relative_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        
        risk_factors = RiskFactors(
            distance_risk=1.0, velocity_risk=0.3, object_type_risk=0.9,
            trajectory_risk=0.9, time_to_collision=0.2, lateral_clearance=0.05
        )
        
        risk_assessments = [(detection, risk_factors, RiskLevel.CRITICAL)]
        
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        # Should select STOP strategy for extremely close obstacles
        self.assertEqual(strategy, AvoidanceStrategy.STOP)
        print(f"✓ Very close obstacle handled: strategy = {strategy.value}")
        
        if trajectory:
            # Should have very short stopping distance
            self.assertLessEqual(trajectory.total_duration, 1.0)
            print(f"✓ Emergency stop trajectory: duration = {trajectory.total_duration:.2f}s")
    
    def test_edge_case_multiple_conflicting_obstacles(self):
        """Test avoidance planning with multiple obstacles in different directions."""
        print("\n=== Testing Edge Case: Multiple Conflicting Obstacles ===")
        
        # Obstacle on the left
        detection1 = ObjectDetection()
        detection1.class_name = "duckiebot"
        detection1.confidence = 0.8
        detection1.distance = 0.6
        detection1.relative_velocity = Vector3(x=-0.2, y=0.1, z=0.0)
        
        risk_factors1 = RiskFactors(
            distance_risk=0.6, velocity_risk=0.4, object_type_risk=0.8,
            trajectory_risk=0.5, time_to_collision=3.0, lateral_clearance=0.3
        )
        
        # Obstacle on the right
        detection2 = ObjectDetection()
        detection2.class_name = "cone"
        detection2.confidence = 0.7
        detection2.distance = 0.7
        detection2.relative_velocity = Vector3(x=-0.1, y=-0.1, z=0.0)
        
        risk_factors2 = RiskFactors(
            distance_risk=0.5, velocity_risk=0.3, object_type_risk=0.6,
            trajectory_risk=0.4, time_to_collision=4.0, lateral_clearance=0.4
        )
        
        risk_assessments = [
            (detection1, risk_factors1, RiskLevel.HIGH),
            (detection2, risk_factors2, RiskLevel.HIGH)
        ]
        
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        
        # Should select appropriate strategy (likely STOP or SLOW due to conflicting obstacles)
        self.assertIn(strategy, [AvoidanceStrategy.STOP, AvoidanceStrategy.SLOW])
        print(f"✓ Multiple conflicting obstacles handled: strategy = {strategy.value}")
        
        if trajectory:
            print(f"✓ Trajectory generated for complex scenario")
    
    def test_performance_under_load(self):
        """Test avoidance planner performance with many objects."""
        print("\n=== Testing Performance Under Load ===")
        
        # Create many objects
        risk_assessments = []
        for i in range(10):  # 10 objects
            detection = ObjectDetection()
            detection.class_name = f"object_{i}"
            detection.confidence = 0.6 + i * 0.03
            detection.distance = 0.5 + i * 0.2
            detection.relative_velocity = Vector3(x=-0.1 - i * 0.05, y=0.0, z=0.0)
            
            risk_factors = RiskFactors(
                distance_risk=0.8 - i * 0.05,
                velocity_risk=0.5 + i * 0.03,
                object_type_risk=0.5,
                trajectory_risk=0.4,
                time_to_collision=2.0 + i * 0.5,
                lateral_clearance=0.4 + i * 0.1
            )
            
            risk_level = RiskLevel.HIGH if i < 3 else RiskLevel.MEDIUM
            risk_assessments.append((detection, risk_factors, risk_level))
        
        # Measure planning time
        start_time = time.time()
        strategy, trajectory = self.planner.plan_avoidance(
            risk_assessments, self.test_vehicle_state
        )
        planning_time = (time.time() - start_time) * 1000  # Convert to ms
        
        # Should complete planning within reasonable time
        max_allowed_time = 100  # 100ms
        self.assertLessEqual(planning_time, max_allowed_time,
                           f"Planning took too long: {planning_time:.2f}ms > {max_allowed_time}ms")
        
        self.assertIsNotNone(strategy)
        print(f"✓ Planning with {len(risk_assessments)} objects completed in {planning_time:.2f}ms")
        print(f"✓ Selected strategy: {strategy.value}")


if __name__ == '__main__':
    # Set up test environment
    print("=" * 80)
    print("AVOIDANCE EXECUTION UNIT TESTS")
    print("=" * 80)
    print("Testing avoidance execution for requirements 3.4 and 3.5")
    print()
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add execution requirement tests
    test_suite.addTest(unittest.makeSuite(TestAvoidanceExecutionRequirements))
    
    # Add robustness tests
    test_suite.addTest(unittest.makeSuite(TestAvoidanceRobustness))
    
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
        print("\n✓ ALL TESTS PASSED - Avoidance execution meets requirements!")
    else:
        print("\n✗ SOME TESTS FAILED - Review implementation")
    
    print("=" * 80)
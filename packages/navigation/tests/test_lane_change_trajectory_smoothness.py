#!/usr/bin/env python3

import unittest
import time
import numpy as np
import math
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

sys.modules['rospy'] = MockRospy()

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

class MockGeometryMsgs:
    class msg:
        Point = MockPoint
        Vector3 = MockVector3

sys.modules['geometry_msgs'] = MockGeometryMsgs()
sys.modules['geometry_msgs.msg'] = MockGeometryMsgs.msg()

# Now import the actual implementation
try:
    from lane_change_trajectory_generator import (
        LaneChangeTrajectoryGenerator, LaneChangeTrajectory, TrajectoryConstraints,
        LaneChangeParameters, TrajectoryWaypoint, TrajectoryValidationResult,
        TrajectoryType
    )
    print("✅ Successfully imported lane change trajectory generator")
except Exception as e:
    print(f"❌ Failed to import lane change trajectory generator: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)


class TestLaneChangeTrajectorySmoothnessAndConstraints(unittest.TestCase):
    """
    Test suite for lane change trajectory generation smoothness and constraint satisfaction.
    
    Tests the quintic polynomial trajectory generation, constraint validation,
    and smoothness optimization for lane change maneuvers.
    """
    
    def setUp(self):
        """Set up test fixtures"""
        # Create trajectory generator instance
        self.trajectory_generator = LaneChangeTrajectoryGenerator()
        
        # Create test data
        self.create_test_data()
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Test setup completed")
    
    def create_test_data(self):
        """Create test data for various scenarios"""
        
        # Standard lane change parameters
        self.standard_lane_change = LaneChangeParameters(
            start_position=MockPoint(x=0.0, y=0.0, z=0.0),
            end_position=MockPoint(x=3.0, y=0.6, z=0.0),  # 3m forward, 0.6m lateral
            start_velocity=MockVector3(x=1.0, y=0.0, z=0.0),
            end_velocity=MockVector3(x=1.0, y=0.0, z=0.0),
            start_heading=0.0,
            end_heading=0.0,
            lane_width=0.6,
            vehicle_length=0.18,
            vehicle_width=0.13
        )
        
        # Aggressive lane change parameters
        self.aggressive_lane_change = LaneChangeParameters(
            start_position=MockPoint(x=0.0, y=0.0, z=0.0),
            end_position=MockPoint(x=2.0, y=0.6, z=0.0),  # Shorter distance
            start_velocity=MockVector3(x=2.0, y=0.0, z=0.0),  # Higher speed
            end_velocity=MockVector3(x=2.0, y=0.0, z=0.0),
            start_heading=0.0,
            end_heading=0.0,
            lane_width=0.6,
            vehicle_length=0.18,
            vehicle_width=0.13
        )
        
        # Conservative constraints
        self.conservative_constraints = TrajectoryConstraints(
            maximum_lateral_acceleration=1.0,  # Low acceleration
            maximum_curvature=1.0,
            maximum_velocity=1.5,
            minimum_radius=0.5,
            comfort_factor=0.9,  # High comfort
            safety_margin=0.4,
            preferred_duration=3.0,  # Longer duration
            maximum_duration=4.0
        )
        
        # Aggressive constraints
        self.aggressive_constraints = TrajectoryConstraints(
            maximum_lateral_acceleration=3.0,  # High acceleration
            maximum_curvature=3.0,
            maximum_velocity=3.0,
            minimum_radius=0.2,
            comfort_factor=0.5,  # Lower comfort
            safety_margin=0.2,
            preferred_duration=1.5,  # Shorter duration
            maximum_duration=2.0
        )
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Test data created")
    
    def test_trajectory_generator_initialization(self):
        """Test that the trajectory generator initializes correctly"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing trajectory generator initialization")
        
        # Check that generator is initialized
        self.assertIsNotNone(self.trajectory_generator)
        self.assertIsInstance(self.trajectory_generator.constraints, TrajectoryConstraints)
        
        # Check default constraint values
        self.assertGreater(self.trajectory_generator.constraints.maximum_lateral_acceleration, 0)
        self.assertGreater(self.trajectory_generator.constraints.maximum_curvature, 0)
        self.assertGreater(self.trajectory_generator.constraints.preferred_duration, 0)
        self.assertGreater(self.trajectory_generator.waypoint_resolution, 0)
        
        # Check performance metrics initialization
        metrics = self.trajectory_generator.get_performance_metrics()
        self.assertEqual(metrics['total_trajectories_generated'], 0)
        self.assertEqual(metrics['successful_generations'], 0)
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Trajectory generator initialization test passed")
    
    def test_optimal_duration_calculation(self):
        """Test optimal duration calculation for different scenarios"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing optimal duration calculation")
        
        # Test with standard parameters
        duration_standard = self.trajectory_generator.calculate_optimal_duration(
            self.standard_lane_change, self.trajectory_generator.constraints
        )
        
        # Test with aggressive parameters
        duration_aggressive = self.trajectory_generator.calculate_optimal_duration(
            self.aggressive_lane_change, self.aggressive_constraints
        )
        
        # Test with conservative constraints
        duration_conservative = self.trajectory_generator.calculate_optimal_duration(
            self.standard_lane_change, self.conservative_constraints
        )
        
        # Validate duration values
        self.assertGreater(duration_standard, 0)
        self.assertGreater(duration_aggressive, 0)
        self.assertGreater(duration_conservative, 0)
        
        # Conservative should take longer than aggressive
        self.assertGreater(duration_conservative, duration_aggressive)
        
        # All durations should be reasonable (between 1 and 5 seconds)
        self.assertGreaterEqual(duration_standard, 1.0)
        self.assertLessEqual(duration_standard, 5.0)
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Duration calculation results:")
        print(f"  Standard duration: {duration_standard:.2f}s")
        print(f"  Aggressive duration: {duration_aggressive:.2f}s")
        print(f"  Conservative duration: {duration_conservative:.2f}s")
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Optimal duration calculation test passed")
    
    def test_quintic_polynomial_coefficients(self):
        """Test quintic polynomial coefficient calculation"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing quintic polynomial coefficients")
        
        duration = 2.0
        coefficients = self.trajectory_generator.calculate_quintic_coefficients(
            self.standard_lane_change, duration
        )
        
        # Validate coefficient structure
        self.assertIn('longitudinal', coefficients)
        self.assertIn('lateral', coefficients)
        self.assertEqual(len(coefficients['longitudinal']), 6)  # Quintic = 6 coefficients
        self.assertEqual(len(coefficients['lateral']), 6)
        
        # Validate boundary conditions
        # At t=0: position should match start position
        coeffs_x = coefficients['longitudinal']
        coeffs_y = coefficients['lateral']
        
        # p(0) = a0
        self.assertAlmostEqual(coeffs_x[0], self.standard_lane_change.start_position.x, places=3)
        self.assertAlmostEqual(coeffs_y[0], self.standard_lane_change.start_position.y, places=3)
        
        # p'(0) = a1
        self.assertAlmostEqual(coeffs_x[1], self.standard_lane_change.start_velocity.x, places=3)
        self.assertAlmostEqual(coeffs_y[1], self.standard_lane_change.start_velocity.y, places=3)
        
        # At t=T: position should match end position
        T = duration
        T2, T3, T4, T5 = T*T, T*T*T, T*T*T*T, T*T*T*T*T
        
        x_final = coeffs_x[0] + coeffs_x[1]*T + coeffs_x[2]*T2 + coeffs_x[3]*T3 + coeffs_x[4]*T4 + coeffs_x[5]*T5
        y_final = coeffs_y[0] + coeffs_y[1]*T + coeffs_y[2]*T2 + coeffs_y[3]*T3 + coeffs_y[4]*T4 + coeffs_y[5]*T5
        
        self.assertAlmostEqual(x_final, self.standard_lane_change.end_position.x, places=2)
        self.assertAlmostEqual(y_final, self.standard_lane_change.end_position.y, places=2)
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Coefficient validation:")
        print(f"  Longitudinal coefficients: {[f'{c:.4f}' for c in coeffs_x]}")
        print(f"  Lateral coefficients: {[f'{c:.4f}' for c in coeffs_y]}")
        print(f"  Boundary condition validation passed")
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Quintic polynomial coefficients test passed")
    
    def test_waypoint_generation(self):
        """Test waypoint generation along trajectory"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing waypoint generation")
        
        duration = 2.0
        coefficients = self.trajectory_generator.calculate_quintic_coefficients(
            self.standard_lane_change, duration
        )
        
        waypoints = self.trajectory_generator.generate_waypoints(
            coefficients, duration, self.standard_lane_change
        )
        
        # Validate waypoint structure
        self.assertGreater(len(waypoints), 0)
        expected_waypoints = int(duration / self.trajectory_generator.waypoint_resolution) + 1
        self.assertEqual(len(waypoints), expected_waypoints)
        
        # Validate waypoint properties
        for i, waypoint in enumerate(waypoints):
            self.assertIsInstance(waypoint, TrajectoryWaypoint)
            self.assertIsNotNone(waypoint.position)
            self.assertIsNotNone(waypoint.velocity)
            self.assertIsNotNone(waypoint.acceleration)
            self.assertIsInstance(waypoint.heading, float)
            self.assertIsInstance(waypoint.curvature, float)
            self.assertGreaterEqual(waypoint.curvature, 0.0)
        
        # Check trajectory smoothness - velocities should be continuous
        for i in range(1, len(waypoints)):
            velocity_change = abs(waypoints[i].velocity.x - waypoints[i-1].velocity.x)
            self.assertLess(velocity_change, 1.0)  # Reasonable velocity change
        
        # Check start and end positions
        start_waypoint = waypoints[0]
        end_waypoint = waypoints[-1]
        
        self.assertAlmostEqual(start_waypoint.position.x, self.standard_lane_change.start_position.x, places=2)
        self.assertAlmostEqual(start_waypoint.position.y, self.standard_lane_change.start_position.y, places=2)
        self.assertAlmostEqual(end_waypoint.position.x, self.standard_lane_change.end_position.x, places=1)
        self.assertAlmostEqual(end_waypoint.position.y, self.standard_lane_change.end_position.y, places=1)
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Waypoint generation results:")
        print(f"  Total waypoints: {len(waypoints)}")
        print(f"  Start position: ({start_waypoint.position.x:.2f}, {start_waypoint.position.y:.2f})")
        print(f"  End position: ({end_waypoint.position.x:.2f}, {end_waypoint.position.y:.2f})")
        print(f"  Max curvature: {max(wp.curvature for wp in waypoints):.3f}")
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Waypoint generation test passed")
    
    def test_trajectory_metrics_calculation(self):
        """Test trajectory metrics calculation"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing trajectory metrics calculation")
        
        # Generate a test trajectory
        trajectory = self.trajectory_generator.generate_lane_change_trajectory(
            self.standard_lane_change
        )
        
        self.assertIsNotNone(trajectory)
        self.assertGreater(len(trajectory.waypoints), 0)
        
        # Calculate metrics manually for verification
        metrics = self.trajectory_generator.calculate_trajectory_metrics(trajectory.waypoints)
        
        # Validate metric structure
        required_metrics = [
            'total_distance', 'max_lateral_acceleration', 'max_curvature',
            'max_velocity', 'average_velocity'
        ]
        
        for metric in required_metrics:
            self.assertIn(metric, metrics)
            self.assertGreaterEqual(metrics[metric], 0.0)
        
        # Validate metric values
        self.assertGreater(metrics['total_distance'], 0.0)
        self.assertLess(metrics['total_distance'], 10.0)  # Reasonable distance
        
        # Compare with trajectory object metrics
        self.assertAlmostEqual(trajectory.total_distance, metrics['total_distance'], places=2)
        self.assertAlmostEqual(trajectory.maximum_lateral_acceleration, metrics['max_lateral_acceleration'], places=2)
        self.assertAlmostEqual(trajectory.maximum_curvature, metrics['max_curvature'], places=3)
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Trajectory metrics:")
        print(f"  Total distance: {metrics['total_distance']:.2f}m")
        print(f"  Max lateral acceleration: {metrics['max_lateral_acceleration']:.2f}m/s²")
        print(f"  Max curvature: {metrics['max_curvature']:.3f}1/m")
        print(f"  Max velocity: {metrics['max_velocity']:.2f}m/s")
        print(f"  Average velocity: {metrics['average_velocity']:.2f}m/s")
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Trajectory metrics calculation test passed")
    
    def test_trajectory_constraint_validation(self):
        """Test trajectory validation against constraints"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing trajectory constraint validation")
        
        # Generate trajectory with conservative constraints
        conservative_trajectory = self.trajectory_generator.generate_lane_change_trajectory(
            self.standard_lane_change, self.conservative_constraints
        )
        
        # Generate trajectory with aggressive constraints
        aggressive_trajectory = self.trajectory_generator.generate_lane_change_trajectory(
            self.aggressive_lane_change, self.aggressive_constraints
        )
        
        # Validate conservative trajectory
        self.assertIsNotNone(conservative_trajectory)
        self.assertTrue(conservative_trajectory.is_feasible)
        self.assertEqual(conservative_trajectory.validation_result, TrajectoryValidationResult.VALID)
        
        # Check constraint satisfaction
        self.assertLessEqual(
            conservative_trajectory.maximum_lateral_acceleration,
            self.conservative_constraints.maximum_lateral_acceleration + 0.01  # Small tolerance
        )
        self.assertLessEqual(
            conservative_trajectory.maximum_curvature,
            self.conservative_constraints.maximum_curvature + 0.01
        )
        
        # Validate aggressive trajectory
        self.assertIsNotNone(aggressive_trajectory)
        # May or may not be feasible depending on parameters
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Constraint validation results:")
        print(f"  Conservative trajectory feasible: {conservative_trajectory.is_feasible}")
        print(f"  Conservative max lateral accel: {conservative_trajectory.maximum_lateral_acceleration:.2f}m/s² (limit: {self.conservative_constraints.maximum_lateral_acceleration:.2f})")
        print(f"  Aggressive trajectory feasible: {aggressive_trajectory.is_feasible}")
        print(f"  Aggressive max lateral accel: {aggressive_trajectory.maximum_lateral_acceleration:.2f}m/s² (limit: {self.aggressive_constraints.maximum_lateral_acceleration:.2f})")
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Trajectory constraint validation test passed")
    
    def test_trajectory_smoothness_properties(self):
        """Test trajectory smoothness properties"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing trajectory smoothness properties")
        
        trajectory = self.trajectory_generator.generate_lane_change_trajectory(
            self.standard_lane_change
        )
        
        self.assertIsNotNone(trajectory)
        self.assertGreater(len(trajectory.waypoints), 2)
        
        waypoints = trajectory.waypoints
        
        # Test velocity continuity
        max_velocity_jump = 0.0
        for i in range(1, len(waypoints)):
            velocity_jump = abs(waypoints[i].velocity.x - waypoints[i-1].velocity.x)
            max_velocity_jump = max(max_velocity_jump, velocity_jump)
        
        # Velocity should be relatively continuous
        self.assertLess(max_velocity_jump, 0.5)  # Reasonable velocity continuity
        
        # Test acceleration smoothness
        max_acceleration_jump = 0.0
        for i in range(1, len(waypoints)):
            accel_jump = abs(waypoints[i].acceleration.y - waypoints[i-1].acceleration.y)
            max_acceleration_jump = max(max_acceleration_jump, accel_jump)
        
        # Test curvature smoothness
        max_curvature_jump = 0.0
        for i in range(1, len(waypoints)):
            curvature_jump = abs(waypoints[i].curvature - waypoints[i-1].curvature)
            max_curvature_jump = max(max_curvature_jump, curvature_jump)
        
        # Test heading continuity
        max_heading_jump = 0.0
        for i in range(1, len(waypoints)):
            heading_diff = abs(waypoints[i].heading - waypoints[i-1].heading)
            # Handle angle wrapping
            if heading_diff > math.pi:
                heading_diff = 2 * math.pi - heading_diff
            max_heading_jump = max(max_heading_jump, heading_diff)
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Smoothness analysis:")
        print(f"  Max velocity jump: {max_velocity_jump:.3f}m/s")
        print(f"  Max acceleration jump: {max_acceleration_jump:.3f}m/s²")
        print(f"  Max curvature jump: {max_curvature_jump:.3f}1/m")
        print(f"  Max heading jump: {max_heading_jump:.3f}rad")
        
        # Smoothness criteria
        self.assertLess(max_velocity_jump, 0.5)
        self.assertLess(max_heading_jump, 0.5)  # Reasonable heading continuity
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Trajectory smoothness properties test passed")
    
    def test_comfort_optimization(self):
        """Test trajectory comfort optimization"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing comfort optimization")
        
        # Generate original trajectory
        original_trajectory = self.trajectory_generator.generate_lane_change_trajectory(
            self.standard_lane_change
        )
        
        self.assertIsNotNone(original_trajectory)
        self.assertTrue(original_trajectory.is_feasible)
        
        # Optimize for comfort
        comfort_factor = 0.7
        optimized_trajectory = self.trajectory_generator.optimize_trajectory_for_comfort(
            original_trajectory, comfort_factor
        )
        
        self.assertIsNotNone(optimized_trajectory)
        
        # Optimized trajectory should have lower lateral acceleration
        self.assertLessEqual(
            optimized_trajectory.maximum_lateral_acceleration,
            original_trajectory.maximum_lateral_acceleration
        )
        
        # Duration and distance should remain similar
        self.assertAlmostEqual(
            optimized_trajectory.total_duration,
            original_trajectory.total_duration,
            places=1
        )
        
        # Number of waypoints should be the same
        self.assertEqual(
            len(optimized_trajectory.waypoints),
            len(original_trajectory.waypoints)
        )
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Comfort optimization results:")
        print(f"  Original max lateral accel: {original_trajectory.maximum_lateral_acceleration:.2f}m/s²")
        print(f"  Optimized max lateral accel: {optimized_trajectory.maximum_lateral_acceleration:.2f}m/s²")
        print(f"  Acceleration reduction: {((original_trajectory.maximum_lateral_acceleration - optimized_trajectory.maximum_lateral_acceleration) / original_trajectory.maximum_lateral_acceleration * 100):.1f}%")
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Comfort optimization test passed")
    
    def test_performance_metrics_tracking(self):
        """Test performance metrics tracking"""
        print(f"\n[TestLaneChangeTrajectorySmoothnessAndConstraints] Testing performance metrics tracking")
        
        # Reset metrics
        self.trajectory_generator.reset_metrics()
        initial_metrics = self.trajectory_generator.get_performance_metrics()
        
        # Generate several trajectories
        for i in range(3):
            trajectory = self.trajectory_generator.generate_lane_change_trajectory(
                self.standard_lane_change
            )
        
        # Check updated metrics
        final_metrics = self.trajectory_generator.get_performance_metrics()
        
        self.assertGreater(final_metrics['total_trajectories_generated'], initial_metrics['total_trajectories_generated'])
        self.assertGreaterEqual(final_metrics['successful_generations'], 0)
        self.assertGreater(final_metrics['average_generation_time_ms'], 0)
        
        # Calculate success rate
        expected_success_rate = (final_metrics['successful_generations'] / 
                               final_metrics['total_trajectories_generated']) * 100
        self.assertAlmostEqual(final_metrics['success_rate'], expected_success_rate, places=1)
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Performance metrics:")
        print(f"  Total trajectories: {final_metrics['total_trajectories_generated']}")
        print(f"  Successful generations: {final_metrics['successful_generations']}")
        print(f"  Success rate: {final_metrics['success_rate']:.1f}%")
        print(f"  Average generation time: {final_metrics['average_generation_time_ms']:.2f}ms")
        
        print(f"[TestLaneChangeTrajectorySmoothnessAndConstraints] Performance metrics tracking test passed")


def run_trajectory_smoothness_tests():
    """Run all trajectory smoothness and constraint satisfaction tests"""
    print(f"\n{'='*70}")
    print(f"RUNNING LANE CHANGE TRAJECTORY SMOOTHNESS AND CONSTRAINT TESTS")
    print(f"{'='*70}")
    
    # Create test suite
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestLaneChangeTrajectorySmoothnessAndConstraints)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout, buffer=False)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n{'='*70}")
    print(f"TRAJECTORY SMOOTHNESS AND CONSTRAINT TEST SUMMARY")
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
    success = run_trajectory_smoothness_tests()
    
    if success:
        print(f"\n✅ All trajectory smoothness and constraint tests passed!")
        exit(0)
    else:
        print(f"\n❌ Some trajectory smoothness and constraint tests failed!")
        exit(1)
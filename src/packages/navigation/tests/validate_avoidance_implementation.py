#!/usr/bin/env python3

"""
Validation script for avoidance planner implementation.

This script validates the implementation without requiring ROS dependencies
by testing the core logic and algorithms.
"""

import sys
import os
import time
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import Enum

# Add src directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# Mock ROS dependencies
class MockROSPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class MockROSVector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class MockROSTwist:
    def __init__(self):
        self.linear = MockROSVector3()
        self.angular = MockROSVector3()

class MockObjectDetection:
    def __init__(self):
        self.class_name = ""
        self.confidence = 0.0
        self.distance = 0.0
        self.relative_velocity = MockROSVector3()

# Mock additional ROS messages
class MockObjectDetectionArray:
    def __init__(self):
        self.detections = []
        self.processing_time = 0.0

class MockLanePose:
    def __init__(self):
        self.d = 0.0
        self.phi = 0.0

class MockGroundProjectionGeometry:
    pass

# Mock rospy
class MockRospy:
    @staticmethod
    def loginfo(msg): print(f"[INFO] {msg}")
    @staticmethod
    def logwarn(msg): print(f"[WARN] {msg}")
    @staticmethod
    def logerr(msg): print(f"[ERROR] {msg}")
    @staticmethod
    def logdebug(msg): pass  # Suppress debug messages

# Replace imports
sys.modules['rospy'] = MockRospy()
sys.modules['geometry_msgs.msg'] = type('MockGeometryMsgs', (), {
    'Point': MockROSPoint,
    'Vector3': MockROSVector3,
    'Twist': MockROSTwist
})()
sys.modules['duckietown_msgs.msg'] = type('MockDuckietownMsgs', (), {
    'ObjectDetection': MockObjectDetection,
    'ObjectDetectionArray': MockObjectDetectionArray,
    'LanePose': MockLanePose
})()
sys.modules['image_processing.ground_projection_geometry'] = type('MockGroundProjection', (), {
    'GroundProjectionGeometry': MockGroundProjectionGeometry,
    'Point': MockROSPoint
})()

# Now import our modules
from avoidance_planner import (
    AvoidancePlanner, AvoidanceStrategy, AvoidanceState, 
    AvoidanceTrajectory, AvoidanceMetrics
)
from risk_assessment_engine import RiskLevel, RiskFactors, VehicleState


def test_requirement_3_3_strategy_selection():
    """Test Requirement 3.3: Appropriate avoidance strategy selection."""
    print("\n" + "="*60)
    print("TESTING REQUIREMENT 3.3: STRATEGY SELECTION")
    print("="*60)
    
    planner = AvoidancePlanner()
    
    vehicle_state = VehicleState(
        position=MockROSPoint(x=0.0, y=0.0, z=0.0),
        velocity=MockROSVector3(x=0.8, y=0.0, z=0.0),
        heading=0.0,
        timestamp=time.time()
    )
    
    test_cases = [
        {
            'name': 'Critical Distance - Should Select STOP',
            'distance': 0.2,  # Below critical threshold
            'ttc': 2.0,
            'risk_level': RiskLevel.CRITICAL,
            'expected_strategy': AvoidanceStrategy.STOP
        },
        {
            'name': 'Critical TTC - Should Select STOP',
            'distance': 0.8,
            'ttc': 0.8,  # Below critical TTC threshold
            'risk_level': RiskLevel.CRITICAL,
            'expected_strategy': AvoidanceStrategy.STOP
        },
        {
            'name': 'Moderate Risk - Should Select SLOW',
            'distance': 0.7,
            'ttc': 2.5,
            'risk_level': RiskLevel.HIGH,
            'expected_strategy': AvoidanceStrategy.SLOW
        },
        {
            'name': 'Safe Conditions - Should Select NONE',
            'distance': 2.0,
            'ttc': 5.0,
            'risk_level': RiskLevel.LOW,
            'expected_strategy': AvoidanceStrategy.NONE
        }
    ]
    
    passed_tests = 0
    total_tests = len(test_cases)
    
    for test_case in test_cases:
        print(f"\nTest: {test_case['name']}")
        
        detection = MockObjectDetection()
        detection.class_name = "test_object"
        detection.confidence = 0.8
        detection.distance = test_case['distance']
        detection.relative_velocity = MockROSVector3(x=-0.3, y=0.0, z=0.0)
        
        risk_factors = RiskFactors(
            distance_risk=0.8 if test_case['risk_level'] == RiskLevel.CRITICAL else 0.5,
            velocity_risk=0.5,
            object_type_risk=0.5,
            trajectory_risk=0.5,
            time_to_collision=test_case['ttc'],
            lateral_clearance=0.4
        )
        
        if test_case['risk_level'] == RiskLevel.LOW:
            risk_assessments = []  # No high-risk objects
        else:
            risk_assessments = [(detection, risk_factors, test_case['risk_level'])]
        
        strategy, trajectory = planner.plan_avoidance(risk_assessments, vehicle_state)
        
        if strategy == test_case['expected_strategy']:
            print(f"  ✓ PASSED: Selected {strategy.value} as expected")
            passed_tests += 1
        else:
            print(f"  ✗ FAILED: Expected {test_case['expected_strategy'].value}, got {strategy.value}")
        
        print(f"  Distance: {test_case['distance']:.2f}m, TTC: {test_case['ttc']:.2f}s")
    
    print(f"\nRequirement 3.3 Results: {passed_tests}/{total_tests} tests passed")
    return passed_tests == total_tests


def test_requirement_3_4_safety_margins():
    """Test Requirement 3.4: Safe distance maintenance (50cm)."""
    print("\n" + "="*60)
    print("TESTING REQUIREMENT 3.4: SAFETY MARGINS")
    print("="*60)
    
    planner = AvoidancePlanner()
    
    # Test 1: Configuration meets requirement
    required_margin = 0.5  # 50cm
    config_test = planner.minimum_safety_margin >= required_margin
    
    print(f"Configuration Test:")
    print(f"  Required margin: {required_margin:.2f}m")
    print(f"  Configured margin: {planner.minimum_safety_margin:.2f}m")
    print(f"  ✓ PASSED" if config_test else f"  ✗ FAILED")
    
    # Test 2: Trajectory validation enforces safety margins
    print(f"\nTrajectory Validation Test:")
    
    # Create unsafe trajectory (too close to obstacle)
    unsafe_trajectory = AvoidanceTrajectory(
        waypoints=[
            MockROSPoint(x=0.0, y=0.0, z=0.0),
            MockROSPoint(x=0.3, y=0.0, z=0.0),
            MockROSPoint(x=0.6, y=0.0, z=0.0)
        ],
        velocities=[0.5, 0.4, 0.3],
        angular_velocities=[0.0, 0.0, 0.0],
        timestamps=[time.time(), time.time() + 0.5, time.time() + 1.0],
        total_duration=1.0,
        safety_margin=0.3,  # Below required margin
        strategy=AvoidanceStrategy.SLOW
    )
    
    # Create obstacle too close
    detection = MockObjectDetection()
    detection.distance = 0.4  # 40cm - violates 50cm requirement
    detection.relative_velocity = MockROSVector3(x=0.0, y=0.0, z=0.0)
    
    risk_factors = RiskFactors(
        distance_risk=0.8, velocity_risk=0.3, object_type_risk=0.5,
        trajectory_risk=0.6, time_to_collision=2.0, 
        lateral_clearance=0.3  # Below required 50cm
    )
    
    high_risk_objects = [(detection, risk_factors, RiskLevel.HIGH)]
    
    is_safe = planner._validate_trajectory_safety(
        unsafe_trajectory, high_risk_objects, time.time()
    )
    
    validation_test = not is_safe  # Should be rejected as unsafe
    
    print(f"  Unsafe trajectory (30cm clearance): {'✓ REJECTED' if validation_test else '✗ ACCEPTED'}")
    
    # Test 3: Safe trajectory is accepted
    safe_trajectory = AvoidanceTrajectory(
        waypoints=[
            MockROSPoint(x=0.0, y=0.0, z=0.0),
            MockROSPoint(x=0.5, y=0.8, z=0.0),
            MockROSPoint(x=1.0, y=0.0, z=0.0)
        ],
        velocities=[0.5, 0.4, 0.3],
        angular_velocities=[0.0, 0.2, -0.2],
        timestamps=[time.time(), time.time() + 0.5, time.time() + 1.0],
        total_duration=1.0,
        safety_margin=0.6,  # Above required margin
        strategy=AvoidanceStrategy.SWERVE
    )
    
    # Update obstacle to maintain safe distance
    risk_factors.lateral_clearance = 0.7  # Above required 50cm
    
    is_safe = planner._validate_trajectory_safety(
        safe_trajectory, high_risk_objects, time.time()
    )
    
    safe_validation_test = is_safe  # Should be accepted as safe
    
    print(f"  Safe trajectory (70cm clearance): {'✓ ACCEPTED' if safe_validation_test else '✗ REJECTED'}")
    
    all_passed = config_test and validation_test and safe_validation_test
    print(f"\nRequirement 3.4 Results: {'✓ ALL PASSED' if all_passed else '✗ SOME FAILED'}")
    return all_passed


def test_requirement_3_5_return_to_normal():
    """Test Requirement 3.5: Return to normal lane following."""
    print("\n" + "="*60)
    print("TESTING REQUIREMENT 3.5: RETURN TO NORMAL")
    print("="*60)
    
    planner = AvoidancePlanner()
    
    vehicle_state = VehicleState(
        position=MockROSPoint(x=0.0, y=0.0, z=0.0),
        velocity=MockROSVector3(x=0.8, y=0.0, z=0.0),
        heading=0.0,
        timestamp=time.time()
    )
    
    # Step 1: Start with high-risk situation
    detection = MockObjectDetection()
    detection.class_name = "duckiebot"
    detection.confidence = 0.8
    detection.distance = 0.6
    detection.relative_velocity = MockROSVector3(x=-0.3, y=0.0, z=0.0)
    
    risk_factors = RiskFactors(
        distance_risk=0.7, velocity_risk=0.5, object_type_risk=0.8,
        trajectory_risk=0.6, time_to_collision=2.0, lateral_clearance=0.4
    )
    
    risk_assessments = [(detection, risk_factors, RiskLevel.HIGH)]
    
    # Should trigger avoidance
    strategy1, trajectory1 = planner.plan_avoidance(risk_assessments, vehicle_state)
    avoidance_triggered = strategy1 != AvoidanceStrategy.NONE
    
    print(f"Step 1 - Avoidance Triggered:")
    print(f"  Strategy: {strategy1.value}")
    print(f"  State: {planner.current_state.value}")
    print(f"  ✓ PASSED" if avoidance_triggered else "  ✗ FAILED")
    
    # Step 2: Simulate path becoming clear
    clear_risk_assessments = []  # No objects
    
    # Multiple calls to simulate time passing with clear path
    for i in range(3):
        time.sleep(0.1)  # Small delay
        strategy_clear, trajectory_clear = planner.plan_avoidance(
            clear_risk_assessments, vehicle_state
        )
    
    # Should return to normal
    returned_to_normal = strategy_clear == AvoidanceStrategy.NONE
    
    print(f"\nStep 2 - Return to Normal:")
    print(f"  Final strategy: {strategy_clear.value}")
    print(f"  Final state: {planner.current_state.value}")
    print(f"  ✓ PASSED" if returned_to_normal else "  ✗ FAILED")
    
    all_passed = avoidance_triggered and returned_to_normal
    print(f"\nRequirement 3.5 Results: {'✓ ALL PASSED' if all_passed else '✗ SOME FAILED'}")
    return all_passed


def test_trajectory_generation():
    """Test trajectory generation for all strategies."""
    print("\n" + "="*60)
    print("TESTING TRAJECTORY GENERATION")
    print("="*60)
    
    planner = AvoidancePlanner()
    
    vehicle_state = VehicleState(
        position=MockROSPoint(x=0.0, y=0.0, z=0.0),
        velocity=MockROSVector3(x=0.8, y=0.0, z=0.0),
        heading=0.0,
        timestamp=time.time()
    )
    
    tests_passed = 0
    total_tests = 3
    
    # Test STOP trajectory
    print(f"\nTesting STOP trajectory generation:")
    try:
        stop_trajectory = planner._generate_stop_trajectory(vehicle_state, time.time())
        
        stop_tests = [
            len(stop_trajectory.waypoints) > 0,
            stop_trajectory.total_duration > 0,
            stop_trajectory.velocities[-1] <= 0.1,  # Should end with low/zero velocity
            all(v >= 0 for v in stop_trajectory.velocities),  # All velocities non-negative
            stop_trajectory.strategy == AvoidanceStrategy.STOP
        ]
        
        if all(stop_tests):
            print(f"  ✓ PASSED: {len(stop_trajectory.waypoints)} waypoints, {stop_trajectory.total_duration:.2f}s")
            tests_passed += 1
        else:
            print(f"  ✗ FAILED: Some trajectory properties invalid")
    except Exception as e:
        print(f"  ✗ FAILED: Exception {e}")
    
    # Test SLOW trajectory
    print(f"\nTesting SLOW trajectory generation:")
    try:
        slow_trajectory = planner._generate_slow_trajectory(vehicle_state, [], time.time())
        
        initial_speed = vehicle_state.velocity.x
        target_speed = initial_speed * planner.slow_speed_factor
        
        slow_tests = [
            len(slow_trajectory.waypoints) > 0,
            slow_trajectory.total_duration > 0,
            abs(slow_trajectory.velocities[-1] - target_speed) < 0.2,  # Close to target speed
            slow_trajectory.strategy == AvoidanceStrategy.SLOW
        ]
        
        if all(slow_tests):
            print(f"  ✓ PASSED: {len(slow_trajectory.waypoints)} waypoints, target speed {target_speed:.2f}m/s")
            tests_passed += 1
        else:
            print(f"  ✗ FAILED: Some trajectory properties invalid")
    except Exception as e:
        print(f"  ✗ FAILED: Exception {e}")
    
    # Test SWERVE trajectory
    print(f"\nTesting SWERVE trajectory generation:")
    try:
        swerve_trajectory = planner._generate_swerve_trajectory(vehicle_state, [], None, time.time())
        
        y_positions = [wp.y for wp in swerve_trajectory.waypoints]
        max_lateral_offset = max(abs(y) for y in y_positions)
        
        swerve_tests = [
            len(swerve_trajectory.waypoints) > 0,
            swerve_trajectory.total_duration > 0,
            max_lateral_offset > 0.1,  # Should have lateral motion
            abs(y_positions[-1]) < 0.2,  # Should return close to center
            all(abs(av) <= planner.max_angular_velocity for av in swerve_trajectory.angular_velocities),
            swerve_trajectory.strategy == AvoidanceStrategy.SWERVE
        ]
        
        if all(swerve_tests):
            print(f"  ✓ PASSED: {len(swerve_trajectory.waypoints)} waypoints, max offset {max_lateral_offset:.2f}m")
            tests_passed += 1
        else:
            print(f"  ✗ FAILED: Some trajectory properties invalid")
    except Exception as e:
        print(f"  ✗ FAILED: Exception {e}")
    
    print(f"\nTrajectory Generation Results: {tests_passed}/{total_tests} tests passed")
    return tests_passed == total_tests


def test_performance_metrics():
    """Test performance monitoring and metrics collection."""
    print("\n" + "="*60)
    print("TESTING PERFORMANCE METRICS")
    print("="*60)
    
    planner = AvoidancePlanner()
    
    # Get initial metrics
    metrics = planner.get_performance_metrics()
    
    required_fields = [
        'timestamp', 'current_state', 'active_strategy', 'total_executions',
        'success_rate', 'strategy_distribution', 'safety_margins', 'thresholds'
    ]
    
    metrics_test = all(field in metrics for field in required_fields)
    
    print(f"Metrics Structure Test:")
    for field in required_fields:
        present = field in metrics
        print(f"  {field}: {'✓' if present else '✗'}")
    
    print(f"\nSample Metrics:")
    print(f"  Current state: {metrics['current_state']}")
    print(f"  Success rate: {metrics['success_rate']:.1f}%")
    print(f"  Safety margins: {metrics['safety_margins']}")
    
    print(f"\nPerformance Metrics Results: {'✓ PASSED' if metrics_test else '✗ FAILED'}")
    return metrics_test


def main():
    """Run all validation tests."""
    print("AVOIDANCE PLANNER IMPLEMENTATION VALIDATION")
    print("=" * 80)
    print("Validating implementation against requirements 3.3, 3.4, and 3.5")
    print()
    
    test_results = []
    
    # Run all tests
    test_results.append(("Requirement 3.3 - Strategy Selection", test_requirement_3_3_strategy_selection()))
    test_results.append(("Requirement 3.4 - Safety Margins", test_requirement_3_4_safety_margins()))
    test_results.append(("Requirement 3.5 - Return to Normal", test_requirement_3_5_return_to_normal()))
    test_results.append(("Trajectory Generation", test_trajectory_generation()))
    test_results.append(("Performance Metrics", test_performance_metrics()))
    
    # Print summary
    print("\n" + "=" * 80)
    print("VALIDATION SUMMARY")
    print("=" * 80)
    
    passed_tests = 0
    total_tests = len(test_results)
    
    for test_name, result in test_results:
        status = "✓ PASSED" if result else "✗ FAILED"
        print(f"{test_name}: {status}")
        if result:
            passed_tests += 1
    
    print(f"\nOverall Results: {passed_tests}/{total_tests} test suites passed")
    
    if passed_tests == total_tests:
        print("\n🎉 ALL VALIDATIONS PASSED!")
        print("The avoidance planner implementation meets all requirements:")
        print("  ✓ 3.3: Selects appropriate avoidance strategies (stop, slow, swerve)")
        print("  ✓ 3.4: Maintains safe distances of at least 50cm from obstacles")
        print("  ✓ 3.5: Returns to normal lane following when path is clear")
        print("  ✓ Generates smooth trajectories for all strategies")
        print("  ✓ Provides comprehensive performance monitoring")
    else:
        print(f"\n⚠️  {total_tests - passed_tests} VALIDATION(S) FAILED")
        print("Review the implementation to address failing requirements.")
    
    print("=" * 80)
    return passed_tests == total_tests


if __name__ == '__main__':
    success = main()
    exit(0 if success else 1)
#!/usr/bin/env python3
"""
Unit tests for Enhanced Vehicle Model accuracy and functionality.

Tests the enhanced vehicle model's ability to predict motion, apply constraints,
and integrate calibration data for accurate Duckiebot dynamics modeling.
"""

import unittest
import numpy as np
import sys
import os
import tempfile
# import yaml  # Not available in test environment

# Add the lane_controller package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'include'))

from lane_controller.enhanced_vehicle_model import (
    EnhancedVehicleModel, VehicleParameters, VehicleState
)


class TestEnhancedVehicleModel(unittest.TestCase):
    """Test cases for enhanced vehicle model"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.default_params = VehicleParameters(
            wheelbase=0.1,
            wheel_radius=0.0318,
            vehicle_width=0.13,
            vehicle_length=0.18,
            mass=1.2,
            max_linear_velocity=0.5,
            max_angular_velocity=2.0,
            max_linear_acceleration=1.0,
            max_angular_acceleration=3.0,
            moment_of_inertia=0.01,
            friction_coefficient=0.8,
            drag_coefficient=0.1,
            motor_time_constant=0.1,
            motor_deadband=0.05,
            motor_saturation=1.0
        )
        self.vehicle_model = EnhancedVehicleModel(self.default_params)
    
    def test_vehicle_model_initialization(self):
        """Test vehicle model initializes correctly"""
        self.assertIsNotNone(self.vehicle_model)
        self.assertEqual(self.vehicle_model.params.wheelbase, 0.1)
        self.assertEqual(self.vehicle_model.params.wheel_radius, 0.0318)
        self.assertIsNotNone(self.vehicle_model.state)
    
    def test_parameter_validation(self):
        """Test parameter validation functionality"""
        # Test with valid parameters
        valid_model = EnhancedVehicleModel(self.default_params)
        diagnostics = valid_model.get_model_diagnostics()
        self.assertTrue(diagnostics['parameters_valid'])
        
        # Test with invalid parameters
        invalid_params = VehicleParameters(
            wheelbase=-0.1,  # Invalid negative wheelbase
            wheel_radius=0.0318,
            mass=-1.0  # Invalid negative mass
        )
        invalid_model = EnhancedVehicleModel(invalid_params)
        invalid_diagnostics = invalid_model.get_model_diagnostics()
        self.assertFalse(invalid_diagnostics['parameters_valid'])
        
        print(f"Parameter validation test - valid: {diagnostics['parameters_valid']}, "
              f"invalid: {invalid_diagnostics['parameters_valid']}")
    
    def test_differential_drive_kinematics(self):
        """Test differential drive kinematic calculations"""
        # Test forward kinematics
        left_wheel_vel = 1.0  # rad/s
        right_wheel_vel = 2.0  # rad/s
        
        v, omega = self.vehicle_model._diff_drive_forward_kinematics(left_wheel_vel, right_wheel_vel)
        
        # Expected values based on differential drive equations
        expected_v = self.default_params.wheel_radius * (left_wheel_vel + right_wheel_vel) / 2.0
        expected_omega = self.default_params.wheel_radius * (right_wheel_vel - left_wheel_vel) / self.default_params.wheelbase
        
        self.assertAlmostEqual(v, expected_v, places=4)
        self.assertAlmostEqual(omega, expected_omega, places=4)
        
        # Test inverse kinematics
        left_calc, right_calc = self.vehicle_model._diff_drive_inverse_kinematics(v, omega)
        
        self.assertAlmostEqual(left_calc, left_wheel_vel, places=4)
        self.assertAlmostEqual(right_calc, right_wheel_vel, places=4)
        
        print(f"Kinematics test - v: {v:.4f}, omega: {omega:.4f}, "
              f"wheels: [{left_calc:.4f}, {right_calc:.4f}]")
    
    def test_motion_prediction_straight_line(self):
        """Test motion prediction for straight line motion"""
        # Initialize state
        initial_state = VehicleState(
            x=0.0, y=0.0, theta=0.0,
            lateral_error=0.0, heading_error=0.0,
            linear_velocity=0.0, angular_velocity=0.0
        )
        self.vehicle_model.update_state(initial_state)
        
        # Predict straight line motion
        control_input = (0.2, 0.0)  # 0.2 m/s forward, no rotation
        dt = 0.1
        
        predicted_state = self.vehicle_model.predict_motion(control_input, dt)
        
        # Check predictions
        self.assertGreater(predicted_state.x, 0.0, "Should move forward")
        self.assertAlmostEqual(predicted_state.y, 0.0, places=3, msg="Should not move sideways")
        self.assertAlmostEqual(predicted_state.theta, 0.0, places=3, msg="Should maintain heading")
        self.assertAlmostEqual(predicted_state.linear_velocity, 0.2, places=2, msg="Should match commanded velocity")
        
        print(f"Straight line prediction - x: {predicted_state.x:.4f}, y: {predicted_state.y:.4f}, "
              f"theta: {predicted_state.theta:.4f}, v: {predicted_state.linear_velocity:.3f}")
    
    def test_motion_prediction_turning(self):
        """Test motion prediction for turning motion"""
        # Initialize state
        initial_state = VehicleState(
            x=0.0, y=0.0, theta=0.0,
            lateral_error=0.0, heading_error=0.0,
            linear_velocity=0.0, angular_velocity=0.0
        )
        self.vehicle_model.update_state(initial_state)
        
        # Predict turning motion
        control_input = (0.1, 1.0)  # 0.1 m/s forward, 1.0 rad/s rotation
        dt = 0.1
        
        predicted_state = self.vehicle_model.predict_motion(control_input, dt)
        
        # Check predictions
        self.assertGreater(predicted_state.theta, 0.0, "Should turn left (positive theta)")
        self.assertAlmostEqual(predicted_state.angular_velocity, 1.0, places=2, msg="Should match commanded angular velocity")
        
        print(f"Turning prediction - x: {predicted_state.x:.4f}, y: {predicted_state.y:.4f}, "
              f"theta: {predicted_state.theta:.4f}, omega: {predicted_state.angular_velocity:.3f}")
    
    def test_constraint_checking(self):
        """Test constraint violation detection"""
        # Test state within constraints
        valid_state = VehicleState(
            linear_velocity=0.3,  # Within max velocity
            angular_velocity=1.5,  # Within max angular velocity
            timestamp=1.0
        )
        
        violations = self.vehicle_model._check_state_constraints(valid_state)
        self.assertEqual(len(violations), 0, "Valid state should have no violations")
        
        # Test state violating constraints
        invalid_state = VehicleState(
            linear_velocity=0.8,  # Exceeds max velocity (0.5)
            angular_velocity=3.0,  # Exceeds max angular velocity (2.0)
            timestamp=2.0
        )
        
        violations = self.vehicle_model._check_state_constraints(invalid_state)
        self.assertGreater(len(violations), 0, "Invalid state should have violations")
        
        print(f"Constraint test - valid violations: {len(violations)}, "
              f"invalid violations: {len(violations)}")
    
    def test_motor_dynamics(self):
        """Test motor dynamics and calibration application"""
        # Test with default calibration
        v_cmd, omega_cmd = 0.2, 0.5
        dt = 0.1
        
        v_actual, omega_actual = self.vehicle_model._apply_motor_dynamics(v_cmd, omega_cmd, dt)
        
        # Motor dynamics should modify the commanded values
        self.assertIsInstance(v_actual, float)
        self.assertIsInstance(omega_actual, float)
        self.assertGreaterEqual(v_actual, 0.0)
        
        # Test with motor calibration
        self.vehicle_model.params.trim_left = 0.1
        self.vehicle_model.params.trim_right = -0.05
        self.vehicle_model.params.gain_left = 0.9
        self.vehicle_model.params.gain_right = 1.1
        
        v_calib, omega_calib = self.vehicle_model._apply_motor_dynamics(v_cmd, omega_cmd, dt)
        
        # Calibrated values should be different
        self.assertNotEqual(v_actual, v_calib)
        
        print(f"Motor dynamics test - original: [{v_actual:.3f}, {omega_actual:.3f}], "
              f"calibrated: [{v_calib:.3f}, {omega_calib:.3f}]")
    
    def test_kinematic_constraints_retrieval(self):
        """Test kinematic constraints retrieval"""
        constraints = self.vehicle_model.get_kinematic_constraints()
        
        # Check constraint structure
        self.assertIn('linear_velocity', constraints)
        self.assertIn('angular_velocity', constraints)
        self.assertIn('linear_acceleration', constraints)
        self.assertIn('angular_acceleration', constraints)
        
        # Check constraint values
        v_min, v_max = constraints['linear_velocity']
        self.assertEqual(v_min, 0.0)
        self.assertEqual(v_max, self.default_params.max_linear_velocity)
        
        omega_min, omega_max = constraints['angular_velocity']
        self.assertEqual(omega_min, -self.default_params.max_angular_velocity)
        self.assertEqual(omega_max, self.default_params.max_angular_velocity)
        
        print(f"Kinematic constraints - v: [{v_min:.1f}, {v_max:.1f}], "
              f"omega: [{omega_min:.1f}, {omega_max:.1f}]")
    
    def test_dynamic_constraints_retrieval(self):
        """Test dynamic constraints retrieval"""
        constraints = self.vehicle_model.get_dynamic_constraints()
        
        # Check constraint structure
        self.assertIn('max_lateral_acceleration', constraints)
        self.assertIn('max_centripetal_acceleration', constraints)
        self.assertIn('motor_time_constant', constraints)
        self.assertIn('motor_deadband', constraints)
        
        # Check constraint values
        max_lat_accel = constraints['max_lateral_acceleration']
        expected_lat_accel = self.default_params.friction_coefficient * 9.81
        self.assertAlmostEqual(max_lat_accel, expected_lat_accel, places=2)
        
        print(f"Dynamic constraints - lat_accel: {max_lat_accel:.2f}, "
              f"motor_tau: {constraints['motor_time_constant']:.2f}")
    
    def test_calibration_data_loading(self):
        """Test calibration data loading functionality"""
        # Test that the method exists and runs without yaml
        self.vehicle_model._load_calibration_data()
        
        # Test that calibration data structure exists
        self.assertIsInstance(self.vehicle_model.calibration_data, dict)
        
        print(f"Calibration test - loaded data keys: {list(self.vehicle_model.calibration_data.keys())}")
    
    def test_model_diagnostics(self):
        """Test model diagnostics reporting"""
        diagnostics = self.vehicle_model.get_model_diagnostics()
        
        # Check diagnostic structure
        required_keys = [
            'parameters_valid', 'calibration_loaded', 'recent_constraint_violations',
            'recent_predictions', 'prediction_success_rate', 'wheelbase', 'wheel_radius'
        ]
        
        for key in required_keys:
            self.assertIn(key, diagnostics, f"Missing diagnostic key: {key}")
        
        # Check diagnostic values
        self.assertIsInstance(diagnostics['parameters_valid'], bool)
        self.assertIsInstance(diagnostics['prediction_success_rate'], float)
        self.assertGreaterEqual(diagnostics['prediction_success_rate'], 0.0)
        self.assertLessEqual(diagnostics['prediction_success_rate'], 1.0)
        
        print(f"Diagnostics test - valid: {diagnostics['parameters_valid']}, "
              f"success_rate: {diagnostics['prediction_success_rate']:.2f}")
    
    def test_angle_normalization(self):
        """Test angle normalization functionality"""
        # Test various angles
        test_angles = [0.0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi, -np.pi/2, -np.pi, -3*np.pi/2]
        
        for angle in test_angles:
            normalized = self.vehicle_model._normalize_angle(angle)
            self.assertGreaterEqual(normalized, -np.pi, f"Angle {angle} normalized to {normalized} < -π")
            self.assertLessEqual(normalized, np.pi, f"Angle {angle} normalized to {normalized} > π")
        
        # Test specific cases
        self.assertAlmostEqual(self.vehicle_model._normalize_angle(2*np.pi), 0.0, places=6)
        self.assertAlmostEqual(self.vehicle_model._normalize_angle(-2*np.pi), 0.0, places=6)
        self.assertAlmostEqual(self.vehicle_model._normalize_angle(3*np.pi), -np.pi, places=6)
        
        print("Angle normalization test passed")


class TestVehicleModelIntegration(unittest.TestCase):
    """Integration tests for vehicle model with control systems"""
    
    def setUp(self):
        """Set up integration test fixtures"""
        self.vehicle_model = EnhancedVehicleModel()
    
    def test_state_update_sequence(self):
        """Test sequence of state updates"""
        states = [
            VehicleState(lateral_error=0.0, heading_error=0.0, linear_velocity=0.1, timestamp=1.0),
            VehicleState(lateral_error=0.02, heading_error=0.1, linear_velocity=0.15, timestamp=2.0),
            VehicleState(lateral_error=0.01, heading_error=0.05, linear_velocity=0.2, timestamp=3.0),
        ]
        
        for state in states:
            self.vehicle_model.update_state(state)
            
            # Verify state was updated
            self.assertEqual(self.vehicle_model.state.timestamp, state.timestamp)
            self.assertEqual(self.vehicle_model.state.lateral_error, state.lateral_error)
        
        print(f"State sequence test - final state: d={self.vehicle_model.state.lateral_error:.3f}, "
              f"phi={self.vehicle_model.state.heading_error:.3f}")
    
    def test_prediction_validation_sequence(self):
        """Test prediction and validation sequence"""
        # Initialize with known state
        initial_state = VehicleState(
            x=0.0, y=0.0, theta=0.0,
            lateral_error=0.05, heading_error=0.1,
            linear_velocity=0.1, angular_velocity=0.0,
            timestamp=1.0
        )
        self.vehicle_model.update_state(initial_state)
        
        # Perform multiple predictions
        control_inputs = [(0.2, 0.0), (0.2, 0.5), (0.15, -0.3), (0.1, 0.0)]
        dt = 0.1
        
        for i, control_input in enumerate(control_inputs):
            predicted_state = self.vehicle_model.predict_motion(control_input, dt)
            
            # Verify prediction is valid
            self.assertIsNotNone(predicted_state)
            self.assertTrue(np.isfinite(predicted_state.x))
            self.assertTrue(np.isfinite(predicted_state.y))
            self.assertTrue(np.isfinite(predicted_state.theta))
            
            # Update state for next prediction
            self.vehicle_model.update_state(predicted_state)
        
        # Check prediction history
        diagnostics = self.vehicle_model.get_model_diagnostics()
        self.assertGreater(diagnostics['recent_predictions'], 0)
        
        print(f"Prediction sequence test - predictions: {diagnostics['recent_predictions']}, "
              f"success_rate: {diagnostics['prediction_success_rate']:.2f}")


def run_vehicle_model_tests():
    """Run all vehicle model tests and return results"""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestEnhancedVehicleModel))
    suite.addTests(loader.loadTestsFromTestCase(TestVehicleModelIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    print("Running Enhanced Vehicle Model Tests...")
    print("=" * 60)
    
    success = run_vehicle_model_tests()
    
    print("=" * 60)
    if success:
        print("✓ All vehicle model tests passed!")
    else:
        print("✗ Some vehicle model tests failed!")
    
    exit(0 if success else 1)
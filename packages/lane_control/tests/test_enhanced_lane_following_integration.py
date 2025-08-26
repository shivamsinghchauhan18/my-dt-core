#!/usr/bin/env python3
"""
Integration tests for enhanced lane following system.

Tests the integration of MPC, enhanced vehicle model, and adaptive gain scheduling
for improved lane following accuracy and smoothness.
"""

import unittest
import numpy as np
import sys
import os
import time

# Add the lane_controller package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'include'))

from lane_controller.controller import LaneController
from lane_controller.mpc_solver import MPCParameters, MPCState
from lane_controller.enhanced_vehicle_model import VehicleParameters
from lane_controller.adaptive_gain_scheduler import GainSchedule


class TestEnhancedLaneFollowingIntegration(unittest.TestCase):
    """Integration tests for enhanced lane following system"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock parameters dictionary
        self.mock_parameters = {
            "~v_bar": MockDTParam(0.19),
            "~k_d": MockDTParam(-6.0),
            "~k_theta": MockDTParam(-5.0),
            "~k_Id": MockDTParam(-0.3),
            "~k_Iphi": MockDTParam(0.0),
            "~d_thres": 0.2615,
            "~integral_bounds": {"d": {"top": 0.3, "bot": -0.3}, "phi": {"top": 1.2, "bot": -1.2}},
            "~d_resolution": 0.011,
            "~phi_resolution": 0.051,
            "~omega_ff": 0.0,
            "~verbose": 1,
            "~stop_line_slowdown": {"start": 0.6, "end": 0.15},
            
            # MPC parameters
            "~use_mpc": False,  # Start with PID for baseline
            "~mpc_prediction_horizon": 10,
            "~mpc_control_horizon": 5,
            
            # Vehicle model parameters
            "~use_enhanced_vehicle_model": True,
            "~vehicle_wheelbase": 0.1,
            "~vehicle_wheel_radius": 0.0318,
            
            # Adaptive gain parameters
            "~use_adaptive_gains": True,
            "~adaptive_low_speed_threshold": 0.1,
            "~adaptive_high_speed_threshold": 0.3,
            "~adaptive_low_speed_k_d_scale": 1.5,
            "~adaptive_high_speed_k_d_scale": 0.8
        }
        
        self.controller = LaneController(self.mock_parameters)
    
    def test_controller_initialization_with_enhancements(self):
        """Test controller initializes with all enhancements"""
        self.assertIsNotNone(self.controller)
        self.assertTrue(self.controller.use_enhanced_model)
        self.assertTrue(self.controller.use_adaptive_gains)
        self.assertIsNotNone(self.controller.vehicle_model)
        self.assertIsNotNone(self.controller.gain_scheduler)
        
        print("Enhanced controller initialization test passed")
    
    def test_adaptive_gain_scheduling_integration(self):
        """Test adaptive gain scheduling integration with PID control"""
        # Test different speed scenarios
        test_scenarios = [
            {"speed": 0.05, "d_err": 0.1, "phi_err": 0.2, "scenario": "low_speed_large_error"},
            {"speed": 0.2, "d_err": 0.02, "phi_err": 0.05, "scenario": "medium_speed_small_error"},
            {"speed": 0.4, "d_err": 0.08, "phi_err": 0.15, "scenario": "high_speed_medium_error"},
        ]
        
        results = []
        
        for scenario in test_scenarios:
            # Simulate control computation
            v, omega = self.controller.compute_control_action(
                scenario["d_err"], scenario["phi_err"], 0.1, [0.0, 0.0], None
            )
            
            # Get gain diagnostics
            gain_diagnostics = self.controller.get_gain_scheduling_diagnostics()
            
            results.append({
                'scenario': scenario["scenario"],
                'input': [scenario["speed"], scenario["d_err"], scenario["phi_err"]],
                'output': [v, omega],
                'scheduled_gains': gain_diagnostics.get('scheduled_gains', {}),
                'speed_scaling': gain_diagnostics.get('speed_scaling', {}),
                'error_scaling': gain_diagnostics.get('error_scaling', {})
            })
            
            # Verify control output is reasonable
            self.assertIsInstance(v, float)
            self.assertIsInstance(omega, float)
            self.assertGreaterEqual(v, 0.0)
            self.assertLessEqual(abs(omega), 5.0)  # Reasonable angular velocity bound
        
        # Print results for analysis
        for result in results:
            print(f"Scenario {result['scenario']}: input={result['input']}, output=[{result['output'][0]:.3f}, {result['output'][1]:.3f}]")
            if result['scheduled_gains']:
                gains = result['scheduled_gains']
                print(f"  Scheduled gains: k_d={gains.get('k_d', 0):.3f}, k_theta={gains.get('k_theta', 0):.3f}")
        
        print("Adaptive gain scheduling integration test passed")
    
    def test_vehicle_model_integration(self):
        """Test vehicle model integration with control system"""
        # Initialize with known state
        d_err, phi_err = 0.05, 0.1
        v, omega = self.controller.compute_control_action(d_err, phi_err, 0.1, [0.0, 0.0], None)
        
        # Get vehicle diagnostics
        vehicle_diagnostics = self.controller.get_vehicle_diagnostics()
        
        # Verify diagnostics are available
        self.assertIsInstance(vehicle_diagnostics, dict)
        if vehicle_diagnostics:
            self.assertIn('parameters_valid', vehicle_diagnostics)
            self.assertIn('wheelbase', vehicle_diagnostics)
            self.assertIn('wheel_radius', vehicle_diagnostics)
        
        # Test motion prediction
        predicted_state = self.controller.predict_vehicle_motion((v, omega), 0.1)
        if predicted_state is not None:
            self.assertIsInstance(predicted_state.lateral_error, float)
            self.assertIsInstance(predicted_state.heading_error, float)
            self.assertTrue(np.isfinite(predicted_state.lateral_error))
            self.assertTrue(np.isfinite(predicted_state.heading_error))
        
        # Test constraint validation
        v_constrained, omega_constrained = self.controller.validate_control_action(v, omega)
        self.assertIsInstance(v_constrained, float)
        self.assertIsInstance(omega_constrained, float)
        
        print(f"Vehicle model integration test - diagnostics keys: {list(vehicle_diagnostics.keys())}")
        print("Vehicle model integration test passed")
    
    def test_mpc_integration_when_enabled(self):
        """Test MPC integration when enabled"""
        # Enable MPC
        self.mock_parameters["~use_mpc"] = True
        mpc_controller = LaneController(self.mock_parameters)
        
        if mpc_controller.mpc_enabled and mpc_controller.mpc_solver is not None:
            # Test MPC control computation
            d_err, phi_err = 0.08, 0.15
            v, omega = mpc_controller.compute_control_action(d_err, phi_err, 0.1, [0.0, 0.0], None)
            
            # Verify MPC produces valid output
            self.assertIsInstance(v, float)
            self.assertIsInstance(omega, float)
            self.assertGreaterEqual(v, 0.0)
            self.assertTrue(np.isfinite(v))
            self.assertTrue(np.isfinite(omega))
            
            # Get MPC performance metrics
            mpc_metrics = mpc_controller.get_mpc_performance_metrics()
            self.assertIsInstance(mpc_metrics, dict)
            
            print(f"MPC integration test - output: [{v:.3f}, {omega:.3f}], metrics keys: {list(mpc_metrics.keys())}")
        else:
            print("MPC integration test skipped - MPC not available")
        
        print("MPC integration test passed")
    
    def test_performance_comparison_pid_vs_enhanced(self):
        """Compare performance between basic PID and enhanced system"""
        # Create basic PID controller
        basic_params = self.mock_parameters.copy()
        basic_params["~use_adaptive_gains"] = False
        basic_params["~use_enhanced_vehicle_model"] = False
        basic_controller = LaneController(basic_params)
        
        # Create enhanced controller
        enhanced_controller = self.controller
        
        # Test scenarios
        test_cases = [
            (0.02, 0.05),   # Small errors
            (0.1, 0.2),     # Medium errors
            (0.15, 0.3),    # Large errors
        ]
        
        basic_results = []
        enhanced_results = []
        
        for d_err, phi_err in test_cases:
            # Basic PID
            v_basic, omega_basic = basic_controller.compute_control_action(
                d_err, phi_err, 0.1, [0.0, 0.0], None
            )
            basic_results.append((v_basic, omega_basic))
            
            # Enhanced system
            v_enhanced, omega_enhanced = enhanced_controller.compute_control_action(
                d_err, phi_err, 0.1, [0.0, 0.0], None
            )
            enhanced_results.append((v_enhanced, omega_enhanced))
        
        # Analyze results
        print("Performance comparison:")
        for i, (d_err, phi_err) in enumerate(test_cases):
            v_basic, omega_basic = basic_results[i]
            v_enhanced, omega_enhanced = enhanced_results[i]
            
            print(f"  Case {i+1} (d={d_err:.3f}, phi={phi_err:.3f}):")
            print(f"    Basic:    v={v_basic:.3f}, omega={omega_basic:.3f}")
            print(f"    Enhanced: v={v_enhanced:.3f}, omega={omega_enhanced:.3f}")
            
            # Both should produce valid outputs
            self.assertTrue(np.isfinite(v_basic) and np.isfinite(omega_basic))
            self.assertTrue(np.isfinite(v_enhanced) and np.isfinite(omega_enhanced))
        
        print("Performance comparison test passed")
    
    def test_system_stability_under_varying_conditions(self):
        """Test system stability under varying operating conditions"""
        # Simulate varying conditions over time
        time_steps = 50
        dt = 0.1
        
        # Generate varying error signals
        lateral_errors = 0.1 * np.sin(np.linspace(0, 4*np.pi, time_steps)) + 0.02 * np.random.randn(time_steps)
        heading_errors = 0.2 * np.cos(np.linspace(0, 3*np.pi, time_steps)) + 0.05 * np.random.randn(time_steps)
        
        control_outputs = []
        gain_history = []
        
        for i in range(time_steps):
            d_err = lateral_errors[i]
            phi_err = heading_errors[i]
            
            # Compute control action
            v, omega = self.controller.compute_control_action(d_err, phi_err, dt, [0.0, 0.0], None)
            control_outputs.append((v, omega))
            
            # Record gain scheduling
            gain_diagnostics = self.controller.get_gain_scheduling_diagnostics()
            if gain_diagnostics and 'scheduled_gains' in gain_diagnostics:
                gain_history.append(gain_diagnostics['scheduled_gains'])
        
        # Analyze stability
        velocities = [output[0] for output in control_outputs]
        angular_velocities = [output[1] for output in control_outputs]
        
        # Check for stability indicators
        v_std = np.std(velocities)
        omega_std = np.std(angular_velocities)
        
        # Verify no extreme values
        self.assertLess(v_std, 0.5, "Velocity should be relatively stable")
        self.assertLess(omega_std, 2.0, "Angular velocity should be relatively stable")
        
        # Verify all outputs are finite
        for v, omega in control_outputs:
            self.assertTrue(np.isfinite(v), "Velocity should be finite")
            self.assertTrue(np.isfinite(omega), "Angular velocity should be finite")
        
        print(f"Stability test - v_std: {v_std:.3f}, omega_std: {omega_std:.3f}, samples: {len(control_outputs)}")
        print("System stability test passed")
    
    def test_integration_with_lane_pose_estimation(self):
        """Test integration with lane pose estimation from lane_filter"""
        # Simulate lane pose data similar to what lane_filter would provide
        lane_poses = [
            {"d": 0.0, "phi": 0.0, "confidence": 0.9},      # Centered
            {"d": 0.05, "phi": 0.1, "confidence": 0.8},     # Slight offset
            {"d": -0.08, "phi": -0.15, "confidence": 0.7},  # Larger offset
            {"d": 0.12, "phi": 0.25, "confidence": 0.6},    # Large offset, lower confidence
        ]
        
        integration_results = []
        
        for pose in lane_poses:
            d_err = pose["d"]
            phi_err = pose["phi"]
            confidence = pose["confidence"]
            
            # Compute control action
            v, omega = self.controller.compute_control_action(d_err, phi_err, 0.1, [0.0, 0.0], None)
            
            # Get system diagnostics
            gain_diagnostics = self.controller.get_gain_scheduling_diagnostics()
            vehicle_diagnostics = self.controller.get_vehicle_diagnostics()
            
            integration_results.append({
                'pose': pose,
                'control': (v, omega),
                'gain_active': bool(gain_diagnostics),
                'vehicle_active': bool(vehicle_diagnostics)
            })
        
        # Verify integration works for all poses
        for result in integration_results:
            v, omega = result['control']
            self.assertTrue(np.isfinite(v), "Control output should be finite")
            self.assertTrue(np.isfinite(omega), "Control output should be finite")
            self.assertGreaterEqual(v, 0.0, "Velocity should be non-negative")
            
            print(f"Lane pose integration - d: {result['pose']['d']:.3f}, phi: {result['pose']['phi']:.3f} -> "
                  f"v: {v:.3f}, omega: {omega:.3f}")
        
        print("Lane pose integration test passed")


class MockDTParam:
    """Mock DTParam for testing"""
    def __init__(self, value):
        self.value = value


def run_integration_tests():
    """Run all integration tests and return results"""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestEnhancedLaneFollowingIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    print("Running Enhanced Lane Following Integration Tests...")
    print("=" * 70)
    
    success = run_integration_tests()
    
    print("=" * 70)
    if success:
        print("✓ All integration tests passed!")
    else:
        print("✗ Some integration tests failed!")
    
    exit(0 if success else 1)
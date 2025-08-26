#!/usr/bin/env python3
"""
Unit tests for MPC optimization accuracy in lane control.

Tests the MPC solver's ability to optimize control actions for various
lane following scenarios and validate constraint satisfaction.
"""

import unittest
import numpy as np
import sys
import os

# Add the lane_controller package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'include'))

from lane_controller.mpc_solver import MPCSolver, MPCParameters, MPCState, MPCResult


class TestMPCOptimizationAccuracy(unittest.TestCase):
    """Test cases for MPC optimization accuracy"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.default_params = MPCParameters(
            prediction_horizon=10,
            control_horizon=5,
            dt=0.1,
            max_velocity=0.5,
            max_angular_velocity=2.0,
            velocity_weight=1.0,
            angular_weight=1.0,
            lateral_weight=10.0,
            heading_weight=5.0,
            control_smoothness_weight=1.0,
            adaptive_horizon=False  # Disable for consistent testing
        )
        self.mpc_solver = MPCSolver(self.default_params)
    
    def test_mpc_solver_initialization(self):
        """Test MPC solver initializes correctly"""
        self.assertIsNotNone(self.mpc_solver)
        self.assertEqual(self.mpc_solver.params.prediction_horizon, 10)
        self.assertEqual(self.mpc_solver.params.control_horizon, 5)
        self.assertEqual(self.mpc_solver.current_horizon, 10)
    
    def test_straight_line_following(self):
        """Test MPC optimization for straight line following"""
        # Test case: small lateral error, no heading error
        state = MPCState(
            lateral_error=0.05,  # 5cm to the right
            heading_error=0.0,   # pointing straight
            velocity=0.2,
            angular_velocity=0.0
        )
        
        result = self.mpc_solver.solve(state)
        
        # Verify optimization completed
        self.assertIsInstance(result, MPCResult)
        self.assertTrue(result.convergence_status, "MPC should converge for simple straight line case")
        
        # Verify control bounds
        self.assertGreaterEqual(result.linear_velocity, 0.0)
        self.assertLessEqual(result.linear_velocity, self.default_params.max_velocity)
        self.assertGreaterEqual(result.angular_velocity, -self.default_params.max_angular_velocity)
        self.assertLessEqual(result.angular_velocity, self.default_params.max_angular_velocity)
        
        # For small lateral error, angular velocity should correct towards center
        self.assertLess(result.angular_velocity, 0.0, "Should turn left to correct rightward error")
        
        # Optimization should be reasonably fast
        self.assertLess(result.optimization_time, 0.1, "Optimization should complete within 100ms")
        
        print(f"Straight line test - v: {result.linear_velocity:.3f}, ω: {result.angular_velocity:.3f}, "
              f"cost: {result.cost_value:.4f}, time: {result.optimization_time*1000:.1f}ms")
    
    def test_curve_following(self):
        """Test MPC optimization for curve following"""
        # Test case: heading error indicating curve
        state = MPCState(
            lateral_error=0.0,
            heading_error=0.3,   # 17 degrees heading error
            velocity=0.15,
            angular_velocity=0.0
        )
        
        result = self.mpc_solver.solve(state)
        
        # Verify optimization completed
        self.assertTrue(result.convergence_status, "MPC should converge for curve following")
        
        # For positive heading error, should turn right (negative angular velocity)
        self.assertLess(result.angular_velocity, 0.0, "Should turn right to correct heading error")
        
        # Verify reasonable control magnitude
        self.assertLess(abs(result.angular_velocity), 1.5, "Angular velocity should be reasonable for curve")
        
        print(f"Curve following test - v: {result.linear_velocity:.3f}, ω: {result.angular_velocity:.3f}, "
              f"cost: {result.cost_value:.4f}, time: {result.optimization_time*1000:.1f}ms")
    
    def test_large_error_correction(self):
        """Test MPC optimization for large error correction"""
        # Test case: large lateral and heading errors
        state = MPCState(
            lateral_error=0.15,  # 15cm lateral error
            heading_error=-0.5,  # -28 degrees heading error
            velocity=0.1,
            angular_velocity=0.0
        )
        
        result = self.mpc_solver.solve(state)
        
        # Should still converge even with large errors
        self.assertTrue(result.convergence_status, "MPC should handle large errors")
        
        # Should produce corrective control action
        self.assertLess(result.angular_velocity, 0.0, "Should turn left for negative heading error")
        
        # Control should be within bounds despite large errors
        self.assertLessEqual(abs(result.angular_velocity), self.default_params.max_angular_velocity)
        
        print(f"Large error test - v: {result.linear_velocity:.3f}, ω: {result.angular_velocity:.3f}, "
              f"cost: {result.cost_value:.4f}, time: {result.optimization_time*1000:.1f}ms")
    
    def test_constraint_satisfaction(self):
        """Test that MPC satisfies all constraints"""
        test_states = [
            MPCState(0.1, 0.2, 0.3, 0.0),   # Mixed errors
            MPCState(-0.08, -0.3, 0.15, 0.0), # Negative errors
            MPCState(0.0, 0.0, 0.2, 0.0),   # No errors
            MPCState(0.2, 0.6, 0.05, 0.0),  # Large errors
        ]
        
        for i, state in enumerate(test_states):
            with self.subTest(test_case=i):
                result = self.mpc_solver.solve(state)
                
                # Check velocity constraints
                self.assertGreaterEqual(result.linear_velocity, 0.0, 
                                      f"Velocity should be non-negative for test {i}")
                self.assertLessEqual(result.linear_velocity, self.default_params.max_velocity,
                                   f"Velocity should be within bounds for test {i}")
                
                # Check angular velocity constraints
                self.assertGreaterEqual(result.angular_velocity, -self.default_params.max_angular_velocity,
                                      f"Angular velocity should be within bounds for test {i}")
                self.assertLessEqual(result.angular_velocity, self.default_params.max_angular_velocity,
                                   f"Angular velocity should be within bounds for test {i}")
                
                # Check constraint violations
                self.assertLessEqual(result.constraint_violations, 2, 
                                   f"Should have minimal constraint violations for test {i}")
    
    def test_optimization_performance(self):
        """Test MPC optimization performance requirements"""
        # Test multiple scenarios for performance consistency
        test_states = [
            MPCState(0.05, 0.1, 0.2, 0.0),
            MPCState(-0.03, -0.15, 0.18, 0.0),
            MPCState(0.12, 0.25, 0.1, 0.0),
            MPCState(0.0, 0.0, 0.25, 0.0),
        ]
        
        optimization_times = []
        convergence_count = 0
        
        for state in test_states:
            result = self.mpc_solver.solve(state)
            optimization_times.append(result.optimization_time)
            if result.convergence_status:
                convergence_count += 1
        
        # Performance requirements
        avg_time = np.mean(optimization_times)
        max_time = np.max(optimization_times)
        convergence_rate = convergence_count / len(test_states)
        
        # Verify performance metrics
        self.assertLess(avg_time, 0.05, f"Average optimization time should be < 50ms, got {avg_time*1000:.1f}ms")
        self.assertLess(max_time, 0.1, f"Max optimization time should be < 100ms, got {max_time*1000:.1f}ms")
        self.assertGreaterEqual(convergence_rate, 0.8, f"Convergence rate should be >= 80%, got {convergence_rate*100:.1f}%")
        
        print(f"Performance test - avg: {avg_time*1000:.1f}ms, max: {max_time*1000:.1f}ms, "
              f"convergence: {convergence_rate*100:.1f}%")
    
    def test_trajectory_prediction_accuracy(self):
        """Test accuracy of trajectory prediction"""
        state = MPCState(
            lateral_error=0.08,
            heading_error=0.2,
            velocity=0.2,
            angular_velocity=0.0
        )
        
        result = self.mpc_solver.solve(state)
        
        # Verify trajectory prediction shape
        self.assertEqual(result.predicted_trajectory.shape[0], self.default_params.prediction_horizon)
        self.assertEqual(result.predicted_trajectory.shape[1], 4)  # [d, phi, v, omega]
        
        # Verify trajectory starts from current state
        initial_pred = result.predicted_trajectory[0]
        self.assertAlmostEqual(initial_pred[0], state.lateral_error, places=3, 
                              msg="Predicted trajectory should start from current lateral error")
        
        # Verify trajectory shows error reduction over time
        final_lateral_error = result.predicted_trajectory[-1, 0]
        self.assertLess(abs(final_lateral_error), abs(state.lateral_error), 
                       "Trajectory should show lateral error reduction")
        
        print(f"Trajectory test - initial error: {state.lateral_error:.3f}, "
              f"final predicted: {final_lateral_error:.3f}")
    
    def test_adaptive_horizon_functionality(self):
        """Test adaptive horizon adjustment"""
        # Enable adaptive horizon
        adaptive_params = MPCParameters(
            prediction_horizon=10,
            control_horizon=5,
            dt=0.1,
            max_velocity=0.5,
            max_angular_velocity=2.0,
            velocity_weight=1.0,
            angular_weight=1.0,
            lateral_weight=10.0,
            heading_weight=5.0,
            control_smoothness_weight=1.0,
            adaptive_horizon=True
        )
        adaptive_solver = MPCSolver(adaptive_params)
        
        # Test high speed scenario (should increase horizon)
        high_speed_state = MPCState(0.05, 0.1, 0.35, 0.0)
        result_high = adaptive_solver.solve(high_speed_state)
        
        # Test low speed scenario (should decrease horizon)
        low_speed_state = MPCState(0.05, 0.1, 0.05, 0.0)
        result_low = adaptive_solver.solve(low_speed_state)
        
        # Both should converge
        self.assertTrue(result_high.convergence_status, "High speed case should converge")
        self.assertTrue(result_low.convergence_status, "Low speed case should converge")
        
        print(f"Adaptive horizon test - high speed horizon: {adaptive_solver.current_horizon}, "
              f"low speed horizon: {adaptive_solver.current_horizon}")
    
    def test_cost_function_behavior(self):
        """Test cost function produces reasonable values"""
        # Test different error magnitudes
        test_cases = [
            (0.0, 0.0, "no_error"),
            (0.05, 0.1, "small_error"),
            (0.15, 0.3, "large_error")
        ]
        
        costs = []
        for lateral_err, heading_err, case_name in test_cases:
            state = MPCState(lateral_err, heading_err, 0.2, 0.0)
            result = self.mpc_solver.solve(state)
            costs.append(result.cost_value)
            
            print(f"Cost test {case_name} - lateral: {lateral_err:.3f}, heading: {heading_err:.3f}, "
                  f"cost: {result.cost_value:.4f}")
        
        # Cost should increase with error magnitude
        self.assertLess(costs[0], costs[1], "Cost should increase with small errors")
        self.assertLess(costs[1], costs[2], "Cost should increase with large errors")
        
        # All costs should be finite and positive
        for i, cost in enumerate(costs):
            self.assertGreater(cost, 0, f"Cost should be positive for case {i}")
            self.assertNotEqual(cost, float('inf'), f"Cost should be finite for case {i}")


class TestMPCIntegration(unittest.TestCase):
    """Integration tests for MPC with lane controller"""
    
    def test_mpc_parameter_validation(self):
        """Test MPC parameter validation"""
        # Test invalid parameters
        invalid_params = MPCParameters(
            prediction_horizon=0,  # Invalid
            control_horizon=5,
            dt=0.1
        )
        
        # Should handle invalid parameters gracefully
        try:
            solver = MPCSolver(invalid_params)
            # If it doesn't raise an exception, it should at least use safe defaults
            self.assertGreater(solver.params.prediction_horizon, 0)
        except (ValueError, AssertionError):
            # Expected behavior for invalid parameters
            pass
    
    def test_mpc_state_edge_cases(self):
        """Test MPC with edge case states"""
        solver = MPCSolver(MPCParameters())
        
        # Test extreme states
        extreme_states = [
            MPCState(1.0, 3.0, 0.0, 0.0),    # Very large errors
            MPCState(-1.0, -3.0, 0.5, 2.0), # Large negative errors
            MPCState(0.0, 0.0, 0.0, 0.0),    # Zero state
        ]
        
        for i, state in enumerate(extreme_states):
            result = solver.solve(state)
            
            # Should produce valid results even for extreme cases
            self.assertIsInstance(result, MPCResult)
            self.assertIsFinite(result.linear_velocity)
            self.assertIsFinite(result.angular_velocity)
            
            print(f"Edge case {i} - input: [{state.lateral_error:.2f}, {state.heading_error:.2f}], "
                  f"output: [{result.linear_velocity:.3f}, {result.angular_velocity:.3f}]")


def run_mpc_tests():
    """Run all MPC tests and return results"""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestMPCOptimizationAccuracy))
    suite.addTests(loader.loadTestsFromTestCase(TestMPCIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    print("Running MPC Optimization Accuracy Tests...")
    print("=" * 60)
    
    success = run_mpc_tests()
    
    print("=" * 60)
    if success:
        print("✓ All MPC tests passed!")
    else:
        print("✗ Some MPC tests failed!")
    
    exit(0 if success else 1)
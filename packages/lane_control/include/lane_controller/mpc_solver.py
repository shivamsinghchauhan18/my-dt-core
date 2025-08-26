#!/usr/bin/env python3
"""
Model Predictive Control (MPC) Solver for Duckiebot Lane Following

This module implements an MPC solver that optimizes control actions over a prediction horizon
to achieve optimal lane following performance with trajectory optimization and constraints.
"""

import numpy as np
import time
from typing import Tuple, Optional, Dict, Any
try:
    import rospy
except ImportError:
    # Mock rospy for testing without ROS
    class MockRospy:
        @staticmethod
        def loginfo(msg, *args): print(f"[INFO] {msg % args if args else msg}")
        @staticmethod
        def logdebug(msg, *args): pass  # Suppress debug in tests
        @staticmethod
        def logwarn(msg, *args): print(f"[WARN] {msg % args if args else msg}")
        @staticmethod
        def logerr(msg, *args): print(f"[ERROR] {msg % args if args else msg}")
        @staticmethod
        def get_logger(): return None
        class core:
            @staticmethod
            def is_initialized(): return False
    rospy = MockRospy()

try:
    from scipy.optimize import minimize
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    # Simple gradient descent fallback
    def minimize(fun, x0, method=None, bounds=None, constraints=None, options=None):
        class Result:
            def __init__(self, x, fun_val, success=True):
                self.x = x
                self.fun = fun_val
                self.success = success
        
        # Simple gradient descent optimization
        x = np.array(x0)
        learning_rate = 0.01
        for _ in range(50):  # Simple iteration limit
            # Numerical gradient
            grad = np.zeros_like(x)
            eps = 1e-6
            f0 = fun(x)
            for i in range(len(x)):
                x_plus = x.copy()
                x_plus[i] += eps
                grad[i] = (fun(x_plus) - f0) / eps
            
            # Update with bounds checking
            x_new = x - learning_rate * grad
            if bounds:
                for i, (low, high) in enumerate(bounds):
                    x_new[i] = np.clip(x_new[i], low, high)
            x = x_new
        
        return Result(x, fun(x))
from dataclasses import dataclass


@dataclass
class MPCParameters:
    """MPC configuration parameters"""
    prediction_horizon: int = 10  # Number of prediction steps
    control_horizon: int = 5      # Number of control steps
    dt: float = 0.1              # Time step
    max_velocity: float = 0.5    # Maximum linear velocity
    max_angular_velocity: float = 2.0  # Maximum angular velocity
    velocity_weight: float = 1.0  # Weight for velocity tracking
    angular_weight: float = 1.0   # Weight for angular velocity
    lateral_weight: float = 10.0  # Weight for lateral error
    heading_weight: float = 5.0   # Weight for heading error
    control_smoothness_weight: float = 1.0  # Weight for control smoothness
    adaptive_horizon: bool = True  # Enable adaptive horizon based on speed


@dataclass
class MPCState:
    """Current state for MPC optimization"""
    lateral_error: float = 0.0
    heading_error: float = 0.0
    velocity: float = 0.0
    angular_velocity: float = 0.0
    timestamp: float = 0.0


@dataclass
class MPCResult:
    """MPC optimization result"""
    linear_velocity: float
    angular_velocity: float
    predicted_trajectory: np.ndarray
    optimization_time: float
    cost_value: float
    convergence_status: bool
    constraint_violations: int


class MPCSolver:
    """
    Model Predictive Control solver for Duckiebot lane following.
    
    Implements trajectory optimization with constraints for smooth and optimal
    lane following behavior.
    """
    
    def __init__(self, parameters: MPCParameters):
        """
        Initialize MPC solver with given parameters.
        
        Args:
            parameters: MPC configuration parameters
        """
        self.params = parameters
        self.state_history = []
        self.control_history = []
        self.optimization_times = []
        self.cost_history = []
        
        # Adaptive parameters
        self.current_horizon = parameters.prediction_horizon
        self.current_weights = self._initialize_weights()
        
        # Logging setup
        self.logger = rospy.get_logger() if rospy.core.is_initialized() else None
        
        rospy.loginfo("[MPCSolver] Initialized MPC solver with horizon: %d, dt: %.3f, scipy: %s", 
                     self.params.prediction_horizon, self.params.dt, SCIPY_AVAILABLE)
    
    def _initialize_weights(self) -> Dict[str, float]:
        """Initialize cost function weights"""
        return {
            'velocity': self.params.velocity_weight,
            'angular': self.params.angular_weight,
            'lateral': self.params.lateral_weight,
            'heading': self.params.heading_weight,
            'smoothness': self.params.control_smoothness_weight
        }
    
    def solve(self, current_state: MPCState, reference_trajectory: Optional[np.ndarray] = None) -> MPCResult:
        """
        Solve MPC optimization problem for current state.
        
        Args:
            current_state: Current vehicle state
            reference_trajectory: Optional reference trajectory to follow
            
        Returns:
            MPCResult containing optimal control action and diagnostics
        """
        start_time = time.time()
        
        # Update adaptive parameters
        self._update_adaptive_parameters(current_state)
        
        # Set up optimization problem
        initial_guess = self._generate_initial_guess(current_state)
        bounds = self._generate_bounds()
        constraints = self._generate_constraints()
        
        rospy.logdebug("[MPCSolver] Starting optimization with horizon: %d, state: [d=%.3f, phi=%.3f, v=%.3f]",
                      self.current_horizon, current_state.lateral_error, 
                      current_state.heading_error, current_state.velocity)
        
        try:
            # Solve optimization problem
            result = minimize(
                fun=lambda x: self._cost_function(x, current_state, reference_trajectory),
                x0=initial_guess,
                method='SLSQP',
                bounds=bounds,
                constraints=constraints,
                options={'maxiter': 100, 'ftol': 1e-6}
            )
            
            optimization_time = time.time() - start_time
            
            # Extract optimal control action
            optimal_v = result.x[0]
            optimal_omega = result.x[1]
            
            # Generate predicted trajectory
            predicted_trajectory = self._predict_trajectory(result.x, current_state)
            
            # Count constraint violations
            constraint_violations = self._count_constraint_violations(result.x)
            
            # Log optimization results
            rospy.logdebug("[MPCSolver] Optimization completed: success=%s, cost=%.4f, time=%.3fms, violations=%d",
                          result.success, result.fun, optimization_time * 1000, constraint_violations)
            
            if optimization_time > 0.05:  # Log if optimization takes too long
                rospy.logwarn("[MPCSolver] Slow optimization: %.3fms (target: <50ms)", optimization_time * 1000)
            
            # Store history for analysis
            self._update_history(current_state, [optimal_v, optimal_omega], optimization_time, result.fun)
            
            return MPCResult(
                linear_velocity=optimal_v,
                angular_velocity=optimal_omega,
                predicted_trajectory=predicted_trajectory,
                optimization_time=optimization_time,
                cost_value=result.fun,
                convergence_status=result.success,
                constraint_violations=constraint_violations
            )
            
        except Exception as e:
            rospy.logerr("[MPCSolver] Optimization failed: %s", str(e))
            # Return safe fallback control
            return MPCResult(
                linear_velocity=min(current_state.velocity, 0.1),
                angular_velocity=0.0,
                predicted_trajectory=np.zeros((self.current_horizon, 4)),
                optimization_time=time.time() - start_time,
                cost_value=float('inf'),
                convergence_status=False,
                constraint_violations=0
            )
    
    def _update_adaptive_parameters(self, state: MPCState):
        """Update MPC parameters based on current state"""
        if not self.params.adaptive_horizon:
            return
            
        # Adapt horizon based on velocity
        if state.velocity > 0.3:
            self.current_horizon = min(self.params.prediction_horizon + 5, 20)
        elif state.velocity < 0.1:
            self.current_horizon = max(self.params.prediction_horizon - 3, 5)
        else:
            self.current_horizon = self.params.prediction_horizon
            
        # Adapt weights based on error magnitude
        if abs(state.lateral_error) > 0.15:
            self.current_weights['lateral'] = self.params.lateral_weight * 2.0
        else:
            self.current_weights['lateral'] = self.params.lateral_weight
            
        if abs(state.heading_error) > 0.5:
            self.current_weights['heading'] = self.params.heading_weight * 1.5
        else:
            self.current_weights['heading'] = self.params.heading_weight
            
        rospy.logdebug("[MPCSolver] Adaptive parameters: horizon=%d, lateral_weight=%.1f, heading_weight=%.1f",
                      self.current_horizon, self.current_weights['lateral'], self.current_weights['heading'])
    
    def _generate_initial_guess(self, state: MPCState) -> np.ndarray:
        """Generate initial guess for optimization"""
        # Control variables: [v_0, omega_0, v_1, omega_1, ..., v_N-1, omega_N-1]
        control_dim = self.params.control_horizon * 2
        
        # Start with current velocity and simple angular velocity based on heading error
        initial_v = max(0.05, min(state.velocity, self.params.max_velocity))
        initial_omega = np.clip(-state.heading_error * 2.0, 
                               -self.params.max_angular_velocity, 
                               self.params.max_angular_velocity)
        
        initial_guess = np.zeros(control_dim)
        for i in range(self.params.control_horizon):
            initial_guess[2*i] = initial_v * (0.9 ** i)  # Gradually decrease velocity
            initial_guess[2*i + 1] = initial_omega * (0.8 ** i)  # Gradually decrease angular velocity
            
        return initial_guess
    
    def _generate_bounds(self) -> list:
        """Generate bounds for optimization variables"""
        bounds = []
        for i in range(self.params.control_horizon):
            # Velocity bounds
            bounds.append((0.0, self.params.max_velocity))
            # Angular velocity bounds
            bounds.append((-self.params.max_angular_velocity, self.params.max_angular_velocity))
        return bounds
    
    def _generate_constraints(self) -> list:
        """Generate constraints for optimization"""
        constraints = []
        
        # Control smoothness constraints
        def control_smoothness_constraint(x):
            violations = []
            for i in range(self.params.control_horizon - 1):
                # Velocity change constraint
                dv = abs(x[2*(i+1)] - x[2*i])
                violations.append(0.2 - dv)  # Max velocity change of 0.2 m/s per step
                
                # Angular velocity change constraint
                domega = abs(x[2*(i+1) + 1] - x[2*i + 1])
                violations.append(1.0 - domega)  # Max angular velocity change of 1.0 rad/s per step
            return np.array(violations)
        
        constraints.append({
            'type': 'ineq',
            'fun': control_smoothness_constraint
        })
        
        return constraints
    
    def _cost_function(self, x: np.ndarray, state: MPCState, reference_trajectory: Optional[np.ndarray]) -> float:
        """
        MPC cost function to minimize.
        
        Args:
            x: Control sequence [v_0, omega_0, v_1, omega_1, ...]
            state: Current state
            reference_trajectory: Optional reference trajectory
            
        Returns:
            Total cost value
        """
        total_cost = 0.0
        
        # Simulate forward to get predicted states
        predicted_states = self._simulate_forward(x, state)
        
        # State tracking costs
        for i, pred_state in enumerate(predicted_states):
            # Lateral error cost
            lateral_cost = self.current_weights['lateral'] * (pred_state[0] ** 2)
            
            # Heading error cost
            heading_cost = self.current_weights['heading'] * (pred_state[1] ** 2)
            
            # Velocity tracking cost (prefer nominal velocity)
            velocity_cost = self.current_weights['velocity'] * ((pred_state[2] - 0.2) ** 2)
            
            total_cost += lateral_cost + heading_cost + velocity_cost
        
        # Control effort costs
        for i in range(self.params.control_horizon):
            v = x[2*i]
            omega = x[2*i + 1]
            
            # Angular velocity penalty
            angular_cost = self.current_weights['angular'] * (omega ** 2)
            total_cost += angular_cost
        
        # Control smoothness costs
        for i in range(self.params.control_horizon - 1):
            dv = x[2*(i+1)] - x[2*i]
            domega = x[2*(i+1) + 1] - x[2*i + 1]
            
            smoothness_cost = self.current_weights['smoothness'] * (dv**2 + domega**2)
            total_cost += smoothness_cost
        
        return total_cost
    
    def _simulate_forward(self, control_sequence: np.ndarray, initial_state: MPCState) -> np.ndarray:
        """
        Simulate vehicle forward using control sequence.
        
        Args:
            control_sequence: Control inputs [v_0, omega_0, v_1, omega_1, ...]
            initial_state: Initial state
            
        Returns:
            Predicted states over horizon
        """
        states = np.zeros((self.current_horizon, 4))  # [d, phi, v, omega]
        
        # Initial state
        current_d = initial_state.lateral_error
        current_phi = initial_state.heading_error
        current_v = initial_state.velocity
        current_omega = initial_state.angular_velocity
        
        for i in range(self.current_horizon):
            # Get control inputs (repeat last control if beyond control horizon)
            if i < self.params.control_horizon:
                v_cmd = control_sequence[2*i]
                omega_cmd = control_sequence[2*i + 1]
            else:
                v_cmd = control_sequence[2*(self.params.control_horizon-1)]
                omega_cmd = control_sequence[2*(self.params.control_horizon-1) + 1]
            
            # Simple kinematic model for lane following
            # Lateral error dynamics: d_dot = v * sin(phi)
            # Heading error dynamics: phi_dot = omega
            
            current_d += current_v * np.sin(current_phi) * self.params.dt
            current_phi += current_omega * self.params.dt
            current_v = v_cmd  # Assume perfect velocity tracking
            current_omega = omega_cmd  # Assume perfect angular velocity tracking
            
            states[i] = [current_d, current_phi, current_v, current_omega]
        
        return states
    
    def _predict_trajectory(self, control_sequence: np.ndarray, initial_state: MPCState) -> np.ndarray:
        """Generate predicted trajectory for visualization"""
        return self._simulate_forward(control_sequence, initial_state)
    
    def _count_constraint_violations(self, control_sequence: np.ndarray) -> int:
        """Count number of constraint violations"""
        violations = 0
        
        # Check control bounds
        for i in range(self.params.control_horizon):
            v = control_sequence[2*i]
            omega = control_sequence[2*i + 1]
            
            if v < 0 or v > self.params.max_velocity:
                violations += 1
            if abs(omega) > self.params.max_angular_velocity:
                violations += 1
        
        # Check smoothness constraints
        for i in range(self.params.control_horizon - 1):
            dv = abs(control_sequence[2*(i+1)] - control_sequence[2*i])
            domega = abs(control_sequence[2*(i+1) + 1] - control_sequence[2*i + 1])
            
            if dv > 0.2:
                violations += 1
            if domega > 1.0:
                violations += 1
        
        return violations
    
    def _update_history(self, state: MPCState, control: list, opt_time: float, cost: float):
        """Update history for performance monitoring"""
        self.state_history.append([state.lateral_error, state.heading_error, state.velocity])
        self.control_history.append(control)
        self.optimization_times.append(opt_time)
        self.cost_history.append(cost)
        
        # Keep only recent history
        max_history = 100
        if len(self.state_history) > max_history:
            self.state_history = self.state_history[-max_history:]
            self.control_history = self.control_history[-max_history:]
            self.optimization_times = self.optimization_times[-max_history:]
            self.cost_history = self.cost_history[-max_history:]
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get MPC performance metrics"""
        if not self.optimization_times:
            return {}
        
        return {
            'avg_optimization_time': np.mean(self.optimization_times),
            'max_optimization_time': np.max(self.optimization_times),
            'avg_cost': np.mean(self.cost_history) if self.cost_history else 0,
            'current_horizon': self.current_horizon,
            'total_optimizations': len(self.optimization_times)
        }
    
    def update_parameters(self, new_params: MPCParameters):
        """Update MPC parameters during runtime"""
        self.params = new_params
        self.current_weights = self._initialize_weights()
        rospy.loginfo("[MPCSolver] Updated MPC parameters: horizon=%d, dt=%.3f", 
                     self.params.prediction_horizon, self.params.dt)
#!/usr/bin/env python3
"""
Enhanced Vehicle Model for Duckiebot Lane Control

This module implements an advanced vehicle dynamics model with Duckiebot-specific
parameters, kinematic and dynamic constraints, and calibration data integration.
"""

import numpy as np
import time
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
import os
try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

try:
    import rospy
except ImportError:
    # Mock rospy for testing
    class MockRospy:
        @staticmethod
        def loginfo(msg, *args): print(f"[INFO] {msg % args if args else msg}")
        @staticmethod
        def logdebug(msg, *args): pass
        @staticmethod
        def logwarn(msg, *args): print(f"[WARN] {msg % args if args else msg}")
        @staticmethod
        def logerr(msg, *args): print(f"[ERROR] {msg % args if args else msg}")
    rospy = MockRospy()


@dataclass
class VehicleParameters:
    """Duckiebot-specific vehicle parameters"""
    # Physical parameters
    wheelbase: float = 0.1  # Distance between wheels (m)
    wheel_radius: float = 0.0318  # Wheel radius (m)
    vehicle_width: float = 0.13  # Vehicle width (m)
    vehicle_length: float = 0.18  # Vehicle length (m)
    mass: float = 1.2  # Vehicle mass (kg)
    
    # Kinematic constraints
    max_linear_velocity: float = 0.5  # Maximum linear velocity (m/s)
    max_angular_velocity: float = 2.0  # Maximum angular velocity (rad/s)
    max_linear_acceleration: float = 1.0  # Maximum linear acceleration (m/s²)
    max_angular_acceleration: float = 3.0  # Maximum angular acceleration (rad/s²)
    
    # Dynamic parameters
    moment_of_inertia: float = 0.01  # Moment of inertia (kg⋅m²)
    friction_coefficient: float = 0.8  # Tire-ground friction coefficient
    drag_coefficient: float = 0.1  # Air drag coefficient
    
    # Motor parameters
    motor_time_constant: float = 0.1  # Motor response time constant (s)
    motor_deadband: float = 0.05  # Motor deadband (normalized)
    motor_saturation: float = 1.0  # Motor saturation limit
    
    # Calibration parameters
    trim_left: float = 0.0  # Left motor trim
    trim_right: float = 0.0  # Right motor trim
    gain_left: float = 1.0  # Left motor gain
    gain_right: float = 1.0  # Right motor gain


@dataclass
class VehicleState:
    """Complete vehicle state representation"""
    # Position and orientation
    x: float = 0.0  # Global x position (m)
    y: float = 0.0  # Global y position (m)
    theta: float = 0.0  # Global heading angle (rad)
    
    # Lane-relative state
    lateral_error: float = 0.0  # Lateral deviation from lane center (m)
    heading_error: float = 0.0  # Heading error relative to lane (rad)
    
    # Velocities
    linear_velocity: float = 0.0  # Linear velocity (m/s)
    angular_velocity: float = 0.0  # Angular velocity (rad/s)
    
    # Accelerations
    linear_acceleration: float = 0.0  # Linear acceleration (m/s²)
    angular_acceleration: float = 0.0  # Angular acceleration (rad/s²)
    
    # Motor states
    left_wheel_velocity: float = 0.0  # Left wheel velocity (rad/s)
    right_wheel_velocity: float = 0.0  # Right wheel velocity (rad/s)
    
    # Timestamp
    timestamp: float = 0.0


class EnhancedVehicleModel:
    """
    Enhanced vehicle dynamics model for Duckiebot with advanced kinematics,
    dynamics, and calibration integration.
    """
    
    def __init__(self, parameters: Optional[VehicleParameters] = None):
        """
        Initialize enhanced vehicle model.
        
        Args:
            parameters: Vehicle parameters, uses defaults if None
        """
        self.params = parameters or VehicleParameters()
        self.state = VehicleState()
        self.previous_state = VehicleState()
        
        # Model validation and calibration
        self.calibration_data = {}
        self.model_validation_metrics = {}
        self.constraint_violations = []
        
        # Performance monitoring
        self.prediction_history = []
        self.validation_errors = []
        
        rospy.loginfo("[EnhancedVehicleModel] Initialized with wheelbase=%.3fm, wheel_radius=%.4fm",
                     self.params.wheelbase, self.params.wheel_radius)
        
        # Load calibration data
        self._load_calibration_data()
        self._validate_parameters()
    
    def _load_calibration_data(self):
        """Load calibration data from ground_projection package"""
        try:
            # Try to load calibration from standard Duckietown locations
            calibration_paths = [
                "/data/config/calibrations/camera_extrinsic/default.yaml",
                "/data/config/calibrations/kinematics/default.yaml",
                "src/packages/ground_projection/config/ground_projection_node/default.yaml"
            ]
            
            if YAML_AVAILABLE:
                for path in calibration_paths:
                    if os.path.exists(path):
                        with open(path, 'r') as f:
                            calib_data = yaml.safe_load(f)
                            self.calibration_data.update(calib_data)
                            rospy.logdebug("[EnhancedVehicleModel] Loaded calibration from: %s", path)
            else:
                rospy.logdebug("[EnhancedVehicleModel] YAML not available, skipping calibration file loading")
            
            # Extract relevant parameters from calibration
            if 'baseline' in self.calibration_data:
                self.params.wheelbase = self.calibration_data['baseline']
                rospy.loginfo("[EnhancedVehicleModel] Updated wheelbase from calibration: %.3fm", 
                             self.params.wheelbase)
            
            if 'gain' in self.calibration_data:
                gain = self.calibration_data['gain']
                if isinstance(gain, dict):
                    self.params.gain_left = gain.get('left', 1.0)
                    self.params.gain_right = gain.get('right', 1.0)
                    rospy.loginfo("[EnhancedVehicleModel] Updated motor gains: left=%.3f, right=%.3f",
                                 self.params.gain_left, self.params.gain_right)
            
            if 'trim' in self.calibration_data:
                trim = self.calibration_data['trim']
                if isinstance(trim, dict):
                    self.params.trim_left = trim.get('left', 0.0)
                    self.params.trim_right = trim.get('right', 0.0)
                    rospy.loginfo("[EnhancedVehicleModel] Updated motor trims: left=%.3f, right=%.3f",
                                 self.params.trim_left, self.params.trim_right)
                                 
        except Exception as e:
            rospy.logwarn("[EnhancedVehicleModel] Could not load calibration data: %s", str(e))
            rospy.loginfo("[EnhancedVehicleModel] Using default parameters")
    
    def _validate_parameters(self):
        """Validate vehicle parameters for physical consistency"""
        validation_results = {}
        
        # Check physical constraints
        if self.params.wheelbase <= 0:
            rospy.logerr("[EnhancedVehicleModel] Invalid wheelbase: %.3f", self.params.wheelbase)
            validation_results['wheelbase'] = False
        else:
            validation_results['wheelbase'] = True
        
        if self.params.wheel_radius <= 0:
            rospy.logerr("[EnhancedVehicleModel] Invalid wheel radius: %.4f", self.params.wheel_radius)
            validation_results['wheel_radius'] = False
        else:
            validation_results['wheel_radius'] = True
        
        # Check kinematic limits
        if self.params.max_linear_velocity <= 0:
            rospy.logwarn("[EnhancedVehicleModel] Invalid max linear velocity: %.3f", 
                         self.params.max_linear_velocity)
            validation_results['max_velocity'] = False
        else:
            validation_results['max_velocity'] = True
        
        # Check dynamic parameters
        if self.params.mass <= 0:
            rospy.logwarn("[EnhancedVehicleModel] Invalid mass: %.3f", self.params.mass)
            validation_results['mass'] = False
        else:
            validation_results['mass'] = True
        
        self.model_validation_metrics = validation_results
        
        # Log validation summary
        valid_params = sum(validation_results.values())
        total_params = len(validation_results)
        rospy.loginfo("[EnhancedVehicleModel] Parameter validation: %d/%d valid", 
                     valid_params, total_params)
        
        if valid_params < total_params:
            rospy.logwarn("[EnhancedVehicleModel] Some parameters failed validation, using defaults")
    
    def update_state(self, new_state: VehicleState):
        """Update vehicle state with validation"""
        self.previous_state = self.state
        self.state = new_state
        self.state.timestamp = time.time()
        
        # Validate state constraints
        constraint_violations = self._check_state_constraints(new_state)
        if constraint_violations:
            self.constraint_violations.extend(constraint_violations)
            rospy.logdebug("[EnhancedVehicleModel] State constraint violations: %s", 
                          ', '.join(constraint_violations))
    
    def _check_state_constraints(self, state: VehicleState) -> list:
        """Check if state violates physical constraints"""
        violations = []
        
        # Velocity constraints
        if abs(state.linear_velocity) > self.params.max_linear_velocity:
            violations.append(f"linear_velocity: {state.linear_velocity:.3f} > {self.params.max_linear_velocity:.3f}")
        
        if abs(state.angular_velocity) > self.params.max_angular_velocity:
            violations.append(f"angular_velocity: {state.angular_velocity:.3f} > {self.params.max_angular_velocity:.3f}")
        
        # Acceleration constraints (if previous state available)
        if self.previous_state.timestamp > 0:
            dt = state.timestamp - self.previous_state.timestamp
            if dt > 0:
                linear_accel = (state.linear_velocity - self.previous_state.linear_velocity) / dt
                angular_accel = (state.angular_velocity - self.previous_state.angular_velocity) / dt
                
                if abs(linear_accel) > self.params.max_linear_acceleration:
                    violations.append(f"linear_acceleration: {linear_accel:.3f} > {self.params.max_linear_acceleration:.3f}")
                
                if abs(angular_accel) > self.params.max_angular_acceleration:
                    violations.append(f"angular_acceleration: {angular_accel:.3f} > {self.params.max_angular_acceleration:.3f}")
        
        return violations
    
    def predict_motion(self, control_input: Tuple[float, float], dt: float) -> VehicleState:
        """
        Predict vehicle motion using enhanced dynamics model.
        
        Args:
            control_input: (linear_velocity_cmd, angular_velocity_cmd)
            dt: Time step for prediction
            
        Returns:
            Predicted vehicle state
        """
        v_cmd, omega_cmd = control_input
        
        rospy.logdebug("[EnhancedVehicleModel] Predicting motion: v_cmd=%.3f, omega_cmd=%.3f, dt=%.3f",
                      v_cmd, omega_cmd, dt)
        
        # Apply motor dynamics and constraints
        v_actual, omega_actual = self._apply_motor_dynamics(v_cmd, omega_cmd, dt)
        
        # Kinematic model prediction
        predicted_state = self._kinematic_prediction(v_actual, omega_actual, dt)
        
        # Apply dynamic effects
        predicted_state = self._apply_dynamic_effects(predicted_state, dt)
        
        # Validate prediction
        self._validate_prediction(predicted_state)
        
        rospy.logdebug("[EnhancedVehicleModel] Predicted state: x=%.3f, y=%.3f, theta=%.3f, v=%.3f, omega=%.3f",
                      predicted_state.x, predicted_state.y, predicted_state.theta,
                      predicted_state.linear_velocity, predicted_state.angular_velocity)
        
        return predicted_state
    
    def _apply_motor_dynamics(self, v_cmd: float, omega_cmd: float, dt: float) -> Tuple[float, float]:
        """Apply motor dynamics and calibration"""
        # Convert to wheel velocities
        left_wheel_cmd, right_wheel_cmd = self._diff_drive_inverse_kinematics(v_cmd, omega_cmd)
        
        # Apply motor calibration
        left_wheel_cmd = (left_wheel_cmd + self.params.trim_left) * self.params.gain_left
        right_wheel_cmd = (right_wheel_cmd + self.params.trim_right) * self.params.gain_right
        
        # Apply motor deadband
        if abs(left_wheel_cmd) < self.params.motor_deadband:
            left_wheel_cmd = 0.0
        if abs(right_wheel_cmd) < self.params.motor_deadband:
            right_wheel_cmd = 0.0
        
        # Apply motor saturation
        left_wheel_cmd = np.clip(left_wheel_cmd, -self.params.motor_saturation, self.params.motor_saturation)
        right_wheel_cmd = np.clip(right_wheel_cmd, -self.params.motor_saturation, self.params.motor_saturation)
        
        # Apply first-order motor dynamics
        tau = self.params.motor_time_constant
        alpha = dt / (tau + dt)
        
        left_wheel_actual = (1 - alpha) * self.state.left_wheel_velocity + alpha * left_wheel_cmd
        right_wheel_actual = (1 - alpha) * self.state.right_wheel_velocity + alpha * right_wheel_cmd
        
        # Convert back to body velocities
        v_actual, omega_actual = self._diff_drive_forward_kinematics(left_wheel_actual, right_wheel_actual)
        
        rospy.logdebug("[EnhancedVehicleModel] Motor dynamics: cmd=[%.3f,%.3f] -> actual=[%.3f,%.3f]",
                      v_cmd, omega_cmd, v_actual, omega_actual)
        
        return v_actual, omega_actual
    
    def _diff_drive_inverse_kinematics(self, v: float, omega: float) -> Tuple[float, float]:
        """Convert body velocities to wheel velocities"""
        # Differential drive kinematics
        left_wheel_vel = (v - omega * self.params.wheelbase / 2.0) / self.params.wheel_radius
        right_wheel_vel = (v + omega * self.params.wheelbase / 2.0) / self.params.wheel_radius
        return left_wheel_vel, right_wheel_vel
    
    def _diff_drive_forward_kinematics(self, left_wheel_vel: float, right_wheel_vel: float) -> Tuple[float, float]:
        """Convert wheel velocities to body velocities"""
        # Forward differential drive kinematics
        v = self.params.wheel_radius * (left_wheel_vel + right_wheel_vel) / 2.0
        omega = self.params.wheel_radius * (right_wheel_vel - left_wheel_vel) / self.params.wheelbase
        return v, omega
    
    def _kinematic_prediction(self, v: float, omega: float, dt: float) -> VehicleState:
        """Predict motion using kinematic model"""
        predicted_state = VehicleState()
        
        # Copy current state
        predicted_state.x = self.state.x
        predicted_state.y = self.state.y
        predicted_state.theta = self.state.theta
        predicted_state.lateral_error = self.state.lateral_error
        predicted_state.heading_error = self.state.heading_error
        
        # Kinematic integration
        if abs(omega) < 1e-6:  # Straight line motion
            predicted_state.x += v * np.cos(self.state.theta) * dt
            predicted_state.y += v * np.sin(self.state.theta) * dt
        else:  # Curved motion
            # Instantaneous center of rotation
            R = v / omega
            
            # Update position
            predicted_state.x += R * (np.sin(self.state.theta + omega * dt) - np.sin(self.state.theta))
            predicted_state.y += R * (-np.cos(self.state.theta + omega * dt) + np.cos(self.state.theta))
        
        # Update orientation
        predicted_state.theta += omega * dt
        predicted_state.theta = self._normalize_angle(predicted_state.theta)
        
        # Update lane-relative state (simplified)
        predicted_state.lateral_error += v * np.sin(self.state.heading_error) * dt
        predicted_state.heading_error += omega * dt
        predicted_state.heading_error = self._normalize_angle(predicted_state.heading_error)
        
        # Update velocities
        predicted_state.linear_velocity = v
        predicted_state.angular_velocity = omega
        
        return predicted_state
    
    def _apply_dynamic_effects(self, state: VehicleState, dt: float) -> VehicleState:
        """Apply dynamic effects like friction and inertia"""
        # Simple dynamic effects
        
        # Apply friction to reduce velocity
        friction_factor = 1.0 - self.params.friction_coefficient * dt * 0.1
        state.linear_velocity *= friction_factor
        state.angular_velocity *= friction_factor
        
        # Apply drag (velocity-dependent)
        drag_factor = 1.0 - self.params.drag_coefficient * abs(state.linear_velocity) * dt * 0.1
        state.linear_velocity *= max(0.0, drag_factor)
        
        return state
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def _validate_prediction(self, predicted_state: VehicleState):
        """Validate prediction against known constraints"""
        # Check for NaN or infinite values
        state_values = [
            predicted_state.x, predicted_state.y, predicted_state.theta,
            predicted_state.lateral_error, predicted_state.heading_error,
            predicted_state.linear_velocity, predicted_state.angular_velocity
        ]
        
        for i, value in enumerate(state_values):
            if not np.isfinite(value):
                rospy.logerr("[EnhancedVehicleModel] Invalid prediction value at index %d: %s", i, value)
                # Reset to safe values
                if i < 3:  # Position/orientation
                    state_values[i] = 0.0
                else:  # Velocities/errors
                    state_values[i] = 0.0
        
        # Update prediction history for analysis
        self.prediction_history.append({
            'timestamp': time.time(),
            'state': predicted_state,
            'valid': all(np.isfinite(v) for v in state_values)
        })
        
        # Keep limited history
        if len(self.prediction_history) > 100:
            self.prediction_history = self.prediction_history[-100:]
    
    def get_kinematic_constraints(self) -> Dict[str, Tuple[float, float]]:
        """Get kinematic constraints for optimization"""
        return {
            'linear_velocity': (0.0, self.params.max_linear_velocity),
            'angular_velocity': (-self.params.max_angular_velocity, self.params.max_angular_velocity),
            'linear_acceleration': (-self.params.max_linear_acceleration, self.params.max_linear_acceleration),
            'angular_acceleration': (-self.params.max_angular_acceleration, self.params.max_angular_acceleration)
        }
    
    def get_dynamic_constraints(self) -> Dict[str, float]:
        """Get dynamic constraints for control"""
        return {
            'max_lateral_acceleration': self.params.friction_coefficient * 9.81,  # Friction limit
            'max_centripetal_acceleration': self.params.max_linear_velocity**2 / (self.params.wheelbase * 2),
            'motor_time_constant': self.params.motor_time_constant,
            'motor_deadband': self.params.motor_deadband
        }
    
    def update_parameters(self, new_params: VehicleParameters):
        """Update vehicle parameters and re-validate"""
        rospy.loginfo("[EnhancedVehicleModel] Updating parameters")
        self.params = new_params
        self._validate_parameters()
        
        # Clear constraint violations on parameter update
        self.constraint_violations = []
    
    def get_model_diagnostics(self) -> Dict[str, Any]:
        """Get model diagnostics and performance metrics"""
        recent_violations = [v for v in self.constraint_violations if time.time() - v.get('timestamp', 0) < 60]
        recent_predictions = [p for p in self.prediction_history if time.time() - p['timestamp'] < 60]
        
        return {
            'parameters_valid': all(self.model_validation_metrics.values()),
            'calibration_loaded': bool(self.calibration_data),
            'recent_constraint_violations': len(recent_violations),
            'recent_predictions': len(recent_predictions),
            'prediction_success_rate': np.mean([p['valid'] for p in recent_predictions]) if recent_predictions else 1.0,
            'wheelbase': self.params.wheelbase,
            'wheel_radius': self.params.wheel_radius,
            'motor_gains': [self.params.gain_left, self.params.gain_right],
            'motor_trims': [self.params.trim_left, self.params.trim_right]
        }
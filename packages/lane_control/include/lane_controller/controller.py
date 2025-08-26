import numpy as np
import time
from typing import Optional, Dict, Any, Tuple

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

from .mpc_solver import MPCSolver, MPCParameters, MPCState, MPCResult
from .enhanced_vehicle_model import EnhancedVehicleModel, VehicleParameters, VehicleState
from .adaptive_gain_scheduler import AdaptiveGainScheduler, GainSchedule


class LaneController:
    """
    The Lane Controller can be used to compute control commands from pose estimations.

    The control commands are in terms of linear and angular velocity (v, omega). The input are errors in the relative
    pose of the Duckiebot in the current lane.

    This implementation is a simple PI(D) controller.

    Args:
        ~v_bar (:obj:`float`): Nominal velocity in m/s
        ~k_d (:obj:`float`): Proportional term for lateral deviation
        ~k_theta (:obj:`float`): Proportional term for heading deviation
        ~k_Id (:obj:`float`): integral term for lateral deviation
        ~k_Iphi (:obj:`float`): integral term for lateral deviation
        ~d_thres (:obj:`float`): Maximum value for lateral error
        ~theta_thres (:obj:`float`): Maximum value for heading error
        ~d_offset (:obj:`float`): Goal offset from center of the lane
        ~integral_bounds (:obj:`dict`): Bounds for integral term
        ~d_resolution (:obj:`float`): Resolution of lateral position estimate
        ~phi_resolution (:obj:`float`): Resolution of heading estimate
        ~omega_ff (:obj:`float`): Feedforward part of controller
        ~verbose (:obj:`bool`): Verbosity level (0,1,2)
        ~stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

    """

    def __init__(self, parameters):
        self.parameters = parameters
        self.d_I = 0.0
        self.phi_I = 0.0
        self.prev_d_err = 0.0
        self.prev_phi_err = 0.0
        
        # Initialize MPC solver
        self.mpc_enabled = parameters.get("~use_mpc", False)
        if self.mpc_enabled:
            mpc_params = self._initialize_mpc_parameters(parameters)
            self.mpc_solver = MPCSolver(mpc_params)
            rospy.loginfo("[LaneController] MPC solver initialized")
        else:
            self.mpc_solver = None
            rospy.loginfo("[LaneController] Using traditional PID controller")
        
        # Performance monitoring
        self.control_history = []
        self.performance_metrics = {}
        self.last_mpc_time = 0.0
        
        # Enhanced vehicle model
        self.use_enhanced_model = parameters.get("~use_enhanced_vehicle_model", True)
        if self.use_enhanced_model:
            vehicle_params = self._initialize_vehicle_parameters(parameters)
            self.vehicle_model = EnhancedVehicleModel(vehicle_params)
            rospy.loginfo("[LaneController] Enhanced vehicle model initialized")
        else:
            self.vehicle_model = None
            rospy.loginfo("[LaneController] Using simplified vehicle model")
        
        # Adaptive gain scheduler
        self.use_adaptive_gains = parameters.get("~use_adaptive_gains", True)
        if self.use_adaptive_gains:
            gain_schedule = self._initialize_gain_schedule(parameters)
            self.gain_scheduler = AdaptiveGainScheduler(gain_schedule)
            rospy.loginfo("[LaneController] Adaptive gain scheduler initialized")
        else:
            self.gain_scheduler = None
            rospy.loginfo("[LaneController] Using fixed gains")

    def update_parameters(self, parameters):
        """Updates parameters of LaneController object.

        Args:
            parameters (:obj:`dict`): dictionary containing the new parameters for LaneController object.
        """
        self.parameters = parameters
        
        # Update MPC parameters if enabled
        if self.mpc_enabled and self.mpc_solver is not None:
            mpc_params = self._initialize_mpc_parameters(parameters)
            self.mpc_solver.update_parameters(mpc_params)
            rospy.logdebug("[LaneController] Updated MPC parameters")
        
        # Update vehicle model parameters if enabled
        if self.use_enhanced_model and self.vehicle_model is not None:
            vehicle_params = self._initialize_vehicle_parameters(parameters)
            self.vehicle_model.update_parameters(vehicle_params)
            rospy.logdebug("[LaneController] Updated vehicle model parameters")
        
        # Update gain scheduler parameters if enabled
        if self.use_adaptive_gains and self.gain_scheduler is not None:
            gain_schedule = self._initialize_gain_schedule(parameters)
            self.gain_scheduler.update_schedule_parameters(gain_schedule)
            rospy.logdebug("[LaneController] Updated gain scheduler parameters")

    def compute_control_action(self, d_err, phi_err, dt, wheels_cmd_exec, stop_line_distance):
        """Main function, computes the control action given the current error signals.

        Given an estimate of the error, computes a control action (tuple of linear and angular velocity). This is done
        via either MPC optimization or basic PI(D) controller with anti-reset windup logic.

        Args:
            d_err (:obj:`float`): error in meters in the lateral direction
            phi_err (:obj:`float`): error in radians in the heading direction
            dt (:obj:`float`): time since last command update
            wheels_cmd_exec (:obj:`bool`): confirmation that the wheel commands have been executed (to avoid
                                           integration while the robot does not move)
            stop_line_distance (:obj:`float`):  distance of the stop line, None if not detected.
        Returns:
            v (:obj:`float`): requested linear velocity in meters/second
            omega (:obj:`float`): requested angular velocity in radians/second
        """
        
        # Use MPC if enabled and available
        if self.mpc_enabled and self.mpc_solver is not None:
            return self._compute_mpc_control(d_err, phi_err, dt, wheels_cmd_exec, stop_line_distance)
        else:
            return self._compute_pid_control(d_err, phi_err, dt, wheels_cmd_exec, stop_line_distance)
    
    def _compute_mpc_control(self, d_err, phi_err, dt, wheels_cmd_exec, stop_line_distance):
        """Compute control action using MPC optimization"""
        start_time = time.time()
        
        # Prepare current state for MPC
        current_velocity = self.compute_velocity(stop_line_distance)
        current_state = MPCState(
            lateral_error=d_err,
            heading_error=phi_err,
            velocity=current_velocity,
            angular_velocity=0.0,  # Will be optimized
            timestamp=start_time
        )
        
        rospy.logdebug("[LaneController] MPC input state: d_err=%.4f, phi_err=%.4f, v=%.3f", 
                      d_err, phi_err, current_velocity)
        
        try:
            # Solve MPC optimization
            mpc_result = self.mpc_solver.solve(current_state)
            
            # Log MPC performance metrics
            rospy.logdebug("[LaneController] MPC result: v=%.3f, omega=%.3f, cost=%.4f, time=%.3fms, converged=%s, violations=%d",
                          mpc_result.linear_velocity, mpc_result.angular_velocity, 
                          mpc_result.cost_value, mpc_result.optimization_time * 1000,
                          mpc_result.convergence_status, mpc_result.constraint_violations)
            
            # Monitor optimization performance
            if mpc_result.optimization_time > 0.1:
                rospy.logwarn("[LaneController] MPC optimization slow: %.3fms", mpc_result.optimization_time * 1000)
            
            if not mpc_result.convergence_status:
                rospy.logwarn("[LaneController] MPC optimization failed to converge, cost=%.4f", mpc_result.cost_value)
            
            if mpc_result.constraint_violations > 0:
                rospy.logwarn("[LaneController] MPC constraint violations: %d", mpc_result.constraint_violations)
            
            # Update performance tracking
            self._update_performance_metrics(mpc_result)
            
            # Apply velocity constraints based on stop line
            v = self._apply_stop_line_constraints(mpc_result.linear_velocity, stop_line_distance)
            omega = mpc_result.angular_velocity
            
            rospy.logdebug("[LaneController] Final MPC control: v=%.3f, omega=%.3f", v, omega)
            
            # Update vehicle state and validate control action
            self.update_vehicle_state(d_err, phi_err, v, omega)
            v, omega = self.validate_control_action(v, omega)
            
            return v, omega
            
        except Exception as e:
            rospy.logerr("[LaneController] MPC computation failed: %s, falling back to PID", str(e))
            # Fallback to PID control
            return self._compute_pid_control(d_err, phi_err, dt, wheels_cmd_exec, stop_line_distance)
    
    def _compute_pid_control(self, d_err, phi_err, dt, wheels_cmd_exec, stop_line_distance):
        """Compute control action using traditional PID controller with adaptive gains"""
        rospy.logdebug("[LaneController] Using PID control: d_err=%.4f, phi_err=%.4f", d_err, phi_err)
        
        if dt is not None:
            self.integrate_errors(d_err, phi_err, dt)

        self.d_I = self.adjust_integral(
            d_err, self.d_I, self.parameters["~integral_bounds"]["d"], self.parameters["~d_resolution"]
        )
        self.phi_I = self.adjust_integral(
            phi_err,
            self.phi_I,
            self.parameters["~integral_bounds"]["phi"],
            self.parameters["~phi_resolution"],
        )

        self.reset_if_needed(d_err, phi_err, wheels_cmd_exec)

        # Get adaptive gains if available
        if self.use_adaptive_gains and self.gain_scheduler is not None:
            # Update gain scheduler state
            current_v = self.compute_velocity(stop_line_distance)
            self.gain_scheduler.update_operating_state(current_v, d_err, phi_err, self.d_I, self.phi_I)
            
            # Compute scheduled gains
            scheduled_gains = self.gain_scheduler.compute_scheduled_gains()
            
            # Use scheduled gains
            k_d = scheduled_gains['k_d']
            k_theta = scheduled_gains['k_theta']
            k_Id = scheduled_gains['k_Id']
            k_Iphi = scheduled_gains['k_Iphi']
            
            rospy.logdebug("[LaneController] Using adaptive gains: k_d=%.3f, k_theta=%.3f, k_Id=%.3f, k_Iphi=%.3f",
                          k_d, k_theta, k_Id, k_Iphi)
        else:
            # Use fixed gains
            k_d = self.parameters["~k_d"].value
            k_theta = self.parameters["~k_theta"].value
            k_Id = self.parameters["~k_Id"].value
            k_Iphi = self.parameters["~k_Iphi"].value
            
            rospy.logdebug("[LaneController] Using fixed gains: k_d=%.3f, k_theta=%.3f, k_Id=%.3f, k_Iphi=%.3f",
                          k_d, k_theta, k_Id, k_Iphi)

        # Compute control action with selected gains
        omega = (
            k_d * d_err
            + k_theta * phi_err
            + k_Id * self.d_I
            + k_Iphi * self.phi_I
        )

        self.prev_d_err = d_err
        self.prev_phi_err = phi_err

        v = self.compute_velocity(stop_line_distance)
        
        # Update performance history for adaptive learning
        if self.use_adaptive_gains and self.gain_scheduler is not None:
            self.gain_scheduler.update_performance_history(d_err, phi_err, (v, omega))
        
        rospy.logdebug("[LaneController] PID control result: v=%.3f, omega=%.3f, d_I=%.4f, phi_I=%.4f", 
                      v, omega, self.d_I, self.phi_I)
        
        # Update vehicle state and validate control action
        self.update_vehicle_state(d_err, phi_err, v, omega)
        v, omega = self.validate_control_action(v, omega)

        return v, omega

    def compute_velocity(self, stop_line_distance):
        """Linearly decrease velocity if approaching a stop line.

        If a stop line is detected, the velocity is linearly decreased to achieve a better stopping position,
        otherwise the nominal velocity is returned.

        Args:
            stop_line_distance (:obj:`float`): distance of the stop line, None if not detected.
        """
        if stop_line_distance is None:
            return self.parameters["~v_bar"].value
        else:

            d1, d2 = (
                self.parameters["~stop_line_slowdown"]["start"],
                self.parameters["~stop_line_slowdown"]["end"],
            )
            # d1 -> v_bar, d2 -> v_bar/2
            c = (0.5 * (d1 - stop_line_distance) + (stop_line_distance - d2)) / (d1 - d2)
            v_new = self.parameters["~v_bar"].value * c
            v = np.max(
                [self.parameters["~v_bar"].value / 2.0, np.min([self.parameters["~v_bar"].value, v_new])]
            )
            return v

    def integrate_errors(self, d_err, phi_err, dt):
        """Integrates error signals in lateral and heading direction.
        Args:
            d_err (:obj:`float`): error in meters in the lateral direction
            phi_err (:obj:`float`): error in radians in the heading direction
            dt (:obj:`float`): time delay in seconds
        """
        self.d_I += d_err * dt
        self.phi_I += phi_err * dt

    def reset_if_needed(self, d_err, phi_err, wheels_cmd_exec):
        """Resets the integral error if needed.

        Resets the integral errors in `d` and `phi` if either the error sign changes, or if the robot is completely
        stopped (i.e. intersections).

        Args:
            d_err (:obj:`float`): error in meters in the lateral direction
            phi_err (:obj:`float`): error in radians in the heading direction
            wheels_cmd_exec (:obj:`bool`): confirmation that the wheel commands have been executed (to avoid
                                           integration while the robot does not move)
        """
        if np.sign(d_err) != np.sign(self.prev_d_err):
            self.d_I = 0
        if np.sign(phi_err) != np.sign(self.prev_phi_err):
            self.phi_I = 0
        if wheels_cmd_exec[0] == 0 and wheels_cmd_exec[1] == 0:
            self.d_I = 0
            self.phi_I = 0

    @staticmethod
    def adjust_integral(error, integral, bounds, resolution):
        """Bounds the integral error to avoid windup.

        Adjusts the integral error to remain in defined bounds, and cancels it if the error is smaller than the
        resolution of the error estimation.

        Args:
            error (:obj:`float`): current error value
            integral (:obj:`float`): current integral value
            bounds (:obj:`dict`): contains minimum and maximum value for the integral
            resolution (:obj:`float`): resolution of the error estimate

        Returns:
            integral (:obj:`float`): adjusted integral value
        """
        if integral > bounds["top"]:
            integral = bounds["top"]
        elif integral < bounds["bot"]:
            integral = bounds["bot"]
        elif abs(error) < resolution:
            integral = 0
        return integral
    
    def _initialize_mpc_parameters(self, parameters) -> MPCParameters:
        """Initialize MPC parameters from ROS parameters"""
        return MPCParameters(
            prediction_horizon=parameters.get("~mpc_prediction_horizon", 10),
            control_horizon=parameters.get("~mpc_control_horizon", 5),
            dt=parameters.get("~mpc_dt", 0.1),
            max_velocity=parameters.get("~mpc_max_velocity", 0.5),
            max_angular_velocity=parameters.get("~mpc_max_angular_velocity", 2.0),
            velocity_weight=parameters.get("~mpc_velocity_weight", 1.0),
            angular_weight=parameters.get("~mpc_angular_weight", 1.0),
            lateral_weight=parameters.get("~mpc_lateral_weight", 10.0),
            heading_weight=parameters.get("~mpc_heading_weight", 5.0),
            control_smoothness_weight=parameters.get("~mpc_smoothness_weight", 1.0),
            adaptive_horizon=parameters.get("~mpc_adaptive_horizon", True)
        )
    
    def _apply_stop_line_constraints(self, mpc_velocity: float, stop_line_distance: Optional[float]) -> float:
        """Apply stop line velocity constraints to MPC result"""
        if stop_line_distance is None:
            return mpc_velocity
        
        # Use existing stop line velocity computation but respect MPC bounds
        stop_line_velocity = self.compute_velocity(stop_line_distance)
        
        # Take minimum of MPC result and stop line constraint
        constrained_velocity = min(mpc_velocity, stop_line_velocity)
        
        rospy.logdebug("[LaneController] Stop line constraint: MPC=%.3f, stop_line=%.3f, final=%.3f, distance=%.3f",
                      mpc_velocity, stop_line_velocity, constrained_velocity, stop_line_distance)
        
        return constrained_velocity
    
    def _update_performance_metrics(self, mpc_result: MPCResult):
        """Update performance metrics for monitoring"""
        current_time = time.time()
        
        # Store control action history
        self.control_history.append({
            'timestamp': current_time,
            'linear_velocity': mpc_result.linear_velocity,
            'angular_velocity': mpc_result.angular_velocity,
            'optimization_time': mpc_result.optimization_time,
            'cost_value': mpc_result.cost_value,
            'convergence_status': mpc_result.convergence_status,
            'constraint_violations': mpc_result.constraint_violations
        })
        
        # Keep only recent history
        max_history = 50
        if len(self.control_history) > max_history:
            self.control_history = self.control_history[-max_history:]
        
        # Update performance metrics
        if len(self.control_history) >= 5:
            recent_times = [h['optimization_time'] for h in self.control_history[-10:]]
            recent_costs = [h['cost_value'] for h in self.control_history[-10:] if h['cost_value'] != float('inf')]
            recent_convergence = [h['convergence_status'] for h in self.control_history[-10:]]
            
            self.performance_metrics = {
                'avg_optimization_time': np.mean(recent_times),
                'max_optimization_time': np.max(recent_times),
                'avg_cost': np.mean(recent_costs) if recent_costs else float('inf'),
                'convergence_rate': np.mean(recent_convergence),
                'total_controls': len(self.control_history)
            }
            
            # Log performance summary periodically
            if current_time - self.last_mpc_time > 5.0:  # Every 5 seconds
                rospy.loginfo("[LaneController] MPC Performance: avg_time=%.2fms, convergence=%.1f%%, avg_cost=%.3f",
                             self.performance_metrics['avg_optimization_time'] * 1000,
                             self.performance_metrics['convergence_rate'] * 100,
                             self.performance_metrics['avg_cost'])
                self.last_mpc_time = current_time
    
    def get_mpc_performance_metrics(self) -> Dict[str, Any]:
        """Get current MPC performance metrics"""
        if self.mpc_solver is not None:
            solver_metrics = self.mpc_solver.get_performance_metrics()
            solver_metrics.update(self.performance_metrics)
            return solver_metrics
        return self.performance_metrics
    
    def enable_mpc(self, enable: bool = True):
        """Enable or disable MPC control"""
        if enable and self.mpc_solver is None:
            mpc_params = self._initialize_mpc_parameters(self.parameters)
            self.mpc_solver = MPCSolver(mpc_params)
            rospy.loginfo("[LaneController] MPC solver enabled")
        
        self.mpc_enabled = enable
        rospy.loginfo("[LaneController] MPC control %s", "enabled" if enable else "disabled")
    
    def _initialize_vehicle_parameters(self, parameters) -> VehicleParameters:
        """Initialize vehicle parameters from ROS parameters"""
        return VehicleParameters(
            wheelbase=parameters.get("~vehicle_wheelbase", 0.1),
            wheel_radius=parameters.get("~vehicle_wheel_radius", 0.0318),
            vehicle_width=parameters.get("~vehicle_width", 0.13),
            vehicle_length=parameters.get("~vehicle_length", 0.18),
            mass=parameters.get("~vehicle_mass", 1.2),
            max_linear_velocity=parameters.get("~vehicle_max_linear_velocity", 0.5),
            max_angular_velocity=parameters.get("~vehicle_max_angular_velocity", 2.0),
            max_linear_acceleration=parameters.get("~vehicle_max_linear_acceleration", 1.0),
            max_angular_acceleration=parameters.get("~vehicle_max_angular_acceleration", 3.0),
            moment_of_inertia=parameters.get("~vehicle_moment_of_inertia", 0.01),
            friction_coefficient=parameters.get("~vehicle_friction_coefficient", 0.8),
            drag_coefficient=parameters.get("~vehicle_drag_coefficient", 0.1),
            motor_time_constant=parameters.get("~vehicle_motor_time_constant", 0.1),
            motor_deadband=parameters.get("~vehicle_motor_deadband", 0.05),
            motor_saturation=parameters.get("~vehicle_motor_saturation", 1.0)
        )
    
    def update_vehicle_state(self, d_err: float, phi_err: float, v: float, omega: float):
        """Update vehicle model state"""
        if self.vehicle_model is not None:
            current_time = time.time()
            
            # Create vehicle state from current measurements
            vehicle_state = VehicleState(
                lateral_error=d_err,
                heading_error=phi_err,
                linear_velocity=v,
                angular_velocity=omega,
                timestamp=current_time
            )
            
            # Update vehicle model
            self.vehicle_model.update_state(vehicle_state)
            
            rospy.logdebug("[LaneController] Updated vehicle state: d=%.4f, phi=%.4f, v=%.3f, omega=%.3f",
                          d_err, phi_err, v, omega)
    
    def predict_vehicle_motion(self, control_input: Tuple[float, float], dt: float) -> Optional[VehicleState]:
        """Predict vehicle motion using enhanced model"""
        if self.vehicle_model is not None:
            try:
                predicted_state = self.vehicle_model.predict_motion(control_input, dt)
                
                rospy.logdebug("[LaneController] Motion prediction: input=[%.3f,%.3f], dt=%.3f -> d=%.4f, phi=%.4f",
                              control_input[0], control_input[1], dt,
                              predicted_state.lateral_error, predicted_state.heading_error)
                
                return predicted_state
            except Exception as e:
                rospy.logwarn("[LaneController] Motion prediction failed: %s", str(e))
                return None
        return None
    
    def get_kinematic_constraints(self) -> Dict[str, Tuple[float, float]]:
        """Get kinematic constraints from vehicle model"""
        if self.vehicle_model is not None:
            constraints = self.vehicle_model.get_kinematic_constraints()
            rospy.logdebug("[LaneController] Kinematic constraints: %s", constraints)
            return constraints
        else:
            # Default constraints
            return {
                'linear_velocity': (0.0, 0.5),
                'angular_velocity': (-2.0, 2.0),
                'linear_acceleration': (-1.0, 1.0),
                'angular_acceleration': (-3.0, 3.0)
            }
    
    def get_dynamic_constraints(self) -> Dict[str, float]:
        """Get dynamic constraints from vehicle model"""
        if self.vehicle_model is not None:
            constraints = self.vehicle_model.get_dynamic_constraints()
            rospy.logdebug("[LaneController] Dynamic constraints: %s", constraints)
            return constraints
        else:
            # Default constraints
            return {
                'max_lateral_acceleration': 7.8,  # 0.8 * 9.81
                'max_centripetal_acceleration': 2.5,
                'motor_time_constant': 0.1,
                'motor_deadband': 0.05
            }
    
    def get_vehicle_diagnostics(self) -> Dict[str, Any]:
        """Get vehicle model diagnostics"""
        if self.vehicle_model is not None:
            diagnostics = self.vehicle_model.get_model_diagnostics()
            rospy.logdebug("[LaneController] Vehicle diagnostics: %s", diagnostics)
            return diagnostics
        return {}
    
    def validate_control_action(self, v: float, omega: float) -> Tuple[float, float]:
        """Validate and constrain control action using vehicle model"""
        if self.vehicle_model is not None:
            # Get constraints
            kinematic_constraints = self.get_kinematic_constraints()
            
            # Apply velocity constraints
            v_min, v_max = kinematic_constraints['linear_velocity']
            v_constrained = np.clip(v, v_min, v_max)
            
            omega_min, omega_max = kinematic_constraints['angular_velocity']
            omega_constrained = np.clip(omega, omega_min, omega_max)
            
            # Log constraint application
            if v != v_constrained or omega != omega_constrained:
                rospy.logdebug("[LaneController] Applied constraints: v [%.3f->%.3f], omega [%.3f->%.3f]",
                              v, v_constrained, omega, omega_constrained)
            
            return v_constrained, omega_constrained
        
        return v, omega
    
    def _initialize_gain_schedule(self, parameters) -> GainSchedule:
        """Initialize gain schedule from ROS parameters"""
        return GainSchedule(
            base_k_d=parameters.get("~k_d", -6.0).value if hasattr(parameters.get("~k_d", -6.0), 'value') else parameters.get("~k_d", -6.0),
            base_k_theta=parameters.get("~k_theta", -5.0).value if hasattr(parameters.get("~k_theta", -5.0), 'value') else parameters.get("~k_theta", -5.0),
            base_k_Id=parameters.get("~k_Id", -0.3).value if hasattr(parameters.get("~k_Id", -0.3), 'value') else parameters.get("~k_Id", -0.3),
            base_k_Iphi=parameters.get("~k_Iphi", 0.0).value if hasattr(parameters.get("~k_Iphi", 0.0), 'value') else parameters.get("~k_Iphi", 0.0),
            low_speed_threshold=parameters.get("~adaptive_low_speed_threshold", 0.1),
            high_speed_threshold=parameters.get("~adaptive_high_speed_threshold", 0.3),
            low_speed_k_d_scale=parameters.get("~adaptive_low_speed_k_d_scale", 1.5),
            low_speed_k_theta_scale=parameters.get("~adaptive_low_speed_k_theta_scale", 1.2),
            high_speed_k_d_scale=parameters.get("~adaptive_high_speed_k_d_scale", 0.8),
            high_speed_k_theta_scale=parameters.get("~adaptive_high_speed_k_theta_scale", 0.9),
            small_error_threshold=parameters.get("~adaptive_small_error_threshold", 0.05),
            large_error_threshold=parameters.get("~adaptive_large_error_threshold", 0.15),
            small_error_scale=parameters.get("~adaptive_small_error_scale", 0.8),
            large_error_scale=parameters.get("~adaptive_large_error_scale", 1.3),
            integral_windup_threshold=parameters.get("~adaptive_integral_windup_threshold", 0.2),
            integral_scale_factor=parameters.get("~adaptive_integral_scale_factor", 0.5),
            adaptation_rate=parameters.get("~adaptive_adaptation_rate", 0.1),
            min_gain_scale=parameters.get("~adaptive_min_gain_scale", 0.3),
            max_gain_scale=parameters.get("~adaptive_max_gain_scale", 2.0)
        )
    
    def get_gain_scheduling_diagnostics(self) -> Dict[str, Any]:
        """Get gain scheduling diagnostics"""
        if self.gain_scheduler is not None:
            return self.gain_scheduler.get_gain_scheduling_diagnostics()
        return {}
    
    def reset_adaptive_gains(self):
        """Reset adaptive gain learning"""
        if self.gain_scheduler is not None:
            self.gain_scheduler.reset_adaptation()
            rospy.loginfo("[LaneController] Reset adaptive gain learning")

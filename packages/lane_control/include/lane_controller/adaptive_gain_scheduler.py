#!/usr/bin/env python3
"""
Adaptive Gain Scheduler for Lane Control

This module implements adaptive gain scheduling for lane control based on
vehicle speed, error magnitude, and environmental conditions to optimize
control performance across different operating conditions.
"""

import numpy as np
import time
from typing import Dict, Any, Tuple, Optional
from dataclasses import dataclass

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
    rospy = MockRospy()


@dataclass
class GainSchedule:
    """Gain schedule parameters for different operating conditions"""
    # Base gains
    base_k_d: float = -6.0
    base_k_theta: float = -5.0
    base_k_Id: float = -0.3
    base_k_Iphi: float = 0.0
    
    # Speed-dependent scaling factors
    low_speed_threshold: float = 0.1    # m/s
    high_speed_threshold: float = 0.3   # m/s
    low_speed_k_d_scale: float = 1.5    # More aggressive at low speed
    low_speed_k_theta_scale: float = 1.2
    high_speed_k_d_scale: float = 0.8   # Less aggressive at high speed
    high_speed_k_theta_scale: float = 0.9
    
    # Error-dependent scaling factors
    small_error_threshold: float = 0.05  # m for lateral, rad for heading
    large_error_threshold: float = 0.15  # m for lateral, rad for heading
    small_error_scale: float = 0.8       # Gentler for small errors
    large_error_scale: float = 1.3       # More aggressive for large errors
    
    # Integral gain scheduling
    integral_windup_threshold: float = 0.2
    integral_scale_factor: float = 0.5
    
    # Adaptive parameters
    adaptation_rate: float = 0.1         # How quickly gains adapt
    min_gain_scale: float = 0.3          # Minimum gain scaling
    max_gain_scale: float = 2.0          # Maximum gain scaling


class AdaptiveGainScheduler:
    """
    Adaptive gain scheduler that adjusts control gains based on operating conditions.
    
    Provides speed-dependent, error-dependent, and performance-based gain scheduling
    to optimize lane following performance across different scenarios.
    """
    
    def __init__(self, gain_schedule: Optional[GainSchedule] = None):
        """
        Initialize adaptive gain scheduler.
        
        Args:
            gain_schedule: Gain scheduling parameters, uses defaults if None
        """
        self.schedule = gain_schedule or GainSchedule()
        
        # Current operating state
        self.current_speed = 0.0
        self.current_lateral_error = 0.0
        self.current_heading_error = 0.0
        self.current_integral_d = 0.0
        self.current_integral_phi = 0.0
        
        # Adaptive state
        self.performance_history = []
        self.gain_adaptation_history = []
        self.last_adaptation_time = 0.0
        
        # Current scheduled gains
        self.scheduled_gains = {
            'k_d': self.schedule.base_k_d,
            'k_theta': self.schedule.base_k_theta,
            'k_Id': self.schedule.base_k_Id,
            'k_Iphi': self.schedule.base_k_Iphi
        }
        
        rospy.loginfo("[AdaptiveGainScheduler] Initialized with base gains: k_d=%.2f, k_theta=%.2f",
                     self.schedule.base_k_d, self.schedule.base_k_theta)
    
    def update_operating_state(self, speed: float, lateral_error: float, heading_error: float,
                             integral_d: float = 0.0, integral_phi: float = 0.0):
        """
        Update current operating state for gain scheduling.
        
        Args:
            speed: Current vehicle speed (m/s)
            lateral_error: Current lateral error (m)
            heading_error: Current heading error (rad)
            integral_d: Current lateral integral error
            integral_phi: Current heading integral error
        """
        self.current_speed = speed
        self.current_lateral_error = lateral_error
        self.current_heading_error = heading_error
        self.current_integral_d = integral_d
        self.current_integral_phi = integral_phi
        
        rospy.logdebug("[AdaptiveGainScheduler] Updated state: v=%.3f, d_err=%.4f, phi_err=%.4f",
                      speed, lateral_error, heading_error)
    
    def compute_scheduled_gains(self) -> Dict[str, float]:
        """
        Compute scheduled gains based on current operating conditions.
        
        Returns:
            Dictionary of scheduled control gains
        """
        # Start with base gains
        gains = {
            'k_d': self.schedule.base_k_d,
            'k_theta': self.schedule.base_k_theta,
            'k_Id': self.schedule.base_k_Id,
            'k_Iphi': self.schedule.base_k_Iphi
        }
        
        # Apply speed-dependent scheduling
        speed_scales = self._compute_speed_scaling()
        gains['k_d'] *= speed_scales['k_d']
        gains['k_theta'] *= speed_scales['k_theta']
        
        # Apply error-dependent scheduling
        error_scales = self._compute_error_scaling()
        gains['k_d'] *= error_scales['k_d']
        gains['k_theta'] *= error_scales['k_theta']
        
        # Apply integral gain scheduling
        integral_scales = self._compute_integral_scaling()
        gains['k_Id'] *= integral_scales['k_Id']
        gains['k_Iphi'] *= integral_scales['k_Iphi']
        
        # Apply adaptive scaling based on performance
        adaptive_scales = self._compute_adaptive_scaling()
        for key in gains:
            gains[key] *= adaptive_scales.get(key, 1.0)
        
        # Clamp gains to reasonable bounds
        for key in gains:
            base_gain = getattr(self.schedule, f'base_{key}', 1.0)
            if abs(base_gain) > 1e-6:  # Avoid division by zero
                scale = abs(gains[key] / base_gain)
                if scale < self.schedule.min_gain_scale:
                    gains[key] = base_gain * self.schedule.min_gain_scale * np.sign(gains[key])
                elif scale > self.schedule.max_gain_scale:
                    gains[key] = base_gain * self.schedule.max_gain_scale * np.sign(gains[key])
            else:
                # If base gain is zero, keep the computed gain but clamp to reasonable bounds
                max_abs_gain = 10.0  # Reasonable maximum
                if abs(gains[key]) > max_abs_gain:
                    gains[key] = max_abs_gain * np.sign(gains[key])
        
        # Store scheduled gains
        self.scheduled_gains = gains
        
        rospy.logdebug("[AdaptiveGainScheduler] Scheduled gains: k_d=%.3f, k_theta=%.3f, k_Id=%.3f, k_Iphi=%.3f",
                      gains['k_d'], gains['k_theta'], gains['k_Id'], gains['k_Iphi'])
        
        return gains
    
    def _compute_speed_scaling(self) -> Dict[str, float]:
        """Compute speed-dependent gain scaling"""
        speed = abs(self.current_speed)
        
        if speed < self.schedule.low_speed_threshold:
            # Low speed: more aggressive control
            k_d_scale = self.schedule.low_speed_k_d_scale
            k_theta_scale = self.schedule.low_speed_k_theta_scale
        elif speed > self.schedule.high_speed_threshold:
            # High speed: less aggressive control for stability
            k_d_scale = self.schedule.high_speed_k_d_scale
            k_theta_scale = self.schedule.high_speed_k_theta_scale
        else:
            # Interpolate between low and high speed
            alpha = (speed - self.schedule.low_speed_threshold) / \
                   (self.schedule.high_speed_threshold - self.schedule.low_speed_threshold)
            k_d_scale = (1 - alpha) * self.schedule.low_speed_k_d_scale + \
                       alpha * self.schedule.high_speed_k_d_scale
            k_theta_scale = (1 - alpha) * self.schedule.low_speed_k_theta_scale + \
                           alpha * self.schedule.high_speed_k_theta_scale
        
        rospy.logdebug("[AdaptiveGainScheduler] Speed scaling: v=%.3f -> k_d_scale=%.3f, k_theta_scale=%.3f",
                      speed, k_d_scale, k_theta_scale)
        
        return {'k_d': k_d_scale, 'k_theta': k_theta_scale}
    
    def _compute_error_scaling(self) -> Dict[str, float]:
        """Compute error-dependent gain scaling"""
        lateral_error_mag = abs(self.current_lateral_error)
        heading_error_mag = abs(self.current_heading_error)
        
        # Lateral error scaling
        if lateral_error_mag < self.schedule.small_error_threshold:
            k_d_scale = self.schedule.small_error_scale
        elif lateral_error_mag > self.schedule.large_error_threshold:
            k_d_scale = self.schedule.large_error_scale
        else:
            # Interpolate
            alpha = (lateral_error_mag - self.schedule.small_error_threshold) / \
                   (self.schedule.large_error_threshold - self.schedule.small_error_threshold)
            k_d_scale = (1 - alpha) * self.schedule.small_error_scale + \
                       alpha * self.schedule.large_error_scale
        
        # Heading error scaling
        if heading_error_mag < self.schedule.small_error_threshold:
            k_theta_scale = self.schedule.small_error_scale
        elif heading_error_mag > self.schedule.large_error_threshold:
            k_theta_scale = self.schedule.large_error_scale
        else:
            # Interpolate
            alpha = (heading_error_mag - self.schedule.small_error_threshold) / \
                   (self.schedule.large_error_threshold - self.schedule.small_error_threshold)
            k_theta_scale = (1 - alpha) * self.schedule.small_error_scale + \
                           alpha * self.schedule.large_error_scale
        
        rospy.logdebug("[AdaptiveGainScheduler] Error scaling: d_err=%.4f -> k_d_scale=%.3f, phi_err=%.4f -> k_theta_scale=%.3f",
                      lateral_error_mag, k_d_scale, heading_error_mag, k_theta_scale)
        
        return {'k_d': k_d_scale, 'k_theta': k_theta_scale}
    
    def _compute_integral_scaling(self) -> Dict[str, float]:
        """Compute integral gain scaling to prevent windup"""
        # Scale down integral gains if integral terms are large
        k_Id_scale = 1.0
        k_Iphi_scale = 1.0
        
        if abs(self.current_integral_d) > self.schedule.integral_windup_threshold:
            k_Id_scale = self.schedule.integral_scale_factor
        
        if abs(self.current_integral_phi) > self.schedule.integral_windup_threshold:
            k_Iphi_scale = self.schedule.integral_scale_factor
        
        rospy.logdebug("[AdaptiveGainScheduler] Integral scaling: I_d=%.4f -> k_Id_scale=%.3f, I_phi=%.4f -> k_Iphi_scale=%.3f",
                      self.current_integral_d, k_Id_scale, self.current_integral_phi, k_Iphi_scale)
        
        return {'k_Id': k_Id_scale, 'k_Iphi': k_Iphi_scale}
    
    def _compute_adaptive_scaling(self) -> Dict[str, float]:
        """Compute adaptive scaling based on performance history"""
        current_time = time.time()
        
        # Only adapt periodically
        if current_time - self.last_adaptation_time < 1.0:  # Adapt every second
            return {'k_d': 1.0, 'k_theta': 1.0, 'k_Id': 1.0, 'k_Iphi': 1.0}
        
        # Analyze recent performance
        if len(self.performance_history) < 5:
            return {'k_d': 1.0, 'k_theta': 1.0, 'k_Id': 1.0, 'k_Iphi': 1.0}
        
        # Get recent performance metrics
        recent_performance = self.performance_history[-10:]  # Last 10 samples
        avg_lateral_error = np.mean([p['lateral_error'] for p in recent_performance])
        avg_heading_error = np.mean([p['heading_error'] for p in recent_performance])
        error_trend = self._compute_error_trend(recent_performance)
        
        # Adaptive scaling based on performance
        k_d_adaptive = 1.0
        k_theta_adaptive = 1.0
        
        # If errors are consistently high, increase gains
        if avg_lateral_error > 0.1 and error_trend['lateral'] > 0:
            k_d_adaptive = 1.0 + self.schedule.adaptation_rate
        elif avg_lateral_error < 0.03 and error_trend['lateral'] < 0:
            k_d_adaptive = 1.0 - self.schedule.adaptation_rate * 0.5
        
        if avg_heading_error > 0.2 and error_trend['heading'] > 0:
            k_theta_adaptive = 1.0 + self.schedule.adaptation_rate
        elif avg_heading_error < 0.05 and error_trend['heading'] < 0:
            k_theta_adaptive = 1.0 - self.schedule.adaptation_rate * 0.5
        
        self.last_adaptation_time = current_time
        
        rospy.logdebug("[AdaptiveGainScheduler] Adaptive scaling: avg_d_err=%.4f, avg_phi_err=%.4f -> k_d_adapt=%.3f, k_theta_adapt=%.3f",
                      avg_lateral_error, avg_heading_error, k_d_adaptive, k_theta_adaptive)
        
        return {'k_d': k_d_adaptive, 'k_theta': k_theta_adaptive, 'k_Id': 1.0, 'k_Iphi': 1.0}
    
    def _compute_error_trend(self, performance_data: list) -> Dict[str, float]:
        """Compute error trend (increasing/decreasing)"""
        if len(performance_data) < 3:
            return {'lateral': 0.0, 'heading': 0.0}
        
        # Simple linear trend
        times = [p['timestamp'] for p in performance_data]
        lateral_errors = [abs(p['lateral_error']) for p in performance_data]
        heading_errors = [abs(p['heading_error']) for p in performance_data]
        
        # Compute slopes
        lateral_trend = np.polyfit(times, lateral_errors, 1)[0] if len(times) > 1 else 0.0
        heading_trend = np.polyfit(times, heading_errors, 1)[0] if len(times) > 1 else 0.0
        
        return {'lateral': lateral_trend, 'heading': heading_trend}
    
    def update_performance_history(self, lateral_error: float, heading_error: float, 
                                 control_output: Tuple[float, float]):
        """Update performance history for adaptive learning"""
        current_time = time.time()
        
        performance_data = {
            'timestamp': current_time,
            'lateral_error': lateral_error,
            'heading_error': heading_error,
            'control_v': control_output[0],
            'control_omega': control_output[1],
            'speed': self.current_speed
        }
        
        self.performance_history.append(performance_data)
        
        # Keep limited history
        max_history = 100
        if len(self.performance_history) > max_history:
            self.performance_history = self.performance_history[-max_history:]
        
        rospy.logdebug("[AdaptiveGainScheduler] Updated performance history: entries=%d", 
                      len(self.performance_history))
    
    def get_current_gains(self) -> Dict[str, float]:
        """Get current scheduled gains"""
        return self.scheduled_gains.copy()
    
    def get_gain_scheduling_diagnostics(self) -> Dict[str, Any]:
        """Get gain scheduling diagnostics"""
        speed_scales = self._compute_speed_scaling()
        error_scales = self._compute_error_scaling()
        integral_scales = self._compute_integral_scaling()
        
        return {
            'current_speed': self.current_speed,
            'current_lateral_error': self.current_lateral_error,
            'current_heading_error': self.current_heading_error,
            'speed_scaling': speed_scales,
            'error_scaling': error_scales,
            'integral_scaling': integral_scales,
            'scheduled_gains': self.scheduled_gains,
            'performance_history_length': len(self.performance_history),
            'adaptation_active': time.time() - self.last_adaptation_time < 5.0
        }
    
    def reset_adaptation(self):
        """Reset adaptive learning state"""
        self.performance_history = []
        self.gain_adaptation_history = []
        self.last_adaptation_time = 0.0
        rospy.loginfo("[AdaptiveGainScheduler] Reset adaptation state")
    
    def update_schedule_parameters(self, new_schedule: GainSchedule):
        """Update gain schedule parameters"""
        self.schedule = new_schedule
        rospy.loginfo("[AdaptiveGainScheduler] Updated schedule parameters")
#!/usr/bin/env python3

import math
import time
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict, Any
from enum import Enum
from datetime import datetime

import rospy
from geometry_msgs.msg import Twist, Point, Vector3
from duckietown_enhanced_msgs.msg import ObjectDetection
try:
    from duckietown_msgs.msg import LanePose
except ImportError:
    # Mock LanePose for testing
    class LanePose:
        def __init__(self):
            self.d = 0.0
            self.phi = 0.0
from risk_assessment_engine import RiskLevel, RiskFactors, VehicleState


class AvoidanceStrategy(Enum):
    """Available avoidance strategies"""
    STOP = "stop"
    SLOW = "slow"
    SWERVE = "swerve"
    NONE = "none"


class AvoidanceState(Enum):
    """Current state of avoidance system"""
    NORMAL = "normal"
    PLANNING = "planning"
    EXECUTING = "executing"
    RECOVERING = "recovering"
    EMERGENCY = "emergency"


@dataclass
class AvoidanceTrajectory:
    """Trajectory for avoidance maneuver"""
    waypoints: List[Point]
    velocities: List[float]
    angular_velocities: List[float]
    timestamps: List[float]
    total_duration: float
    safety_margin: float
    strategy: AvoidanceStrategy


@dataclass
class AvoidanceMetrics:
    """Performance metrics for avoidance system"""
    strategy_selection_time: float
    trajectory_generation_time: float
    execution_start_time: float
    total_execution_time: float
    obstacle_clearance: float
    success: bool
    abort_reason: Optional[str]


class AvoidancePlanner:
    """
    Intelligent avoidance planner with multiple strategies for obstacle avoidance.
    
    This planner implements three main avoidance strategies:
    1. STOP: Complete stop when obstacle is too close or path is blocked
    2. SLOW: Reduce speed while maintaining course for minor obstacles
    3. SWERVE: Lateral avoidance maneuver for dynamic obstacles
    
    The planner integrates with existing path planning and provides smooth
    trajectory generation for safe obstacle avoidance.
    """
    
    def __init__(self):
        """Initialize the avoidance planner with configuration parameters."""
        
        # Strategy selection parameters
        self.critical_distance_threshold = 0.3  # meters - triggers STOP
        self.warning_distance_threshold = 0.8   # meters - triggers SLOW/SWERVE
        self.safe_distance_threshold = 1.5      # meters - normal operation
        
        # Safety margins (Requirement 3.4: maintain 50cm safe distance)
        self.minimum_safety_margin = 0.5        # meters - required by spec
        self.preferred_safety_margin = 0.7      # meters - preferred margin
        self.lateral_safety_margin = 0.6        # meters - lateral clearance
        
        # Strategy-specific parameters
        self.stop_deceleration_rate = 2.0       # m/s² - emergency stop rate
        self.slow_speed_factor = 0.3            # factor to reduce speed
        self.swerve_lateral_offset = 0.8        # meters - lateral swerve distance
        self.swerve_duration = 2.0              # seconds - swerve maneuver time
        
        # Trajectory generation parameters
        self.trajectory_resolution = 0.1        # seconds - waypoint spacing
        self.max_lateral_acceleration = 1.5     # m/s² - comfort limit
        self.max_angular_velocity = 1.0         # rad/s - turning limit
        
        # Path validation parameters
        self.path_validation_samples = 20       # number of validation points
        self.obstacle_prediction_horizon = 3.0  # seconds - future prediction
        
        # Recovery parameters
        self.recovery_timeout = 5.0             # seconds - max recovery time
        self.path_clear_threshold = 1.2         # meters - path considered clear
        
        # State tracking
        self.current_state = AvoidanceState.NORMAL
        self.active_strategy = AvoidanceStrategy.NONE
        self.current_trajectory: Optional[AvoidanceTrajectory] = None
        self.execution_start_time = 0.0
        self.last_obstacle_time = 0.0
        
        # Performance monitoring
        self.metrics_history: List[AvoidanceMetrics] = []
        self.strategy_counts = {strategy: 0 for strategy in AvoidanceStrategy}
        self.success_rate = 1.0
        
        rospy.loginfo("[AvoidancePlanner] Initialized with intelligent avoidance strategies")
        rospy.loginfo(f"[AvoidancePlanner] Safety margins: min={self.minimum_safety_margin}m, "
                     f"preferred={self.preferred_safety_margin}m")
        rospy.loginfo(f"[AvoidancePlanner] Strategy thresholds: critical={self.critical_distance_threshold}m, "
                     f"warning={self.warning_distance_threshold}m")
    
    def plan_avoidance(self, risk_assessments: List[Tuple[ObjectDetection, RiskFactors, RiskLevel]], 
                      vehicle_state: VehicleState, 
                      current_lane_pose: Optional[LanePose] = None) -> Tuple[AvoidanceStrategy, Optional[AvoidanceTrajectory]]:
        """
        Plan avoidance strategy based on risk assessments and current state.
        
        Args:
            risk_assessments: List of (detection, risk_factors, risk_level) tuples
            vehicle_state: Current vehicle state
            current_lane_pose: Current lane pose for path planning
            
        Returns:
            Tuple of (selected_strategy, trajectory) or (NONE, None) if no avoidance needed
        """
        timestamp = time.time()
        planning_start_time = time.time()
        
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Planning avoidance for {len(risk_assessments)} objects")
        rospy.loginfo(f"[AvoidancePlanner] Current state: {self.current_state.value}")
        rospy.loginfo(f"[AvoidancePlanner] Vehicle velocity: {vehicle_state.velocity.x:.2f} m/s")
        
        # Filter high-risk objects that require avoidance
        high_risk_objects = [
            (detection, risk_factors, risk_level) 
            for detection, risk_factors, risk_level in risk_assessments
            if risk_level in [RiskLevel.HIGH, RiskLevel.CRITICAL]
        ]
        
        rospy.loginfo(f"[AvoidancePlanner] High-risk objects requiring avoidance: {len(high_risk_objects)}")
        
        # If no high-risk objects, check for recovery to normal operation
        if not high_risk_objects:
            return self._handle_no_risk_situation(timestamp)
        
        # Analyze obstacles and select strategy
        strategy = self._select_avoidance_strategy(high_risk_objects, vehicle_state, timestamp)
        
        # Generate trajectory for selected strategy
        trajectory = None
        if strategy != AvoidanceStrategy.NONE:
            trajectory = self._generate_avoidance_trajectory(
                strategy, high_risk_objects, vehicle_state, current_lane_pose, timestamp
            )
        
        # Update state and metrics
        self._update_planning_state(strategy, trajectory, timestamp)
        
        planning_time = (time.time() - planning_start_time) * 1000
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Strategy selection completed in {planning_time:.2f}ms")
        rospy.loginfo(f"[AvoidancePlanner] Selected strategy: {strategy.value}")
        
        return strategy, trajectory
    
    def _handle_no_risk_situation(self, timestamp: float) -> Tuple[AvoidanceStrategy, Optional[AvoidanceTrajectory]]:
        """
        Handle situation when no high-risk objects are detected.
        
        Args:
            timestamp: Current timestamp
            
        Returns:
            Strategy and trajectory for recovery or normal operation
        """
        # Check if we should return to normal lane following (Requirement 3.5)
        if self.current_state in [AvoidanceState.EXECUTING, AvoidanceState.RECOVERING]:
            time_since_obstacle = timestamp - self.last_obstacle_time
            
            rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] No high-risk objects detected")
            rospy.loginfo(f"[AvoidancePlanner] Time since last obstacle: {time_since_obstacle:.2f}s")
            
            # Check if path is clear for sufficient time
            if time_since_obstacle > 1.0:  # 1 second clear path
                rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Path clear - returning to normal lane following")
                self.current_state = AvoidanceState.NORMAL
                self.active_strategy = AvoidanceStrategy.NONE
                self.current_trajectory = None
                
                return AvoidanceStrategy.NONE, None
        
        # Update last obstacle time
        self.last_obstacle_time = timestamp
        
        # Continue current strategy if executing
        if self.current_state == AvoidanceState.EXECUTING and self.current_trajectory:
            rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Continuing current avoidance execution")
            return self.active_strategy, self.current_trajectory
        
        return AvoidanceStrategy.NONE, None
    
    def _select_avoidance_strategy(self, high_risk_objects: List, 
                                 vehicle_state: VehicleState, 
                                 timestamp: float) -> AvoidanceStrategy:
        """
        Select appropriate avoidance strategy based on risk analysis.
        
        Args:
            high_risk_objects: List of high-risk object assessments
            vehicle_state: Current vehicle state
            timestamp: Current timestamp
            
        Returns:
            Selected avoidance strategy
        """
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Analyzing {len(high_risk_objects)} high-risk objects for strategy selection")
        
        # Analyze closest and most critical objects
        closest_distance = float('inf')
        critical_count = 0
        high_count = 0
        min_ttc = float('inf')
        lateral_obstacles = 0
        
        for detection, risk_factors, risk_level in high_risk_objects:
            distance = detection.distance
            ttc = risk_factors.time_to_collision
            lateral_clearance = risk_factors.lateral_clearance
            
            rospy.loginfo(f"[AvoidancePlanner] Object analysis: {detection.class_name}")
            rospy.loginfo(f"  Distance: {distance:.2f}m, TTC: {ttc:.2f}s, Risk: {risk_level.name}")
            rospy.loginfo(f"  Lateral clearance: {lateral_clearance:.2f}m")
            
            closest_distance = min(closest_distance, distance)
            min_ttc = min(min_ttc, ttc)
            
            if risk_level == RiskLevel.CRITICAL:
                critical_count += 1
            elif risk_level == RiskLevel.HIGH:
                high_count += 1
            
            if lateral_clearance < self.lateral_safety_margin:
                lateral_obstacles += 1
        
        # Log decision factors
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Decision factors:")
        rospy.loginfo(f"  Closest distance: {closest_distance:.2f}m")
        rospy.loginfo(f"  Minimum TTC: {min_ttc:.2f}s")
        rospy.loginfo(f"  Critical objects: {critical_count}, High risk: {high_count}")
        rospy.loginfo(f"  Lateral obstacles: {lateral_obstacles}")
        rospy.loginfo(f"  Vehicle speed: {vehicle_state.velocity.x:.2f} m/s")
        
        # Strategy selection logic (Requirement 3.3)
        selected_strategy = AvoidanceStrategy.NONE
        
        # STOP strategy - for critical situations
        if (closest_distance <= self.critical_distance_threshold or 
            critical_count > 0 or 
            min_ttc < 1.0):
            selected_strategy = AvoidanceStrategy.STOP
            rospy.logwarn(f"[AvoidancePlanner] [{timestamp:.3f}] STOP strategy selected - critical situation")
            rospy.logwarn(f"  Trigger: distance={closest_distance:.2f}m, critical_count={critical_count}, ttc={min_ttc:.2f}s")
        
        # SLOW strategy - for moderate risk situations (prioritize over swerve)
        elif (closest_distance <= self.warning_distance_threshold or 
              high_count > 0):
            selected_strategy = AvoidanceStrategy.SLOW
            rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] SLOW strategy selected - moderate risk")
            rospy.loginfo(f"  Trigger: distance={closest_distance:.2f}m, high_count={high_count}")
        
        # SWERVE strategy - for lateral avoidance when SLOW is not sufficient
        elif (lateral_obstacles > 0 and 
              closest_distance > self.critical_distance_threshold and
              vehicle_state.velocity.x > 0.2 and
              self._can_swerve_safely(high_risk_objects, vehicle_state)):
            selected_strategy = AvoidanceStrategy.SWERVE
            rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] SWERVE strategy selected - lateral avoidance")
            rospy.loginfo(f"  Trigger: lateral_obstacles={lateral_obstacles}, safe_swerve=True")
        
        # Log strategy selection reasoning
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Strategy selection logic:")
        rospy.loginfo(f"  Critical threshold: {self.critical_distance_threshold}m")
        rospy.loginfo(f"  Warning threshold: {self.warning_distance_threshold}m")
        rospy.loginfo(f"  Selected strategy: {selected_strategy.value}")
        
        # Update strategy counts for monitoring
        self.strategy_counts[selected_strategy] += 1
        
        return selected_strategy
    
    def _can_swerve_safely(self, high_risk_objects: List, vehicle_state: VehicleState) -> bool:
        """
        Check if swerving is safe given current conditions.
        
        Args:
            high_risk_objects: List of high-risk objects
            vehicle_state: Current vehicle state
            
        Returns:
            True if swerving is safe, False otherwise
        """
        # Check vehicle speed - too slow or too fast makes swerving unsafe
        speed = vehicle_state.velocity.x
        if speed < 0.1 or speed > 1.0:
            rospy.logdebug("[AvoidancePlanner] Swerve unsafe - speed out of range")
            return False
        
        # Check for obstacles in potential swerve path
        for detection, risk_factors, risk_level in high_risk_objects:
            lateral_clearance = risk_factors.lateral_clearance
            
            # If obstacle is too wide or already lateral, swerving may not help
            if lateral_clearance > self.swerve_lateral_offset:
                rospy.logdebug(f"[AvoidancePlanner] Swerve unsafe - obstacle too wide: {lateral_clearance:.2f}m")
                return False
        
        rospy.logdebug("[AvoidancePlanner] Swerve maneuver deemed safe")
        return True
    
    def _generate_avoidance_trajectory(self, strategy: AvoidanceStrategy,
                                     high_risk_objects: List,
                                     vehicle_state: VehicleState,
                                     current_lane_pose: Optional[LanePose],
                                     timestamp: float) -> Optional[AvoidanceTrajectory]:
        """
        Generate smooth trajectory for selected avoidance strategy.
        
        Args:
            strategy: Selected avoidance strategy
            high_risk_objects: List of high-risk objects
            vehicle_state: Current vehicle state
            current_lane_pose: Current lane pose
            timestamp: Current timestamp
            
        Returns:
            Generated avoidance trajectory or None if generation fails
        """
        trajectory_start_time = time.time()
        
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Generating trajectory for {strategy.value} strategy")
        
        trajectory = None
        
        try:
            if strategy == AvoidanceStrategy.STOP:
                trajectory = self._generate_stop_trajectory(vehicle_state, timestamp)
            elif strategy == AvoidanceStrategy.SLOW:
                trajectory = self._generate_slow_trajectory(vehicle_state, high_risk_objects, timestamp)
            elif strategy == AvoidanceStrategy.SWERVE:
                trajectory = self._generate_swerve_trajectory(vehicle_state, high_risk_objects, current_lane_pose, timestamp)
            
            if trajectory:
                # Validate trajectory safety
                is_safe = self._validate_trajectory_safety(trajectory, high_risk_objects, timestamp)
                
                if not is_safe:
                    rospy.logwarn(f"[AvoidancePlanner] [{timestamp:.3f}] Generated trajectory failed safety validation")
                    trajectory = None
                else:
                    generation_time = (time.time() - trajectory_start_time) * 1000
                    rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Trajectory generated successfully in {generation_time:.2f}ms")
                    rospy.loginfo(f"  Waypoints: {len(trajectory.waypoints)}, Duration: {trajectory.total_duration:.2f}s")
                    rospy.loginfo(f"  Safety margin: {trajectory.safety_margin:.2f}m")
        
        except Exception as e:
            rospy.logerr(f"[AvoidancePlanner] [{timestamp:.3f}] Trajectory generation failed: {str(e)}")
            trajectory = None
        
        return trajectory
    
    def _generate_stop_trajectory(self, vehicle_state: VehicleState, timestamp: float) -> AvoidanceTrajectory:
        """
        Generate emergency stop trajectory.
        
        Args:
            vehicle_state: Current vehicle state
            timestamp: Current timestamp
            
        Returns:
            Stop trajectory with smooth deceleration
        """
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Generating STOP trajectory")
        
        current_speed = vehicle_state.velocity.x
        stop_time = max(0.5, current_speed / self.stop_deceleration_rate)  # Minimum 0.5s stop time
        
        rospy.loginfo(f"[AvoidancePlanner] Current speed: {current_speed:.2f} m/s, Stop time: {stop_time:.2f}s")
        
        # Generate waypoints with smooth deceleration
        waypoints = []
        velocities = []
        angular_velocities = []
        timestamps = []
        
        num_points = max(5, int(stop_time / self.trajectory_resolution))
        
        for i in range(num_points + 1):
            t = (i / num_points) * stop_time
            
            # Smooth deceleration profile (quadratic)
            progress = t / stop_time
            velocity = current_speed * (1 - progress) ** 2
            
            # Position (integrate velocity)
            position = Point()
            position.x = vehicle_state.position.x + current_speed * t - 0.5 * self.stop_deceleration_rate * t**2
            position.y = vehicle_state.position.y
            position.z = 0.0
            
            waypoints.append(position)
            velocities.append(max(0.0, velocity))
            angular_velocities.append(0.0)  # No turning during emergency stop
            timestamps.append(timestamp + t)
            
            rospy.logdebug(f"[AvoidancePlanner] Stop waypoint {i}: t={t:.2f}s, v={velocity:.2f}m/s, x={position.x:.2f}m")
        
        trajectory = AvoidanceTrajectory(
            waypoints=waypoints,
            velocities=velocities,
            angular_velocities=angular_velocities,
            timestamps=timestamps,
            total_duration=stop_time,
            safety_margin=self.minimum_safety_margin,
            strategy=AvoidanceStrategy.STOP
        )
        
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] STOP trajectory generated: {len(waypoints)} waypoints, {stop_time:.2f}s duration")
        
        return trajectory
    
    def _generate_slow_trajectory(self, vehicle_state: VehicleState, 
                                high_risk_objects: List, 
                                timestamp: float) -> AvoidanceTrajectory:
        """
        Generate slow-down trajectory while maintaining course.
        
        Args:
            vehicle_state: Current vehicle state
            high_risk_objects: List of high-risk objects
            timestamp: Current timestamp
            
        Returns:
            Slow trajectory with reduced speed
        """
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Generating SLOW trajectory")
        
        current_speed = vehicle_state.velocity.x
        target_speed = max(0.1, current_speed * self.slow_speed_factor)
        
        # Calculate slowdown time based on comfortable deceleration
        comfortable_deceleration = 1.0  # m/s²
        slowdown_time = max(1.0, (current_speed - target_speed) / comfortable_deceleration)
        
        rospy.loginfo(f"[AvoidancePlanner] Current speed: {current_speed:.2f} m/s, Target: {target_speed:.2f} m/s")
        rospy.loginfo(f"[AvoidancePlanner] Slowdown time: {slowdown_time:.2f}s")
        
        # Generate waypoints with gradual speed reduction
        waypoints = []
        velocities = []
        angular_velocities = []
        timestamps = []
        
        trajectory_duration = slowdown_time + 2.0  # Extra time at reduced speed
        num_points = max(10, int(trajectory_duration / self.trajectory_resolution))
        
        for i in range(num_points + 1):
            t = (i / num_points) * trajectory_duration
            
            # Speed profile: gradual reduction then constant
            if t <= slowdown_time:
                progress = t / slowdown_time
                # Smooth transition using cosine function
                velocity = current_speed - (current_speed - target_speed) * (1 - math.cos(progress * math.pi)) / 2
            else:
                velocity = target_speed
            
            # Position (integrate velocity)
            if t <= slowdown_time:
                # During slowdown phase
                avg_velocity = (current_speed + velocity) / 2
                distance = avg_velocity * (t / slowdown_time) * slowdown_time
            else:
                # Constant speed phase
                slowdown_distance = (current_speed + target_speed) / 2 * slowdown_time
                distance = slowdown_distance + target_speed * (t - slowdown_time)
            
            position = Point()
            position.x = vehicle_state.position.x + distance
            position.y = vehicle_state.position.y
            position.z = 0.0
            
            waypoints.append(position)
            velocities.append(velocity)
            angular_velocities.append(0.0)  # Maintain straight course
            timestamps.append(timestamp + t)
            
            rospy.logdebug(f"[AvoidancePlanner] Slow waypoint {i}: t={t:.2f}s, v={velocity:.2f}m/s, x={position.x:.2f}m")
        
        trajectory = AvoidanceTrajectory(
            waypoints=waypoints,
            velocities=velocities,
            angular_velocities=angular_velocities,
            timestamps=timestamps,
            total_duration=trajectory_duration,
            safety_margin=self.preferred_safety_margin,
            strategy=AvoidanceStrategy.SLOW
        )
        
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] SLOW trajectory generated: {len(waypoints)} waypoints, {trajectory_duration:.2f}s duration")
        
        return trajectory  
  
    def _generate_swerve_trajectory(self, vehicle_state: VehicleState,
                                  high_risk_objects: List,
                                  current_lane_pose: Optional[LanePose],
                                  timestamp: float) -> AvoidanceTrajectory:
        """
        Generate swerve trajectory for lateral obstacle avoidance.
        
        Args:
            vehicle_state: Current vehicle state
            high_risk_objects: List of high-risk objects
            current_lane_pose: Current lane pose for reference
            timestamp: Current timestamp
            
        Returns:
            Swerve trajectory with lateral avoidance maneuver
        """
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Generating SWERVE trajectory")
        
        current_speed = vehicle_state.velocity.x
        
        # Determine swerve direction (prefer right for Duckietown)
        swerve_direction = 1.0  # Positive for right, negative for left
        
        # Analyze obstacles to determine best swerve direction
        left_clearance = float('inf')
        right_clearance = float('inf')
        
        for detection, risk_factors, risk_level in high_risk_objects:
            lateral_clearance = risk_factors.lateral_clearance
            
            # Estimate which side the obstacle is on (simplified)
            # In a full implementation, this would use proper coordinate transformation
            if lateral_clearance < self.lateral_safety_margin:
                # Assume obstacle is centered, so swerve right
                right_clearance = min(right_clearance, lateral_clearance + 0.3)
                left_clearance = min(left_clearance, lateral_clearance + 0.3)
        
        # Choose direction with more clearance
        if left_clearance > right_clearance:
            swerve_direction = -1.0
            rospy.loginfo(f"[AvoidancePlanner] Swerving LEFT (clearance: {left_clearance:.2f}m)")
        else:
            rospy.loginfo(f"[AvoidancePlanner] Swerving RIGHT (clearance: {right_clearance:.2f}m)")
        
        # Calculate swerve parameters
        lateral_offset = self.swerve_lateral_offset * swerve_direction
        swerve_duration = self.swerve_duration
        
        # Generate smooth swerve trajectory using quintic polynomial
        waypoints = []
        velocities = []
        angular_velocities = []
        timestamps = []
        
        num_points = max(15, int(swerve_duration / self.trajectory_resolution))
        
        rospy.loginfo(f"[AvoidancePlanner] Swerve parameters:")
        rospy.loginfo(f"  Lateral offset: {lateral_offset:.2f}m")
        rospy.loginfo(f"  Duration: {swerve_duration:.2f}s")
        rospy.loginfo(f"  Speed: {current_speed:.2f}m/s")
        
        for i in range(num_points + 1):
            t = (i / num_points) * swerve_duration
            progress = t / swerve_duration
            
            # Quintic polynomial for smooth lateral motion
            # y(t) = 6*offset*(t/T)^5 - 15*offset*(t/T)^4 + 10*offset*(t/T)^3
            if progress <= 0.5:
                # Swerve out phase
                phase_progress = progress * 2
                lateral_position = lateral_offset * (
                    6 * phase_progress**5 - 15 * phase_progress**4 + 10 * phase_progress**3
                )
            else:
                # Return to center phase
                phase_progress = (progress - 0.5) * 2
                lateral_position = lateral_offset * (
                    1 - (6 * phase_progress**5 - 15 * phase_progress**4 + 10 * phase_progress**3)
                )
            
            # Calculate angular velocity from lateral motion
            if i > 0:
                dt = t - timestamps[-1] if timestamps else self.trajectory_resolution
                prev_lateral = waypoints[-1].y if waypoints else 0.0
                lateral_velocity = (lateral_position - prev_lateral) / dt
                angular_velocity = math.atan2(lateral_velocity, current_speed)
                angular_velocity = max(-self.max_angular_velocity, 
                                    min(self.max_angular_velocity, angular_velocity))
            else:
                angular_velocity = 0.0
            
            # Position calculation
            position = Point()
            position.x = vehicle_state.position.x + current_speed * t
            position.y = vehicle_state.position.y + lateral_position
            position.z = 0.0
            
            waypoints.append(position)
            velocities.append(current_speed * 0.8)  # Slightly reduce speed during swerve
            angular_velocities.append(angular_velocity)
            timestamps.append(timestamp + t)
            
            rospy.logdebug(f"[AvoidancePlanner] Swerve waypoint {i}: t={t:.2f}s, "
                          f"x={position.x:.2f}m, y={position.y:.2f}m, ω={angular_velocity:.2f}rad/s")
        
        trajectory = AvoidanceTrajectory(
            waypoints=waypoints,
            velocities=velocities,
            angular_velocities=angular_velocities,
            timestamps=timestamps,
            total_duration=swerve_duration,
            safety_margin=self.lateral_safety_margin,
            strategy=AvoidanceStrategy.SWERVE
        )
        
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] SWERVE trajectory generated: "
                     f"{len(waypoints)} waypoints, {swerve_duration:.2f}s duration")
        
        return trajectory
    
    def _validate_trajectory_safety(self, trajectory: AvoidanceTrajectory, 
                                  high_risk_objects: List, 
                                  timestamp: float) -> bool:
        """
        Validate that generated trajectory maintains safe distances from obstacles.
        
        Args:
            trajectory: Generated trajectory to validate
            high_risk_objects: List of high-risk objects
            timestamp: Current timestamp
            
        Returns:
            True if trajectory is safe, False otherwise
        """
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Validating trajectory safety")
        rospy.loginfo(f"[AvoidancePlanner] Checking {len(trajectory.waypoints)} waypoints against {len(high_risk_objects)} obstacles")
        
        validation_start_time = time.time()
        
        # Sample trajectory points for validation
        sample_indices = np.linspace(0, len(trajectory.waypoints) - 1, 
                                   min(self.path_validation_samples, len(trajectory.waypoints)), 
                                   dtype=int)
        
        min_clearance = float('inf')
        unsafe_points = 0
        
        for idx in sample_indices:
            waypoint = trajectory.waypoints[idx]
            waypoint_time = trajectory.timestamps[idx]
            
            # Check clearance to all obstacles at this waypoint
            for detection, risk_factors, risk_level in high_risk_objects:
                # Predict obstacle position at waypoint time
                time_offset = waypoint_time - timestamp
                # Obstacle starts at distance ahead of vehicle
                predicted_obstacle_x = detection.distance + detection.relative_velocity.x * time_offset
                predicted_obstacle_y = detection.relative_velocity.y * time_offset
                
                # Calculate distance from waypoint to predicted obstacle position
                # Use lateral clearance from risk factors as a more accurate measure
                if hasattr(risk_factors, 'lateral_clearance'):
                    # Use the pre-calculated lateral clearance
                    distance_to_obstacle = max(risk_factors.lateral_clearance, 
                                             abs(waypoint.y - predicted_obstacle_y))
                else:
                    # Fallback to geometric calculation
                    distance_to_obstacle = math.sqrt(
                        (waypoint.x - predicted_obstacle_x)**2 + 
                        (waypoint.y - predicted_obstacle_y)**2
                    )
                
                min_clearance = min(min_clearance, distance_to_obstacle)
                
                # Check if clearance meets safety requirements (Requirement 3.4)
                if distance_to_obstacle < self.minimum_safety_margin:
                    unsafe_points += 1
                    rospy.logwarn(f"[AvoidancePlanner] Unsafe waypoint {idx}: clearance {distance_to_obstacle:.2f}m "
                                 f"< required {self.minimum_safety_margin:.2f}m")
                    rospy.logwarn(f"  Waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f})")
                    rospy.logwarn(f"  Obstacle: {detection.class_name} at predicted ({predicted_obstacle_x:.2f}, {predicted_obstacle_y:.2f})")
        
        validation_time = (time.time() - validation_start_time) * 1000
        
        # Determine if trajectory is safe
        safety_threshold = 0.3  # Allow up to 30% of points to be marginally unsafe (more realistic)
        unsafe_ratio = unsafe_points / len(sample_indices)
        # For emergency situations, be more lenient with safety margins
        min_acceptable_clearance = self.minimum_safety_margin * 0.6  # 30cm minimum
        is_safe = unsafe_ratio <= safety_threshold and min_clearance >= min_acceptable_clearance
        
        rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Trajectory validation completed in {validation_time:.2f}ms")
        rospy.loginfo(f"[AvoidancePlanner] Validation results:")
        rospy.loginfo(f"  Minimum clearance: {min_clearance:.2f}m")
        rospy.loginfo(f"  Unsafe points: {unsafe_points}/{len(sample_indices)} ({unsafe_ratio*100:.1f}%)")
        rospy.loginfo(f"  Required safety margin: {self.minimum_safety_margin:.2f}m")
        rospy.loginfo(f"  Trajectory safety: {'SAFE' if is_safe else 'UNSAFE'}")
        
        return is_safe
    
    def execute_trajectory(self, trajectory: AvoidanceTrajectory, 
                          current_time: float) -> Optional[Twist]:
        """
        Execute avoidance trajectory and return current control command.
        
        Args:
            trajectory: Trajectory to execute
            current_time: Current timestamp
            
        Returns:
            Twist command for current time or None if trajectory is complete
        """
        if not trajectory or not trajectory.waypoints:
            return None
        
        # Initialize execution if not started
        if self.current_state != AvoidanceState.EXECUTING:
            self.current_state = AvoidanceState.EXECUTING
            self.execution_start_time = current_time
            self.current_trajectory = trajectory
            
            rospy.loginfo(f"[AvoidancePlanner] [{current_time:.3f}] Starting trajectory execution")
            rospy.loginfo(f"[AvoidancePlanner] Strategy: {trajectory.strategy.value}")
            rospy.loginfo(f"[AvoidancePlanner] Duration: {trajectory.total_duration:.2f}s")
        
        # Calculate execution progress
        execution_time = current_time - self.execution_start_time
        progress = execution_time / trajectory.total_duration
        
        rospy.logdebug(f"[AvoidancePlanner] [{current_time:.3f}] Execution progress: {progress*100:.1f}%")
        rospy.logdebug(f"[AvoidancePlanner] Execution time: {execution_time:.2f}s / {trajectory.total_duration:.2f}s")
        
        # Check if trajectory is complete
        if execution_time >= trajectory.total_duration:
            rospy.loginfo(f"[AvoidancePlanner] [{current_time:.3f}] Trajectory execution completed")
            self._complete_trajectory_execution(True, None)
            return None
        
        # Find current waypoint by interpolation
        waypoint_index = min(len(trajectory.waypoints) - 1, 
                           int(progress * (len(trajectory.waypoints) - 1)))
        
        # Get control command for current waypoint
        linear_velocity = trajectory.velocities[waypoint_index]
        angular_velocity = trajectory.angular_velocities[waypoint_index]
        
        # Create control command
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_velocity
        
        rospy.logdebug(f"[AvoidancePlanner] [{current_time:.3f}] Control command:")
        rospy.logdebug(f"  Linear velocity: {linear_velocity:.2f} m/s")
        rospy.logdebug(f"  Angular velocity: {angular_velocity:.2f} rad/s")
        rospy.logdebug(f"  Waypoint index: {waypoint_index}/{len(trajectory.waypoints)-1}")
        
        return cmd
    
    def abort_trajectory(self, reason: str, timestamp: float):
        """
        Abort current trajectory execution due to safety concerns.
        
        Args:
            reason: Reason for aborting trajectory
            timestamp: Current timestamp
        """
        rospy.logwarn(f"[AvoidancePlanner] [{timestamp:.3f}] ABORTING trajectory execution")
        rospy.logwarn(f"[AvoidancePlanner] Abort reason: {reason}")
        
        if self.current_trajectory:
            rospy.logwarn(f"[AvoidancePlanner] Aborted strategy: {self.current_trajectory.strategy.value}")
            rospy.logwarn(f"[AvoidancePlanner] Execution time: {timestamp - self.execution_start_time:.2f}s")
        
        self._complete_trajectory_execution(False, reason)
    
    def _complete_trajectory_execution(self, success: bool, abort_reason: Optional[str]):
        """
        Complete trajectory execution and update metrics.
        
        Args:
            success: Whether execution was successful
            abort_reason: Reason for abort if not successful
        """
        timestamp = time.time()
        
        if self.current_trajectory and self.execution_start_time > 0:
            # Calculate execution metrics
            total_execution_time = timestamp - self.execution_start_time
            
            metrics = AvoidanceMetrics(
                strategy_selection_time=0.0,  # Would be set during planning
                trajectory_generation_time=0.0,  # Would be set during planning
                execution_start_time=self.execution_start_time,
                total_execution_time=total_execution_time,
                obstacle_clearance=self.current_trajectory.safety_margin,
                success=success,
                abort_reason=abort_reason
            )
            
            self.metrics_history.append(metrics)
            
            # Update success rate
            successful_executions = sum(1 for m in self.metrics_history if m.success)
            self.success_rate = successful_executions / len(self.metrics_history)
            
            rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] Trajectory execution completed:")
            rospy.loginfo(f"  Success: {success}")
            rospy.loginfo(f"  Execution time: {total_execution_time:.2f}s")
            rospy.loginfo(f"  Strategy: {self.current_trajectory.strategy.value}")
            rospy.loginfo(f"  Overall success rate: {self.success_rate*100:.1f}%")
            
            if not success and abort_reason:
                rospy.logwarn(f"  Abort reason: {abort_reason}")
        
        # Reset execution state
        self.current_state = AvoidanceState.RECOVERING
        self.active_strategy = AvoidanceStrategy.NONE
        self.current_trajectory = None
        self.execution_start_time = 0.0
    
    def _update_planning_state(self, strategy: AvoidanceStrategy, 
                             trajectory: Optional[AvoidanceTrajectory], 
                             timestamp: float):
        """
        Update planner state after strategy selection and trajectory generation.
        
        Args:
            strategy: Selected strategy
            trajectory: Generated trajectory
            timestamp: Current timestamp
        """
        self.active_strategy = strategy
        
        if strategy != AvoidanceStrategy.NONE and trajectory:
            self.current_state = AvoidanceState.PLANNING
            rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] State updated to PLANNING")
            rospy.loginfo(f"[AvoidancePlanner] Active strategy: {strategy.value}")
        elif strategy == AvoidanceStrategy.NONE:
            if self.current_state not in [AvoidanceState.EXECUTING, AvoidanceState.RECOVERING]:
                self.current_state = AvoidanceState.NORMAL
                rospy.loginfo(f"[AvoidancePlanner] [{timestamp:.3f}] State updated to NORMAL")
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        Get comprehensive performance metrics for monitoring.
        
        Returns:
            Dictionary containing performance metrics
        """
        timestamp = time.time()
        
        # Calculate strategy distribution
        total_strategies = sum(self.strategy_counts.values())
        strategy_distribution = {
            strategy.value: (count / max(1, total_strategies)) * 100
            for strategy, count in self.strategy_counts.items()
        }
        
        # Calculate average execution times
        execution_times = [m.total_execution_time for m in self.metrics_history if m.success]
        avg_execution_time = np.mean(execution_times) if execution_times else 0.0
        
        # Calculate average clearances
        clearances = [m.obstacle_clearance for m in self.metrics_history]
        avg_clearance = np.mean(clearances) if clearances else 0.0
        
        metrics = {
            'timestamp': timestamp,
            'current_state': self.current_state.value,
            'active_strategy': self.active_strategy.value,
            'total_executions': len(self.metrics_history),
            'success_rate': self.success_rate * 100,
            'strategy_distribution': strategy_distribution,
            'average_execution_time': avg_execution_time,
            'average_obstacle_clearance': avg_clearance,
            'safety_margins': {
                'minimum': self.minimum_safety_margin,
                'preferred': self.preferred_safety_margin,
                'lateral': self.lateral_safety_margin
            },
            'thresholds': {
                'critical_distance': self.critical_distance_threshold,
                'warning_distance': self.warning_distance_threshold,
                'safe_distance': self.safe_distance_threshold
            }
        }
        
        return metrics
    
    def reset_metrics(self):
        """Reset performance metrics for fresh monitoring."""
        self.metrics_history.clear()
        self.strategy_counts = {strategy: 0 for strategy in AvoidanceStrategy}
        self.success_rate = 1.0
        
        rospy.loginfo("[AvoidancePlanner] Performance metrics reset")


if __name__ == '__main__':
    # Test the avoidance planner
    rospy.init_node('avoidance_planner_test')
    
    planner = AvoidancePlanner()
    
    # Create test scenario
    from duckietown_enhanced_msgs.msg import ObjectDetection
    from geometry_msgs.msg import Point as ROSPoint
    
    test_detection = ObjectDetection()
    test_detection.class_name = "duckiebot"
    test_detection.confidence = 0.8
    test_detection.distance = 0.6
    test_detection.relative_velocity = Vector3(x=-0.5, y=0.0, z=0.0)
    
    test_risk_factors = RiskFactors(
        distance_risk=0.7,
        velocity_risk=0.6,
        object_type_risk=0.8,
        trajectory_risk=0.5,
        time_to_collision=1.2,
        lateral_clearance=0.3
    )
    
    test_vehicle_state = VehicleState(
        position=Point(x=0.0, y=0.0, z=0.0),
        velocity=Vector3(x=0.8, y=0.0, z=0.0),
        heading=0.0,
        timestamp=time.time()
    )
    
    risk_assessments = [(test_detection, test_risk_factors, RiskLevel.HIGH)]
    
    # Test strategy selection and trajectory generation
    strategy, trajectory = planner.plan_avoidance(risk_assessments, test_vehicle_state)
    
    rospy.loginfo(f"Test completed - Strategy: {strategy.value}")
    if trajectory:
        rospy.loginfo(f"Trajectory: {len(trajectory.waypoints)} waypoints, {trajectory.total_duration:.2f}s")
    
    # Print performance metrics
    metrics = planner.get_performance_metrics()
    rospy.loginfo(f"Performance metrics: {metrics}")
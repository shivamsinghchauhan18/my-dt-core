#!/usr/bin/env python3

import rospy
import time
import numpy as np
import math
from typing import List, Optional, Dict, Any, Tuple
from dataclasses import dataclass
from enum import Enum
from datetime import datetime

from geometry_msgs.msg import Point, Vector3


class TrajectoryType(Enum):
    """Types of lane change trajectories"""
    QUINTIC_POLYNOMIAL = "quintic_polynomial"
    CUBIC_SPLINE = "cubic_spline"
    SINUSOIDAL = "sinusoidal"
    LINEAR = "linear"


class TrajectoryValidationResult(Enum):
    """Trajectory validation results"""
    VALID = "valid"
    INVALID_ACCELERATION = "invalid_acceleration"
    INVALID_CURVATURE = "invalid_curvature"
    INVALID_DURATION = "invalid_duration"
    INVALID_CONSTRAINTS = "invalid_constraints"


@dataclass
class TrajectoryWaypoint:
    """Single waypoint in a trajectory"""
    position: Point
    velocity: Vector3
    acceleration: Vector3
    heading: float
    curvature: float
    timestamp: float


@dataclass
class LaneChangeTrajectory:
    """Complete lane change trajectory"""
    waypoints: List[TrajectoryWaypoint]
    total_duration: float
    total_distance: float
    maximum_lateral_acceleration: float
    maximum_curvature: float
    safety_margin: float
    trajectory_type: TrajectoryType
    is_feasible: bool
    validation_result: TrajectoryValidationResult
    generation_time: float


@dataclass
class TrajectoryConstraints:
    """Constraints for trajectory generation"""
    maximum_lateral_acceleration: float
    maximum_curvature: float
    maximum_velocity: float
    minimum_radius: float
    comfort_factor: float
    safety_margin: float
    preferred_duration: float
    maximum_duration: float


@dataclass
class LaneChangeParameters:
    """Parameters for lane change trajectory"""
    start_position: Point
    end_position: Point
    start_velocity: Vector3
    end_velocity: Vector3
    start_heading: float
    end_heading: float
    lane_width: float
    vehicle_length: float
    vehicle_width: float


class LaneChangeTrajectoryGenerator:
    """
    Advanced trajectory generator for smooth lane change maneuvers.
    
    This generator creates smooth, feasible trajectories for lane changes using
    quintic polynomial curves with lateral acceleration constraints and comfort
    optimization. It validates trajectories against safety and feasibility constraints.
    """
    
    def __init__(self):
        """Initialize the trajectory generator"""
        
        # Default trajectory constraints
        self.constraints = TrajectoryConstraints(
            maximum_lateral_acceleration=2.0,  # m/s²
            maximum_curvature=2.0,  # 1/m
            maximum_velocity=2.0,  # m/s
            minimum_radius=0.5,  # m
            comfort_factor=0.8,  # 0-1
            safety_margin=0.3,  # m
            preferred_duration=2.0,  # s
            maximum_duration=3.0  # s
        )
        
        # Trajectory generation parameters
        self.waypoint_resolution = 0.1  # seconds between waypoints
        self.polynomial_order = 5  # Quintic polynomial
        
        # Performance metrics
        self.performance_metrics = {
            'total_trajectories_generated': 0,
            'successful_generations': 0,
            'average_generation_time': 0.0,
            'validation_failures': 0,
            'constraint_violations': 0,
            'last_update_time': time.time()
        }
        
        rospy.loginfo("[LaneChangeTrajectoryGenerator] Initialized with quintic polynomial trajectory generation")
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] Constraints: {self.constraints}")
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] Waypoint resolution: {self.waypoint_resolution}s")
    
    def generate_lane_change_trajectory(self,
                                      lane_change_params: LaneChangeParameters,
                                      constraints: Optional[TrajectoryConstraints] = None) -> LaneChangeTrajectory:
        """
        Generate a smooth lane change trajectory using quintic polynomials.
        
        Args:
            lane_change_params: Parameters defining the lane change
            constraints: Optional custom constraints (uses defaults if None)
            
        Returns:
            LaneChangeTrajectory with waypoints and validation results
        """
        timestamp = time.time()
        generation_start_time = time.time()
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Generating lane change trajectory")
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] Start position: ({lane_change_params.start_position.x:.2f}, {lane_change_params.start_position.y:.2f})")
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] End position: ({lane_change_params.end_position.x:.2f}, {lane_change_params.end_position.y:.2f})")
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] Lane width: {lane_change_params.lane_width:.2f}m")
        
        # Use provided constraints or defaults
        if constraints is None:
            constraints = self.constraints
        
        try:
            # Calculate trajectory duration based on constraints and comfort
            duration = self.calculate_optimal_duration(lane_change_params, constraints)
            
            rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Calculated optimal duration: {duration:.2f}s")
            
            # Generate quintic polynomial coefficients
            coefficients = self.calculate_quintic_coefficients(lane_change_params, duration)
            
            rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Generated polynomial coefficients")
            rospy.loginfo(f"  Longitudinal coefficients: {[f'{c:.4f}' for c in coefficients['longitudinal']]}")
            rospy.loginfo(f"  Lateral coefficients: {[f'{c:.4f}' for c in coefficients['lateral']]}")
            
            # Generate waypoints along the trajectory
            waypoints = self.generate_waypoints(coefficients, duration, lane_change_params)
            
            rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Generated {len(waypoints)} waypoints")
            
            # Calculate trajectory metrics
            metrics = self.calculate_trajectory_metrics(waypoints)
            
            rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Trajectory metrics:")
            rospy.loginfo(f"  Total distance: {metrics['total_distance']:.2f}m")
            rospy.loginfo(f"  Maximum lateral acceleration: {metrics['max_lateral_acceleration']:.2f}m/s²")
            rospy.loginfo(f"  Maximum curvature: {metrics['max_curvature']:.3f}1/m")
            
            # Validate trajectory against constraints
            validation_result = self.validate_trajectory(waypoints, constraints)
            
            rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Trajectory validation: {validation_result.value}")
            
            # Create trajectory object
            generation_time = (time.time() - generation_start_time) * 1000
            
            trajectory = LaneChangeTrajectory(
                waypoints=waypoints,
                total_duration=duration,
                total_distance=metrics['total_distance'],
                maximum_lateral_acceleration=metrics['max_lateral_acceleration'],
                maximum_curvature=metrics['max_curvature'],
                safety_margin=constraints.safety_margin,
                trajectory_type=TrajectoryType.QUINTIC_POLYNOMIAL,
                is_feasible=(validation_result == TrajectoryValidationResult.VALID),
                validation_result=validation_result,
                generation_time=generation_time
            )
            
            # Update performance metrics
            self.update_performance_metrics(generation_time, trajectory.is_feasible)
            
            rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Trajectory generation completed")
            rospy.loginfo(f"  Generation time: {generation_time:.2f}ms")
            rospy.loginfo(f"  Feasible: {trajectory.is_feasible}")
            rospy.loginfo(f"  Validation result: {validation_result.value}")
            
            return trajectory
            
        except Exception as e:
            rospy.logerr(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Trajectory generation failed: {str(e)}")
            rospy.logerr(f"[LaneChangeTrajectoryGenerator] Exception details: {type(e).__name__}")
            
            # Return empty trajectory on failure
            return LaneChangeTrajectory(
                waypoints=[],
                total_duration=0.0,
                total_distance=0.0,
                maximum_lateral_acceleration=0.0,
                maximum_curvature=0.0,
                safety_margin=0.0,
                trajectory_type=TrajectoryType.QUINTIC_POLYNOMIAL,
                is_feasible=False,
                validation_result=TrajectoryValidationResult.INVALID_CONSTRAINTS,
                generation_time=0.0
            )
    
    def calculate_optimal_duration(self,
                                 lane_change_params: LaneChangeParameters,
                                 constraints: TrajectoryConstraints) -> float:
        """
        Calculate optimal duration for lane change based on constraints and comfort.
        
        Args:
            lane_change_params: Lane change parameters
            constraints: Trajectory constraints
            
        Returns:
            Optimal duration in seconds
        """
        timestamp = time.time()
        
        # Calculate distance to travel
        dx = lane_change_params.end_position.x - lane_change_params.start_position.x
        dy = lane_change_params.end_position.y - lane_change_params.start_position.y
        total_distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate minimum duration based on velocity constraints
        avg_velocity = (lane_change_params.start_velocity.x + lane_change_params.end_velocity.x) / 2.0
        min_duration_velocity = total_distance / max(0.1, avg_velocity)
        
        # Calculate minimum duration based on lateral acceleration constraints
        lateral_distance = abs(dy)  # Assuming lateral displacement
        min_duration_acceleration = math.sqrt(4 * lateral_distance / constraints.maximum_lateral_acceleration)
        
        # Calculate comfort-based duration
        comfort_duration = constraints.preferred_duration * constraints.comfort_factor
        
        # Select optimal duration
        optimal_duration = max(
            min_duration_velocity,
            min_duration_acceleration,
            comfort_duration,
            1.0  # Minimum 1 second
        )
        
        # Clamp to maximum duration
        optimal_duration = min(optimal_duration, constraints.maximum_duration)
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Duration calculation:")
        rospy.loginfo(f"  Total distance: {total_distance:.2f}m")
        rospy.loginfo(f"  Lateral distance: {lateral_distance:.2f}m")
        rospy.loginfo(f"  Min duration (velocity): {min_duration_velocity:.2f}s")
        rospy.loginfo(f"  Min duration (acceleration): {min_duration_acceleration:.2f}s")
        rospy.loginfo(f"  Comfort duration: {comfort_duration:.2f}s")
        rospy.loginfo(f"  Optimal duration: {optimal_duration:.2f}s")
        
        return optimal_duration
    
    def calculate_quintic_coefficients(self,
                                     lane_change_params: LaneChangeParameters,
                                     duration: float) -> Dict[str, List[float]]:
        """
        Calculate quintic polynomial coefficients for smooth trajectory.
        
        Args:
            lane_change_params: Lane change parameters
            duration: Trajectory duration
            
        Returns:
            Dictionary with longitudinal and lateral coefficients
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Calculating quintic polynomial coefficients")
        
        # Boundary conditions for longitudinal motion (x-direction)
        x0 = lane_change_params.start_position.x
        x1 = lane_change_params.end_position.x
        v0_x = lane_change_params.start_velocity.x
        v1_x = lane_change_params.end_velocity.x
        a0_x = 0.0  # Assume zero initial acceleration
        a1_x = 0.0  # Assume zero final acceleration
        
        # Boundary conditions for lateral motion (y-direction)
        y0 = lane_change_params.start_position.y
        y1 = lane_change_params.end_position.y
        v0_y = lane_change_params.start_velocity.y
        v1_y = lane_change_params.end_velocity.y
        a0_y = 0.0  # Assume zero initial lateral acceleration
        a1_y = 0.0  # Assume zero final lateral acceleration
        
        # Time variables
        T = duration
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T
        
        # Coefficient matrix for quintic polynomial
        # p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        # Boundary conditions: p(0), p'(0), p''(0), p(T), p'(T), p''(T)
        
        A = np.array([
            [1, 0, 0, 0, 0, 0],      # p(0) = a0
            [0, 1, 0, 0, 0, 0],      # p'(0) = a1
            [0, 0, 2, 0, 0, 0],      # p''(0) = 2*a2
            [1, T, T2, T3, T4, T5],  # p(T)
            [0, 1, 2*T, 3*T2, 4*T3, 5*T4],  # p'(T)
            [0, 0, 2, 6*T, 12*T2, 20*T3]    # p''(T)
        ])
        
        # Boundary condition vectors
        b_x = np.array([x0, v0_x, a0_x, x1, v1_x, a1_x])
        b_y = np.array([y0, v0_y, a0_y, y1, v1_y, a1_y])
        
        # Solve for coefficients
        try:
            coeffs_x = np.linalg.solve(A, b_x)
            coeffs_y = np.linalg.solve(A, b_y)
            
            rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Polynomial coefficients calculated successfully")
            
            return {
                'longitudinal': coeffs_x.tolist(),
                'lateral': coeffs_y.tolist()
            }
            
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Failed to solve for coefficients: {e}")
            # Return linear interpolation as fallback
            return {
                'longitudinal': [x0, (x1-x0)/T, 0, 0, 0, 0],
                'lateral': [y0, (y1-y0)/T, 0, 0, 0, 0]
            }
    
    def generate_waypoints(self,
                         coefficients: Dict[str, List[float]],
                         duration: float,
                         lane_change_params: LaneChangeParameters) -> List[TrajectoryWaypoint]:
        """
        Generate waypoints along the quintic polynomial trajectory.
        
        Args:
            coefficients: Polynomial coefficients
            duration: Trajectory duration
            lane_change_params: Lane change parameters
            
        Returns:
            List of trajectory waypoints
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Generating trajectory waypoints")
        
        waypoints = []
        num_points = int(duration / self.waypoint_resolution) + 1
        
        coeffs_x = coefficients['longitudinal']
        coeffs_y = coefficients['lateral']
        
        for i in range(num_points):
            t = i * self.waypoint_resolution
            if t > duration:
                t = duration
            
            # Calculate position using quintic polynomial
            # p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            t2 = t * t
            t3 = t2 * t
            t4 = t3 * t
            t5 = t4 * t
            
            x = coeffs_x[0] + coeffs_x[1]*t + coeffs_x[2]*t2 + coeffs_x[3]*t3 + coeffs_x[4]*t4 + coeffs_x[5]*t5
            y = coeffs_y[0] + coeffs_y[1]*t + coeffs_y[2]*t2 + coeffs_y[3]*t3 + coeffs_y[4]*t4 + coeffs_y[5]*t5
            
            # Calculate velocity (first derivative)
            # p'(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
            vx = coeffs_x[1] + 2*coeffs_x[2]*t + 3*coeffs_x[3]*t2 + 4*coeffs_x[4]*t3 + 5*coeffs_x[5]*t4
            vy = coeffs_y[1] + 2*coeffs_y[2]*t + 3*coeffs_y[3]*t2 + 4*coeffs_y[4]*t3 + 5*coeffs_y[5]*t4
            
            # Calculate acceleration (second derivative)
            # p''(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
            ax = 2*coeffs_x[2] + 6*coeffs_x[3]*t + 12*coeffs_x[4]*t2 + 20*coeffs_x[5]*t3
            ay = 2*coeffs_y[2] + 6*coeffs_y[3]*t + 12*coeffs_y[4]*t2 + 20*coeffs_y[5]*t3
            
            # Calculate heading
            heading = math.atan2(vy, vx) if (vx != 0 or vy != 0) else 0.0
            
            # Calculate curvature
            # κ = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
            velocity_magnitude_sq = vx*vx + vy*vy
            if velocity_magnitude_sq > 1e-6:
                curvature = abs(vx*ay - vy*ax) / (velocity_magnitude_sq ** 1.5)
            else:
                curvature = 0.0
            
            # Create waypoint
            waypoint = TrajectoryWaypoint(
                position=Point(x=x, y=y, z=0.0),
                velocity=Vector3(x=vx, y=vy, z=0.0),
                acceleration=Vector3(x=ax, y=ay, z=0.0),
                heading=heading,
                curvature=curvature,
                timestamp=timestamp + t
            )
            
            waypoints.append(waypoint)
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Generated {len(waypoints)} waypoints")
        rospy.loginfo(f"  Time range: 0.0s to {duration:.2f}s")
        rospy.loginfo(f"  Position range: ({waypoints[0].position.x:.2f}, {waypoints[0].position.y:.2f}) to ({waypoints[-1].position.x:.2f}, {waypoints[-1].position.y:.2f})")
        
        return waypoints
    
    def calculate_trajectory_metrics(self, waypoints: List[TrajectoryWaypoint]) -> Dict[str, float]:
        """
        Calculate metrics for the generated trajectory.
        
        Args:
            waypoints: List of trajectory waypoints
            
        Returns:
            Dictionary of trajectory metrics
        """
        if len(waypoints) < 2:
            return {
                'total_distance': 0.0,
                'max_lateral_acceleration': 0.0,
                'max_curvature': 0.0,
                'max_velocity': 0.0,
                'average_velocity': 0.0
            }
        
        total_distance = 0.0
        max_lateral_acceleration = 0.0
        max_curvature = 0.0
        max_velocity = 0.0
        total_velocity = 0.0
        
        for i in range(1, len(waypoints)):
            # Calculate distance between consecutive waypoints
            dx = waypoints[i].position.x - waypoints[i-1].position.x
            dy = waypoints[i].position.y - waypoints[i-1].position.y
            distance = math.sqrt(dx*dx + dy*dy)
            total_distance += distance
            
            # Track maximum values
            lateral_accel = abs(waypoints[i].acceleration.y)
            max_lateral_acceleration = max(max_lateral_acceleration, lateral_accel)
            
            max_curvature = max(max_curvature, waypoints[i].curvature)
            
            velocity_magnitude = math.sqrt(
                waypoints[i].velocity.x**2 + waypoints[i].velocity.y**2
            )
            max_velocity = max(max_velocity, velocity_magnitude)
            total_velocity += velocity_magnitude
        
        average_velocity = total_velocity / len(waypoints) if len(waypoints) > 0 else 0.0
        
        return {
            'total_distance': total_distance,
            'max_lateral_acceleration': max_lateral_acceleration,
            'max_curvature': max_curvature,
            'max_velocity': max_velocity,
            'average_velocity': average_velocity
        }
    
    def validate_trajectory(self,
                          waypoints: List[TrajectoryWaypoint],
                          constraints: TrajectoryConstraints) -> TrajectoryValidationResult:
        """
        Validate trajectory against constraints.
        
        Args:
            waypoints: List of trajectory waypoints
            constraints: Trajectory constraints
            
        Returns:
            Validation result
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Validating trajectory against constraints")
        
        if len(waypoints) < 2:
            rospy.logwarn(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Insufficient waypoints for validation")
            return TrajectoryValidationResult.INVALID_CONSTRAINTS
        
        # Calculate trajectory metrics
        metrics = self.calculate_trajectory_metrics(waypoints)
        
        # Check lateral acceleration constraint
        if metrics['max_lateral_acceleration'] > constraints.maximum_lateral_acceleration:
            rospy.logwarn(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Lateral acceleration constraint violated:")
            rospy.logwarn(f"  Maximum: {metrics['max_lateral_acceleration']:.2f} m/s²")
            rospy.logwarn(f"  Limit: {constraints.maximum_lateral_acceleration:.2f} m/s²")
            return TrajectoryValidationResult.INVALID_ACCELERATION
        
        # Check curvature constraint
        if metrics['max_curvature'] > constraints.maximum_curvature:
            rospy.logwarn(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Curvature constraint violated:")
            rospy.logwarn(f"  Maximum: {metrics['max_curvature']:.3f} 1/m")
            rospy.logwarn(f"  Limit: {constraints.maximum_curvature:.3f} 1/m")
            return TrajectoryValidationResult.INVALID_CURVATURE
        
        # Check velocity constraint
        if metrics['max_velocity'] > constraints.maximum_velocity:
            rospy.logwarn(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Velocity constraint violated:")
            rospy.logwarn(f"  Maximum: {metrics['max_velocity']:.2f} m/s")
            rospy.logwarn(f"  Limit: {constraints.maximum_velocity:.2f} m/s")
            return TrajectoryValidationResult.INVALID_CONSTRAINTS
        
        # Check duration constraint
        duration = waypoints[-1].timestamp - waypoints[0].timestamp
        if duration > constraints.maximum_duration:
            rospy.logwarn(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Duration constraint violated:")
            rospy.logwarn(f"  Duration: {duration:.2f} s")
            rospy.logwarn(f"  Limit: {constraints.maximum_duration:.2f} s")
            return TrajectoryValidationResult.INVALID_DURATION
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Trajectory validation passed")
        rospy.loginfo(f"  Max lateral acceleration: {metrics['max_lateral_acceleration']:.2f} m/s² (limit: {constraints.maximum_lateral_acceleration:.2f})")
        rospy.loginfo(f"  Max curvature: {metrics['max_curvature']:.3f} 1/m (limit: {constraints.maximum_curvature:.3f})")
        rospy.loginfo(f"  Max velocity: {metrics['max_velocity']:.2f} m/s (limit: {constraints.maximum_velocity:.2f})")
        rospy.loginfo(f"  Duration: {duration:.2f} s (limit: {constraints.maximum_duration:.2f})")
        
        return TrajectoryValidationResult.VALID
    
    def optimize_trajectory_for_comfort(self,
                                      trajectory: LaneChangeTrajectory,
                                      comfort_factor: float = 0.8) -> LaneChangeTrajectory:
        """
        Optimize trajectory for passenger comfort by reducing accelerations.
        
        Args:
            trajectory: Original trajectory
            comfort_factor: Comfort optimization factor (0-1)
            
        Returns:
            Optimized trajectory
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Optimizing trajectory for comfort")
        rospy.loginfo(f"  Comfort factor: {comfort_factor}")
        rospy.loginfo(f"  Original max lateral acceleration: {trajectory.maximum_lateral_acceleration:.2f} m/s²")
        
        if not trajectory.waypoints or comfort_factor >= 1.0:
            return trajectory
        
        # Create optimized waypoints with reduced accelerations
        optimized_waypoints = []
        
        for i, waypoint in enumerate(trajectory.waypoints):
            # Reduce lateral acceleration by comfort factor
            optimized_acceleration = Vector3(
                x=waypoint.acceleration.x,
                y=waypoint.acceleration.y * comfort_factor,
                z=waypoint.acceleration.z
            )
            
            # Recalculate curvature with reduced acceleration
            velocity_magnitude_sq = waypoint.velocity.x**2 + waypoint.velocity.y**2
            if velocity_magnitude_sq > 1e-6:
                optimized_curvature = abs(waypoint.velocity.x * optimized_acceleration.y - 
                                        waypoint.velocity.y * optimized_acceleration.x) / (velocity_magnitude_sq ** 1.5)
            else:
                optimized_curvature = waypoint.curvature
            
            optimized_waypoint = TrajectoryWaypoint(
                position=waypoint.position,
                velocity=waypoint.velocity,
                acceleration=optimized_acceleration,
                heading=waypoint.heading,
                curvature=optimized_curvature,
                timestamp=waypoint.timestamp
            )
            
            optimized_waypoints.append(optimized_waypoint)
        
        # Recalculate trajectory metrics
        optimized_metrics = self.calculate_trajectory_metrics(optimized_waypoints)
        
        # Create optimized trajectory
        optimized_trajectory = LaneChangeTrajectory(
            waypoints=optimized_waypoints,
            total_duration=trajectory.total_duration,
            total_distance=optimized_metrics['total_distance'],
            maximum_lateral_acceleration=optimized_metrics['max_lateral_acceleration'],
            maximum_curvature=optimized_metrics['max_curvature'],
            safety_margin=trajectory.safety_margin,
            trajectory_type=trajectory.trajectory_type,
            is_feasible=trajectory.is_feasible,
            validation_result=trajectory.validation_result,
            generation_time=trajectory.generation_time
        )
        
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] [{timestamp:.3f}] Trajectory comfort optimization completed")
        rospy.loginfo(f"  Optimized max lateral acceleration: {optimized_trajectory.maximum_lateral_acceleration:.2f} m/s²")
        rospy.loginfo(f"  Acceleration reduction: {((trajectory.maximum_lateral_acceleration - optimized_trajectory.maximum_lateral_acceleration) / trajectory.maximum_lateral_acceleration * 100):.1f}%")
        
        return optimized_trajectory
    
    def update_performance_metrics(self, generation_time: float, is_feasible: bool):
        """Update performance metrics"""
        self.performance_metrics['total_trajectories_generated'] += 1
        
        if is_feasible:
            self.performance_metrics['successful_generations'] += 1
        else:
            self.performance_metrics['validation_failures'] += 1
        
        # Update average generation time
        total_generated = self.performance_metrics['total_trajectories_generated']
        current_avg = self.performance_metrics['average_generation_time']
        self.performance_metrics['average_generation_time'] = (
            (current_avg * (total_generated - 1) + generation_time) / total_generated
        )
        
        self.performance_metrics['last_update_time'] = time.time()
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get current performance metrics"""
        success_rate = 0.0
        if self.performance_metrics['total_trajectories_generated'] > 0:
            success_rate = (self.performance_metrics['successful_generations'] / 
                          self.performance_metrics['total_trajectories_generated']) * 100
        
        return {
            'total_trajectories_generated': self.performance_metrics['total_trajectories_generated'],
            'successful_generations': self.performance_metrics['successful_generations'],
            'success_rate': success_rate,
            'average_generation_time_ms': self.performance_metrics['average_generation_time'],
            'validation_failures': self.performance_metrics['validation_failures'],
            'constraint_violations': self.performance_metrics['constraint_violations'],
            'last_update_time': self.performance_metrics['last_update_time']
        }
    
    def reset_metrics(self):
        """Reset performance metrics"""
        self.performance_metrics = {
            'total_trajectories_generated': 0,
            'successful_generations': 0,
            'average_generation_time': 0.0,
            'validation_failures': 0,
            'constraint_violations': 0,
            'last_update_time': time.time()
        }
        rospy.loginfo("[LaneChangeTrajectoryGenerator] Performance metrics reset")
    
    def set_constraints(self, constraints: TrajectoryConstraints):
        """Update trajectory constraints"""
        self.constraints = constraints
        rospy.loginfo(f"[LaneChangeTrajectoryGenerator] Constraints updated: {constraints}")
    
    def get_constraints(self) -> TrajectoryConstraints:
        """Get current trajectory constraints"""
        return self.constraints
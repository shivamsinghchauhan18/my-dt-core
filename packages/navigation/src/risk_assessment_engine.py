#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple
from enum import IntEnum

import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3
from duckietown_msgs.msg import ObjectDetection, ObjectDetectionArray
from image_processing.ground_projection_geometry import GroundProjectionGeometry, Point as GPPoint


class RiskLevel(IntEnum):
    """Risk level enumeration matching ObjectDetection message constants"""
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    CRITICAL = 3


@dataclass
class RiskFactors:
    """Container for individual risk factors used in assessment"""
    distance_risk: float
    velocity_risk: float
    object_type_risk: float
    trajectory_risk: float
    time_to_collision: float
    lateral_clearance: float


@dataclass
class VehicleState:
    """Current vehicle state for risk assessment"""
    position: Point
    velocity: Vector3
    heading: float
    timestamp: float


@dataclass
class TrajectoryPrediction:
    """Predicted trajectory information"""
    intersection_point: Optional[Point]
    time_to_intersection: float
    collision_probability: float
    minimum_distance: float


class RiskAssessmentEngine:
    """
    Advanced risk assessment engine for collision prediction and avoidance.
    
    This engine evaluates collision risk based on multiple factors including:
    - Object distance and relative velocity
    - Object type and associated risk weights
    - Trajectory intersection prediction
    - Time-to-collision analysis
    - Lateral clearance assessment
    
    The engine uses ground projection utilities to convert image coordinates
    to real-world coordinates for accurate distance and trajectory calculations.
    """
    
    def __init__(self, ground_projector: Optional[GroundProjectionGeometry] = None):
        """
        Initialize the risk assessment engine.
        
        Args:
            ground_projector: Ground projection geometry for coordinate transformations
        """
        self.ground_projector = ground_projector
        
        # Risk assessment parameters
        self.safe_distance_threshold = 0.5  # meters
        self.warning_distance_threshold = 1.0  # meters
        self.critical_distance_threshold = 0.3  # meters
        self.max_assessment_distance = 5.0  # meters
        
        # Time-to-collision thresholds
        self.critical_ttc_threshold = 1.0  # seconds
        self.warning_ttc_threshold = 3.0  # seconds
        
        # Object type risk weights
        self.object_risk_weights = {
            'duckiebot': 0.8,
            'duckie': 0.3,
            'cone': 0.6,
            'barrier': 0.9,
            'person': 1.0,
            'vehicle': 0.9,
            'unknown': 0.5
        }
        
        # Velocity risk parameters
        self.velocity_risk_threshold = 0.5  # m/s
        self.max_velocity_risk = 2.0  # m/s
        
        # Trajectory prediction parameters
        self.prediction_horizon = 5.0  # seconds
        self.trajectory_samples = 50
        
        # Safety margins
        self.lateral_safety_margin = 0.4  # meters
        self.longitudinal_safety_margin = 0.6  # meters
        
        rospy.loginfo("[RiskAssessmentEngine] Initialized with comprehensive risk assessment capabilities")
        rospy.loginfo(f"[RiskAssessmentEngine] Safe distance threshold: {self.safe_distance_threshold}m")
        rospy.loginfo(f"[RiskAssessmentEngine] Critical TTC threshold: {self.critical_ttc_threshold}s")
        rospy.loginfo(f"[RiskAssessmentEngine] Prediction horizon: {self.prediction_horizon}s")
    
    def assess_collision_risk(self, detections: List[ObjectDetection], 
                            vehicle_state: VehicleState) -> List[Tuple[ObjectDetection, RiskFactors, RiskLevel]]:
        """
        Assess collision risk for all detected objects.
        
        Args:
            detections: List of detected objects
            vehicle_state: Current vehicle state
            
        Returns:
            List of tuples containing (detection, risk_factors, risk_level)
        """
        timestamp = time.time()
        rospy.loginfo(f"[RiskAssessmentEngine] [{timestamp:.3f}] Starting risk assessment for {len(detections)} objects")
        
        risk_assessments = []
        
        for i, detection in enumerate(detections):
            start_time = time.time()
            
            # Calculate individual risk factors
            risk_factors = self._calculate_risk_factors(detection, vehicle_state)
            
            # Determine overall risk level
            risk_level = self._determine_risk_level(risk_factors)
            
            # Log detailed risk assessment
            processing_time = (time.time() - start_time) * 1000
            self._log_risk_assessment(detection, risk_factors, risk_level, processing_time, i)
            
            risk_assessments.append((detection, risk_factors, risk_level))
        
        total_time = (time.time() - timestamp) * 1000
        rospy.loginfo(f"[RiskAssessmentEngine] [{timestamp:.3f}] Completed risk assessment in {total_time:.2f}ms")
        
        return risk_assessments
    
    def _calculate_risk_factors(self, detection: ObjectDetection, 
                              vehicle_state: VehicleState) -> RiskFactors:
        """
        Calculate individual risk factors for a detected object.
        
        Args:
            detection: Object detection data
            vehicle_state: Current vehicle state
            
        Returns:
            RiskFactors object containing all calculated risk components
        """
        timestamp = time.time()
        
        # Distance-based risk
        distance = detection.distance
        distance_risk = self._calculate_distance_risk(distance)
        
        # Velocity-based risk
        relative_velocity = detection.relative_velocity
        velocity_magnitude = math.sqrt(
            relative_velocity.x**2 + relative_velocity.y**2 + relative_velocity.z**2
        )
        velocity_risk = self._calculate_velocity_risk(velocity_magnitude, distance)
        
        # Object type risk
        object_type_risk = self._calculate_object_type_risk(detection.class_name)
        
        # Trajectory intersection risk
        trajectory_prediction = self._predict_trajectory_intersection(detection, vehicle_state)
        trajectory_risk = trajectory_prediction.collision_probability
        
        # Time to collision
        ttc = self._calculate_time_to_collision(detection, vehicle_state)
        
        # Lateral clearance
        lateral_clearance = self._calculate_lateral_clearance(detection, vehicle_state)
        
        risk_factors = RiskFactors(
            distance_risk=distance_risk,
            velocity_risk=velocity_risk,
            object_type_risk=object_type_risk,
            trajectory_risk=trajectory_risk,
            time_to_collision=ttc,
            lateral_clearance=lateral_clearance
        )
        
        # Log detailed risk factor calculations
        rospy.logdebug(f"[RiskAssessmentEngine] [{timestamp:.3f}] Risk factors for {detection.class_name}:")
        rospy.logdebug(f"  Distance: {distance:.2f}m -> Risk: {distance_risk:.3f}")
        rospy.logdebug(f"  Velocity: {velocity_magnitude:.2f}m/s -> Risk: {velocity_risk:.3f}")
        rospy.logdebug(f"  Object type: {detection.class_name} -> Risk: {object_type_risk:.3f}")
        rospy.logdebug(f"  Trajectory risk: {trajectory_risk:.3f}")
        rospy.logdebug(f"  Time to collision: {ttc:.2f}s")
        rospy.logdebug(f"  Lateral clearance: {lateral_clearance:.2f}m")
        
        return risk_factors
    
    def _calculate_distance_risk(self, distance: float) -> float:
        """
        Calculate risk based on object distance.
        
        Args:
            distance: Distance to object in meters
            
        Returns:
            Risk value between 0.0 and 1.0
        """
        # Use a single exponential decay function for monotonic behavior
        # Risk = 1.0 at distance 0, decaying exponentially
        decay_rate = 2.0  # Controls how quickly risk decreases with distance
        risk = math.exp(-distance / decay_rate)
        
        # Apply additional scaling based on thresholds
        if distance <= self.critical_distance_threshold:
            risk = max(risk, 0.9)  # Ensure high risk for critical distances
        elif distance <= self.safe_distance_threshold:
            risk = max(risk, 0.3)  # Moderate risk for intermediate distances
        
        return max(0.0, min(1.0, risk))
    
    def _calculate_velocity_risk(self, velocity_magnitude: float, distance: float) -> float:
        """
        Calculate risk based on relative velocity and distance.
        
        Args:
            velocity_magnitude: Magnitude of relative velocity
            distance: Distance to object
            
        Returns:
            Risk value between 0.0 and 1.0
        """
        if velocity_magnitude < 0.1:  # Nearly stationary
            return 0.1
        
        # Higher velocity increases risk, but effect diminishes with distance
        velocity_factor = min(1.0, velocity_magnitude / self.max_velocity_risk)
        distance_factor = max(0.1, 1.0 / (1.0 + distance))
        
        risk = velocity_factor * distance_factor
        
        # Additional risk if object is approaching
        if velocity_magnitude > self.velocity_risk_threshold:
            risk *= 1.5
        
        return max(0.0, min(1.0, risk))
    
    def _calculate_object_type_risk(self, object_class: str) -> float:
        """
        Calculate risk based on object type.
        
        Args:
            object_class: Class name of detected object
            
        Returns:
            Risk weight for the object type
        """
        return self.object_risk_weights.get(object_class.lower(), 0.5)
    
    def _predict_trajectory_intersection(self, detection: ObjectDetection, 
                                       vehicle_state: VehicleState) -> TrajectoryPrediction:
        """
        Predict trajectory intersection and collision probability.
        
        Args:
            detection: Object detection data
            vehicle_state: Current vehicle state
            
        Returns:
            TrajectoryPrediction with intersection analysis
        """
        timestamp = time.time()
        
        # Convert detection position to ground coordinates if ground projector available
        if self.ground_projector:
            # Use bounding box center for position estimation
            center_x = (detection.top_left.x + detection.bottom_right.x) / 2
            center_y = (detection.top_left.y + detection.bottom_right.y) / 2
            
            # Convert to ground coordinates (simplified - would need proper implementation)
            object_position = Point()
            object_position.x = detection.distance * math.cos(math.atan2(center_x - 320, 480))
            object_position.y = detection.distance * math.sin(math.atan2(center_x - 320, 480))
            object_position.z = 0.0
        else:
            # Fallback to simple distance-based positioning
            object_position = Point()
            object_position.x = detection.distance
            object_position.y = 0.0
            object_position.z = 0.0
        
        # Predict future positions
        dt = 0.1  # Time step for prediction
        collision_probability = 0.0
        intersection_point = None
        time_to_intersection = float('inf')
        minimum_distance = float('inf')
        
        for t in np.arange(0, self.prediction_horizon, dt):
            # Predict vehicle position
            vehicle_future_x = vehicle_state.position.x + vehicle_state.velocity.x * t
            vehicle_future_y = vehicle_state.position.y + vehicle_state.velocity.y * t
            
            # Predict object position
            object_future_x = object_position.x + detection.relative_velocity.x * t
            object_future_y = object_position.y + detection.relative_velocity.y * t
            
            # Calculate distance between predicted positions
            distance = math.sqrt(
                (vehicle_future_x - object_future_x)**2 + 
                (vehicle_future_y - object_future_y)**2
            )
            
            if distance < minimum_distance:
                minimum_distance = distance
            
            # Check for potential collision
            if distance < self.lateral_safety_margin:
                if intersection_point is None:
                    intersection_point = Point()
                    intersection_point.x = (vehicle_future_x + object_future_x) / 2
                    intersection_point.y = (vehicle_future_y + object_future_y) / 2
                    intersection_point.z = 0.0
                    time_to_intersection = t
                
                # Increase collision probability based on proximity
                collision_probability = max(collision_probability, 
                                          1.0 - (distance / self.lateral_safety_margin))
        
        prediction = TrajectoryPrediction(
            intersection_point=intersection_point,
            time_to_intersection=time_to_intersection,
            collision_probability=collision_probability,
            minimum_distance=minimum_distance
        )
        
        rospy.logdebug(f"[RiskAssessmentEngine] [{timestamp:.3f}] Trajectory prediction:")
        rospy.logdebug(f"  Intersection point: {intersection_point}")
        rospy.logdebug(f"  Time to intersection: {time_to_intersection:.2f}s")
        rospy.logdebug(f"  Collision probability: {collision_probability:.3f}")
        rospy.logdebug(f"  Minimum distance: {minimum_distance:.2f}m")
        
        return prediction
    
    def _calculate_time_to_collision(self, detection: ObjectDetection, 
                                   vehicle_state: VehicleState) -> float:
        """
        Calculate time to collision based on current trajectories.
        
        Args:
            detection: Object detection data
            vehicle_state: Current vehicle state
            
        Returns:
            Time to collision in seconds (inf if no collision predicted)
        """
        # Relative velocity in the direction of the object
        relative_velocity_magnitude = math.sqrt(
            detection.relative_velocity.x**2 + 
            detection.relative_velocity.y**2
        )
        
        if relative_velocity_magnitude < 0.01:  # Nearly zero relative velocity
            return float('inf')
        
        # Simple TTC calculation (assuming head-on approach)
        ttc = detection.distance / relative_velocity_magnitude
        
        # Apply safety factor for uncertainty
        ttc *= 0.8
        
        return max(0.0, ttc)
    
    def _calculate_lateral_clearance(self, detection: ObjectDetection, 
                                   vehicle_state: VehicleState) -> float:
        """
        Calculate lateral clearance to the object.
        
        Args:
            detection: Object detection data
            vehicle_state: Current vehicle state
            
        Returns:
            Lateral clearance in meters
        """
        # Simplified lateral clearance calculation
        # In a full implementation, this would use proper coordinate transformations
        
        # Estimate lateral offset based on bounding box position
        image_center_x = 320  # Assuming 640px width
        bbox_center_x = (detection.top_left.x + detection.bottom_right.x) / 2
        
        # Convert pixel offset to lateral distance (simplified)
        lateral_offset = (bbox_center_x - image_center_x) / image_center_x
        lateral_distance = abs(lateral_offset * detection.distance * 0.5)  # Rough approximation
        
        return lateral_distance
    
    def _determine_risk_level(self, risk_factors: RiskFactors) -> RiskLevel:
        """
        Determine overall risk level based on individual risk factors.
        
        Args:
            risk_factors: Calculated risk factors
            
        Returns:
            Overall risk level
        """
        # Weighted combination of risk factors
        weights = {
            'distance': 0.3,
            'velocity': 0.2,
            'object_type': 0.15,
            'trajectory': 0.25,
            'ttc': 0.1
        }
        
        # Calculate weighted risk score
        risk_score = (
            weights['distance'] * risk_factors.distance_risk +
            weights['velocity'] * risk_factors.velocity_risk +
            weights['object_type'] * risk_factors.object_type_risk +
            weights['trajectory'] * risk_factors.trajectory_risk
        )
        
        # Time-to-collision override
        if risk_factors.time_to_collision < self.critical_ttc_threshold:
            risk_score = max(risk_score, 0.9)
        elif risk_factors.time_to_collision < self.warning_ttc_threshold:
            risk_score = max(risk_score, 0.5)
        
        # Lateral clearance consideration
        if risk_factors.lateral_clearance < self.lateral_safety_margin:
            risk_score *= 1.2  # Reduced multiplier for more balanced scoring
        
        # Determine risk level based on score with adjusted thresholds
        if risk_score >= 0.75:
            return RiskLevel.CRITICAL
        elif risk_score >= 0.55:
            return RiskLevel.HIGH
        elif risk_score >= 0.3:
            return RiskLevel.MEDIUM
        else:
            return RiskLevel.LOW
    
    def _log_risk_assessment(self, detection: ObjectDetection, risk_factors: RiskFactors, 
                           risk_level: RiskLevel, processing_time: float, object_index: int):
        """
        Log comprehensive risk assessment results.
        
        Args:
            detection: Object detection data
            risk_factors: Calculated risk factors
            risk_level: Determined risk level
            processing_time: Time taken for assessment
            object_index: Index of object in detection array
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[RiskAssessmentEngine] [{timestamp:.3f}] Object {object_index} Risk Assessment:")
        rospy.loginfo(f"  Class: {detection.class_name} | Confidence: {detection.confidence:.2f}")
        rospy.loginfo(f"  Distance: {detection.distance:.2f}m | Risk Level: {risk_level.name}")
        rospy.loginfo(f"  Risk Factors:")
        rospy.loginfo(f"    Distance Risk: {risk_factors.distance_risk:.3f}")
        rospy.loginfo(f"    Velocity Risk: {risk_factors.velocity_risk:.3f}")
        rospy.loginfo(f"    Object Type Risk: {risk_factors.object_type_risk:.3f}")
        rospy.loginfo(f"    Trajectory Risk: {risk_factors.trajectory_risk:.3f}")
        rospy.loginfo(f"  Safety Metrics:")
        rospy.loginfo(f"    Time to Collision: {risk_factors.time_to_collision:.2f}s")
        rospy.loginfo(f"    Lateral Clearance: {risk_factors.lateral_clearance:.2f}m")
        rospy.loginfo(f"  Processing Time: {processing_time:.2f}ms")
        
        # Log decision thresholds for transparency
        if risk_level == RiskLevel.CRITICAL:
            rospy.logwarn(f"[RiskAssessmentEngine] CRITICAL RISK detected for {detection.class_name}!")
            rospy.logwarn(f"  Immediate action required - Emergency stop may be triggered")
        elif risk_level == RiskLevel.HIGH:
            rospy.logwarn(f"[RiskAssessmentEngine] HIGH RISK detected for {detection.class_name}")
            rospy.logwarn(f"  Avoidance maneuver recommended")
    
    def get_risk_statistics(self, risk_assessments: List[Tuple[ObjectDetection, RiskFactors, RiskLevel]]) -> dict:
        """
        Generate risk assessment statistics for monitoring.
        
        Args:
            risk_assessments: List of risk assessment results
            
        Returns:
            Dictionary containing risk statistics
        """
        if not risk_assessments:
            return {
                'total_objects': 0,
                'risk_distribution': {'LOW': 0, 'MEDIUM': 0, 'HIGH': 0, 'CRITICAL': 0},
                'average_distance': 0.0,
                'minimum_ttc': float('inf'),
                'critical_objects': []
            }
        
        risk_counts = {'LOW': 0, 'MEDIUM': 0, 'HIGH': 0, 'CRITICAL': 0}
        distances = []
        ttcs = []
        critical_objects = []
        
        for detection, risk_factors, risk_level in risk_assessments:
            risk_counts[risk_level.name] += 1
            distances.append(detection.distance)
            ttcs.append(risk_factors.time_to_collision)
            
            if risk_level == RiskLevel.CRITICAL:
                critical_objects.append({
                    'class': detection.class_name,
                    'distance': detection.distance,
                    'ttc': risk_factors.time_to_collision
                })
        
        return {
            'total_objects': len(risk_assessments),
            'risk_distribution': risk_counts,
            'average_distance': np.mean(distances) if distances else 0.0,
            'minimum_ttc': min(ttcs) if ttcs else float('inf'),
            'critical_objects': critical_objects
        }
#!/usr/bin/env python3

import rospy
import time
import numpy as np
from typing import List, Optional, Dict, Any, Tuple
from dataclasses import dataclass
from enum import Enum
from datetime import datetime

from geometry_msgs.msg import Point, Vector3
from duckietown_enhanced_msgs.msg import ObjectDetection


class LaneChangeDecision(Enum):
    """Lane change decision types"""
    NO_CHANGE = "no_change"
    CHANGE_LEFT = "change_left"
    CHANGE_RIGHT = "change_right"
    ABORT_CHANGE = "abort_change"


class LaneChangeReason(Enum):
    """Reasons for lane change decisions"""
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    PATH_OPTIMIZATION = "path_optimization"
    TRAFFIC_FLOW = "traffic_flow"
    EMERGENCY_MANEUVER = "emergency_maneuver"
    NONE = "none"


@dataclass
class LaneState:
    """Represents the state of a lane"""
    lane_id: str
    is_current: bool
    is_available: bool
    width: float
    center_line_offset: float
    obstacles: List[ObjectDetection]
    gap_length: float
    gap_start_distance: float
    safety_score: float
    timestamp: float


@dataclass
class GapAnalysis:
    """Results of gap analysis for lane change"""
    gap_available: bool
    gap_length: float
    gap_start_distance: float
    gap_end_distance: float
    leading_vehicle_distance: float
    following_vehicle_distance: float
    safety_margin_front: float
    safety_margin_rear: float
    confidence: float


@dataclass
class DecisionCriteria:
    """Criteria used for lane change decision making"""
    obstacle_clearance_required: float
    minimum_gap_length: float
    minimum_safety_margin: float
    maximum_lateral_acceleration: float
    decision_confidence_threshold: float
    emergency_threshold: float
    comfort_factor: float


@dataclass
class DecisionMatrix:
    """Decision matrix with weighted criteria"""
    safety_weight: float
    efficiency_weight: float
    comfort_weight: float
    urgency_weight: float
    feasibility_weight: float


class LaneChangeDecisionEngine:
    """
    Advanced lane change decision engine with multi-criteria evaluation.
    
    This engine evaluates lane change opportunities using multiple criteria including
    safety, efficiency, comfort, and feasibility. It performs gap analysis for
    adjacent lanes and uses a weighted decision matrix to determine the best action.
    """
    
    def __init__(self):
        """Initialize the lane change decision engine"""
        
        # Configuration parameters
        self.criteria = DecisionCriteria(
            obstacle_clearance_required=1.5,  # meters
            minimum_gap_length=4.0,  # meters
            minimum_safety_margin=1.0,  # meters
            maximum_lateral_acceleration=2.0,  # m/s²
            decision_confidence_threshold=0.7,  # 0-1
            emergency_threshold=0.9,  # 0-1
            comfort_factor=0.8  # 0-1
        )
        
        # Decision matrix weights (should sum to 1.0)
        self.decision_matrix = DecisionMatrix(
            safety_weight=0.4,
            efficiency_weight=0.2,
            comfort_weight=0.15,
            urgency_weight=0.15,
            feasibility_weight=0.1
        )
        
        # State tracking
        self.current_lane_state: Optional[LaneState] = None
        self.adjacent_lanes: Dict[str, LaneState] = {}
        self.last_decision_time = 0.0
        self.decision_history: List[Tuple[float, LaneChangeDecision, float]] = []
        
        # Performance metrics
        self.performance_metrics = {
            'total_evaluations': 0,
            'successful_decisions': 0,
            'average_evaluation_time': 0.0,
            'gap_analyses_performed': 0,
            'safety_violations_detected': 0,
            'emergency_decisions': 0,
            'last_update_time': time.time()
        }
        
        rospy.loginfo("[LaneChangeDecisionEngine] Initialized with multi-criteria evaluation")
        rospy.loginfo(f"[LaneChangeDecisionEngine] Decision criteria: {self.criteria}")
        rospy.loginfo(f"[LaneChangeDecisionEngine] Decision matrix weights: {self.decision_matrix}")
    
    def evaluate_lane_change_opportunity(self, 
                                       current_lane: LaneState,
                                       adjacent_lanes: Dict[str, LaneState],
                                       vehicle_state: Dict[str, Any],
                                       obstacles: List[ObjectDetection]) -> Tuple[LaneChangeDecision, LaneChangeReason, float]:
        """
        Evaluate lane change opportunities using multi-criteria analysis.
        
        Args:
            current_lane: Current lane state
            adjacent_lanes: Dictionary of adjacent lane states
            vehicle_state: Current vehicle state information
            obstacles: List of detected obstacles
            
        Returns:
            Tuple of (decision, reason, confidence_score)
        """
        timestamp = time.time()
        evaluation_start_time = time.time()
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Starting lane change evaluation")
        rospy.loginfo(f"[LaneChangeDecisionEngine] Current lane: {current_lane.lane_id}")
        rospy.loginfo(f"[LaneChangeDecisionEngine] Adjacent lanes available: {list(adjacent_lanes.keys())}")
        rospy.loginfo(f"[LaneChangeDecisionEngine] Total obstacles: {len(obstacles)}")
        
        try:
            # Update state
            self.current_lane_state = current_lane
            self.adjacent_lanes = adjacent_lanes
            
            # Perform comprehensive evaluation
            decision_scores = {}
            gap_analyses = {}
            
            # Evaluate each potential lane change option
            for lane_id, lane_state in adjacent_lanes.items():
                rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Evaluating lane change to {lane_id}")
                
                # Perform gap analysis
                gap_analysis = self.perform_gap_analysis(lane_state, vehicle_state, obstacles)
                gap_analyses[lane_id] = gap_analysis
                
                rospy.loginfo(f"[LaneChangeDecisionEngine] Gap analysis for {lane_id}:")
                rospy.loginfo(f"  Gap available: {gap_analysis.gap_available}")
                rospy.loginfo(f"  Gap length: {gap_analysis.gap_length:.2f}m")
                rospy.loginfo(f"  Safety margins: front={gap_analysis.safety_margin_front:.2f}m, rear={gap_analysis.safety_margin_rear:.2f}m")
                rospy.loginfo(f"  Confidence: {gap_analysis.confidence:.2f}")
                
                # Calculate decision score using multi-criteria evaluation
                decision_score = self.calculate_decision_score(
                    current_lane, lane_state, gap_analysis, vehicle_state, obstacles
                )
                decision_scores[lane_id] = decision_score
                
                rospy.loginfo(f"[LaneChangeDecisionEngine] Decision score for {lane_id}: {decision_score:.3f}")
            
            # Determine best decision based on scores
            decision, reason, confidence = self.select_best_decision(
                decision_scores, gap_analyses, current_lane, vehicle_state
            )
            
            # Update performance metrics
            evaluation_time = (time.time() - evaluation_start_time) * 1000
            self.update_performance_metrics(evaluation_time, decision, confidence)
            
            # Log decision results
            rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Decision Results:")
            rospy.loginfo(f"  Decision: {decision.value}")
            rospy.loginfo(f"  Reason: {reason.value}")
            rospy.loginfo(f"  Confidence: {confidence:.3f}")
            rospy.loginfo(f"  Evaluation time: {evaluation_time:.2f}ms")
            
            # Store decision in history
            self.decision_history.append((timestamp, decision, confidence))
            self.last_decision_time = timestamp
            
            return decision, reason, confidence
            
        except Exception as e:
            rospy.logerr(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Evaluation failed: {str(e)}")
            rospy.logerr(f"[LaneChangeDecisionEngine] Exception details: {type(e).__name__}")
            return LaneChangeDecision.NO_CHANGE, LaneChangeReason.NONE, 0.0
    
    def perform_gap_analysis(self, 
                           target_lane: LaneState, 
                           vehicle_state: Dict[str, Any],
                           obstacles: List[ObjectDetection]) -> GapAnalysis:
        """
        Perform detailed gap analysis for a target lane.
        
        Args:
            target_lane: Target lane to analyze
            vehicle_state: Current vehicle state
            obstacles: List of obstacles
            
        Returns:
            GapAnalysis results
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Performing gap analysis for lane {target_lane.lane_id}")
        
        # Filter obstacles in target lane
        lane_obstacles = [obs for obs in obstacles if self.is_obstacle_in_lane(obs, target_lane)]
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] Found {len(lane_obstacles)} obstacles in target lane")
        
        # Find leading and following vehicles
        vehicle_position = vehicle_state.get('position', {'x': 0.0, 'y': 0.0})
        vehicle_velocity = vehicle_state.get('velocity', {'x': 0.0, 'y': 0.0})
        
        leading_vehicle = None
        following_vehicle = None
        leading_distance = float('inf')
        following_distance = float('inf')
        
        for obstacle in lane_obstacles:
            # Calculate relative position (simplified - assumes obstacles are in front/behind)
            relative_distance = obstacle.distance
            
            # Determine if obstacle is leading or following
            if relative_distance > 0:  # Obstacle ahead
                if relative_distance < leading_distance:
                    leading_distance = relative_distance
                    leading_vehicle = obstacle
            else:  # Obstacle behind (would need more sophisticated detection)
                abs_distance = abs(relative_distance)
                if abs_distance < following_distance:
                    following_distance = abs_distance
                    following_vehicle = obstacle
        
        # Calculate gap metrics
        gap_start_distance = following_distance if following_vehicle else 0.0
        gap_end_distance = leading_distance if leading_vehicle else 100.0  # Assume clear ahead
        gap_length = gap_end_distance - gap_start_distance
        
        # Calculate safety margins
        required_deceleration_distance = self.calculate_deceleration_distance(vehicle_velocity.get('x', 0.0))
        safety_margin_front = max(0.0, leading_distance - required_deceleration_distance - self.criteria.minimum_safety_margin)
        safety_margin_rear = max(0.0, following_distance - self.criteria.minimum_safety_margin)
        
        # Determine if gap is available
        gap_available = (
            gap_length >= self.criteria.minimum_gap_length and
            safety_margin_front >= self.criteria.minimum_safety_margin and
            safety_margin_rear >= self.criteria.minimum_safety_margin
        )
        
        # Calculate confidence based on gap quality
        confidence = self.calculate_gap_confidence(gap_length, safety_margin_front, safety_margin_rear)
        
        # Update performance metrics
        self.performance_metrics['gap_analyses_performed'] += 1
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Gap analysis results:")
        rospy.loginfo(f"  Gap length: {gap_length:.2f}m (required: {self.criteria.minimum_gap_length:.2f}m)")
        rospy.loginfo(f"  Leading vehicle distance: {leading_distance:.2f}m")
        rospy.loginfo(f"  Following vehicle distance: {following_distance:.2f}m")
        rospy.loginfo(f"  Safety margins: front={safety_margin_front:.2f}m, rear={safety_margin_rear:.2f}m")
        rospy.loginfo(f"  Gap available: {gap_available}")
        rospy.loginfo(f"  Confidence: {confidence:.3f}")
        
        return GapAnalysis(
            gap_available=gap_available,
            gap_length=gap_length,
            gap_start_distance=gap_start_distance,
            gap_end_distance=gap_end_distance,
            leading_vehicle_distance=leading_distance,
            following_vehicle_distance=following_distance,
            safety_margin_front=safety_margin_front,
            safety_margin_rear=safety_margin_rear,
            confidence=confidence
        )
    
    def calculate_decision_score(self,
                               current_lane: LaneState,
                               target_lane: LaneState,
                               gap_analysis: GapAnalysis,
                               vehicle_state: Dict[str, Any],
                               obstacles: List[ObjectDetection]) -> float:
        """
        Calculate decision score using weighted multi-criteria evaluation.
        
        Args:
            current_lane: Current lane state
            target_lane: Target lane state
            gap_analysis: Gap analysis results
            vehicle_state: Vehicle state information
            obstacles: List of obstacles
            
        Returns:
            Decision score (0.0 to 1.0)
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Calculating decision score for lane {target_lane.lane_id}")
        
        # Safety score (0.0 to 1.0)
        safety_score = self.calculate_safety_score(gap_analysis, current_lane, target_lane, obstacles)
        
        # Efficiency score (0.0 to 1.0)
        efficiency_score = self.calculate_efficiency_score(current_lane, target_lane, vehicle_state)
        
        # Comfort score (0.0 to 1.0)
        comfort_score = self.calculate_comfort_score(gap_analysis, vehicle_state)
        
        # Urgency score (0.0 to 1.0)
        urgency_score = self.calculate_urgency_score(current_lane, obstacles)
        
        # Feasibility score (0.0 to 1.0)
        feasibility_score = self.calculate_feasibility_score(gap_analysis, vehicle_state)
        
        # Calculate weighted total score
        total_score = (
            self.decision_matrix.safety_weight * safety_score +
            self.decision_matrix.efficiency_weight * efficiency_score +
            self.decision_matrix.comfort_weight * comfort_score +
            self.decision_matrix.urgency_weight * urgency_score +
            self.decision_matrix.feasibility_weight * feasibility_score
        )
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Decision score breakdown:")
        rospy.loginfo(f"  Safety: {safety_score:.3f} (weight: {self.decision_matrix.safety_weight})")
        rospy.loginfo(f"  Efficiency: {efficiency_score:.3f} (weight: {self.decision_matrix.efficiency_weight})")
        rospy.loginfo(f"  Comfort: {comfort_score:.3f} (weight: {self.decision_matrix.comfort_weight})")
        rospy.loginfo(f"  Urgency: {urgency_score:.3f} (weight: {self.decision_matrix.urgency_weight})")
        rospy.loginfo(f"  Feasibility: {feasibility_score:.3f} (weight: {self.decision_matrix.feasibility_weight})")
        rospy.loginfo(f"  Total score: {total_score:.3f}")
        
        return total_score
    
    def calculate_safety_score(self, 
                             gap_analysis: GapAnalysis, 
                             current_lane: LaneState,
                             target_lane: LaneState,
                             obstacles: List[ObjectDetection]) -> float:
        """Calculate safety score for lane change decision"""
        
        # Base safety from gap analysis
        gap_safety = gap_analysis.confidence if gap_analysis.gap_available else 0.0
        
        # Safety margin factor
        margin_factor = min(1.0, (gap_analysis.safety_margin_front + gap_analysis.safety_margin_rear) / 
                           (2 * self.criteria.minimum_safety_margin))
        
        # Obstacle density factor
        current_obstacles = len([obs for obs in obstacles if self.is_obstacle_in_lane(obs, current_lane)])
        target_obstacles = len([obs for obs in obstacles if self.is_obstacle_in_lane(obs, target_lane)])
        obstacle_factor = max(0.0, 1.0 - (target_obstacles - current_obstacles) * 0.2)
        
        safety_score = gap_safety * margin_factor * obstacle_factor
        
        return max(0.0, min(1.0, safety_score))
    
    def calculate_efficiency_score(self, 
                                 current_lane: LaneState,
                                 target_lane: LaneState,
                                 vehicle_state: Dict[str, Any]) -> float:
        """Calculate efficiency score for lane change decision"""
        
        # Lane speed potential (simplified)
        current_speed_potential = 1.0 - len(current_lane.obstacles) * 0.1
        target_speed_potential = 1.0 - len(target_lane.obstacles) * 0.1
        
        # Path optimization factor
        path_improvement = max(0.0, target_speed_potential - current_speed_potential)
        
        efficiency_score = path_improvement
        
        return max(0.0, min(1.0, efficiency_score))
    
    def calculate_comfort_score(self, 
                              gap_analysis: GapAnalysis,
                              vehicle_state: Dict[str, Any]) -> float:
        """Calculate comfort score for lane change decision"""
        
        # Smooth maneuver potential
        gap_comfort = min(1.0, gap_analysis.gap_length / (2 * self.criteria.minimum_gap_length))
        
        # Acceleration comfort
        vehicle_speed = vehicle_state.get('velocity', {}).get('x', 0.0)
        required_lateral_accel = self.estimate_lateral_acceleration(vehicle_speed, gap_analysis.gap_length)
        accel_comfort = max(0.0, 1.0 - required_lateral_accel / self.criteria.maximum_lateral_acceleration)
        
        comfort_score = gap_comfort * accel_comfort * self.criteria.comfort_factor
        
        return max(0.0, min(1.0, comfort_score))
    
    def calculate_urgency_score(self, 
                              current_lane: LaneState,
                              obstacles: List[ObjectDetection]) -> float:
        """Calculate urgency score for lane change decision"""
        
        # Check for blocking obstacles in current lane
        blocking_obstacles = [obs for obs in obstacles 
                            if self.is_obstacle_in_lane(obs, current_lane) and obs.distance < 3.0]
        
        if len(blocking_obstacles) > 0:
            # High urgency if obstacles are close
            min_distance = min(obs.distance for obs in blocking_obstacles)
            urgency_score = max(0.0, 1.0 - min_distance / 3.0)
        else:
            urgency_score = 0.0
        
        return max(0.0, min(1.0, urgency_score))
    
    def calculate_feasibility_score(self, 
                                  gap_analysis: GapAnalysis,
                                  vehicle_state: Dict[str, Any]) -> float:
        """Calculate feasibility score for lane change decision"""
        
        # Basic feasibility from gap analysis
        if not gap_analysis.gap_available:
            return 0.0
        
        # Time feasibility
        vehicle_speed = vehicle_state.get('velocity', {}).get('x', 0.0)
        maneuver_time = self.estimate_maneuver_time(vehicle_speed, gap_analysis.gap_length)
        time_feasibility = min(1.0, 3.0 / max(0.1, maneuver_time))  # Prefer maneuvers under 3 seconds
        
        # Confidence feasibility
        confidence_feasibility = gap_analysis.confidence
        
        feasibility_score = time_feasibility * confidence_feasibility
        
        return max(0.0, min(1.0, feasibility_score))
    
    def select_best_decision(self,
                           decision_scores: Dict[str, float],
                           gap_analyses: Dict[str, GapAnalysis],
                           current_lane: LaneState,
                           vehicle_state: Dict[str, Any]) -> Tuple[LaneChangeDecision, LaneChangeReason, float]:
        """
        Select the best lane change decision based on scores and thresholds.
        
        Args:
            decision_scores: Dictionary of lane IDs to decision scores
            gap_analyses: Dictionary of lane IDs to gap analyses
            current_lane: Current lane state
            vehicle_state: Vehicle state information
            
        Returns:
            Tuple of (decision, reason, confidence)
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Selecting best decision from {len(decision_scores)} options")
        
        # Find the highest scoring option
        if not decision_scores:
            rospy.loginfo(f"[LaneChangeDecisionEngine] No lane change options available")
            return LaneChangeDecision.NO_CHANGE, LaneChangeReason.NONE, 1.0
        
        best_lane = max(decision_scores.keys(), key=lambda k: decision_scores[k])
        best_score = decision_scores[best_lane]
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] Best option: {best_lane} with score {best_score:.3f}")
        rospy.loginfo(f"[LaneChangeDecisionEngine] Decision threshold: {self.criteria.decision_confidence_threshold}")
        
        # Check if score meets threshold
        if best_score < self.criteria.decision_confidence_threshold:
            rospy.loginfo(f"[LaneChangeDecisionEngine] Score below threshold, no lane change")
            return LaneChangeDecision.NO_CHANGE, LaneChangeReason.NONE, best_score
        
        # Determine decision type and reason
        if "left" in best_lane.lower():
            decision = LaneChangeDecision.CHANGE_LEFT
        elif "right" in best_lane.lower():
            decision = LaneChangeDecision.CHANGE_RIGHT
        else:
            decision = LaneChangeDecision.NO_CHANGE
        
        # Determine reason based on urgency and efficiency
        urgency_score = self.calculate_urgency_score(current_lane, [])
        if urgency_score > 0.7:
            reason = LaneChangeReason.OBSTACLE_AVOIDANCE
        elif best_score > 0.8:
            reason = LaneChangeReason.PATH_OPTIMIZATION
        else:
            reason = LaneChangeReason.TRAFFIC_FLOW
        
        # Check for emergency conditions
        if best_score > self.criteria.emergency_threshold:
            reason = LaneChangeReason.EMERGENCY_MANEUVER
            self.performance_metrics['emergency_decisions'] += 1
        
        rospy.loginfo(f"[LaneChangeDecisionEngine] [{timestamp:.3f}] Final decision:")
        rospy.loginfo(f"  Decision: {decision.value}")
        rospy.loginfo(f"  Reason: {reason.value}")
        rospy.loginfo(f"  Confidence: {best_score:.3f}")
        
        return decision, reason, best_score
    
    def is_obstacle_in_lane(self, obstacle: ObjectDetection, lane: LaneState) -> bool:
        """
        Determine if an obstacle is in a specific lane (simplified implementation).
        
        Args:
            obstacle: Object detection
            lane: Lane state
            
        Returns:
            True if obstacle is in the lane
        """
        # Simplified implementation - would need more sophisticated lane detection
        # For now, assume obstacles are distributed based on their lateral position
        return True  # Placeholder - would implement proper lane association
    
    def calculate_deceleration_distance(self, velocity: float) -> float:
        """Calculate required deceleration distance for safe stopping"""
        max_deceleration = 3.0  # m/s²
        return (velocity * velocity) / (2 * max_deceleration)
    
    def calculate_gap_confidence(self, gap_length: float, front_margin: float, rear_margin: float) -> float:
        """Calculate confidence score for a gap"""
        length_factor = min(1.0, gap_length / self.criteria.minimum_gap_length)
        margin_factor = min(1.0, (front_margin + rear_margin) / (2 * self.criteria.minimum_safety_margin))
        return length_factor * margin_factor
    
    def estimate_lateral_acceleration(self, velocity: float, gap_length: float) -> float:
        """Estimate required lateral acceleration for lane change"""
        if gap_length <= 0 or velocity <= 0:
            return float('inf')
        
        # Simplified calculation assuming sinusoidal trajectory
        lane_width = 0.6  # meters (typical Duckietown lane width)
        maneuver_time = gap_length / velocity
        return (4 * lane_width) / (maneuver_time * maneuver_time)
    
    def estimate_maneuver_time(self, velocity: float, gap_length: float) -> float:
        """Estimate time required for lane change maneuver"""
        if velocity <= 0:
            return float('inf')
        return gap_length / velocity
    
    def update_performance_metrics(self, evaluation_time: float, decision: LaneChangeDecision, confidence: float):
        """Update performance metrics"""
        self.performance_metrics['total_evaluations'] += 1
        
        # Update average evaluation time
        total_evals = self.performance_metrics['total_evaluations']
        current_avg = self.performance_metrics['average_evaluation_time']
        self.performance_metrics['average_evaluation_time'] = (
            (current_avg * (total_evals - 1) + evaluation_time) / total_evals
        )
        
        # Count successful decisions
        if decision != LaneChangeDecision.NO_CHANGE and confidence > self.criteria.decision_confidence_threshold:
            self.performance_metrics['successful_decisions'] += 1
        
        # Check for safety violations
        if confidence < 0.3:  # Low confidence might indicate safety issues
            self.performance_metrics['safety_violations_detected'] += 1
        
        self.performance_metrics['last_update_time'] = time.time()
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get current performance metrics"""
        success_rate = 0.0
        if self.performance_metrics['total_evaluations'] > 0:
            success_rate = (self.performance_metrics['successful_decisions'] / 
                          self.performance_metrics['total_evaluations']) * 100
        
        return {
            'total_evaluations': self.performance_metrics['total_evaluations'],
            'successful_decisions': self.performance_metrics['successful_decisions'],
            'success_rate': success_rate,
            'average_evaluation_time_ms': self.performance_metrics['average_evaluation_time'],
            'gap_analyses_performed': self.performance_metrics['gap_analyses_performed'],
            'safety_violations_detected': self.performance_metrics['safety_violations_detected'],
            'emergency_decisions': self.performance_metrics['emergency_decisions'],
            'last_update_time': self.performance_metrics['last_update_time'],
            'decision_history_length': len(self.decision_history)
        }
    
    def get_decision_history(self, limit: int = 10) -> List[Tuple[float, str, float]]:
        """Get recent decision history"""
        recent_history = self.decision_history[-limit:] if limit > 0 else self.decision_history
        return [(timestamp, decision.value, confidence) for timestamp, decision, confidence in recent_history]
    
    def reset_metrics(self):
        """Reset performance metrics"""
        self.performance_metrics = {
            'total_evaluations': 0,
            'successful_decisions': 0,
            'average_evaluation_time': 0.0,
            'gap_analyses_performed': 0,
            'safety_violations_detected': 0,
            'emergency_decisions': 0,
            'last_update_time': time.time()
        }
        self.decision_history.clear()
        rospy.loginfo("[LaneChangeDecisionEngine] Performance metrics reset")
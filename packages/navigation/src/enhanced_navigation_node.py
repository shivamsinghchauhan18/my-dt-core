#!/usr/bin/env python3

import rospy
import time
from typing import List, Optional, Dict, Any
from datetime import datetime

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_enhanced_msgs.msg import ObjectDetectionArray, ObjectDetection
from geometry_msgs.msg import Point, Vector3
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String, Header
from sensor_msgs.msg import CompressedImage

from risk_assessment_engine import RiskAssessmentEngine, RiskLevel, VehicleState, RiskFactors
from avoidance_planner import AvoidancePlanner, AvoidanceStrategy, AvoidanceState
from lane_change_decision_engine import (
    LaneChangeDecisionEngine, LaneChangeDecision, LaneChangeReason, 
    LaneState, GapAnalysis, DecisionCriteria, DecisionMatrix
)
from lane_change_trajectory_generator import (
    LaneChangeTrajectoryGenerator, LaneChangeTrajectory, TrajectoryConstraints,
    LaneChangeParameters, TrajectoryWaypoint, TrajectoryValidationResult
)


class EnhancedNavigationNode(DTROS):
    """
    Enhanced navigation node with integrated risk assessment capabilities.
    
    This node extends the existing navigation system with advanced risk assessment
    for collision prediction and avoidance. It integrates with the existing
    object detection pipeline and provides real-time risk monitoring.
    
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~risk_assessment_enabled (:obj:`bool`): Enable/disable risk assessment
        ~risk_update_frequency (:obj:`float`): Frequency for risk assessment updates
        ~emergency_stop_threshold (:obj:`str`): Risk level that triggers emergency stop
        ~avoidance_threshold (:obj:`str`): Risk level that triggers avoidance behavior
        ~monitoring_enabled (:obj:`bool`): Enable comprehensive monitoring and logging
        ~performance_logging (:obj:`bool`): Enable performance metrics logging

    Subscribers:
    ~object_detections (:obj:`duckietown_enhanced_msgs.msg.ObjectDetectionArray`): Detected objects
        ~vehicle_state (:obj:`geometry_msgs.msg.Twist`): Current vehicle velocity
        ~camera_image (:obj:`sensor_msgs.msg.CompressedImage`): Camera image for ground projection

    Publishers:
        ~risk_assessment (:obj:`std_msgs.msg.String`): Risk assessment results
    ~navigation_command (:obj:`duckietown_msgs.msg.Twist2DStamped`): Navigation commands
        ~emergency_stop (:obj:`std_msgs.msg.String`): Emergency stop signals
        ~performance_metrics (:obj:`std_msgs.msg.String`): Performance monitoring data
    """
    
    def __init__(self, node_name: str):
        # Initialize the DTROS parent class
        super(EnhancedNavigationNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        
        # Configuration parameters
        self.risk_assessment_enabled = rospy.get_param('~risk_assessment_enabled', True)
        self.risk_update_frequency = rospy.get_param('~risk_update_frequency', 10.0)  # Hz
        self.emergency_stop_threshold = rospy.get_param('~emergency_stop_threshold', 'CRITICAL')
        self.avoidance_threshold = rospy.get_param('~avoidance_threshold', 'HIGH')
        self.monitoring_enabled = rospy.get_param('~monitoring_enabled', True)
        self.performance_logging = rospy.get_param('~performance_logging', True)
        
        # Initialize risk assessment engine
        self.risk_engine = RiskAssessmentEngine()
        
        # Initialize avoidance planner
        self.avoidance_planner = AvoidancePlanner()
        
        # Initialize lane change decision engine
        self.lane_change_engine = LaneChangeDecisionEngine()
        
        # Initialize trajectory generator
        self.trajectory_generator = LaneChangeTrajectoryGenerator()
        
        # State tracking
        self.current_vehicle_state = VehicleState(
            position=Point(x=0.0, y=0.0, z=0.0),
            velocity=Vector3(x=0.0, y=0.0, z=0.0),
            heading=0.0,
            timestamp=time.time()
        )
        
        self.latest_detections: List[ObjectDetection] = []
        self.latest_risk_assessments: List = []
        self.emergency_stop_active = False
        self.last_risk_update = 0.0
        
        # Lane change state tracking
        self.current_lane_state: Optional[LaneState] = None
        self.adjacent_lanes: Dict[str, LaneState] = {}
        self.lane_change_active = False
        self.last_lane_change_evaluation = 0.0
        self.current_trajectory: Optional[LaneChangeTrajectory] = None
        self.trajectory_execution_start_time = 0.0
        
        # Performance monitoring
        self.performance_metrics = {
            'total_assessments': 0,
            'average_assessment_time': 0.0,
            'critical_detections': 0,
            'emergency_stops_triggered': 0,
            'last_update_time': time.time()
        }
        
        # Subscribers
        self.sub_detections = rospy.Subscriber(
            '~object_detections',
            ObjectDetectionArray,
            self.cb_object_detections,
            queue_size=1
        )
        
        self.sub_vehicle_state = rospy.Subscriber(
            '~vehicle_state',
            Twist2DStamped,
            self.cb_vehicle_state,
            queue_size=1
        )
        
        # Publishers
        self.pub_risk_assessment = rospy.Publisher(
            '~risk_assessment',
            String,
            queue_size=1
        )
        
        self.pub_navigation_command = rospy.Publisher(
            '~navigation_command',
            Twist2DStamped,
            queue_size=1
        )
        
        self.pub_emergency_stop = rospy.Publisher(
            '~emergency_stop',
            String,
            queue_size=1
        )
        
        self.pub_performance_metrics = rospy.Publisher(
            '~performance_metrics',
            String,
            queue_size=1
        )
        
        # Initialize risk assessment timer
        if self.risk_assessment_enabled:
            self.risk_timer = rospy.Timer(
                rospy.Duration(1.0 / self.risk_update_frequency),
                self.cb_risk_assessment_timer
            )
        
        # Initialize performance monitoring timer
        if self.performance_logging:
            self.performance_timer = rospy.Timer(
                rospy.Duration(1.0),  # 1 Hz for performance logging
                self.cb_performance_monitoring_timer
            )
        
        rospy.loginfo(f"[EnhancedNavigationNode] Initialized with risk assessment and avoidance capabilities")
        rospy.loginfo(f"[EnhancedNavigationNode] Risk assessment enabled: {self.risk_assessment_enabled}")
        rospy.loginfo(f"[EnhancedNavigationNode] Update frequency: {self.risk_update_frequency} Hz")
        rospy.loginfo(f"[EnhancedNavigationNode] Emergency stop threshold: {self.emergency_stop_threshold}")
        rospy.loginfo(f"[EnhancedNavigationNode] Avoidance threshold: {self.avoidance_threshold}")
        rospy.loginfo(f"[EnhancedNavigationNode] Avoidance planner initialized with intelligent strategies")
    
    def cb_object_detections(self, msg: ObjectDetectionArray):
        """
        Callback for object detection messages.
        
        Args:
            msg: ObjectDetectionArray message containing detected objects
        """
        timestamp = time.time()
        
        # Store latest detections
        self.latest_detections = msg.detections
        
        if self.monitoring_enabled:
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Received {len(msg.detections)} object detections")
            rospy.loginfo(f"[EnhancedNavigationNode] Processing time from detection: {msg.processing_time:.3f}s")
            
            # Log individual detections
            for i, detection in enumerate(msg.detections):
                rospy.loginfo(f"[EnhancedNavigationNode] Detection {i}: {detection.class_name} "
                             f"at {detection.distance:.2f}m (confidence: {detection.confidence:.2f})")
        
        # Trigger immediate risk assessment if enabled
        if self.risk_assessment_enabled and len(self.latest_detections) > 0:
            self.perform_risk_assessment()
            
        # Trigger lane change evaluation if enabled
        self.evaluate_lane_change_opportunities()
    
    def cb_vehicle_state(self, msg: Twist2DStamped):
        """
        Callback for vehicle state updates.
        
        Args:
            msg: Twist message containing current vehicle velocity
        """
        timestamp = time.time()
        
    # Update vehicle state (map Twist2DStamped to internal state)
    self.current_vehicle_state.velocity.x = msg.v
    self.current_vehicle_state.velocity.y = 0.0
    self.current_vehicle_state.velocity.z = 0.0
        self.current_vehicle_state.timestamp = timestamp
        
        # Calculate heading from angular velocity (simplified)
        if abs(msg.omega) > 0.01:
            self.current_vehicle_state.heading += msg.omega * 0.1  # Rough integration
        
        if self.monitoring_enabled:
            rospy.logdebug(f"[EnhancedNavigationNode] [{timestamp:.3f}] Vehicle state updated:")
            rospy.logdebug(f"  v: {msg.v:.2f} m/s")
            rospy.logdebug(f"  omega: {msg.omega:.2f} rad/s")
            rospy.logdebug(f"  Estimated heading: {self.current_vehicle_state.heading:.2f} rad")
    
    def cb_risk_assessment_timer(self, event):
        """
        Timer callback for periodic risk assessment.
        
        Args:
            event: Timer event information
        """
        if len(self.latest_detections) > 0:
            self.perform_risk_assessment()
    
    def cb_performance_monitoring_timer(self, event):
        """
        Timer callback for performance monitoring and logging.
        
        Args:
            event: Timer event information
        """
        self.publish_performance_metrics()
    
    def perform_risk_assessment(self):
        """
        Perform comprehensive risk assessment on current detections.
        """
        if not self.risk_assessment_enabled or len(self.latest_detections) == 0:
            return
        
        timestamp = time.time()
        assessment_start_time = time.time()
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Starting risk assessment for {len(self.latest_detections)} objects")
        
        try:
            # Perform risk assessment
            risk_assessments = self.risk_engine.assess_collision_risk(
                self.latest_detections,
                self.current_vehicle_state
            )
            
            assessment_time = (time.time() - assessment_start_time) * 1000
            
            # Update performance metrics
            self.performance_metrics['total_assessments'] += 1
            self.performance_metrics['average_assessment_time'] = (
                (self.performance_metrics['average_assessment_time'] * 
                 (self.performance_metrics['total_assessments'] - 1) + assessment_time) /
                self.performance_metrics['total_assessments']
            )
            
            # Store latest assessments
            self.latest_risk_assessments = risk_assessments
            
            # Process risk assessment results
            self.process_risk_assessment_results(risk_assessments, assessment_time)
            
            # Publish risk assessment results
            self.publish_risk_assessment_results(risk_assessments, assessment_time)
            
            # Update last assessment time
            self.last_risk_update = timestamp
            
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Risk assessment completed in {assessment_time:.2f}ms")
            
        except Exception as e:
            rospy.logerr(f"[EnhancedNavigationNode] [{timestamp:.3f}] Risk assessment failed: {str(e)}")
            rospy.logerr(f"[EnhancedNavigationNode] Exception details: {type(e).__name__}")
    
    def process_risk_assessment_results(self, risk_assessments: List, assessment_time: float):
        """
        Process risk assessment results and determine appropriate actions.
        
        Args:
            risk_assessments: List of risk assessment results
            assessment_time: Time taken for assessment in milliseconds
        """
        timestamp = time.time()
        
        # Analyze risk levels
        risk_levels = [risk_level for _, _, risk_level in risk_assessments]
        critical_objects = [assessment for assessment in risk_assessments 
                          if assessment[2] == RiskLevel.CRITICAL]
        high_risk_objects = [assessment for assessment in risk_assessments 
                           if assessment[2] == RiskLevel.HIGH]
        
        # Log risk analysis
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Risk Analysis Results:")
        rospy.loginfo(f"  Critical risk objects: {len(critical_objects)}")
        rospy.loginfo(f"  High risk objects: {len(high_risk_objects)}")
        rospy.loginfo(f"  Total objects assessed: {len(risk_assessments)}")
        
        # Check for emergency stop conditions
        if len(critical_objects) > 0 and self.emergency_stop_threshold == 'CRITICAL':
            self.trigger_emergency_stop(critical_objects, "Critical risk objects detected")
        elif len(high_risk_objects) > 0 and self.emergency_stop_threshold == 'HIGH':
            self.trigger_emergency_stop(high_risk_objects, "High risk objects detected")
        
        # Check for avoidance behavior triggers using intelligent avoidance planner
        elif len(high_risk_objects) > 0 and self.avoidance_threshold == 'HIGH':
            self.trigger_intelligent_avoidance(risk_assessments, "High risk avoidance required")
        elif len(critical_objects) > 0 and self.avoidance_threshold == 'CRITICAL':
            self.trigger_intelligent_avoidance(risk_assessments, "Critical risk avoidance required")
        
        # Update performance metrics
        self.performance_metrics['critical_detections'] += len(critical_objects)
        
        # Log detailed object risk information
        for i, (detection, risk_factors, risk_level) in enumerate(risk_assessments):
            rospy.loginfo(f"[EnhancedNavigationNode] Object {i} Risk Details:")
            rospy.loginfo(f"  Class: {detection.class_name} | Distance: {detection.distance:.2f}m")
            rospy.loginfo(f"  Risk Level: {risk_level.name} | TTC: {risk_factors.time_to_collision:.2f}s")
            rospy.loginfo(f"  Risk Factors: D:{risk_factors.distance_risk:.2f} "
                         f"V:{risk_factors.velocity_risk:.2f} T:{risk_factors.object_type_risk:.2f}")
            rospy.loginfo(f"  Safety Margins: Lateral:{risk_factors.lateral_clearance:.2f}m")
    
    def trigger_emergency_stop(self, critical_objects: List, reason: str):
        """
        Trigger emergency stop due to critical risk conditions.
        
        Args:
            critical_objects: List of critical risk objects
            reason: Reason for emergency stop
        """
        timestamp = time.time()
        
        if not self.emergency_stop_active:
            self.emergency_stop_active = True
            self.performance_metrics['emergency_stops_triggered'] += 1
            
            rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] EMERGENCY STOP TRIGGERED!")
            rospy.logwarn(f"[EnhancedNavigationNode] Reason: {reason}")
            rospy.logwarn(f"[EnhancedNavigationNode] Critical objects: {len(critical_objects)}")
            
            # Log critical object details
            for i, (detection, risk_factors, risk_level) in enumerate(critical_objects):
                rospy.logwarn(f"[EnhancedNavigationNode] Critical Object {i}: {detection.class_name} "
                             f"at {detection.distance:.2f}m (TTC: {risk_factors.time_to_collision:.2f}s)")
            
            # Publish emergency stop notification
            stop_msg = String()
            stop_msg.data = f"EMERGENCY_STOP: {reason} - {len(critical_objects)} critical objects"
            self.pub_emergency_stop.publish(stop_msg)
            
            # Send zero velocity command (Twist2DStamped)
            stop_command = Twist2DStamped()
            stop_command.v = 0.0
            stop_command.omega = 0.0
            self.pub_navigation_command.publish(stop_command)
    
    def trigger_intelligent_avoidance(self, risk_assessments: List, reason: str):
        """
        Trigger intelligent avoidance behavior using the avoidance planner.
        
        Args:
            risk_assessments: List of all risk assessments
            reason: Reason for avoidance behavior
        """
        timestamp = time.time()
        
        rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] INTELLIGENT AVOIDANCE TRIGGERED!")
        rospy.logwarn(f"[EnhancedNavigationNode] Reason: {reason}")
        rospy.logwarn(f"[EnhancedNavigationNode] Total risk assessments: {len(risk_assessments)}")
        
        # Use avoidance planner to select strategy and generate trajectory
        try:
            strategy, trajectory = self.avoidance_planner.plan_avoidance(
                risk_assessments, 
                self.current_vehicle_state
            )
            
            rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] Avoidance strategy selected: {strategy.value}")
            
            if trajectory:
                rospy.logwarn(f"[EnhancedNavigationNode] Trajectory generated:")
                rospy.logwarn(f"  Duration: {trajectory.total_duration:.2f}s")
                rospy.logwarn(f"  Waypoints: {len(trajectory.waypoints)}")
                rospy.logwarn(f"  Safety margin: {trajectory.safety_margin:.2f}m")
                
                # Execute the first command from the trajectory
                avoidance_command = self.avoidance_planner.execute_trajectory(trajectory, timestamp)
                
                if avoidance_command:
                    # Convert geometry_msgs/Twist to Twist2DStamped before publishing
                    avoidance_cmd_2d = Twist2DStamped()
                    if hasattr(avoidance_command, 'linear') and hasattr(avoidance_command, 'angular'):
                        avoidance_cmd_2d.v = getattr(avoidance_command.linear, 'x', 0.0)
                        avoidance_cmd_2d.omega = getattr(avoidance_command.angular, 'z', 0.0)
                    self.pub_navigation_command.publish(avoidance_cmd_2d)
                    rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Avoidance command sent:")
                    rospy.loginfo(f"  Linear velocity: {avoidance_cmd_2d.v:.2f} m/s")
                    rospy.loginfo(f"  Angular velocity: {avoidance_cmd_2d.omega:.2f} rad/s")
                else:
                    rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] No avoidance command generated")
            else:
                rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] No trajectory generated - using fallback")
                self.trigger_avoidance_behavior(risk_assessments, reason)
                
        except Exception as e:
            rospy.logerr(f"[EnhancedNavigationNode] [{timestamp:.3f}] Intelligent avoidance failed: {str(e)}")
            rospy.logerr(f"[EnhancedNavigationNode] Falling back to simple avoidance")
            self.trigger_avoidance_behavior(risk_assessments, reason)
    
    def trigger_avoidance_behavior(self, risk_objects: List, reason: str):
        """
        Trigger simple avoidance behavior for high-risk objects (fallback method).
        
        Args:
            risk_objects: List of high-risk objects
            reason: Reason for avoidance behavior
        """
        timestamp = time.time()
        
        rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] SIMPLE AVOIDANCE BEHAVIOR TRIGGERED!")
        rospy.logwarn(f"[EnhancedNavigationNode] Reason: {reason}")
        rospy.logwarn(f"[EnhancedNavigationNode] Risk objects: {len(risk_objects)}")
        
        # Log risk object details
        for i, (detection, risk_factors, risk_level) in enumerate(risk_objects):
            rospy.logwarn(f"[EnhancedNavigationNode] Risk Object {i}: {detection.class_name} "
                         f"at {detection.distance:.2f}m (Risk: {risk_level.name})")
        
        # Simple avoidance strategy - reduce speed
    avoidance_command = Twist2DStamped()
    avoidance_command.v = max(0.1, self.current_vehicle_state.velocity.x * 0.5)
    avoidance_command.omega = 0.0  # Could implement steering avoidance here
        
    self.pub_navigation_command.publish(avoidance_command)
        
    rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Simple avoidance command sent: v={avoidance_command.v:.2f}")
    
    def publish_risk_assessment_results(self, risk_assessments: List, assessment_time: float):
        """
        Publish risk assessment results for monitoring and debugging.
        
        Args:
            risk_assessments: List of risk assessment results
            assessment_time: Time taken for assessment
        """
        timestamp = time.time()
        
        # Generate risk statistics
        stats = self.risk_engine.get_risk_statistics(risk_assessments)
        
        # Create comprehensive risk report
        risk_report = {
            'timestamp': timestamp,
            'assessment_time_ms': assessment_time,
            'total_objects': len(risk_assessments),
            'risk_distribution': stats['risk_distribution'],
            'average_distance': stats['average_distance'],
            'minimum_ttc': stats['minimum_ttc'],
            'critical_objects': stats['critical_objects'],
            'emergency_stop_active': self.emergency_stop_active,
            'vehicle_velocity': {
                'x': self.current_vehicle_state.velocity.x,
                'y': self.current_vehicle_state.velocity.y,
                'z': self.current_vehicle_state.velocity.z
            }
        }
        
        # Publish risk assessment
        risk_msg = String()
        risk_msg.data = str(risk_report)
        self.pub_risk_assessment.publish(risk_msg)
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Risk assessment published:")
        rospy.loginfo(f"  Assessment time: {assessment_time:.2f}ms")
        rospy.loginfo(f"  Risk distribution: {stats['risk_distribution']}")
        rospy.loginfo(f"  Average distance: {stats['average_distance']:.2f}m")
        rospy.loginfo(f"  Minimum TTC: {stats['minimum_ttc']:.2f}s")
    
    def publish_performance_metrics(self):
        """
        Publish performance monitoring metrics.
        """
        timestamp = time.time()
        
        # Update performance metrics
        self.performance_metrics['last_update_time'] = timestamp
        
        # Calculate additional metrics
        uptime = timestamp - rospy.get_time()
        assessments_per_second = (self.performance_metrics['total_assessments'] / 
                                max(1.0, uptime)) if uptime > 0 else 0.0
        
        # Get avoidance planner metrics
        avoidance_metrics = self.avoidance_planner.get_performance_metrics()
        
        performance_report = {
            'timestamp': timestamp,
            'uptime_seconds': uptime,
            'total_assessments': self.performance_metrics['total_assessments'],
            'assessments_per_second': assessments_per_second,
            'average_assessment_time_ms': self.performance_metrics['average_assessment_time'],
            'critical_detections': self.performance_metrics['critical_detections'],
            'emergency_stops_triggered': self.performance_metrics['emergency_stops_triggered'],
            'last_risk_update': self.last_risk_update,
            'risk_assessment_enabled': self.risk_assessment_enabled,
            'emergency_stop_active': self.emergency_stop_active,
            'avoidance_planner': avoidance_metrics
        }
        
        # Publish performance metrics
        perf_msg = String()
        perf_msg.data = str(performance_report)
        self.pub_performance_metrics.publish(perf_msg)
        
        if self.monitoring_enabled:
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Performance Metrics:")
            rospy.loginfo(f"  Total assessments: {self.performance_metrics['total_assessments']}")
            rospy.loginfo(f"  Assessments/sec: {assessments_per_second:.2f}")
            rospy.loginfo(f"  Avg assessment time: {self.performance_metrics['average_assessment_time']:.2f}ms")
            rospy.loginfo(f"  Critical detections: {self.performance_metrics['critical_detections']}")
            rospy.loginfo(f"  Emergency stops: {self.performance_metrics['emergency_stops_triggered']}")
            rospy.loginfo(f"  Avoidance planner state: {avoidance_metrics['current_state']}")
            rospy.loginfo(f"  Avoidance success rate: {avoidance_metrics['success_rate']:.1f}%")
    
    def reset_emergency_stop(self):
        """
        Reset emergency stop state (could be called by external service).
        """
        timestamp = time.time()
        
        if self.emergency_stop_active:
            self.emergency_stop_active = False
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Emergency stop reset")
            
            # Publish reset notification
            reset_msg = String()
            reset_msg.data = "EMERGENCY_STOP_RESET"
            self.pub_emergency_stop.publish(reset_msg)
    
    def evaluate_lane_change_opportunities(self):
        """
        Evaluate lane change opportunities using the decision engine.
        """
        timestamp = time.time()
        
        # Skip if no detections or too recent evaluation
        if (len(self.latest_detections) == 0 or 
            timestamp - self.last_lane_change_evaluation < 0.5):  # Limit to 2 Hz
            return
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Evaluating lane change opportunities")
        
        try:
            # Create current lane state (simplified)
            current_lane = self.create_current_lane_state()
            
            # Create adjacent lane states (simplified)
            adjacent_lanes = self.create_adjacent_lane_states()
            
            # Prepare vehicle state for decision engine
            vehicle_state_dict = {
                'position': {'x': self.current_vehicle_state.position.x, 'y': self.current_vehicle_state.position.y},
                'velocity': {'x': self.current_vehicle_state.velocity.x, 'y': self.current_vehicle_state.velocity.y},
                'heading': self.current_vehicle_state.heading,
                'timestamp': self.current_vehicle_state.timestamp
            }
            
            # Evaluate lane change opportunity
            decision, reason, confidence = self.lane_change_engine.evaluate_lane_change_opportunity(
                current_lane, adjacent_lanes, vehicle_state_dict, self.latest_detections
            )
            
            # Process lane change decision
            self.process_lane_change_decision(decision, reason, confidence)
            
            # Update evaluation timestamp
            self.last_lane_change_evaluation = timestamp
            
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Lane change evaluation completed")
            rospy.loginfo(f"  Decision: {decision.value}")
            rospy.loginfo(f"  Reason: {reason.value}")
            rospy.loginfo(f"  Confidence: {confidence:.3f}")
            
        except Exception as e:
            rospy.logerr(f"[EnhancedNavigationNode] [{timestamp:.3f}] Lane change evaluation failed: {str(e)}")
            rospy.logerr(f"[EnhancedNavigationNode] Exception details: {type(e).__name__}")
    
    def create_current_lane_state(self) -> LaneState:
        """
        Create current lane state from available information.
        
        Returns:
            LaneState representing the current lane
        """
        timestamp = time.time()
        
        # Filter obstacles in current lane (simplified)
        current_lane_obstacles = [obs for obs in self.latest_detections 
                                if obs.distance < 5.0]  # Assume obstacles within 5m are in current lane
        
        # Calculate safety score based on obstacles
        safety_score = max(0.0, 1.0 - len(current_lane_obstacles) * 0.2)
        
        current_lane = LaneState(
            lane_id="current",
            is_current=True,
            is_available=True,
            width=0.6,  # Typical Duckietown lane width
            center_line_offset=0.0,
            obstacles=current_lane_obstacles,
            gap_length=10.0,  # Assume reasonable gap ahead
            gap_start_distance=0.0,
            safety_score=safety_score,
            timestamp=timestamp
        )
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Current lane state created:")
        rospy.loginfo(f"  Obstacles in lane: {len(current_lane_obstacles)}")
        rospy.loginfo(f"  Safety score: {safety_score:.3f}")
        
        return current_lane
    
    def create_adjacent_lane_states(self) -> Dict[str, LaneState]:
        """
        Create adjacent lane states (simplified implementation).
        
        Returns:
            Dictionary of adjacent lane states
        """
        timestamp = time.time()
        adjacent_lanes = {}
        
        # Create left lane (if available)
        left_lane_obstacles = [obs for obs in self.latest_detections 
                             if obs.distance < 5.0]  # Simplified - would need proper lane detection
        
        left_safety_score = max(0.0, 1.0 - len(left_lane_obstacles) * 0.15)
        
        adjacent_lanes["left"] = LaneState(
            lane_id="left",
            is_current=False,
            is_available=True,  # Assume available for now
            width=0.6,
            center_line_offset=-0.6,  # Left of current lane
            obstacles=left_lane_obstacles,
            gap_length=8.0,  # Assume reasonable gap
            gap_start_distance=1.0,
            safety_score=left_safety_score,
            timestamp=timestamp
        )
        
        # Create right lane (if available)
        right_lane_obstacles = [obs for obs in self.latest_detections 
                              if obs.distance < 5.0]  # Simplified
        
        right_safety_score = max(0.0, 1.0 - len(right_lane_obstacles) * 0.15)
        
        adjacent_lanes["right"] = LaneState(
            lane_id="right",
            is_current=False,
            is_available=True,  # Assume available for now
            width=0.6,
            center_line_offset=0.6,  # Right of current lane
            obstacles=right_lane_obstacles,
            gap_length=8.0,  # Assume reasonable gap
            gap_start_distance=1.0,
            safety_score=right_safety_score,
            timestamp=timestamp
        )
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Adjacent lane states created:")
        rospy.loginfo(f"  Left lane safety score: {left_safety_score:.3f}")
        rospy.loginfo(f"  Right lane safety score: {right_safety_score:.3f}")
        
        return adjacent_lanes
    
    def process_lane_change_decision(self, decision: LaneChangeDecision, reason: LaneChangeReason, confidence: float):
        """
        Process lane change decision and take appropriate action.
        
        Args:
            decision: Lane change decision
            reason: Reason for the decision
            confidence: Decision confidence score
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Processing lane change decision:")
        rospy.loginfo(f"  Decision: {decision.value}")
        rospy.loginfo(f"  Reason: {reason.value}")
        rospy.loginfo(f"  Confidence: {confidence:.3f}")
        
        if decision == LaneChangeDecision.NO_CHANGE:
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] No lane change required")
            self.lane_change_active = False
            return
        
        # Check if lane change is safe to initiate
        if confidence < 0.7:
            rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] Lane change confidence too low: {confidence:.3f}")
            return
        
        # Log lane change initiation
        rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] LANE CHANGE DECISION MADE!")
        rospy.logwarn(f"[EnhancedNavigationNode] Direction: {decision.value}")
        rospy.logwarn(f"[EnhancedNavigationNode] Reason: {reason.value}")
        rospy.logwarn(f"[EnhancedNavigationNode] Confidence: {confidence:.3f}")
        
        # Generate trajectory for the lane change
        trajectory = self.generate_lane_change_trajectory(decision, confidence)
        
        if trajectory and trajectory.is_feasible:
            # Set lane change active flag
            self.lane_change_active = True
            self.current_trajectory = trajectory
            self.trajectory_execution_start_time = timestamp
            
            rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] LANE CHANGE TRAJECTORY GENERATED!")
            rospy.logwarn(f"[EnhancedNavigationNode] Trajectory duration: {trajectory.total_duration:.2f}s")
            rospy.logwarn(f"[EnhancedNavigationNode] Trajectory distance: {trajectory.total_distance:.2f}m")
            rospy.logwarn(f"[EnhancedNavigationNode] Max lateral acceleration: {trajectory.maximum_lateral_acceleration:.2f}m/s²")
            rospy.logwarn(f"[EnhancedNavigationNode] Trajectory feasible: {trajectory.is_feasible}")
            
            # Publish lane change decision for coordination with other systems
            lane_change_msg = String()
            lane_change_msg.data = f"LANE_CHANGE_TRAJECTORY: {decision.value} - {reason.value} (confidence: {confidence:.3f}, duration: {trajectory.total_duration:.2f}s)"
            
            # Would publish to lane change coordination topic
            # self.pub_lane_change_decision.publish(lane_change_msg)
            
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Lane change trajectory ready for execution")
        else:
            rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] Failed to generate feasible trajectory - lane change aborted")
            self.lane_change_active = False
            self.current_trajectory = None
        
        # Update performance metrics
        lane_change_metrics = self.lane_change_engine.get_performance_metrics()
        rospy.loginfo(f"[EnhancedNavigationNode] Lane change engine metrics:")
        rospy.loginfo(f"  Total evaluations: {lane_change_metrics['total_evaluations']}")
        rospy.loginfo(f"  Success rate: {lane_change_metrics['success_rate']:.1f}%")
        rospy.loginfo(f"  Average evaluation time: {lane_change_metrics['average_evaluation_time_ms']:.2f}ms")
    
    def generate_lane_change_trajectory(self, decision: LaneChangeDecision, confidence: float) -> Optional[LaneChangeTrajectory]:
        """
        Generate a smooth trajectory for the lane change decision.
        
        Args:
            decision: Lane change decision
            confidence: Decision confidence
            
        Returns:
            Generated trajectory or None if generation fails
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Generating trajectory for {decision.value}")
        
        if decision == LaneChangeDecision.NO_CHANGE:
            return None
        
        try:
            # Create lane change parameters
            lane_change_params = self.create_lane_change_parameters(decision)
            
            # Create trajectory constraints based on confidence and urgency
            constraints = self.create_trajectory_constraints(confidence)
            
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Lane change parameters:")
            rospy.loginfo(f"  Start position: ({lane_change_params.start_position.x:.2f}, {lane_change_params.start_position.y:.2f})")
            rospy.loginfo(f"  End position: ({lane_change_params.end_position.x:.2f}, {lane_change_params.end_position.y:.2f})")
            rospy.loginfo(f"  Lane width: {lane_change_params.lane_width:.2f}m")
            
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Trajectory constraints:")
            rospy.loginfo(f"  Max lateral acceleration: {constraints.maximum_lateral_acceleration:.2f}m/s²")
            rospy.loginfo(f"  Preferred duration: {constraints.preferred_duration:.2f}s")
            rospy.loginfo(f"  Comfort factor: {constraints.comfort_factor:.2f}")
            
            # Generate trajectory
            trajectory = self.trajectory_generator.generate_lane_change_trajectory(
                lane_change_params, constraints
            )
            
            rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Trajectory generation results:")
            rospy.loginfo(f"  Feasible: {trajectory.is_feasible}")
            rospy.loginfo(f"  Validation result: {trajectory.validation_result.value}")
            rospy.loginfo(f"  Generation time: {trajectory.generation_time:.2f}ms")
            
            if trajectory.is_feasible:
                rospy.loginfo(f"  Duration: {trajectory.total_duration:.2f}s")
                rospy.loginfo(f"  Distance: {trajectory.total_distance:.2f}m")
                rospy.loginfo(f"  Waypoints: {len(trajectory.waypoints)}")
                rospy.loginfo(f"  Max lateral acceleration: {trajectory.maximum_lateral_acceleration:.2f}m/s²")
                rospy.loginfo(f"  Max curvature: {trajectory.maximum_curvature:.3f}1/m")
                
                # Optimize for comfort if confidence is high
                if confidence > 0.8:
                    rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Optimizing trajectory for comfort")
                    trajectory = self.trajectory_generator.optimize_trajectory_for_comfort(
                        trajectory, constraints.comfort_factor
                    )
                    rospy.loginfo(f"  Optimized max lateral acceleration: {trajectory.maximum_lateral_acceleration:.2f}m/s²")
            
            return trajectory
            
        except Exception as e:
            rospy.logerr(f"[EnhancedNavigationNode] [{timestamp:.3f}] Trajectory generation failed: {str(e)}")
            rospy.logerr(f"[EnhancedNavigationNode] Exception details: {type(e).__name__}")
            return None
    
    def create_lane_change_parameters(self, decision: LaneChangeDecision) -> LaneChangeParameters:
        """
        Create lane change parameters based on current state and decision.
        
        Args:
            decision: Lane change decision
            
        Returns:
            LaneChangeParameters for trajectory generation
        """
        timestamp = time.time()
        
        # Current position (simplified - would use actual localization)
        start_position = Point(x=0.0, y=0.0, z=0.0)
        
        # Target position based on decision
        lane_width = 0.6  # Standard Duckietown lane width
        lateral_offset = lane_width if decision == LaneChangeDecision.CHANGE_RIGHT else -lane_width
        longitudinal_distance = 3.0  # Distance to complete lane change
        
        end_position = Point(
            x=start_position.x + longitudinal_distance,
            y=start_position.y + lateral_offset,
            z=0.0
        )
        
        # Velocity parameters
        current_speed = max(0.5, self.current_vehicle_state.velocity.x)  # Minimum 0.5 m/s
        start_velocity = Vector3(x=current_speed, y=0.0, z=0.0)
        end_velocity = Vector3(x=current_speed, y=0.0, z=0.0)  # Maintain speed
        
        # Heading parameters
        start_heading = 0.0  # Assume straight initially
        end_heading = 0.0    # Return to straight after lane change
        
        # Vehicle parameters
        vehicle_length = 0.18  # Duckiebot length
        vehicle_width = 0.13   # Duckiebot width
        
        lane_change_params = LaneChangeParameters(
            start_position=start_position,
            end_position=end_position,
            start_velocity=start_velocity,
            end_velocity=end_velocity,
            start_heading=start_heading,
            end_heading=end_heading,
            lane_width=lane_width,
            vehicle_length=vehicle_length,
            vehicle_width=vehicle_width
        )
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Created lane change parameters:")
        rospy.loginfo(f"  Lateral offset: {lateral_offset:.2f}m")
        rospy.loginfo(f"  Longitudinal distance: {longitudinal_distance:.2f}m")
        rospy.loginfo(f"  Current speed: {current_speed:.2f}m/s")
        
        return lane_change_params
    
    def create_trajectory_constraints(self, confidence: float) -> TrajectoryConstraints:
        """
        Create trajectory constraints based on decision confidence and system state.
        
        Args:
            confidence: Decision confidence (0-1)
            
        Returns:
            TrajectoryConstraints for trajectory generation
        """
        timestamp = time.time()
        
        # Adjust constraints based on confidence
        # Higher confidence allows more aggressive maneuvers
        base_max_acceleration = 2.0  # m/s²
        base_comfort_factor = 0.8
        base_preferred_duration = 2.0  # s
        
        # Scale constraints based on confidence
        max_lateral_acceleration = base_max_acceleration * (0.5 + 0.5 * confidence)
        comfort_factor = base_comfort_factor * (0.7 + 0.3 * confidence)
        preferred_duration = base_preferred_duration * (1.5 - 0.5 * confidence)
        
        constraints = TrajectoryConstraints(
            maximum_lateral_acceleration=max_lateral_acceleration,
            maximum_curvature=2.0,  # 1/m
            maximum_velocity=2.0,   # m/s
            minimum_radius=0.3,     # m
            comfort_factor=comfort_factor,
            safety_margin=0.3,      # m
            preferred_duration=preferred_duration,
            maximum_duration=3.0    # s
        )
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Created trajectory constraints:")
        rospy.loginfo(f"  Confidence-based scaling: {confidence:.3f}")
        rospy.loginfo(f"  Max lateral acceleration: {max_lateral_acceleration:.2f}m/s²")
        rospy.loginfo(f"  Comfort factor: {comfort_factor:.2f}")
        rospy.loginfo(f"  Preferred duration: {preferred_duration:.2f}s")
        
        return constraints


if __name__ == '__main__':
    # Create and run the enhanced navigation node
    node = EnhancedNavigationNode('enhanced_navigation_node')
    rospy.spin()
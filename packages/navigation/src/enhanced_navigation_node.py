#!/usr/bin/env python3

import rospy
import time
from typing import List, Optional, Dict, Any
from datetime import datetime

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import ObjectDetectionArray, ObjectDetection
from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import String, Header
from sensor_msgs.msg import CompressedImage

from risk_assessment_engine import RiskAssessmentEngine, RiskLevel, VehicleState, RiskFactors


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
        ~object_detections (:obj:`duckietown_msgs.msg.ObjectDetectionArray`): Detected objects
        ~vehicle_state (:obj:`geometry_msgs.msg.Twist`): Current vehicle velocity
        ~camera_image (:obj:`sensor_msgs.msg.CompressedImage`): Camera image for ground projection

    Publishers:
        ~risk_assessment (:obj:`std_msgs.msg.String`): Risk assessment results
        ~navigation_command (:obj:`geometry_msgs.msg.Twist`): Navigation commands
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
            Twist,
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
            Twist,
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
        
        rospy.loginfo(f"[EnhancedNavigationNode] Initialized with risk assessment capabilities")
        rospy.loginfo(f"[EnhancedNavigationNode] Risk assessment enabled: {self.risk_assessment_enabled}")
        rospy.loginfo(f"[EnhancedNavigationNode] Update frequency: {self.risk_update_frequency} Hz")
        rospy.loginfo(f"[EnhancedNavigationNode] Emergency stop threshold: {self.emergency_stop_threshold}")
        rospy.loginfo(f"[EnhancedNavigationNode] Avoidance threshold: {self.avoidance_threshold}")
    
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
    
    def cb_vehicle_state(self, msg: Twist):
        """
        Callback for vehicle state updates.
        
        Args:
            msg: Twist message containing current vehicle velocity
        """
        timestamp = time.time()
        
        # Update vehicle state
        self.current_vehicle_state.velocity = msg.linear
        self.current_vehicle_state.timestamp = timestamp
        
        # Calculate heading from angular velocity (simplified)
        if abs(msg.angular.z) > 0.01:
            self.current_vehicle_state.heading += msg.angular.z * 0.1  # Rough integration
        
        if self.monitoring_enabled:
            rospy.logdebug(f"[EnhancedNavigationNode] [{timestamp:.3f}] Vehicle state updated:")
            rospy.logdebug(f"  Linear velocity: ({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f})")
            rospy.logdebug(f"  Angular velocity: ({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})")
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
        
        # Check for avoidance behavior triggers
        elif len(high_risk_objects) > 0 and self.avoidance_threshold == 'HIGH':
            self.trigger_avoidance_behavior(high_risk_objects, "High risk avoidance required")
        elif len(critical_objects) > 0 and self.avoidance_threshold == 'CRITICAL':
            self.trigger_avoidance_behavior(critical_objects, "Critical risk avoidance required")
        
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
            
            # Publish emergency stop command
            stop_msg = String()
            stop_msg.data = f"EMERGENCY_STOP: {reason} - {len(critical_objects)} critical objects"
            self.pub_emergency_stop.publish(stop_msg)
            
            # Send zero velocity command
            stop_command = Twist()
            stop_command.linear.x = 0.0
            stop_command.angular.z = 0.0
            self.pub_navigation_command.publish(stop_command)
    
    def trigger_avoidance_behavior(self, risk_objects: List, reason: str):
        """
        Trigger avoidance behavior for high-risk objects.
        
        Args:
            risk_objects: List of high-risk objects
            reason: Reason for avoidance behavior
        """
        timestamp = time.time()
        
        rospy.logwarn(f"[EnhancedNavigationNode] [{timestamp:.3f}] AVOIDANCE BEHAVIOR TRIGGERED!")
        rospy.logwarn(f"[EnhancedNavigationNode] Reason: {reason}")
        rospy.logwarn(f"[EnhancedNavigationNode] Risk objects: {len(risk_objects)}")
        
        # Log risk object details
        for i, (detection, risk_factors, risk_level) in enumerate(risk_objects):
            rospy.logwarn(f"[EnhancedNavigationNode] Risk Object {i}: {detection.class_name} "
                         f"at {detection.distance:.2f}m (Risk: {risk_level.name})")
        
        # Simple avoidance strategy - reduce speed
        avoidance_command = Twist()
        avoidance_command.linear.x = max(0.1, self.current_vehicle_state.velocity.x * 0.5)
        avoidance_command.angular.z = 0.0  # Could implement steering avoidance here
        
        self.pub_navigation_command.publish(avoidance_command)
        
        rospy.loginfo(f"[EnhancedNavigationNode] [{timestamp:.3f}] Avoidance command sent: "
                     f"linear.x={avoidance_command.linear.x:.2f}")
    
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
            'emergency_stop_active': self.emergency_stop_active
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


if __name__ == '__main__':
    # Create and run the enhanced navigation node
    node = EnhancedNavigationNode('enhanced_navigation_node')
    rospy.spin()
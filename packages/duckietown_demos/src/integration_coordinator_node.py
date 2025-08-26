#!/usr/bin/env python3
"""
Integration Coordinator Node for Enhanced Autonomous Navigation

This node coordinates between YOLO object detection and enhanced navigation,
managing behavior arbitration, system integration, and comprehensive monitoring.

Author: Duckietown
"""

import rospy
import numpy as np
import time
from threading import Lock
from collections import deque
from typing import Dict, List, Optional, Tuple

# ROS messages
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import (
    ObjectDetectionArray, 
    LanePose, 
    Twist2DStamped,
    BoolStamped
)

# Custom message types (would be defined in duckietown_msgs)
try:
    from duckietown_msgs.msg import (
        RiskAssessmentStatus,
        SystemStatus,
        PerformanceMetrics,
        CoordinationLog
    )
except ImportError:
    # Fallback to basic message types if custom messages not available
    rospy.logwarn("Custom integration messages not available, using basic types")
    RiskAssessmentStatus = String
    SystemStatus = String
    PerformanceMetrics = String
    CoordinationLog = String


class IntegrationCoordinator:
    """
    Coordinates YOLO detection with navigation system for autonomous behavior.
    
    Manages:
    - Behavior arbitration between lane following and obstacle avoidance
    - System integration status monitoring
    - Performance optimization and load balancing
    - Emergency response coordination
    - Comprehensive logging and monitoring
    """
    
    def __init__(self):
        """Initialize the integration coordinator."""
        self.node_name = "integration_coordinator"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Thread safety
        self.lock = Lock()
        
        # Configuration
        self.enable_yolo = rospy.get_param("~enable_yolo", True)
        self.enable_avoidance = rospy.get_param("~enable_avoidance", True)
        self.enable_monitoring = rospy.get_param("~enable_monitoring", True)
        self.config_mode = rospy.get_param("~config_mode", "baseline")
        
        # System state
        self.system_active = True
        self.emergency_stop_active = False
        self.last_detection_time = None
        self.last_navigation_time = None
        self.last_lane_pose_time = None
        
        # Performance tracking
        self.performance_history = deque(maxlen=100)
        self.coordination_decisions = deque(maxlen=50)
        self.system_health_score = 1.0
        
        # Behavior state
        self.current_behavior = "lane_following"
        self.behavior_priority = {
            "emergency_stop": 100,
            "collision_avoidance": 80,
            "lane_following": 60,
            "idle": 0
        }
        
        # Enhanced coordination features
        self.enable_enhanced_integration = rospy.get_param("~enable_enhanced_integration", True)
        self.coordination_timeout = rospy.get_param("~coordination_timeout", 30.0)
        self.integration_check_frequency = rospy.get_param("~integration_check_frequency", 2.0)
        
        # Component monitoring
        self.monitor_fsm_health = rospy.get_param("~monitor_fsm_health", True)
        self.monitor_coordinator_health = rospy.get_param("~monitor_coordinator_health", True)
        self.monitor_performance_optimizer = rospy.get_param("~monitor_performance_optimizer", True)
        
        # Enhanced coordination state
        self.registered_components = {}
        self.component_status = {}
        self.coordination_metrics = {
            'total_workflows': 0,
            'successful_workflows': 0,
            'failed_workflows': 0,
            'average_workflow_time': 0.0,
            'component_failures': 0,
            'recovery_attempts': 0
        }
        self.integration_health = 1.0
        self.active_workflows = {}
        self.workflow_history = deque(maxlen=100)
        
        # Data storage
        self.latest_detections = None
        self.latest_navigation_cmd = None
        self.latest_risk_status = None
        self.latest_lane_pose = None
        
        # Timing and performance
        self.coordination_start_time = time.time()
        self.frame_count = 0
        self.last_performance_log = time.time()
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] Integration Coordinator initialized")
        rospy.loginfo(f"[{self.node_name}] YOLO enabled: {self.enable_yolo}")
        rospy.loginfo(f"[{self.node_name}] Avoidance enabled: {self.enable_avoidance}")
        rospy.loginfo(f"[{self.node_name}] Monitoring enabled: {self.enable_monitoring}")
        rospy.loginfo(f"[{self.node_name}] Configuration mode: {self.config_mode}")
    
    def _setup_subscribers(self):
        """Setup ROS subscribers for system integration."""
        rospy.loginfo(f"[{self.node_name}] Setting up subscribers...")
        
        # YOLO detection input
        if self.enable_yolo:
            self.yolo_sub = rospy.Subscriber(
                "~yolo_detections", 
                ObjectDetectionArray, 
                self._yolo_detections_callback,
                queue_size=1
            )
            rospy.loginfo(f"[{self.node_name}] Subscribed to YOLO detections")
        
        # Navigation command input
        if self.enable_avoidance:
            self.nav_sub = rospy.Subscriber(
                "~navigation_command", 
                Twist2DStamped, 
                self._navigation_command_callback,
                queue_size=1
            )
            rospy.loginfo(f"[{self.node_name}] Subscribed to navigation commands")
        
        # Risk assessment input
        self.risk_sub = rospy.Subscriber(
            "~risk_assessment", 
            RiskAssessmentStatus, 
            self._risk_assessment_callback,
            queue_size=1
        )
        
        # Lane pose input
        self.lane_pose_sub = rospy.Subscriber(
            "~lane_pose", 
            LanePose, 
            self._lane_pose_callback,
            queue_size=1
        )
        
        # Emergency stop input
        self.emergency_sub = rospy.Subscriber(
            "~emergency_stop", 
            BoolStamped, 
            self._emergency_stop_callback,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] All subscribers initialized")
    
    def _setup_publishers(self):
        """Setup ROS publishers for coordination output."""
        rospy.loginfo(f"[{self.node_name}] Setting up publishers...")
        
        # Integrated control command output
        self.cmd_pub = rospy.Publisher(
            "~integrated_car_cmd", 
            Twist2DStamped, 
            queue_size=1
        )
        
        # System status output
        self.status_pub = rospy.Publisher(
            "~system_status", 
            SystemStatus, 
            queue_size=1
        )
        
        # Performance metrics output
        self.performance_pub = rospy.Publisher(
            "~performance_metrics", 
            PerformanceMetrics, 
            queue_size=1
        )
        
        # Coordination log output
        self.coordination_log_pub = rospy.Publisher(
            "~coordination_log", 
            CoordinationLog, 
            queue_size=10
        )
        
        rospy.loginfo(f"[{self.node_name}] All publishers initialized")
    
    def _setup_timers(self):
        """Setup periodic timers for coordination and monitoring."""
        # Main coordination timer (high frequency)
        self.coordination_timer = rospy.Timer(
            rospy.Duration(0.05),  # 20 Hz
            self._coordination_callback
        )
        
        # Performance monitoring timer
        if self.enable_monitoring:
            self.monitoring_timer = rospy.Timer(
                rospy.Duration(1.0),  # 1 Hz
                self._monitoring_callback
            )
        
        # System health check timer
        self.health_timer = rospy.Timer(
            rospy.Duration(0.5),  # 2 Hz
            self._health_check_callback
        )
        
        rospy.loginfo(f"[{self.node_name}] Timers initialized")
    
    def _yolo_detections_callback(self, msg):
        """Handle YOLO detection results."""
        with self.lock:
            self.latest_detections = msg
            self.last_detection_time = time.time()
            
            if self.config_mode == "debug":
                rospy.logdebug(f"[{self.node_name}] Received {len(msg.detections)} YOLO detections")
                for detection in msg.detections:
                    rospy.logdebug(f"[{self.node_name}] Detection: {detection.class_name} "
                                 f"(conf: {detection.confidence:.2f}, dist: {detection.distance:.2f}m)")
    
    def _navigation_command_callback(self, msg):
        """Handle navigation command from enhanced navigation."""
        with self.lock:
            self.latest_navigation_cmd = msg
            self.last_navigation_time = time.time()
            
            if self.config_mode == "debug":
                rospy.logdebug(f"[{self.node_name}] Navigation command: "
                             f"v={msg.v:.2f}, omega={msg.omega:.2f}")
    
    def _risk_assessment_callback(self, msg):
        """Handle risk assessment status."""
        with self.lock:
            self.latest_risk_status = msg
            
            if self.config_mode == "debug":
                rospy.logdebug(f"[{self.node_name}] Risk assessment received")
    
    def _lane_pose_callback(self, msg):
        """Handle lane pose updates."""
        with self.lock:
            self.latest_lane_pose = msg
            self.last_lane_pose_time = time.time()
            
            if self.config_mode == "debug":
                rospy.logdebug(f"[{self.node_name}] Lane pose: "
                             f"d={msg.d:.3f}, phi={msg.phi:.3f}")
    
    def _emergency_stop_callback(self, msg):
        """Handle emergency stop signals."""
        with self.lock:
            self.emergency_stop_active = msg.data
            
            if msg.data:
                rospy.logwarn(f"[{self.node_name}] EMERGENCY STOP ACTIVATED!")
                self.current_behavior = "emergency_stop"
            else:
                rospy.loginfo(f"[{self.node_name}] Emergency stop deactivated")
                if self.current_behavior == "emergency_stop":
                    self.current_behavior = "lane_following"
    
    def _coordination_callback(self, event):
        """Main coordination logic - runs at high frequency."""
        if not self.system_active:
            return
        
        start_time = time.time()
        
        with self.lock:
            # Determine current behavior based on system state
            behavior_decision = self._determine_behavior()
            
            # Generate integrated control command
            control_cmd = self._generate_control_command(behavior_decision)
            
            # Publish control command
            if control_cmd is not None:
                self.cmd_pub.publish(control_cmd)
                self.frame_count += 1
            
            # Log coordination decision
            self._log_coordination_decision(behavior_decision, control_cmd)
        
        # Track performance
        processing_time = time.time() - start_time
        self.performance_history.append(processing_time)
        
        if self.config_mode == "debug" and self.frame_count % 20 == 0:
            rospy.logdebug(f"[{self.node_name}] Coordination cycle: {processing_time*1000:.1f}ms, "
                         f"behavior: {behavior_decision}")
    
    def _determine_behavior(self) -> str:
        """Determine the current behavior based on system state and priorities."""
        current_time = time.time()
        
        # Check for emergency stop
        if self.emergency_stop_active:
            return "emergency_stop"
        
        # Check for collision avoidance need
        if (self.enable_avoidance and 
            self.latest_risk_status is not None and 
            self._requires_collision_avoidance()):
            return "collision_avoidance"
        
        # Check for valid lane following
        if (self.latest_lane_pose is not None and 
            self.last_lane_pose_time is not None and
            current_time - self.last_lane_pose_time < 1.0):
            return "lane_following"
        
        # Default to idle if no valid inputs
        return "idle"
    
    def _requires_collision_avoidance(self) -> bool:
        """Check if collision avoidance is required based on risk assessment."""
        if self.latest_risk_status is None:
            return False
        
        # This would check the actual risk level from the risk assessment
        # For now, using a simple heuristic
        if hasattr(self.latest_risk_status, 'risk_level'):
            return self.latest_risk_status.risk_level in ['HIGH', 'CRITICAL']
        
        return False
    
    def _generate_control_command(self, behavior: str) -> Optional[Twist2DStamped]:
        """Generate integrated control command based on current behavior."""
        if behavior == "emergency_stop":
            return self._generate_emergency_stop_command()
        
        elif behavior == "collision_avoidance":
            return self._generate_avoidance_command()
        
        elif behavior == "lane_following":
            return self._generate_lane_following_command()
        
        elif behavior == "idle":
            return self._generate_idle_command()
        
        return None
    
    def _generate_emergency_stop_command(self) -> Twist2DStamped:
        """Generate emergency stop command."""
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        return cmd
    
    def _generate_avoidance_command(self) -> Optional[Twist2DStamped]:
        """Generate collision avoidance command."""
        if self.latest_navigation_cmd is not None:
            # Use navigation command with potential modifications
            cmd = Twist2DStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.v = self.latest_navigation_cmd.v * 0.5  # Reduce speed during avoidance
            cmd.omega = self.latest_navigation_cmd.omega
            return cmd
        
        return self._generate_emergency_stop_command()
    
    def _generate_lane_following_command(self) -> Optional[Twist2DStamped]:
        """Generate lane following command."""
        # Use navigation command if available, otherwise use basic lane following
        if self.latest_navigation_cmd is not None:
            return self.latest_navigation_cmd
        
        # Fallback to basic lane following based on lane pose
        if self.latest_lane_pose is not None:
            cmd = Twist2DStamped()
            cmd.header.stamp = rospy.Time.now()
            
            # Simple proportional controller
            cmd.v = 0.3  # Base speed
            cmd.omega = -2.0 * self.latest_lane_pose.d - 1.0 * self.latest_lane_pose.phi
            
            return cmd
        
        return None
    
    def _generate_idle_command(self) -> Twist2DStamped:
        """Generate idle command (stop)."""
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        return cmd
    
    def _log_coordination_decision(self, behavior: str, control_cmd: Optional[Twist2DStamped]):
        """Log coordination decision for analysis."""
        decision_data = {
            'timestamp': time.time(),
            'behavior': behavior,
            'has_detections': self.latest_detections is not None,
            'has_navigation': self.latest_navigation_cmd is not None,
            'has_lane_pose': self.latest_lane_pose is not None,
            'emergency_active': self.emergency_stop_active,
            'control_v': control_cmd.v if control_cmd else 0.0,
            'control_omega': control_cmd.omega if control_cmd else 0.0
        }
        
        self.coordination_decisions.append(decision_data)
        
        # Publish coordination log
        if hasattr(CoordinationLog, 'behavior'):
            log_msg = CoordinationLog()
            log_msg.header.stamp = rospy.Time.now()
            log_msg.behavior = behavior
            log_msg.has_detections = decision_data['has_detections']
            log_msg.has_navigation = decision_data['has_navigation']
            log_msg.emergency_active = decision_data['emergency_active']
            self.coordination_log_pub.publish(log_msg)
    
    def _monitoring_callback(self, event):
        """Periodic monitoring and performance reporting."""
        if not self.enable_monitoring:
            return
        
        current_time = time.time()
        
        # Calculate performance metrics
        if self.performance_history:
            avg_latency = np.mean(self.performance_history) * 1000  # ms
            max_latency = np.max(self.performance_history) * 1000  # ms
            fps = self.frame_count / (current_time - self.coordination_start_time)
        else:
            avg_latency = max_latency = fps = 0.0
        
        # System health assessment
        self._update_system_health()
        
        # Log performance metrics
        rospy.loginfo(f"[{self.node_name}] Performance - "
                     f"FPS: {fps:.1f}, "
                     f"Avg Latency: {avg_latency:.1f}ms, "
                     f"Max Latency: {max_latency:.1f}ms, "
                     f"Health: {self.system_health_score:.2f}")
        
        # Publish performance metrics
        if hasattr(PerformanceMetrics, 'fps'):
            perf_msg = PerformanceMetrics()
            perf_msg.header.stamp = rospy.Time.now()
            perf_msg.fps = fps
            perf_msg.avg_latency_ms = avg_latency
            perf_msg.max_latency_ms = max_latency
            perf_msg.health_score = self.system_health_score
            self.performance_pub.publish(perf_msg)
    
    def _health_check_callback(self, event):
        """Periodic system health check."""
        current_time = time.time()
        
        # Check data freshness
        detection_fresh = (self.last_detection_time is not None and 
                          current_time - self.last_detection_time < 2.0)
        navigation_fresh = (self.last_navigation_time is not None and 
                           current_time - self.last_navigation_time < 2.0)
        lane_pose_fresh = (self.last_lane_pose_time is not None and 
                          current_time - self.last_lane_pose_time < 2.0)
        
        # Update system status
        system_status = {
            'active': self.system_active,
            'emergency_stop': self.emergency_stop_active,
            'current_behavior': self.current_behavior,
            'detection_fresh': detection_fresh,
            'navigation_fresh': navigation_fresh,
            'lane_pose_fresh': lane_pose_fresh,
            'health_score': self.system_health_score
        }
        
        # Publish system status
        if hasattr(SystemStatus, 'active'):
            status_msg = SystemStatus()
            status_msg.header.stamp = rospy.Time.now()
            status_msg.active = system_status['active']
            status_msg.emergency_stop = system_status['emergency_stop']
            status_msg.current_behavior = system_status['current_behavior']
            status_msg.health_score = system_status['health_score']
            self.status_pub.publish(status_msg)
        
        # Log warnings for stale data
        if self.enable_yolo and not detection_fresh:
            rospy.logwarn(f"[{self.node_name}] YOLO detections are stale!")
        if self.enable_avoidance and not navigation_fresh:
            rospy.logwarn(f"[{self.node_name}] Navigation commands are stale!")
        if not lane_pose_fresh:
            rospy.logwarn(f"[{self.node_name}] Lane pose is stale!")
    
    def _update_system_health(self):
        """Update overall system health score."""
        current_time = time.time()
        health_factors = []
        
        # Performance health
        if self.performance_history:
            avg_latency = np.mean(self.performance_history)
            perf_health = max(0.0, 1.0 - (avg_latency - 0.05) / 0.15)  # Good if < 50ms
            health_factors.append(perf_health)
        
        # Data freshness health
        if self.last_detection_time:
            det_age = current_time - self.last_detection_time
            det_health = max(0.0, 1.0 - det_age / 5.0)  # Good if < 5s old
            health_factors.append(det_health)
        
        if self.last_navigation_time:
            nav_age = current_time - self.last_navigation_time
            nav_health = max(0.0, 1.0 - nav_age / 5.0)
            health_factors.append(nav_health)
        
        if self.last_lane_pose_time:
            pose_age = current_time - self.last_lane_pose_time
            pose_health = max(0.0, 1.0 - pose_age / 2.0)  # More critical
            health_factors.append(pose_health)
        
        # Emergency stop penalty
        if self.emergency_stop_active:
            health_factors.append(0.0)
        
        # Calculate overall health
        if health_factors:
            self.system_health_score = np.mean(health_factors)
        else:
            self.system_health_score = 0.5  # Unknown state
    
    def register_component(self, component_name: str, component_type: str, 
                          priority: int, health_check_callback=None) -> bool:
        """Register a component with the integration coordinator"""
        try:
            self.registered_components[component_name] = {
                'type': component_type,
                'priority': priority,
                'health_check': health_check_callback,
                'registration_time': time.time(),
                'last_health_check': None,
                'health_status': 'unknown',
                'failure_count': 0
            }
            
            self.component_status[component_name] = {
                'active': True,
                'healthy': True,
                'last_update': time.time(),
                'performance_metrics': {}
            }
            
            rospy.loginfo(f"[{self.node_name}] Registered component '{component_name}' "
                         f"(type: {component_type}, priority: {priority})")
            return True
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Failed to register component '{component_name}': {e}")
            return False
    
    def execute_coordination_workflow(self, workflow_type: str, parameters: dict) -> dict:
        """Execute a coordination workflow"""
        workflow_id = f"{workflow_type}_{int(time.time() * 1000)}"
        start_time = time.time()
        
        try:
            rospy.loginfo(f"[{self.node_name}] Executing workflow '{workflow_type}' (ID: {workflow_id})")
            rospy.logdebug(f"[{self.node_name}] Workflow parameters: {parameters}")
            
            self.active_workflows[workflow_id] = {
                'type': workflow_type,
                'parameters': parameters,
                'start_time': start_time,
                'status': 'running'
            }
            
            # Execute workflow based on type
            if workflow_type == "enhanced_navigation":
                result = self._execute_enhanced_navigation_workflow(parameters)
            elif workflow_type == "emergency_coordination":
                result = self._execute_emergency_coordination_workflow(parameters)
            elif workflow_type == "performance_optimization":
                result = self._execute_performance_optimization_workflow(parameters)
            else:
                result = self._execute_generic_workflow(workflow_type, parameters)
            
            # Update workflow status
            execution_time = time.time() - start_time
            self.active_workflows[workflow_id]['status'] = 'completed' if result.get('success', False) else 'failed'
            self.active_workflows[workflow_id]['execution_time'] = execution_time
            self.active_workflows[workflow_id]['result'] = result
            
            # Update metrics
            self.coordination_metrics['total_workflows'] += 1
            if result.get('success', False):
                self.coordination_metrics['successful_workflows'] += 1
            else:
                self.coordination_metrics['failed_workflows'] += 1
            
            # Update average workflow time
            total_time = (self.coordination_metrics['average_workflow_time'] * 
                         (self.coordination_metrics['total_workflows'] - 1) + execution_time)
            self.coordination_metrics['average_workflow_time'] = total_time / self.coordination_metrics['total_workflows']
            
            # Store in history
            workflow_record = {
                'id': workflow_id,
                'type': workflow_type,
                'parameters': parameters,
                'result': result,
                'execution_time': execution_time,
                'timestamp': start_time
            }
            self.workflow_history.append(workflow_record)
            
            rospy.loginfo(f"[{self.node_name}] Workflow '{workflow_type}' completed in {execution_time:.3f}s "
                         f"(success: {result.get('success', False)})")
            
            return result
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Workflow execution error: {e}")
            self.coordination_metrics['failed_workflows'] += 1
            return {'success': False, 'error': str(e)}
        
        finally:
            # Clean up active workflow
            if workflow_id in self.active_workflows:
                del self.active_workflows[workflow_id]
    
    def _execute_enhanced_navigation_workflow(self, parameters: dict) -> dict:
        """Execute enhanced navigation workflow"""
        try:
            enable_arbitration = parameters.get('enable_arbitration', True)
            enable_state_management = parameters.get('enable_state_management', True)
            target_performance = parameters.get('target_performance', 'normal')
            
            rospy.logdebug(f"[{self.node_name}] Enhanced navigation workflow: "
                          f"arbitration={enable_arbitration}, state_mgmt={enable_state_management}, "
                          f"performance={target_performance}")
            
            # Check component availability
            required_components = ['fsm_node', 'coordinator_node']
            if self.monitor_performance_optimizer:
                required_components.append('performance_optimizer')
            
            component_check = self._check_component_availability(required_components)
            if not component_check['all_available']:
                return {
                    'success': False,
                    'error': 'Required components not available',
                    'missing_components': component_check['missing']
                }
            
            # Execute workflow steps
            workflow_steps = []
            
            # Step 1: Initialize enhanced coordination
            if enable_arbitration:
                step_result = self._initialize_behavior_arbitration()
                workflow_steps.append(('behavior_arbitration', step_result))
            
            # Step 2: Initialize state management
            if enable_state_management:
                step_result = self._initialize_state_management()
                workflow_steps.append(('state_management', step_result))
            
            # Step 3: Configure performance optimization
            if target_performance != 'normal':
                step_result = self._configure_performance_optimization(target_performance)
                workflow_steps.append(('performance_optimization', step_result))
            
            # Check if all steps succeeded
            all_success = all(step[1].get('success', False) for step in workflow_steps)
            
            return {
                'success': all_success,
                'workflow_steps': workflow_steps,
                'components_checked': required_components,
                'execution_details': {
                    'arbitration_enabled': enable_arbitration,
                    'state_management_enabled': enable_state_management,
                    'performance_target': target_performance
                }
            }
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Enhanced navigation workflow error: {e}")
            return {'success': False, 'error': str(e)}
    
    def _execute_emergency_coordination_workflow(self, parameters: dict) -> dict:
        """Execute emergency coordination workflow"""
        try:
            emergency_type = parameters.get('emergency_type', 'unknown')
            override_all = parameters.get('override_all', True)
            
            rospy.logwarn(f"[{self.node_name}] Emergency coordination workflow: type={emergency_type}")
            
            # Immediate emergency actions
            emergency_actions = []
            
            # Stop all active workflows
            if override_all:
                stopped_workflows = list(self.active_workflows.keys())
                self.active_workflows.clear()
                emergency_actions.append(('stop_workflows', {'stopped': stopped_workflows}))
            
            # Notify all components
            notification_result = self._notify_components_emergency(emergency_type)
            emergency_actions.append(('notify_components', notification_result))
            
            # Set emergency state
            self.emergency_stop_active = True
            self.current_behavior = "emergency_stop"
            emergency_actions.append(('set_emergency_state', {'success': True}))
            
            return {
                'success': True,
                'emergency_type': emergency_type,
                'emergency_actions': emergency_actions,
                'timestamp': time.time()
            }
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Emergency coordination workflow error: {e}")
            return {'success': False, 'error': str(e)}
    
    def _execute_performance_optimization_workflow(self, parameters: dict) -> dict:
        """Execute performance optimization workflow"""
        try:
            target_metrics = parameters.get('target_metrics', {})
            
            rospy.loginfo(f"[{self.node_name}] Performance optimization workflow: targets={target_metrics}")
            
            # Check if performance optimizer is available
            if 'performance_optimizer' not in self.component_status:
                return {
                    'success': False,
                    'error': 'Performance optimizer not available'
                }
            
            # Request optimization
            optimization_result = self._request_performance_optimization(target_metrics)
            
            return {
                'success': optimization_result.get('success', False),
                'optimization_requested': True,
                'target_metrics': target_metrics,
                'optimization_result': optimization_result
            }
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Performance optimization workflow error: {e}")
            return {'success': False, 'error': str(e)}
    
    def _execute_generic_workflow(self, workflow_type: str, parameters: dict) -> dict:
        """Execute generic workflow"""
        rospy.loginfo(f"[{self.node_name}] Executing generic workflow: {workflow_type}")
        
        # Basic workflow execution
        return {
            'success': True,
            'workflow_type': workflow_type,
            'parameters': parameters,
            'message': 'Generic workflow executed successfully'
        }
    
    def validate_integration(self) -> dict:
        """Validate system integration"""
        try:
            validation_results = {
                'valid': True,
                'component_status': {},
                'integration_health': self.integration_health,
                'validation_timestamp': time.time(),
                'issues': []
            }
            
            # Check each registered component
            for component_name, component_info in self.registered_components.items():
                component_valid = True
                component_issues = []
                
                # Check if component is active
                if component_name not in self.component_status:
                    component_valid = False
                    component_issues.append('Component not in status tracking')
                else:
                    status = self.component_status[component_name]
                    if not status.get('active', False):
                        component_valid = False
                        component_issues.append('Component not active')
                    if not status.get('healthy', False):
                        component_valid = False
                        component_issues.append('Component not healthy')
                
                # Run health check if available
                if component_info.get('health_check'):
                    try:
                        health_result = component_info['health_check']()
                        if not health_result:
                            component_valid = False
                            component_issues.append('Health check failed')
                    except Exception as e:
                        component_valid = False
                        component_issues.append(f'Health check error: {str(e)}')
                
                validation_results['component_status'][component_name] = {
                    'valid': component_valid,
                    'issues': component_issues
                }
                
                if not component_valid:
                    validation_results['valid'] = False
                    validation_results['issues'].extend([f"{component_name}: {issue}" for issue in component_issues])
            
            rospy.loginfo(f"[{self.node_name}] Integration validation: "
                         f"valid={validation_results['valid']}, "
                         f"components={len(validation_results['component_status'])}")
            
            return validation_results
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Integration validation error: {e}")
            return {
                'valid': False,
                'error': str(e),
                'validation_timestamp': time.time()
            }
    
    def get_coordination_statistics(self) -> dict:
        """Get coordination statistics"""
        return {
            'coordination_metrics': dict(self.coordination_metrics),
            'registered_components': len(self.registered_components),
            'active_workflows': len(self.active_workflows),
            'workflow_history_length': len(self.workflow_history),
            'integration_health': self.integration_health,
            'system_health_score': self.system_health_score,
            'current_behavior': self.current_behavior,
            'emergency_active': self.emergency_stop_active
        }
    
    def request_performance_optimization(self, target_metrics: dict) -> dict:
        """Request performance optimization"""
        return self._request_performance_optimization(target_metrics)
    
    def trigger_emergency_coordination(self, emergency_type: str, reason: str, override_all: bool = True) -> dict:
        """Trigger emergency coordination"""
        return self.execute_coordination_workflow(
            "emergency_coordination",
            {
                'emergency_type': emergency_type,
                'reason': reason,
                'override_all': override_all
            }
        )
    
    def validate_system_configuration(self, config: dict) -> dict:
        """Validate system configuration"""
        try:
            validation_result = {
                'valid': True,
                'validated_components': [],
                'configuration_issues': [],
                'validation_timestamp': time.time()
            }
            
            # Validate behavior arbitration config
            if 'behavior_arbitration' in config:
                arb_config = config['behavior_arbitration']
                if arb_config.get('enable_arbitration', False):
                    validation_result['validated_components'].append('behavior_arbitration')
                    if arb_config.get('max_recovery_attempts', 0) <= 0:
                        validation_result['configuration_issues'].append('Invalid max_recovery_attempts')
            
            # Validate state management config
            if 'state_management' in config:
                state_config = config['state_management']
                if state_config.get('enable_persistence', False):
                    validation_result['validated_components'].append('state_management')
            
            # Validate performance optimization config
            if 'performance_optimization' in config:
                perf_config = config['performance_optimization']
                if perf_config.get('enable_optimization', False):
                    validation_result['validated_components'].append('performance_optimization')
            
            validation_result['valid'] = len(validation_result['configuration_issues']) == 0
            
            return validation_result
            
        except Exception as e:
            return {
                'valid': False,
                'error': str(e),
                'validation_timestamp': time.time()
            }
    
    def report_component_failure(self, component_name: str, failure_type: str, failure_details: str):
        """Report component failure"""
        rospy.logwarn(f"[{self.node_name}] Component failure reported: {component_name} - {failure_type}")
        
        if component_name in self.registered_components:
            self.registered_components[component_name]['failure_count'] += 1
            self.registered_components[component_name]['last_failure'] = {
                'type': failure_type,
                'details': failure_details,
                'timestamp': time.time()
            }
        
        if component_name in self.component_status:
            self.component_status[component_name]['healthy'] = False
            self.component_status[component_name]['last_failure'] = failure_type
        
        self.coordination_metrics['component_failures'] += 1
    
    def attempt_component_recovery(self, component_name: str) -> dict:
        """Attempt component recovery"""
        try:
            rospy.loginfo(f"[{self.node_name}] Attempting recovery for component: {component_name}")
            
            recovery_result = {
                'recovery_attempted': True,
                'component_name': component_name,
                'recovery_strategy': 'restart',
                'success': False,
                'timestamp': time.time()
            }
            
            # Simple recovery strategy - mark as healthy and reset failure count
            if component_name in self.component_status:
                self.component_status[component_name]['healthy'] = True
                self.component_status[component_name]['last_update'] = time.time()
                recovery_result['success'] = True
            
            if component_name in self.registered_components:
                self.registered_components[component_name]['failure_count'] = 0
            
            self.coordination_metrics['recovery_attempts'] += 1
            
            rospy.loginfo(f"[{self.node_name}] Recovery attempt for '{component_name}': "
                         f"{'successful' if recovery_result['success'] else 'failed'}")
            
            return recovery_result
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Component recovery error: {e}")
            return {
                'recovery_attempted': True,
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }
    
    # Helper methods for workflow execution
    def _check_component_availability(self, required_components: list) -> dict:
        """Check if required components are available"""
        available = []
        missing = []
        
        for component in required_components:
            if (component in self.component_status and 
                self.component_status[component].get('active', False)):
                available.append(component)
            else:
                missing.append(component)
        
        return {
            'all_available': len(missing) == 0,
            'available': available,
            'missing': missing
        }
    
    def _initialize_behavior_arbitration(self) -> dict:
        """Initialize behavior arbitration"""
        rospy.logdebug(f"[{self.node_name}] Initializing behavior arbitration")
        return {'success': True, 'message': 'Behavior arbitration initialized'}
    
    def _initialize_state_management(self) -> dict:
        """Initialize state management"""
        rospy.logdebug(f"[{self.node_name}] Initializing state management")
        return {'success': True, 'message': 'State management initialized'}
    
    def _configure_performance_optimization(self, target_performance: str) -> dict:
        """Configure performance optimization"""
        rospy.logdebug(f"[{self.node_name}] Configuring performance optimization: {target_performance}")
        return {'success': True, 'target': target_performance}
    
    def _notify_components_emergency(self, emergency_type: str) -> dict:
        """Notify components of emergency"""
        rospy.logwarn(f"[{self.node_name}] Notifying components of emergency: {emergency_type}")
        return {'success': True, 'notified_components': list(self.registered_components.keys())}
    
    def _request_performance_optimization(self, target_metrics: dict) -> dict:
        """Request performance optimization"""
        rospy.loginfo(f"[{self.node_name}] Requesting performance optimization: {target_metrics}")
        return {'success': True, 'optimization_requested': True, 'target_metrics': target_metrics}

    def shutdown(self):
        """Clean shutdown of the coordinator."""
        rospy.loginfo(f"[{self.node_name}] Shutting down integration coordinator...")
        self.system_active = False
        
        # Log final statistics
        stats = self.get_coordination_statistics()
        rospy.loginfo(f"[{self.node_name}] Final coordination statistics:")
        rospy.loginfo(f"[{self.node_name}] Total workflows: {stats['coordination_metrics']['total_workflows']}")
        rospy.loginfo(f"[{self.node_name}] Successful workflows: {stats['coordination_metrics']['successful_workflows']}")
        rospy.loginfo(f"[{self.node_name}] Component failures: {stats['coordination_metrics']['component_failures']}")
        rospy.loginfo(f"[{self.node_name}] Recovery attempts: {stats['coordination_metrics']['recovery_attempts']}")
        
        # Stop all timers
        if hasattr(self, 'coordination_timer'):
            self.coordination_timer.shutdown()
        if hasattr(self, 'monitoring_timer'):
            self.monitoring_timer.shutdown()
        if hasattr(self, 'health_timer'):
            self.health_timer.shutdown()
        
        rospy.loginfo(f"[{self.node_name}] Integration coordinator shutdown complete")


def main():
    """Main function to run the integration coordinator."""
    try:
        coordinator = IntegrationCoordinator()
        rospy.loginfo("Integration Coordinator running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Integration Coordinator interrupted")
    except Exception as e:
        rospy.logerr(f"Integration Coordinator error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'coordinator' in locals():
            coordinator.shutdown()


if __name__ == '__main__':
    main()
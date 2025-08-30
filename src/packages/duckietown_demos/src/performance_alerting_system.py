#!/usr/bin/env python3
"""
Performance Alerting System for Enhanced Autonomous Navigation

Provides comprehensive performance monitoring with intelligent alerting,
escalation, and notification capabilities for enhanced components.

Author: Duckietown
"""

import rospy
import time
import threading
import json
import smtplib
from email.mime.text import MimeText
from email.mime.multipart import MimeMultipart
from typing import Dict, List, Optional, Any, Callable
from collections import deque, defaultdict
from dataclasses import dataclass, asdict
from enum import Enum
import numpy as np

# ROS messages
from std_msgs.msg import String, Float32, Bool, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class AlertSeverity(Enum):
    """Alert severity levels"""
    INFO = 1
    WARNING = 2
    CRITICAL = 3
    EMERGENCY = 4


class AlertCategory(Enum):
    """Alert categories"""
    PERFORMANCE = "performance"
    RESOURCE = "resource"
    THERMAL = "thermal"
    COMPONENT = "component"
    SYSTEM = "system"
    SAFETY = "safety"


@dataclass
class PerformanceAlert:
    """Performance alert data structure"""
    alert_id: str
    timestamp: float
    severity: AlertSeverity
    category: AlertCategory
    component: str
    title: str
    description: str
    metrics: Dict[str, float]
    threshold_violated: str
    recommended_actions: List[str]
    escalation_level: int
    acknowledged: bool
    resolved: bool
    resolution_time: Optional[float]
    
    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data['severity'] = self.severity.name
        data['category'] = self.category.value
        return data


@dataclass
class AlertThreshold:
    """Alert threshold configuration"""
    metric_name: str
    warning_threshold: float
    critical_threshold: float
    emergency_threshold: Optional[float]
    comparison_operator: str  # '>', '<', '>=', '<=', '==', '!='
    duration_seconds: float  # How long threshold must be violated
    enabled: bool
    
    def check_violation(self, value: float, duration: float) -> Optional[AlertSeverity]:
        """Check if value violates threshold for required duration"""
        if not self.enabled or duration < self.duration_seconds:
            return None
        
        if self.emergency_threshold is not None:
            if self._compare(value, self.emergency_threshold):
                return AlertSeverity.EMERGENCY
        
        if self._compare(value, self.critical_threshold):
            return AlertSeverity.CRITICAL
        elif self._compare(value, self.warning_threshold):
            return AlertSeverity.WARNING
        
        return None
    
    def _compare(self, value: float, threshold: float) -> bool:
        """Compare value against threshold using specified operator"""
        if self.comparison_operator == '>':
            return value > threshold
        elif self.comparison_operator == '<':
            return value < threshold
        elif self.comparison_operator == '>=':
            return value >= threshold
        elif self.comparison_operator == '<=':
            return value <= threshold
        elif self.comparison_operator == '==':
            return abs(value - threshold) < 1e-6
        elif self.comparison_operator == '!=':
            return abs(value - threshold) >= 1e-6
        else:
            return False


class PerformanceAlertingSystem:
    """
    Comprehensive performance alerting system with intelligent escalation,
    notification, and resolution tracking capabilities.
    """
    
    def __init__(self):
        self.node_name = "performance_alerting_system"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Configuration
        self.monitoring_frequency = rospy.get_param("~monitoring_frequency", 2.0)  # Hz
        self.alert_retention_hours = rospy.get_param("~alert_retention_hours", 24)
        self.escalation_enabled = rospy.get_param("~escalation_enabled", True)
        self.email_notifications = rospy.get_param("~email_notifications", False)
        self.raspberry_pi_mode = rospy.get_param("~raspberry_pi_mode", True)
        
        # Email configuration (if enabled)
        if self.email_notifications:
            self.smtp_server = rospy.get_param("~smtp_server", "smtp.gmail.com")
            self.smtp_port = rospy.get_param("~smtp_port", 587)
            self.email_username = rospy.get_param("~email_username", "")
            self.email_password = rospy.get_param("~email_password", "")
            self.notification_recipients = rospy.get_param("~notification_recipients", [])
        
        # Alert storage
        self.active_alerts: Dict[str, PerformanceAlert] = {}
        self.alert_history: deque = deque(maxlen=1000)
        self.metric_history: Dict[str, deque] = defaultdict(lambda: deque(maxlen=100))
        self.threshold_violations: Dict[str, Dict] = defaultdict(dict)
        
        # Alert thresholds
        self.alert_thresholds: Dict[str, AlertThreshold] = {}
        self._setup_default_thresholds()
        
        # Escalation configuration
        self.escalation_rules = {
            AlertSeverity.WARNING: {'escalate_after_minutes': 10, 'max_level': 2},
            AlertSeverity.CRITICAL: {'escalate_after_minutes': 5, 'max_level': 3},
            AlertSeverity.EMERGENCY: {'escalate_after_minutes': 1, 'max_level': 5}
        }
        
        # Statistics
        self.total_alerts_generated = 0
        self.alerts_by_severity = defaultdict(int)
        self.alerts_by_category = defaultdict(int)
        self.alerts_resolved = 0
        self.average_resolution_time = 0.0
        
        # Threading
        self.alerting_active = True
        self.alerting_lock = threading.Lock()
        
        # Notification callbacks
        self.notification_callbacks: List[Callable] = []
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] Performance Alerting System initialized")
        rospy.loginfo(f"[{self.node_name}] Monitoring frequency: {self.monitoring_frequency} Hz")
        rospy.loginfo(f"[{self.node_name}] Alert retention: {self.alert_retention_hours} hours")
        rospy.loginfo(f"[{self.node_name}] Escalation enabled: {self.escalation_enabled}")
        rospy.loginfo(f"[{self.node_name}] Email notifications: {self.email_notifications}")
        rospy.loginfo(f"[{self.node_name}] Configured thresholds: {len(self.alert_thresholds)}")
    
    def _setup_default_thresholds(self):
        """Setup default alert thresholds for enhanced components"""
        # CPU thresholds
        cpu_warning = 75.0 if self.raspberry_pi_mode else 80.0
        cpu_critical = 90.0 if self.raspberry_pi_mode else 95.0
        cpu_emergency = 98.0
        
        self.alert_thresholds['cpu_percent'] = AlertThreshold(
            metric_name='cpu_percent',
            warning_threshold=cpu_warning,
            critical_threshold=cpu_critical,
            emergency_threshold=cpu_emergency,
            comparison_operator='>',
            duration_seconds=5.0,
            enabled=True
        )
        
        # Memory thresholds
        memory_warning = 70.0 if self.raspberry_pi_mode else 80.0
        memory_critical = 85.0 if self.raspberry_pi_mode else 90.0
        memory_emergency = 95.0
        
        self.alert_thresholds['memory_percent'] = AlertThreshold(
            metric_name='memory_percent',
            warning_threshold=memory_warning,
            critical_threshold=memory_critical,
            emergency_threshold=memory_emergency,
            comparison_operator='>',
            duration_seconds=10.0,
            enabled=True
        )
        
        # Temperature thresholds (Raspberry Pi)
        if self.raspberry_pi_mode:
            self.alert_thresholds['temperature_celsius'] = AlertThreshold(
                metric_name='temperature_celsius',
                warning_threshold=70.0,
                critical_threshold=80.0,
                emergency_threshold=85.0,
                comparison_operator='>',
                duration_seconds=30.0,
                enabled=True
            )
        
        # FPS thresholds
        fps_warning = 15.0 if self.raspberry_pi_mode else 20.0
        fps_critical = 10.0 if self.raspberry_pi_mode else 15.0
        fps_emergency = 5.0
        
        self.alert_thresholds['processing_fps'] = AlertThreshold(
            metric_name='processing_fps',
            warning_threshold=fps_warning,
            critical_threshold=fps_critical,
            emergency_threshold=fps_emergency,
            comparison_operator='<',
            duration_seconds=15.0,
            enabled=True
        )
        
        # Latency thresholds
        latency_warning = 250.0 if self.raspberry_pi_mode else 200.0
        latency_critical = 400.0 if self.raspberry_pi_mode else 300.0
        latency_emergency = 1000.0
        
        self.alert_thresholds['latency_ms'] = AlertThreshold(
            metric_name='latency_ms',
            warning_threshold=latency_warning,
            critical_threshold=latency_critical,
            emergency_threshold=latency_emergency,
            comparison_operator='>',
            duration_seconds=10.0,
            enabled=True
        )
        
        # Queue depth thresholds
        self.alert_thresholds['queue_depth'] = AlertThreshold(
            metric_name='queue_depth',
            warning_threshold=10.0,
            critical_threshold=20.0,
            emergency_threshold=50.0,
            comparison_operator='>',
            duration_seconds=20.0,
            enabled=True
        )
        
        # Error rate thresholds
        self.alert_thresholds['error_rate'] = AlertThreshold(
            metric_name='error_rate',
            warning_threshold=0.05,  # 5% error rate
            critical_threshold=0.15,  # 15% error rate
            emergency_threshold=0.30,  # 30% error rate
            comparison_operator='>',
            duration_seconds=30.0,
            enabled=True
        )
        
        # System efficiency thresholds
        self.alert_thresholds['system_efficiency'] = AlertThreshold(
            metric_name='system_efficiency',
            warning_threshold=0.7,
            critical_threshold=0.5,
            emergency_threshold=0.3,
            comparison_operator='<',
            duration_seconds=60.0,
            enabled=True
        )
        
        rospy.loginfo(f"[{self.node_name}] Configured {len(self.alert_thresholds)} default alert thresholds")
    
    def _setup_subscribers(self):
        """Setup subscribers for performance monitoring data"""
        rospy.loginfo(f"[{self.node_name}] Setting up performance monitoring subscribers...")
        
        # Enhanced performance profiler data
        self.profiler_sub = rospy.Subscriber(
            "/enhanced_performance_profiler/performance_profiles",
            String,
            self._performance_profile_callback,
            queue_size=10
        )
        
        # System performance monitor data
        self.system_sub = rospy.Subscriber(
            "/system_performance_monitor/system_health",
            String,
            self._system_health_callback,
            queue_size=1
        )
        
        # Bottleneck alerts from profiler
        self.bottleneck_sub = rospy.Subscriber(
            "/enhanced_performance_profiler/bottleneck_alerts",
            String,
            self._bottleneck_alert_callback,
            queue_size=10
        )
        
        # Component-specific performance data
        self.component_sub = rospy.Subscriber(
            "~component_metrics",
            String,
            self._component_metrics_callback,
            queue_size=10
        )
        
        # Alert acknowledgments
        self.ack_sub = rospy.Subscriber(
            "~alert_acknowledgment",
            String,
            self._alert_acknowledgment_callback,
            queue_size=10
        )
        
        rospy.loginfo(f"[{self.node_name}] Performance monitoring subscribers initialized")
    
    def _setup_publishers(self):
        """Setup publishers for alert notifications"""
        rospy.loginfo(f"[{self.node_name}] Setting up alert notification publishers...")
        
        # Active alerts
        self.alerts_pub = rospy.Publisher(
            "~active_alerts",
            String,
            queue_size=1
        )
        
        # New alert notifications
        self.new_alert_pub = rospy.Publisher(
            "~new_alert",
            String,
            queue_size=10
        )
        
        # Alert escalations
        self.escalation_pub = rospy.Publisher(
            "~alert_escalation",
            String,
            queue_size=10
        )
        
        # Alert resolutions
        self.resolution_pub = rospy.Publisher(
            "~alert_resolution",
            String,
            queue_size=10
        )
        
        # Alert statistics
        self.stats_pub = rospy.Publisher(
            "~alert_statistics",
            String,
            queue_size=1
        )
        
        # Emergency alerts (high priority)
        self.emergency_pub = rospy.Publisher(
            "~emergency_alert",
            String,
            queue_size=10,
            latch=True  # Latch emergency alerts
        )
        
        rospy.loginfo(f"[{self.node_name}] Alert notification publishers initialized")
    
    def _setup_timers(self):
        """Setup periodic monitoring and escalation timers"""
        # Main alerting timer
        self.alerting_timer = rospy.Timer(
            rospy.Duration(1.0 / self.monitoring_frequency),
            self._alerting_callback
        )
        
        # Escalation timer
        if self.escalation_enabled:
            self.escalation_timer = rospy.Timer(
                rospy.Duration(30.0),  # Check escalations every 30 seconds
                self._escalation_callback
            )
        
        # Cleanup timer
        self.cleanup_timer = rospy.Timer(
            rospy.Duration(300.0),  # Cleanup every 5 minutes
            self._cleanup_callback
        )
        
        # Statistics timer
        self.stats_timer = rospy.Timer(
            rospy.Duration(60.0),  # Publish stats every minute
            self._statistics_callback
        )
        
        rospy.loginfo(f"[{self.node_name}] Alert monitoring timers initialized")
    
    def _performance_profile_callback(self, msg):
        """Handle performance profile data from enhanced profiler"""
        try:
            profile_data = json.loads(msg.data)
            
            # Extract metrics for alerting
            metrics = {
                'component': profile_data.get('component_name', 'unknown'),
                'cpu_percent': profile_data.get('cpu_usage_percent', 0.0),
                'memory_mb': profile_data.get('memory_usage_mb', 0.0),
                'processing_fps': profile_data.get('fps', 0.0),
                'latency_ms': profile_data.get('latency_p50_ms', 0.0),
                'queue_depth': profile_data.get('queue_depth', 0),
                'error_count': profile_data.get('error_count', 0),
                'timestamp': time.time()
            }
            
            # Update metric history
            component = metrics['component']
            for metric_name, value in metrics.items():
                if metric_name not in ['component', 'timestamp'] and isinstance(value, (int, float)):
                    key = f"{component}_{metric_name}"
                    self.metric_history[key].append({'value': value, 'timestamp': metrics['timestamp']})
            
            # Check for threshold violations
            self._check_component_thresholds(component, metrics)
            
            rospy.logdebug(f"[{self.node_name}] Performance profile processed for '{component}': "
                          f"FPS={metrics['processing_fps']:.1f}, CPU={metrics['cpu_percent']:.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing performance profile: {e}")
    
    def _system_health_callback(self, msg):
        """Handle system health data"""
        try:
            if hasattr(msg, 'data'):
                health_data = json.loads(msg.data)
            else:
                health_data = {'overall_health': 1.0, 'timestamp': time.time()}
            
            # Extract system metrics
            metrics = {
                'system_efficiency': health_data.get('overall_health', 1.0),
                'cpu_percent': health_data.get('cpu_percent', 0.0),
                'memory_percent': health_data.get('memory_percent', 0.0),
                'temperature_celsius': health_data.get('temperature_celsius'),
                'timestamp': time.time()
            }
            
            # Update system metric history
            for metric_name, value in metrics.items():
                if metric_name != 'timestamp' and value is not None and isinstance(value, (int, float)):
                    key = f"system_{metric_name}"
                    self.metric_history[key].append({'value': value, 'timestamp': metrics['timestamp']})
            
            # Check system-level thresholds
            self._check_system_thresholds(metrics)
            
            rospy.logdebug(f"[{self.node_name}] System health processed: "
                          f"Efficiency={metrics['system_efficiency']:.2f}, "
                          f"CPU={metrics['cpu_percent']:.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing system health: {e}")
    
    def _bottleneck_alert_callback(self, msg):
        """Handle bottleneck alerts from performance profiler"""
        try:
            bottleneck_data = json.loads(msg.data)
            bottleneck = bottleneck_data.get('bottleneck', {})
            
            # Create high-priority alert for bottlenecks
            alert_id = f"bottleneck_{bottleneck.get('bottleneck_type', 'unknown')}_{int(time.time())}"
            
            alert = PerformanceAlert(
                alert_id=alert_id,
                timestamp=time.time(),
                severity=AlertSeverity.CRITICAL,
                category=AlertCategory.PERFORMANCE,
                component=', '.join(bottleneck.get('affected_components', ['system'])),
                title=f"Performance Bottleneck: {bottleneck.get('bottleneck_type', 'Unknown')}",
                description=bottleneck.get('impact_description', 'Performance bottleneck detected'),
                metrics={'severity': bottleneck.get('severity', 1.0)},
                threshold_violated=bottleneck.get('root_cause', 'Bottleneck detected'),
                recommended_actions=bottleneck.get('recommended_actions', []),
                escalation_level=0,
                acknowledged=False,
                resolved=False,
                resolution_time=None
            )
            
            self._generate_alert(alert)
            
            rospy.logwarn(f"[{self.node_name}] Bottleneck alert generated: {alert.title}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing bottleneck alert: {e}")
    
    def _component_metrics_callback(self, msg):
        """Handle component-specific metrics"""
        try:
            metrics_data = json.loads(msg.data)
            component = metrics_data.get('component', 'unknown')
            
            # Process custom component metrics
            for metric_name, value in metrics_data.items():
                if metric_name != 'component' and isinstance(value, (int, float)):
                    key = f"{component}_{metric_name}"
                    self.metric_history[key].append({'value': value, 'timestamp': time.time()})
            
            # Check custom thresholds if configured
            self._check_custom_thresholds(component, metrics_data)
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing component metrics: {e}")
    
    def _alert_acknowledgment_callback(self, msg):
        """Handle alert acknowledgments"""
        try:
            ack_data = json.loads(msg.data)
            alert_id = ack_data.get('alert_id')
            acknowledged_by = ack_data.get('acknowledged_by', 'system')
            
            if alert_id in self.active_alerts:
                with self.alerting_lock:
                    self.active_alerts[alert_id].acknowledged = True
                    
                rospy.loginfo(f"[{self.node_name}] Alert acknowledged: {alert_id} by {acknowledged_by}")
                
                # Comprehensive logging
                rospy.logdebug(f"[{self.node_name}] Alert acknowledgment event: {alert_id}, "
                              f"By={acknowledged_by}")
            else:
                rospy.logwarn(f"[{self.node_name}] Acknowledgment for unknown alert: {alert_id}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing alert acknowledgment: {e}")
    
    def _check_component_thresholds(self, component: str, metrics: Dict[str, Any]):
        """Check component metrics against configured thresholds"""
        current_time = time.time()
        
        for metric_name, value in metrics.items():
            if metric_name in ['component', 'timestamp'] or not isinstance(value, (int, float)):
                continue
            
            threshold_key = metric_name
            if threshold_key not in self.alert_thresholds:
                continue
            
            threshold = self.alert_thresholds[threshold_key]
            key = f"{component}_{metric_name}"
            
            # Check if threshold is violated
            if key not in self.threshold_violations:
                self.threshold_violations[key] = {'start_time': None, 'last_value': None}
            
            violation_info = self.threshold_violations[key]
            
            # Determine if currently violating threshold
            temp_severity = threshold.check_violation(value, 0)  # Check without duration
            is_violating = temp_severity is not None
            
            if is_violating:
                if violation_info['start_time'] is None:
                    violation_info['start_time'] = current_time
                
                violation_duration = current_time - violation_info['start_time']
                severity = threshold.check_violation(value, violation_duration)
                
                if severity is not None:
                    # Generate alert
                    alert_id = f"{component}_{metric_name}_{severity.name.lower()}"
                    
                    if alert_id not in self.active_alerts:
                        alert = self._create_threshold_alert(
                            alert_id, component, metric_name, value, threshold, severity
                        )
                        self._generate_alert(alert)
                
                violation_info['last_value'] = value
            else:
                # No longer violating, reset
                if violation_info['start_time'] is not None:
                    # Resolve any active alerts for this metric
                    alert_id_pattern = f"{component}_{metric_name}_"
                    for alert_id in list(self.active_alerts.keys()):
                        if alert_id.startswith(alert_id_pattern):
                            self._resolve_alert(alert_id, "Threshold no longer violated")
                
                violation_info['start_time'] = None
                violation_info['last_value'] = None
    
    def _check_system_thresholds(self, metrics: Dict[str, Any]):
        """Check system-level metrics against thresholds"""
        current_time = time.time()
        
        for metric_name, value in metrics.items():
            if metric_name == 'timestamp' or value is None or not isinstance(value, (int, float)):
                continue
            
            if metric_name not in self.alert_thresholds:
                continue
            
            threshold = self.alert_thresholds[metric_name]
            key = f"system_{metric_name}"
            
            # Check threshold violation
            if key not in self.threshold_violations:
                self.threshold_violations[key] = {'start_time': None, 'last_value': None}
            
            violation_info = self.threshold_violations[key]
            
            temp_severity = threshold.check_violation(value, 0)
            is_violating = temp_severity is not None
            
            if is_violating:
                if violation_info['start_time'] is None:
                    violation_info['start_time'] = current_time
                
                violation_duration = current_time - violation_info['start_time']
                severity = threshold.check_violation(value, violation_duration)
                
                if severity is not None:
                    alert_id = f"system_{metric_name}_{severity.name.lower()}"
                    
                    if alert_id not in self.active_alerts:
                        alert = self._create_threshold_alert(
                            alert_id, "system", metric_name, value, threshold, severity
                        )
                        self._generate_alert(alert)
                
                violation_info['last_value'] = value
            else:
                if violation_info['start_time'] is not None:
                    alert_id_pattern = f"system_{metric_name}_"
                    for alert_id in list(self.active_alerts.keys()):
                        if alert_id.startswith(alert_id_pattern):
                            self._resolve_alert(alert_id, "System threshold no longer violated")
                
                violation_info['start_time'] = None
                violation_info['last_value'] = None
    
    def _check_custom_thresholds(self, component: str, metrics: Dict[str, Any]):
        """Check custom component thresholds"""
        # Placeholder for custom threshold checking
        # Can be extended to support component-specific thresholds
        pass
    
    def _create_threshold_alert(self, alert_id: str, component: str, metric_name: str,
                              value: float, threshold: AlertThreshold, severity: AlertSeverity) -> PerformanceAlert:
        """Create an alert for threshold violation"""
        
        # Determine category based on metric
        if 'cpu' in metric_name or 'memory' in metric_name:
            category = AlertCategory.RESOURCE
        elif 'temperature' in metric_name:
            category = AlertCategory.THERMAL
        elif 'fps' in metric_name or 'latency' in metric_name:
            category = AlertCategory.PERFORMANCE
        else:
            category = AlertCategory.COMPONENT
        
        # Generate title and description
        title = f"{severity.name}: {metric_name.replace('_', ' ').title()} Threshold Violated"
        description = f"{component} {metric_name} is {value:.2f}, violating {severity.name.lower()} threshold of {self._get_violated_threshold(threshold, severity):.2f}"
        
        # Generate recommended actions based on metric
        recommended_actions = self._get_recommended_actions(metric_name, severity)
        
        return PerformanceAlert(
            alert_id=alert_id,
            timestamp=time.time(),
            severity=severity,
            category=category,
            component=component,
            title=title,
            description=description,
            metrics={metric_name: value},
            threshold_violated=f"{metric_name} {threshold.comparison_operator} {self._get_violated_threshold(threshold, severity)}",
            recommended_actions=recommended_actions,
            escalation_level=0,
            acknowledged=False,
            resolved=False,
            resolution_time=None
        )
    
    def _get_violated_threshold(self, threshold: AlertThreshold, severity: AlertSeverity) -> float:
        """Get the specific threshold value that was violated"""
        if severity == AlertSeverity.EMERGENCY and threshold.emergency_threshold is not None:
            return threshold.emergency_threshold
        elif severity == AlertSeverity.CRITICAL:
            return threshold.critical_threshold
        else:
            return threshold.warning_threshold
    
    def _get_recommended_actions(self, metric_name: str, severity: AlertSeverity) -> List[str]:
        """Get recommended actions based on metric and severity"""
        actions = []
        
        if 'cpu' in metric_name:
            actions.extend([
                "Reduce processing quality",
                "Enable frame skipping",
                "Optimize algorithms",
                "Check for runaway processes"
            ])
            if severity >= AlertSeverity.CRITICAL:
                actions.extend([
                    "Consider emergency stop",
                    "Restart high-CPU components"
                ])
        
        elif 'memory' in metric_name:
            actions.extend([
                "Clear caches",
                "Reduce buffer sizes",
                "Enable garbage collection",
                "Check for memory leaks"
            ])
            if severity >= AlertSeverity.CRITICAL:
                actions.extend([
                    "Restart memory-intensive components",
                    "Enable swap if available"
                ])
        
        elif 'temperature' in metric_name:
            actions.extend([
                "Reduce CPU-intensive operations",
                "Check cooling system",
                "Reduce processing quality",
                "Monitor ambient temperature"
            ])
            if severity >= AlertSeverity.CRITICAL:
                actions.extend([
                    "Implement thermal throttling",
                    "Consider emergency shutdown"
                ])
        
        elif 'fps' in metric_name:
            actions.extend([
                "Optimize processing pipeline",
                "Reduce image resolution",
                "Enable parallel processing",
                "Check system resources"
            ])
        
        elif 'latency' in metric_name:
            actions.extend([
                "Optimize critical path",
                "Enable asynchronous processing",
                "Reduce computational complexity",
                "Check system load"
            ])
        
        elif 'queue' in metric_name:
            actions.extend([
                "Increase processing rate",
                "Implement queue management",
                "Add load balancing",
                "Check for bottlenecks"
            ])
        
        return actions
    
    def _generate_alert(self, alert: PerformanceAlert):
        """Generate and process a new alert"""
        with self.alerting_lock:
            # Add to active alerts
            self.active_alerts[alert.alert_id] = alert
            
            # Add to history
            self.alert_history.append(alert)
            
            # Update statistics
            self.total_alerts_generated += 1
            self.alerts_by_severity[alert.severity] += 1
            self.alerts_by_category[alert.category] += 1
        
        # Publish new alert
        self._publish_new_alert(alert)
        
        # Send notifications
        self._send_notifications(alert)
        
        # Log alert generation
        rospy.logwarn(f"[{self.node_name}] ALERT GENERATED: {alert.title}")
        rospy.logwarn(f"[{self.node_name}] Component: {alert.component}, Severity: {alert.severity.name}")
        rospy.logwarn(f"[{self.node_name}] Description: {alert.description}")
        
        # Comprehensive logging
        rospy.logdebug(f"[{self.node_name}] Alert generation event:")
        rospy.logdebug(f"[{self.node_name}] - ID: {alert.alert_id}")
        rospy.logdebug(f"[{self.node_name}] - Severity: {alert.severity.name}")
        rospy.logdebug(f"[{self.node_name}] - Category: {alert.category.value}")
        rospy.logdebug(f"[{self.node_name}] - Component: {alert.component}")
        rospy.logdebug(f"[{self.node_name}] - Metrics: {alert.metrics}")
        rospy.logdebug(f"[{self.node_name}] - Actions: {alert.recommended_actions}")
        
        # Real-time monitoring
        rospy.logwarn(f"[{self.node_name}] Alert event: {alert.severity.name} - {alert.component}, "
                     f"Total alerts={self.total_alerts_generated}")
    
    def _resolve_alert(self, alert_id: str, resolution_reason: str = "Automatically resolved"):
        """Resolve an active alert"""
        if alert_id not in self.active_alerts:
            return
        
        with self.alerting_lock:
            alert = self.active_alerts[alert_id]
            alert.resolved = True
            alert.resolution_time = time.time()
            
            # Calculate resolution time
            resolution_duration = alert.resolution_time - alert.timestamp
            
            # Update average resolution time
            if self.alerts_resolved > 0:
                self.average_resolution_time = (
                    (self.average_resolution_time * self.alerts_resolved + resolution_duration) /
                    (self.alerts_resolved + 1)
                )
            else:
                self.average_resolution_time = resolution_duration
            
            self.alerts_resolved += 1
            
            # Remove from active alerts
            del self.active_alerts[alert_id]
        
        # Publish resolution
        self._publish_alert_resolution(alert, resolution_reason)
        
        rospy.loginfo(f"[{self.node_name}] Alert resolved: {alert.title}")
        rospy.loginfo(f"[{self.node_name}] Resolution time: {resolution_duration:.1f} seconds")
        rospy.loginfo(f"[{self.node_name}] Reason: {resolution_reason}")
        
        # Real-time monitoring
        rospy.loginfo(f"[{self.node_name}] Alert resolution event: {alert_id}, "
                     f"Duration={resolution_duration:.1f}s, Total resolved={self.alerts_resolved}")
    
    def _publish_new_alert(self, alert: PerformanceAlert):
        """Publish new alert notification"""
        alert_msg = String()
        alert_msg.data = json.dumps(alert.to_dict())
        self.new_alert_pub.publish(alert_msg)
        
        # Publish emergency alerts separately
        if alert.severity == AlertSeverity.EMERGENCY:
            emergency_msg = String()
            emergency_msg.data = json.dumps({
                'type': 'EMERGENCY_ALERT',
                'alert': alert.to_dict(),
                'timestamp': time.time()
            })
            self.emergency_pub.publish(emergency_msg)
    
    def _publish_alert_resolution(self, alert: PerformanceAlert, reason: str):
        """Publish alert resolution notification"""
        resolution_data = {
            'alert_id': alert.alert_id,
            'title': alert.title,
            'component': alert.component,
            'severity': alert.severity.name,
            'resolution_time': alert.resolution_time,
            'resolution_duration': alert.resolution_time - alert.timestamp,
            'resolution_reason': reason,
            'timestamp': time.time()
        }
        
        resolution_msg = String()
        resolution_msg.data = json.dumps(resolution_data)
        self.resolution_pub.publish(resolution_msg)
    
    def _send_notifications(self, alert: PerformanceAlert):
        """Send notifications for new alerts"""
        # Call registered notification callbacks
        for callback in self.notification_callbacks:
            try:
                callback(alert)
            except Exception as e:
                rospy.logerr(f"[{self.node_name}] Notification callback error: {e}")
        
        # Send email notifications if enabled
        if self.email_notifications and alert.severity >= AlertSeverity.CRITICAL:
            self._send_email_notification(alert)
    
    def _send_email_notification(self, alert: PerformanceAlert):
        """Send email notification for critical alerts"""
        if not self.notification_recipients:
            return
        
        try:
            # Create email message
            msg = MimeMultipart()
            msg['From'] = self.email_username
            msg['To'] = ', '.join(self.notification_recipients)
            msg['Subject'] = f"[DUCKIETOWN ALERT] {alert.severity.name}: {alert.title}"
            
            # Email body
            body = f"""
Duckietown Performance Alert

Alert ID: {alert.alert_id}
Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(alert.timestamp))}
Severity: {alert.severity.name}
Category: {alert.category.value}
Component: {alert.component}

Description:
{alert.description}

Metrics:
{json.dumps(alert.metrics, indent=2)}

Threshold Violated:
{alert.threshold_violated}

Recommended Actions:
{chr(10).join(f"- {action}" for action in alert.recommended_actions)}

This is an automated alert from the Duckietown Performance Monitoring System.
"""
            
            msg.attach(MimeText(body, 'plain'))
            
            # Send email
            server = smtplib.SMTP(self.smtp_server, self.smtp_port)
            server.starttls()
            server.login(self.email_username, self.email_password)
            text = msg.as_string()
            server.sendmail(self.email_username, self.notification_recipients, text)
            server.quit()
            
            rospy.loginfo(f"[{self.node_name}] Email notification sent for alert: {alert.alert_id}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Failed to send email notification: {e}")
    
    def _alerting_callback(self, event):
        """Main alerting callback - publish active alerts"""
        if not self.alerting_active:
            return
        
        try:
            with self.alerting_lock:
                active_alerts_data = {
                    'timestamp': time.time(),
                    'active_count': len(self.active_alerts),
                    'alerts': [alert.to_dict() for alert in self.active_alerts.values()]
                }
            
            # Publish active alerts
            alerts_msg = String()
            alerts_msg.data = json.dumps(active_alerts_data)
            self.alerts_pub.publish(alerts_msg)
            
            rospy.logdebug(f"[{self.node_name}] Published {len(self.active_alerts)} active alerts")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Alerting callback error: {e}")
    
    def _escalation_callback(self, event):
        """Handle alert escalations"""
        if not self.escalation_enabled:
            return
        
        try:
            current_time = time.time()
            escalated_alerts = []
            
            with self.alerting_lock:
                for alert in self.active_alerts.values():
                    if alert.acknowledged or alert.resolved:
                        continue
                    
                    # Check if escalation is needed
                    alert_age_minutes = (current_time - alert.timestamp) / 60.0
                    escalation_rule = self.escalation_rules.get(alert.severity)
                    
                    if escalation_rule and alert_age_minutes >= escalation_rule['escalate_after_minutes']:
                        if alert.escalation_level < escalation_rule['max_level']:
                            alert.escalation_level += 1
                            escalated_alerts.append(alert)
            
            # Publish escalations
            for alert in escalated_alerts:
                self._publish_escalation(alert)
                rospy.logwarn(f"[{self.node_name}] Alert escalated: {alert.title} (Level {alert.escalation_level})")
                
                # Real-time monitoring
                rospy.logwarn(f"[{self.node_name}] Escalation event: {alert.alert_id}, "
                             f"Level={alert.escalation_level}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Escalation callback error: {e}")
    
    def _publish_escalation(self, alert: PerformanceAlert):
        """Publish alert escalation notification"""
        escalation_data = {
            'alert_id': alert.alert_id,
            'title': alert.title,
            'component': alert.component,
            'severity': alert.severity.name,
            'escalation_level': alert.escalation_level,
            'age_minutes': (time.time() - alert.timestamp) / 60.0,
            'timestamp': time.time()
        }
        
        escalation_msg = String()
        escalation_msg.data = json.dumps(escalation_data)
        self.escalation_pub.publish(escalation_msg)
    
    def _cleanup_callback(self, event):
        """Cleanup old alerts and data"""
        try:
            current_time = time.time()
            retention_seconds = self.alert_retention_hours * 3600
            
            # Clean up old metric history
            for key in list(self.metric_history.keys()):
                history = self.metric_history[key]
                while history and (current_time - history[0]['timestamp']) > retention_seconds:
                    history.popleft()
                
                # Remove empty histories
                if not history:
                    del self.metric_history[key]
            
            # Clean up old threshold violations
            for key in list(self.threshold_violations.keys()):
                violation = self.threshold_violations[key]
                if (violation['start_time'] and 
                    (current_time - violation['start_time']) > retention_seconds):
                    del self.threshold_violations[key]
            
            rospy.logdebug(f"[{self.node_name}] Cleanup completed: "
                          f"Metric histories: {len(self.metric_history)}, "
                          f"Threshold violations: {len(self.threshold_violations)}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Cleanup callback error: {e}")
    
    def _statistics_callback(self, event):
        """Publish alert statistics"""
        try:
            with self.alerting_lock:
                stats_data = {
                    'timestamp': time.time(),
                    'total_alerts_generated': self.total_alerts_generated,
                    'active_alerts': len(self.active_alerts),
                    'alerts_resolved': self.alerts_resolved,
                    'average_resolution_time_seconds': self.average_resolution_time,
                    'alerts_by_severity': dict(self.alerts_by_severity),
                    'alerts_by_category': dict(self.alerts_by_category),
                    'configured_thresholds': len(self.alert_thresholds),
                    'monitored_metrics': len(self.metric_history)
                }
            
            # Publish statistics
            stats_msg = String()
            stats_msg.data = json.dumps(stats_data)
            self.stats_pub.publish(stats_msg)
            
            rospy.logdebug(f"[{self.node_name}] Alert statistics: "
                          f"Total={self.total_alerts_generated}, "
                          f"Active={len(self.active_alerts)}, "
                          f"Resolved={self.alerts_resolved}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Statistics callback error: {e}")
    
    def register_notification_callback(self, callback: Callable[[PerformanceAlert], None]):
        """Register a callback for alert notifications"""
        self.notification_callbacks.append(callback)
        rospy.loginfo(f"[{self.node_name}] Registered notification callback")
    
    def add_custom_threshold(self, threshold: AlertThreshold) -> bool:
        """Add a custom alert threshold"""
        try:
            self.alert_thresholds[threshold.metric_name] = threshold
            rospy.loginfo(f"[{self.node_name}] Added custom threshold for '{threshold.metric_name}'")
            return True
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error adding custom threshold: {e}")
            return False
    
    def get_alert_statistics(self) -> Dict[str, Any]:
        """Get comprehensive alert statistics"""
        with self.alerting_lock:
            return {
                'timestamp': time.time(),
                'total_alerts_generated': self.total_alerts_generated,
                'active_alerts': len(self.active_alerts),
                'alerts_resolved': self.alerts_resolved,
                'average_resolution_time_seconds': self.average_resolution_time,
                'alerts_by_severity': dict(self.alerts_by_severity),
                'alerts_by_category': dict(self.alerts_by_category),
                'configured_thresholds': len(self.alert_thresholds),
                'monitored_metrics': len(self.metric_history),
                'escalation_enabled': self.escalation_enabled,
                'email_notifications': self.email_notifications,
                'raspberry_pi_mode': self.raspberry_pi_mode
            }
    
    def shutdown(self):
        """Shutdown the performance alerting system"""
        rospy.loginfo(f"[{self.node_name}] Performance Alerting System shutting down")
        
        self.alerting_active = False
        
        # Shutdown timers
        if hasattr(self, 'alerting_timer'):
            self.alerting_timer.shutdown()
        if hasattr(self, 'escalation_timer'):
            self.escalation_timer.shutdown()
        if hasattr(self, 'cleanup_timer'):
            self.cleanup_timer.shutdown()
        if hasattr(self, 'stats_timer'):
            self.stats_timer.shutdown()
        
        # Log final statistics
        stats = self.get_alert_statistics()
        rospy.loginfo(f"[{self.node_name}] Final alert statistics:")
        rospy.loginfo(f"[{self.node_name}] Total alerts: {stats['total_alerts_generated']}")
        rospy.loginfo(f"[{self.node_name}] Active alerts: {stats['active_alerts']}")
        rospy.loginfo(f"[{self.node_name}] Resolved alerts: {stats['alerts_resolved']}")
        rospy.loginfo(f"[{self.node_name}] Average resolution time: {stats['average_resolution_time_seconds']:.1f}s")
        rospy.loginfo(f"[{self.node_name}] Monitored metrics: {stats['monitored_metrics']}")
        
        rospy.loginfo(f"[{self.node_name}] Performance Alerting System shutdown complete")


def main():
    """Main function to run the performance alerting system"""
    try:
        alerting_system = PerformanceAlertingSystem()
        rospy.loginfo("Performance Alerting System running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Performance Alerting System interrupted")
    except Exception as e:
        rospy.logerr(f"Performance Alerting System error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
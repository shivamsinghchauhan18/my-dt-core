#!/usr/bin/env python3
"""
Safety Status Publisher for comprehensive safety monitoring integration.
Publishes safety status to multiple ROS topics and integrates with visualization tools.
"""

import time
import rospy
from duckietown_msgs.msg import SafetyStatus, FSMState
from std_msgs.msg import String, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class SafetyStatusPublisher:
    """
    Publisher for safety status information with visualization integration.
    Provides comprehensive safety status publishing and visualization support.
    """
    
    def __init__(self, node_name):
        self.node_name = node_name
        
        # Safety status publishing parameters
        self.publish_rate = rospy.get_param("~safety_publish_rate", 1.0)  # Hz
        self.enable_diagnostics = rospy.get_param("~enable_diagnostics", True)
        self.enable_visualization = rospy.get_param("~enable_visualization", True)
        self.enable_system_health = rospy.get_param("~enable_system_health", True)
        
        # Safety status tracking
        self.last_safety_status = None
        self.safety_status_history = []
        self.max_history_length = rospy.get_param("~max_history_length", 100)
        self.safety_event_count = 0
        self.last_publish_time = time.time()
        
        # Publishers for safety status
        self.pub_safety_status = rospy.Publisher(
            "~safety_status", SafetyStatus, queue_size=1, latch=True
        )
        
        # Diagnostic status publisher
        if self.enable_diagnostics:
            self.pub_diagnostics = rospy.Publisher(
                "/diagnostics", DiagnosticArray, queue_size=1
            )
        
        # System health publishers
        if self.enable_system_health:
            self.pub_system_health = rospy.Publisher(
                "~system_health_score", Float32, queue_size=1
            )
            self.pub_emergency_status = rospy.Publisher(
                "~emergency_active", Bool, queue_size=1
            )
            self.pub_safety_level = rospy.Publisher(
                "~safety_level", String, queue_size=1
            )
        
        # Visualization publishers
        if self.enable_visualization:
            self.pub_safety_markers = rospy.Publisher(
                "~safety_visualization", MarkerArray, queue_size=1
            )
            self.pub_health_indicator = rospy.Publisher(
                "~health_indicator", Marker, queue_size=1
            )
        
        # Safety status timer
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.cb_publish_timer
        )
        
        rospy.loginfo(f"[{self.node_name}] Safety Status Publisher initialized")
        rospy.loginfo(f"[{self.node_name}] Publish rate: {self.publish_rate}Hz")
        rospy.loginfo(f"[{self.node_name}] Diagnostics: {self.enable_diagnostics}")
        rospy.loginfo(f"[{self.node_name}] Visualization: {self.enable_visualization}")
        rospy.loginfo(f"[{self.node_name}] System health: {self.enable_system_health}")
    
    def publish_safety_status(self, safety_status):
        """
        Publish comprehensive safety status information.
        
        Args:
            safety_status: SafetyStatus message to publish
        """
        if not safety_status:
            return
        
        current_time = time.time()
        self.last_safety_status = safety_status
        self.safety_event_count += 1
        
        # Add to history
        self.safety_status_history.append({
            'timestamp': current_time,
            'status': safety_status,
            'event_count': self.safety_event_count
        })
        
        # Trim history
        if len(self.safety_status_history) > self.max_history_length:
            self.safety_status_history = self.safety_status_history[-self.max_history_length:]
        
        # Publish main safety status
        self.pub_safety_status.publish(safety_status)
        
        # Publish diagnostic information
        if self.enable_diagnostics:
            self._publish_diagnostics(safety_status)
        
        # Publish system health information
        if self.enable_system_health:
            self._publish_system_health(safety_status)
        
        # Publish visualization
        if self.enable_visualization:
            self._publish_visualization(safety_status)
        
        # Log safety status publishing
        rospy.logdebug(f"[{self.node_name}] Safety status published: Level={safety_status.safety_level}, Score={safety_status.system_health_score:.1f}")
        
        # Real-time monitoring
        publish_interval = current_time - self.last_publish_time
        rospy.logdebug(f"[{self.node_name}] Safety status publishing event: Interval={publish_interval:.3f}s, Event count={self.safety_event_count}")
        
        self.last_publish_time = current_time
    
    def _publish_diagnostics(self, safety_status):
        """Publish diagnostic information for safety monitoring."""
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = rospy.Time.now()
        
        # Overall safety diagnostic
        overall_diag = DiagnosticStatus()
        overall_diag.name = f"{self.node_name}/safety_monitor"
        overall_diag.hardware_id = "safety_system"
        
        # Set diagnostic level based on safety status
        if safety_status.safety_level == SafetyStatus.SAFE:
            overall_diag.level = DiagnosticStatus.OK
            overall_diag.message = "Safety system operating normally"
        elif safety_status.safety_level == SafetyStatus.WARNING:
            overall_diag.level = DiagnosticStatus.WARN
            overall_diag.message = f"Safety warnings active: {len(safety_status.active_warnings)}"
        elif safety_status.safety_level == SafetyStatus.CRITICAL:
            overall_diag.level = DiagnosticStatus.ERROR
            overall_diag.message = f"Critical safety conditions detected"
        else:  # EMERGENCY
            overall_diag.level = DiagnosticStatus.ERROR
            overall_diag.message = f"EMERGENCY: {safety_status.emergency_reason}"
        
        # Add diagnostic values
        overall_diag.values = [
            KeyValue("safety_level", str(safety_status.safety_level)),
            KeyValue("system_health_score", f"{safety_status.system_health_score:.1f}"),
            KeyValue("emergency_stop_active", str(safety_status.emergency_stop_active)),
            KeyValue("active_warnings", str(len(safety_status.active_warnings))),
            KeyValue("cpu_temperature", f"{safety_status.cpu_temperature:.1f}°C"),
            KeyValue("memory_usage", f"{safety_status.memory_usage:.1f}%"),
            KeyValue("lane_detection_confidence", f"{safety_status.lane_detection_confidence:.2f}"),
            KeyValue("object_detection_fps", f"{safety_status.object_detection_fps:.1f}"),
            KeyValue("control_loop_frequency", f"{safety_status.control_loop_frequency:.1f}Hz")
        ]
        
        diagnostic_array.status.append(overall_diag)
        
        # Hardware diagnostic
        hardware_diag = DiagnosticStatus()
        hardware_diag.name = f"{self.node_name}/hardware_health"
        hardware_diag.hardware_id = "system_hardware"
        
        if safety_status.hardware_health == SafetyStatus.HEALTH_OK:
            hardware_diag.level = DiagnosticStatus.OK
            hardware_diag.message = "Hardware health normal"
        elif safety_status.hardware_health == SafetyStatus.HEALTH_WARNING:
            hardware_diag.level = DiagnosticStatus.WARN
            hardware_diag.message = "Hardware health warnings"
        else:  # HEALTH_CRITICAL
            hardware_diag.level = DiagnosticStatus.ERROR
            hardware_diag.message = "Critical hardware conditions"
        
        hardware_diag.values = [
            KeyValue("hardware_health", str(safety_status.hardware_health)),
            KeyValue("cpu_temperature", f"{safety_status.cpu_temperature:.1f}°C"),
            KeyValue("memory_usage", f"{safety_status.memory_usage:.1f}%")
        ]
        
        diagnostic_array.status.append(hardware_diag)
        
        # Sensor diagnostic
        sensor_diag = DiagnosticStatus()
        sensor_diag.name = f"{self.node_name}/sensor_health"
        sensor_diag.hardware_id = "sensor_system"
        
        sensor_failures = 0
        if safety_status.camera_status == SafetyStatus.SENSOR_FAILED:
            sensor_failures += 1
        if safety_status.imu_status == SafetyStatus.SENSOR_FAILED:
            sensor_failures += 1
        if safety_status.encoder_status == SafetyStatus.SENSOR_FAILED:
            sensor_failures += 1
        
        if sensor_failures == 0:
            sensor_diag.level = DiagnosticStatus.OK
            sensor_diag.message = "All sensors operational"
        elif sensor_failures == 1:
            sensor_diag.level = DiagnosticStatus.WARN
            sensor_diag.message = f"One sensor failure detected"
        else:
            sensor_diag.level = DiagnosticStatus.ERROR
            sensor_diag.message = f"Multiple sensor failures: {sensor_failures}"
        
        sensor_diag.values = [
            KeyValue("camera_status", str(safety_status.camera_status)),
            KeyValue("imu_status", str(safety_status.imu_status)),
            KeyValue("encoder_status", str(safety_status.encoder_status)),
            KeyValue("sensor_failures", str(sensor_failures))
        ]
        
        diagnostic_array.status.append(sensor_diag)
        
        # Publish diagnostics
        self.pub_diagnostics.publish(diagnostic_array)
        
        rospy.logdebug(f"[{self.node_name}] Diagnostic information published: {len(diagnostic_array.status)} diagnostics")
    
    def _publish_system_health(self, safety_status):
        """Publish system health information."""
        # System health score
        health_score_msg = Float32()
        health_score_msg.data = safety_status.system_health_score
        self.pub_system_health.publish(health_score_msg)
        
        # Emergency status
        emergency_msg = Bool()
        emergency_msg.data = safety_status.emergency_stop_active
        self.pub_emergency_status.publish(emergency_msg)
        
        # Safety level
        safety_level_names = {
            SafetyStatus.SAFE: "SAFE",
            SafetyStatus.WARNING: "WARNING",
            SafetyStatus.CRITICAL: "CRITICAL",
            SafetyStatus.EMERGENCY: "EMERGENCY"
        }
        
        level_msg = String()
        level_msg.data = safety_level_names.get(safety_status.safety_level, "UNKNOWN")
        self.pub_safety_level.publish(level_msg)
        
        rospy.logdebug(f"[{self.node_name}] System health published: Score={safety_status.system_health_score:.1f}, Level={level_msg.data}")
    
    def _publish_visualization(self, safety_status):
        """Publish visualization markers for safety status."""
        current_time = rospy.Time.now()
        
        # Safety status marker array
        marker_array = MarkerArray()
        
        # Health indicator marker
        health_marker = Marker()
        health_marker.header.frame_id = "base_link"
        health_marker.header.stamp = current_time
        health_marker.ns = "safety_health"
        health_marker.id = 0
        health_marker.type = Marker.SPHERE
        health_marker.action = Marker.ADD
        
        # Position above robot
        health_marker.pose.position.x = 0.0
        health_marker.pose.position.y = 0.0
        health_marker.pose.position.z = 0.5
        health_marker.pose.orientation.w = 1.0
        
        # Size based on health score
        health_score_normalized = safety_status.system_health_score / 100.0
        health_marker.scale.x = 0.1 + (health_score_normalized * 0.2)
        health_marker.scale.y = 0.1 + (health_score_normalized * 0.2)
        health_marker.scale.z = 0.1 + (health_score_normalized * 0.2)
        
        # Color based on safety level
        if safety_status.safety_level == SafetyStatus.SAFE:
            health_marker.color.r = 0.0
            health_marker.color.g = 1.0
            health_marker.color.b = 0.0
        elif safety_status.safety_level == SafetyStatus.WARNING:
            health_marker.color.r = 1.0
            health_marker.color.g = 1.0
            health_marker.color.b = 0.0
        elif safety_status.safety_level == SafetyStatus.CRITICAL:
            health_marker.color.r = 1.0
            health_marker.color.g = 0.5
            health_marker.color.b = 0.0
        else:  # EMERGENCY
            health_marker.color.r = 1.0
            health_marker.color.g = 0.0
            health_marker.color.b = 0.0
        
        health_marker.color.a = 0.8
        health_marker.lifetime = rospy.Duration(2.0)
        
        marker_array.markers.append(health_marker)
        
        # Warning indicators
        for i, warning in enumerate(safety_status.active_warnings[:5]):  # Show up to 5 warnings
            warning_marker = Marker()
            warning_marker.header.frame_id = "base_link"
            warning_marker.header.stamp = current_time
            warning_marker.ns = "safety_warnings"
            warning_marker.id = i + 1
            warning_marker.type = Marker.TEXT_VIEW_FACING
            warning_marker.action = Marker.ADD
            
            # Position warnings around robot
            angle = (i * 2.0 * 3.14159) / 5.0
            warning_marker.pose.position.x = 0.3 * cos(angle)
            warning_marker.pose.position.y = 0.3 * sin(angle)
            warning_marker.pose.position.z = 0.3 + (i * 0.1)
            warning_marker.pose.orientation.w = 1.0
            
            warning_marker.scale.z = 0.05
            warning_marker.color.r = 1.0
            warning_marker.color.g = 0.0
            warning_marker.color.b = 0.0
            warning_marker.color.a = 1.0
            
            # Truncate warning text
            warning_text = warning[:20] + "..." if len(warning) > 20 else warning
            warning_marker.text = warning_text
            warning_marker.lifetime = rospy.Duration(5.0)
            
            marker_array.markers.append(warning_marker)
        
        # Publish visualization
        self.pub_safety_markers.publish(marker_array)
        self.pub_health_indicator.publish(health_marker)
        
        rospy.logdebug(f"[{self.node_name}] Safety visualization published: {len(marker_array.markers)} markers")
    
    def cb_publish_timer(self, event):
        """Timer callback for periodic safety status publishing."""
        if self.last_safety_status:
            # Update timestamp
            self.last_safety_status.header.stamp = rospy.Time.now()
            
            # Republish for latch behavior
            self.pub_safety_status.publish(self.last_safety_status)
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Periodic safety status update: Timestamp={rospy.Time.now()}")
    
    def get_safety_statistics(self):
        """Get safety status statistics."""
        if not self.safety_status_history:
            return {}
        
        recent_history = self.safety_status_history[-10:]  # Last 10 events
        
        # Calculate statistics
        avg_health_score = sum(h['status'].system_health_score for h in recent_history) / len(recent_history)
        
        safety_levels = [h['status'].safety_level for h in recent_history]
        emergency_count = safety_levels.count(SafetyStatus.EMERGENCY)
        critical_count = safety_levels.count(SafetyStatus.CRITICAL)
        warning_count = safety_levels.count(SafetyStatus.WARNING)
        
        return {
            'total_events': self.safety_event_count,
            'history_length': len(self.safety_status_history),
            'avg_health_score': avg_health_score,
            'recent_emergency_count': emergency_count,
            'recent_critical_count': critical_count,
            'recent_warning_count': warning_count,
            'last_publish_time': self.last_publish_time
        }
    
    def shutdown(self):
        """Shutdown safety status publisher."""
        rospy.loginfo(f"[{self.node_name}] Shutting down Safety Status Publisher")
        
        # Stop timer
        if hasattr(self, 'publish_timer'):
            self.publish_timer.shutdown()
        
        # Log final statistics
        stats = self.get_safety_statistics()
        rospy.loginfo(f"[{self.node_name}] Final safety statistics:")
        rospy.loginfo(f"[{self.node_name}] Total events: {stats.get('total_events', 0)}")
        rospy.loginfo(f"[{self.node_name}] Average health score: {stats.get('avg_health_score', 0.0):.1f}")
        
        rospy.loginfo(f"[{self.node_name}] Safety Status Publisher shutdown complete")


# Helper function for cos calculation (simplified)
def cos(angle):
    """Simple cosine approximation."""
    import math
    return math.cos(angle)


def sin(angle):
    """Simple sine approximation."""
    import math
    return math.sin(angle)
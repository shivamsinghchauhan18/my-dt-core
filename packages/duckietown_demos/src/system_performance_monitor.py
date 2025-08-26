#!/usr/bin/env python3
"""
System Performance Monitor for Enhanced Autonomous Navigation

Monitors performance across all integrated components and provides
comprehensive system health reporting and alerting.

Author: Duckietown
"""

import rospy
import psutil
import time
import numpy as np
from threading import Lock
from collections import deque
from typing import Dict, List, Optional

# ROS messages
from std_msgs.msg import String, Float32, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Custom message types (fallback to basic types if not available)
try:
    from duckietown_msgs.msg import PerformanceMetrics, SystemHealthStatus
except ImportError:
    rospy.logwarn("Custom performance messages not available, using basic types")
    PerformanceMetrics = String
    SystemHealthStatus = String


class SystemPerformanceMonitor:
    """
    Monitors system performance across all integrated components.
    
    Tracks:
    - CPU and memory usage
    - Component-specific performance metrics
    - System health indicators
    - Performance alerts and warnings
    - Resource utilization trends
    """
    
    def __init__(self):
        """Initialize the system performance monitor."""
        self.node_name = "system_performance_monitor"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Thread safety
        self.lock = Lock()
        
        # Configuration
        self.monitoring_frequency = rospy.get_param("~monitoring_frequency", 2.0)  # Hz
        self.alert_thresholds = {
            'cpu_percent': rospy.get_param("~cpu_alert_threshold", 80.0),
            'memory_percent': rospy.get_param("~memory_alert_threshold", 85.0),
            'latency_ms': rospy.get_param("~latency_alert_threshold", 200.0),
            'fps_min': rospy.get_param("~fps_alert_threshold", 15.0)
        }
        
        # Performance data storage
        self.performance_data = {
            'yolo': deque(maxlen=100),
            'navigation': deque(maxlen=100),
            'integration': deque(maxlen=100)
        }
        
        # System metrics
        self.system_metrics = deque(maxlen=100)
        self.alert_history = deque(maxlen=50)
        
        # Component status
        self.component_status = {
            'yolo_detector': {'active': False, 'last_update': None},
            'navigation': {'active': False, 'last_update': None},
            'integration': {'active': False, 'last_update': None}
        }
        
        # Health scoring
        self.overall_health_score = 1.0
        self.component_health_scores = {}
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] System Performance Monitor initialized")
        rospy.loginfo(f"[{self.node_name}] Monitoring frequency: {self.monitoring_frequency} Hz")
        rospy.loginfo(f"[{self.node_name}] Alert thresholds: {self.alert_thresholds}")
    
    def _setup_subscribers(self):
        """Setup subscribers for performance data from all components."""
        rospy.loginfo(f"[{self.node_name}] Setting up performance subscribers...")
        
        # YOLO detector performance
        self.yolo_perf_sub = rospy.Subscriber(
            "~yolo_performance",
            PerformanceMetrics,
            self._yolo_performance_callback,
            queue_size=1
        )
        
        # Navigation performance
        self.nav_perf_sub = rospy.Subscriber(
            "~navigation_performance",
            PerformanceMetrics,
            self._navigation_performance_callback,
            queue_size=1
        )
        
        # Integration coordinator performance
        self.integration_perf_sub = rospy.Subscriber(
            "~integration_performance",
            PerformanceMetrics,
            self._integration_performance_callback,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Performance subscribers initialized")
    
    def _setup_publishers(self):
        """Setup publishers for system health reporting."""
        rospy.loginfo(f"[{self.node_name}] Setting up health publishers...")
        
        # Overall system health
        self.health_pub = rospy.Publisher(
            "~system_health",
            SystemHealthStatus,
            queue_size=1
        )
        
        # Diagnostic information (ROS standard)
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics",
            DiagnosticArray,
            queue_size=1
        )
        
        # Performance alerts
        self.alerts_pub = rospy.Publisher(
            "~performance_alerts",
            String,
            queue_size=10
        )
        
        rospy.loginfo(f"[{self.node_name}] Health publishers initialized")
    
    def _setup_timers(self):
        """Setup periodic monitoring timers."""
        # Main monitoring timer
        self.monitoring_timer = rospy.Timer(
            rospy.Duration(1.0 / self.monitoring_frequency),
            self._monitoring_callback
        )
        
        # System metrics collection timer
        self.system_timer = rospy.Timer(
            rospy.Duration(1.0),  # 1 Hz for system metrics
            self._system_metrics_callback
        )
        
        # Health reporting timer
        self.health_timer = rospy.Timer(
            rospy.Duration(5.0),  # 0.2 Hz for health reports
            self._health_reporting_callback
        )
        
        rospy.loginfo(f"[{self.node_name}] Monitoring timers initialized")
    
    def _yolo_performance_callback(self, msg):
        """Handle YOLO detector performance data."""
        with self.lock:
            current_time = time.time()
            
            # Extract performance metrics
            if hasattr(msg, 'fps'):
                perf_data = {
                    'timestamp': current_time,
                    'fps': msg.fps,
                    'avg_latency_ms': msg.avg_latency_ms,
                    'max_latency_ms': msg.max_latency_ms,
                    'memory_usage_mb': getattr(msg, 'memory_usage_mb', 0),
                    'gpu_utilization': getattr(msg, 'gpu_utilization', 0)
                }
            else:
                # Fallback for basic message types
                perf_data = {
                    'timestamp': current_time,
                    'fps': 0,
                    'avg_latency_ms': 0,
                    'max_latency_ms': 0,
                    'memory_usage_mb': 0,
                    'gpu_utilization': 0
                }
            
            self.performance_data['yolo'].append(perf_data)
            self.component_status['yolo_detector']['active'] = True
            self.component_status['yolo_detector']['last_update'] = current_time
            
            # Check for performance alerts
            self._check_performance_alerts('yolo', perf_data)
            
            rospy.logdebug(f"[{self.node_name}] YOLO performance: "
                         f"FPS={perf_data['fps']:.1f}, "
                         f"Latency={perf_data['avg_latency_ms']:.1f}ms")
    
    def _navigation_performance_callback(self, msg):
        """Handle navigation system performance data."""
        with self.lock:
            current_time = time.time()
            
            # Extract performance metrics
            if hasattr(msg, 'fps'):
                perf_data = {
                    'timestamp': current_time,
                    'fps': msg.fps,
                    'avg_latency_ms': msg.avg_latency_ms,
                    'max_latency_ms': msg.max_latency_ms,
                    'risk_assessments_per_sec': getattr(msg, 'risk_assessments_per_sec', 0),
                    'avoidance_activations': getattr(msg, 'avoidance_activations', 0)
                }
            else:
                # Fallback for basic message types
                perf_data = {
                    'timestamp': current_time,
                    'fps': 0,
                    'avg_latency_ms': 0,
                    'max_latency_ms': 0,
                    'risk_assessments_per_sec': 0,
                    'avoidance_activations': 0
                }
            
            self.performance_data['navigation'].append(perf_data)
            self.component_status['navigation']['active'] = True
            self.component_status['navigation']['last_update'] = current_time
            
            # Check for performance alerts
            self._check_performance_alerts('navigation', perf_data)
            
            rospy.logdebug(f"[{self.node_name}] Navigation performance: "
                         f"FPS={perf_data['fps']:.1f}, "
                         f"Latency={perf_data['avg_latency_ms']:.1f}ms")
    
    def _integration_performance_callback(self, msg):
        """Handle integration coordinator performance data."""
        with self.lock:
            current_time = time.time()
            
            # Extract performance metrics
            if hasattr(msg, 'fps'):
                perf_data = {
                    'timestamp': current_time,
                    'fps': msg.fps,
                    'avg_latency_ms': msg.avg_latency_ms,
                    'max_latency_ms': msg.max_latency_ms,
                    'coordination_decisions_per_sec': getattr(msg, 'coordination_decisions_per_sec', 0),
                    'behavior_switches': getattr(msg, 'behavior_switches', 0)
                }
            else:
                # Fallback for basic message types
                perf_data = {
                    'timestamp': current_time,
                    'fps': 0,
                    'avg_latency_ms': 0,
                    'max_latency_ms': 0,
                    'coordination_decisions_per_sec': 0,
                    'behavior_switches': 0
                }
            
            self.performance_data['integration'].append(perf_data)
            self.component_status['integration']['active'] = True
            self.component_status['integration']['last_update'] = current_time
            
            # Check for performance alerts
            self._check_performance_alerts('integration', perf_data)
            
            rospy.logdebug(f"[{self.node_name}] Integration performance: "
                         f"FPS={perf_data['fps']:.1f}, "
                         f"Latency={perf_data['avg_latency_ms']:.1f}ms")
    
    def _monitoring_callback(self, event):
        """Main monitoring callback - analyze performance trends."""
        with self.lock:
            current_time = time.time()
            
            # Update component health scores
            self._update_component_health_scores()
            
            # Check for stale components
            self._check_component_staleness(current_time)
            
            # Calculate overall system health
            self._calculate_overall_health()
    
    def _system_metrics_callback(self, event):
        """Collect system-level metrics (CPU, memory, etc.)."""
        try:
            # Get system metrics
            cpu_percent = psutil.cpu_percent(interval=None)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            
            # Get process-specific metrics for ROS nodes
            ros_processes = []
            for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
                try:
                    if 'ros' in proc.info['name'].lower() or 'python' in proc.info['name'].lower():
                        ros_processes.append(proc.info)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            system_data = {
                'timestamp': time.time(),
                'cpu_percent': cpu_percent,
                'memory_percent': memory.percent,
                'memory_available_gb': memory.available / (1024**3),
                'disk_percent': disk.percent,
                'disk_free_gb': disk.free / (1024**3),
                'ros_process_count': len(ros_processes),
                'total_ros_cpu': sum(p.get('cpu_percent', 0) for p in ros_processes),
                'total_ros_memory': sum(p.get('memory_percent', 0) for p in ros_processes)
            }
            
            with self.lock:
                self.system_metrics.append(system_data)
            
            # Check for system-level alerts
            self._check_system_alerts(system_data)
            
            rospy.logdebug(f"[{self.node_name}] System metrics: "
                         f"CPU={cpu_percent:.1f}%, "
                         f"Memory={memory.percent:.1f}%, "
                         f"ROS processes={len(ros_processes)}")
            
        except Exception as e:
            rospy.logwarn(f"[{self.node_name}] Error collecting system metrics: {e}")
    
    def _health_reporting_callback(self, event):
        """Periodic health reporting and diagnostics publication."""
        with self.lock:
            # Publish system health status
            self._publish_health_status()
            
            # Publish ROS diagnostics
            self._publish_diagnostics()
            
            # Log health summary
            self._log_health_summary()
    
    def _check_performance_alerts(self, component: str, perf_data: Dict):
        """Check performance data against alert thresholds."""
        alerts = []
        
        # Check FPS
        if perf_data['fps'] < self.alert_thresholds['fps_min']:
            alerts.append(f"{component} FPS below threshold: {perf_data['fps']:.1f} < {self.alert_thresholds['fps_min']}")
        
        # Check latency
        if perf_data['avg_latency_ms'] > self.alert_thresholds['latency_ms']:
            alerts.append(f"{component} latency above threshold: {perf_data['avg_latency_ms']:.1f}ms > {self.alert_thresholds['latency_ms']}ms")
        
        # Publish alerts
        for alert in alerts:
            self._publish_alert(f"PERFORMANCE_ALERT", alert)
            rospy.logwarn(f"[{self.node_name}] {alert}")
    
    def _check_system_alerts(self, system_data: Dict):
        """Check system metrics against alert thresholds."""
        alerts = []
        
        # Check CPU usage
        if system_data['cpu_percent'] > self.alert_thresholds['cpu_percent']:
            alerts.append(f"High CPU usage: {system_data['cpu_percent']:.1f}% > {self.alert_thresholds['cpu_percent']}%")
        
        # Check memory usage
        if system_data['memory_percent'] > self.alert_thresholds['memory_percent']:
            alerts.append(f"High memory usage: {system_data['memory_percent']:.1f}% > {self.alert_thresholds['memory_percent']}%")
        
        # Check disk space
        if system_data['disk_percent'] > 90.0:
            alerts.append(f"Low disk space: {system_data['disk_percent']:.1f}% used")
        
        # Publish alerts
        for alert in alerts:
            self._publish_alert("SYSTEM_ALERT", alert)
            rospy.logwarn(f"[{self.node_name}] {alert}")
    
    def _check_component_staleness(self, current_time: float):
        """Check if components have gone stale."""
        stale_threshold = 5.0  # seconds
        
        for component, status in self.component_status.items():
            if status['active'] and status['last_update']:
                age = current_time - status['last_update']
                if age > stale_threshold:
                    alert = f"Component {component} has gone stale (last update {age:.1f}s ago)"
                    self._publish_alert("STALENESS_ALERT", alert)
                    rospy.logwarn(f"[{self.node_name}] {alert}")
                    status['active'] = False
    
    def _update_component_health_scores(self):
        """Update health scores for each component based on recent performance."""
        for component, data_queue in self.performance_data.items():
            if not data_queue:
                self.component_health_scores[component] = 0.0
                continue
            
            # Get recent data (last 10 samples)
            recent_data = list(data_queue)[-10:]
            
            health_factors = []
            
            # FPS health factor
            fps_values = [d['fps'] for d in recent_data if d['fps'] > 0]
            if fps_values:
                avg_fps = np.mean(fps_values)
                fps_health = min(1.0, avg_fps / self.alert_thresholds['fps_min'])
                health_factors.append(fps_health)
            
            # Latency health factor
            latency_values = [d['avg_latency_ms'] for d in recent_data if d['avg_latency_ms'] > 0]
            if latency_values:
                avg_latency = np.mean(latency_values)
                latency_health = max(0.0, 1.0 - (avg_latency - 50.0) / 150.0)  # Good if < 50ms
                health_factors.append(latency_health)
            
            # Calculate component health
            if health_factors:
                self.component_health_scores[component] = np.mean(health_factors)
            else:
                self.component_health_scores[component] = 0.5  # Unknown
    
    def _calculate_overall_health(self):
        """Calculate overall system health score."""
        health_factors = []
        
        # Component health factors
        for score in self.component_health_scores.values():
            health_factors.append(score)
        
        # System resource health factors
        if self.system_metrics:
            latest_system = self.system_metrics[-1]
            
            # CPU health
            cpu_health = max(0.0, 1.0 - (latest_system['cpu_percent'] - 50.0) / 50.0)
            health_factors.append(cpu_health)
            
            # Memory health
            memory_health = max(0.0, 1.0 - (latest_system['memory_percent'] - 60.0) / 40.0)
            health_factors.append(memory_health)
        
        # Calculate overall health
        if health_factors:
            self.overall_health_score = np.mean(health_factors)
        else:
            self.overall_health_score = 0.0
    
    def _publish_alert(self, alert_type: str, message: str):
        """Publish a performance alert."""
        alert_data = {
            'timestamp': time.time(),
            'type': alert_type,
            'message': message
        }
        
        self.alert_history.append(alert_data)
        
        # Publish as string message
        alert_msg = String()
        alert_msg.data = f"[{alert_type}] {message}"
        self.alerts_pub.publish(alert_msg)
    
    def _publish_health_status(self):
        """Publish overall system health status."""
        if hasattr(SystemHealthStatus, 'overall_health'):
            health_msg = SystemHealthStatus()
            health_msg.header = Header()
            health_msg.header.stamp = rospy.Time.now()
            health_msg.overall_health = self.overall_health_score
            
            # Add component health scores
            for component, score in self.component_health_scores.items():
                setattr(health_msg, f"{component}_health", score)
            
            self.health_pub.publish(health_msg)
    
    def _publish_diagnostics(self):
        """Publish ROS diagnostics information."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()
        
        # Overall system status
        system_status = DiagnosticStatus()
        system_status.name = "Enhanced Autonomous Navigation System"
        system_status.hardware_id = "duckiebot"
        
        if self.overall_health_score > 0.8:
            system_status.level = DiagnosticStatus.OK
            system_status.message = "System operating normally"
        elif self.overall_health_score > 0.6:
            system_status.level = DiagnosticStatus.WARN
            system_status.message = "System performance degraded"
        else:
            system_status.level = DiagnosticStatus.ERROR
            system_status.message = "System performance critical"
        
        # Add key values
        system_status.values.append(KeyValue("Overall Health", f"{self.overall_health_score:.2f}"))
        
        if self.system_metrics:
            latest = self.system_metrics[-1]
            system_status.values.append(KeyValue("CPU Usage", f"{latest['cpu_percent']:.1f}%"))
            system_status.values.append(KeyValue("Memory Usage", f"{latest['memory_percent']:.1f}%"))
        
        diag_array.status.append(system_status)
        
        # Component-specific diagnostics
        for component, score in self.component_health_scores.items():
            comp_status = DiagnosticStatus()
            comp_status.name = f"Enhanced Navigation - {component.title()}"
            comp_status.hardware_id = "duckiebot"
            
            if score > 0.8:
                comp_status.level = DiagnosticStatus.OK
                comp_status.message = f"{component} operating normally"
            elif score > 0.6:
                comp_status.level = DiagnosticStatus.WARN
                comp_status.message = f"{component} performance degraded"
            else:
                comp_status.level = DiagnosticStatus.ERROR
                comp_status.message = f"{component} performance critical"
            
            comp_status.values.append(KeyValue("Health Score", f"{score:.2f}"))
            comp_status.values.append(KeyValue("Active", str(self.component_status.get(component, {}).get('active', False))))
            
            diag_array.status.append(comp_status)
        
        self.diagnostics_pub.publish(diag_array)
    
    def _log_health_summary(self):
        """Log a summary of system health."""
        rospy.loginfo(f"[{self.node_name}] System Health Summary:")
        rospy.loginfo(f"[{self.node_name}]   Overall Health: {self.overall_health_score:.2f}")
        
        for component, score in self.component_health_scores.items():
            active = self.component_status.get(component, {}).get('active', False)
            rospy.loginfo(f"[{self.node_name}]   {component.title()}: {score:.2f} ({'Active' if active else 'Inactive'})")
        
        if self.system_metrics:
            latest = self.system_metrics[-1]
            rospy.loginfo(f"[{self.node_name}]   System: CPU={latest['cpu_percent']:.1f}%, "
                         f"Memory={latest['memory_percent']:.1f}%, "
                         f"ROS Processes={latest['ros_process_count']}")


def main():
    """Main function to run the system performance monitor."""
    try:
        monitor = SystemPerformanceMonitor()
        rospy.loginfo("System Performance Monitor running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("System Performance Monitor interrupted")
    except Exception as e:
        rospy.logerr(f"System Performance Monitor error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
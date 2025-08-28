#!/usr/bin/env python3
"""
Ultra Debugging and Real-time Monitoring System for Progressive Deployment
Provides comprehensive system monitoring, bottleneck detection, and debugging capabilities
"""

import os
import sys
import time
import json
import threading
import subprocess
import psutil
import logging
from datetime import datetime, timedelta
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, asdict
from collections import deque, defaultdict
import queue
import signal

# Configure comprehensive logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [MONITOR] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


@dataclass
class SystemMetrics:
    """Comprehensive system metrics data structure"""
    timestamp: float
    cpu_percent: float
    cpu_load_1m: float
    cpu_load_5m: float
    cpu_load_15m: float
    memory_percent: float
    memory_available_gb: float
    memory_used_gb: float
    disk_usage_percent: float
    disk_free_gb: float
    temperature_celsius: Optional[float]
    network_bytes_sent: int
    network_bytes_recv: int
    active_processes: int
    ros_topics_count: int
    ros_nodes_count: int
    docker_containers_count: int
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class BottleneckAlert:
    """Bottleneck detection alert"""
    timestamp: float
    severity: str  # LOW, MEDIUM, HIGH, CRITICAL
    category: str  # CPU, MEMORY, DISK, NETWORK, ROS, DOCKER
    metric_name: str
    current_value: float
    threshold: float
    description: str
    suggested_action: str
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ComponentHealth:
    """Health status of a deployed component"""
    component_name: str
    status: str  # HEALTHY, WARNING, CRITICAL, FAILED
    last_check: float
    metrics: Dict[str, Any]
    errors: List[str]
    warnings: List[str]
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class RealTimeMonitor:
    """
    Comprehensive real-time monitoring system with bottleneck detection
    """
    
    def __init__(self, robot_host: str, robot_user: str = "duckie"):
        self.robot_host = robot_host
        self.robot_user = robot_user
        self.monitoring_active = False
        self.monitor_thread = None
        self.alert_thread = None
        
        # Monitoring configuration
        self.sampling_interval = 2.0  # seconds
        self.history_size = 300  # keep 10 minutes of data at 2s intervals
        self.alert_cooldown = 30.0  # seconds between similar alerts
        
        # Data storage
        self.metrics_history: deque = deque(maxlen=self.history_size)
        self.alerts_history: deque = deque(maxlen=1000)
        self.component_health: Dict[str, ComponentHealth] = {}
        self.alert_queue = queue.Queue()
        
        # Thresholds for bottleneck detection
        self.thresholds = {
            'cpu_percent': {'warning': 70.0, 'critical': 90.0},
            'cpu_load_1m': {'warning': 2.0, 'critical': 4.0},
            'memory_percent': {'warning': 80.0, 'critical': 95.0},
            'disk_usage_percent': {'warning': 85.0, 'critical': 95.0},
            'temperature_celsius': {'warning': 70.0, 'critical': 80.0},
            'ros_topics_count': {'warning': 50, 'critical': 100},
            'docker_containers_count': {'warning': 15, 'critical': 25}
        }
        
        # Alert tracking
        self.last_alerts = defaultdict(float)
        
        # Log files
        self.log_dir = "/tmp/ultra_monitoring"
        os.makedirs(self.log_dir, exist_ok=True)
        self.metrics_log = f"{self.log_dir}/metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        self.alerts_log = f"{self.log_dir}/alerts_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        self.debug_log = f"{self.log_dir}/debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        
        logger.info(f"Ultra monitoring initialized for {robot_host}")
        logger.info(f"Logs will be saved to: {self.log_dir}")
    
    def ssh_command(self, command: str) -> Tuple[bool, str]:
        """Execute SSH command and return success status and output"""
        try:
            result = subprocess.run(
                ["ssh", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no", 
                 f"{self.robot_user}@{self.robot_host}", command],
                capture_output=True, text=True, timeout=10
            )
            return result.returncode == 0, result.stdout + result.stderr
        except subprocess.TimeoutExpired:
            return False, "SSH command timeout"
        except Exception as e:
            return False, f"SSH error: {str(e)}"
    
    def collect_system_metrics(self) -> Optional[SystemMetrics]:
        """Collect comprehensive system metrics from robot"""
        try:
            # System metrics collection command
            metrics_cmd = """
python3 -c "
import psutil
import json
import subprocess
import os

# CPU metrics
cpu_percent = psutil.cpu_percent(interval=1)
load_avg = os.getloadavg()

# Memory metrics
memory = psutil.virtual_memory()

# Disk metrics
disk = psutil.disk_usage('/')

# Network metrics
network = psutil.net_io_counters()

# Temperature (Raspberry Pi specific)
try:
    temp_result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
    if temp_result.returncode == 0:
        temp_str = temp_result.stdout.strip()
        temperature = float(temp_str.split('=')[1].replace('°C', '')) if '=' in temp_str else None
    else:
        temperature = None
except:
    temperature = None

# Process count
active_processes = len(psutil.pids())

# ROS metrics
try:
    ros_topics_result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
    ros_topics_count = len(ros_topics_result.stdout.strip().split('\\n')) if ros_topics_result.returncode == 0 and ros_topics_result.stdout.strip() else 0
except:
    ros_topics_count = 0

try:
    ros_nodes_result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=5)
    ros_nodes_count = len(ros_nodes_result.stdout.strip().split('\\n')) if ros_nodes_result.returncode == 0 and ros_nodes_result.stdout.strip() else 0
except:
    ros_nodes_count = 0

# Docker metrics
try:
    docker_result = subprocess.run(['docker', 'ps', '-q'], capture_output=True, text=True, timeout=5)
    docker_containers_count = len(docker_result.stdout.strip().split('\\n')) if docker_result.returncode == 0 and docker_result.stdout.strip() else 0
except:
    docker_containers_count = 0

# Compile metrics
metrics = {
    'timestamp': $(date +%s.%N),
    'cpu_percent': cpu_percent,
    'cpu_load_1m': load_avg[0],
    'cpu_load_5m': load_avg[1],
    'cpu_load_15m': load_avg[2],
    'memory_percent': memory.percent,
    'memory_available_gb': memory.available / (1024**3),
    'memory_used_gb': memory.used / (1024**3),
    'disk_usage_percent': disk.percent,
    'disk_free_gb': disk.free / (1024**3),
    'temperature_celsius': temperature,
    'network_bytes_sent': network.bytes_sent,
    'network_bytes_recv': network.bytes_recv,
    'active_processes': active_processes,
    'ros_topics_count': ros_topics_count,
    'ros_nodes_count': ros_nodes_count,
    'docker_containers_count': docker_containers_count
}

print(json.dumps(metrics))
"
"""
            
            success, output = self.ssh_command(metrics_cmd)
            if not success:
                logger.error(f"Failed to collect metrics: {output}")
                return None
            
            try:
                metrics_data = json.loads(output.strip())
                return SystemMetrics(**metrics_data)
            except json.JSONDecodeError as e:
                logger.error(f"Failed to parse metrics JSON: {e}")
                logger.debug(f"Raw output: {output}")
                return None
                
        except Exception as e:
            logger.error(f"Error collecting system metrics: {e}")
            return None
    
    def detect_bottlenecks(self, metrics: SystemMetrics) -> List[BottleneckAlert]:
        """Detect system bottlenecks and performance issues"""
        alerts = []
        current_time = time.time()
        
        # Check each metric against thresholds
        for metric_name, thresholds in self.thresholds.items():
            if not hasattr(metrics, metric_name):
                continue
                
            value = getattr(metrics, metric_name)
            if value is None:
                continue
            
            # Determine severity
            severity = None
            threshold = None
            
            if value >= thresholds.get('critical', float('inf')):
                severity = 'CRITICAL'
                threshold = thresholds['critical']
            elif value >= thresholds.get('warning', float('inf')):
                severity = 'WARNING'
                threshold = thresholds['warning']
            
            if severity:
                # Check alert cooldown
                alert_key = f"{metric_name}_{severity}"
                if current_time - self.last_alerts[alert_key] < self.alert_cooldown:
                    continue
                
                self.last_alerts[alert_key] = current_time
                
                # Generate alert
                alert = self._create_bottleneck_alert(
                    metric_name, value, threshold, severity, current_time
                )
                alerts.append(alert)
        
        # Custom bottleneck detection logic
        alerts.extend(self._detect_custom_bottlenecks(metrics, current_time))
        
        return alerts
    
    def _create_bottleneck_alert(self, metric_name: str, value: float, 
                                threshold: float, severity: str, timestamp: float) -> BottleneckAlert:
        """Create a bottleneck alert with appropriate description and action"""
        
        descriptions = {
            'cpu_percent': f"High CPU usage: {value:.1f}%",
            'cpu_load_1m': f"High CPU load: {value:.2f}",
            'memory_percent': f"High memory usage: {value:.1f}%",
            'disk_usage_percent': f"High disk usage: {value:.1f}%",
            'temperature_celsius': f"High temperature: {value:.1f}°C",
            'ros_topics_count': f"Too many ROS topics: {int(value)}",
            'docker_containers_count': f"Too many Docker containers: {int(value)}"
        }
        
        actions = {
            'cpu_percent': "Check for runaway processes, reduce processing load",
            'cpu_load_1m': "Investigate high load processes, consider optimization",
            'memory_percent': "Check memory leaks, restart heavy processes",
            'disk_usage_percent': "Clean up disk space, check log files",
            'temperature_celsius': "Check cooling, reduce processing load",
            'ros_topics_count': "Review active ROS nodes, stop unnecessary topics",
            'docker_containers_count': "Stop unused containers, check for container leaks"
        }
        
        categories = {
            'cpu_percent': 'CPU',
            'cpu_load_1m': 'CPU',
            'memory_percent': 'MEMORY',
            'disk_usage_percent': 'DISK',
            'temperature_celsius': 'THERMAL',
            'ros_topics_count': 'ROS',
            'docker_containers_count': 'DOCKER'
        }
        
        return BottleneckAlert(
            timestamp=timestamp,
            severity=severity,
            category=categories.get(metric_name, 'SYSTEM'),
            metric_name=metric_name,
            current_value=value,
            threshold=threshold,
            description=descriptions.get(metric_name, f"Metric {metric_name} exceeded threshold"),
            suggested_action=actions.get(metric_name, "Investigate and optimize")
        )
    
    def _detect_custom_bottlenecks(self, metrics: SystemMetrics, timestamp: float) -> List[BottleneckAlert]:
        """Detect custom bottlenecks based on complex conditions"""
        alerts = []
        
        # Memory leak detection
        if len(self.metrics_history) >= 10:
            recent_memory = [m.memory_percent for m in list(self.metrics_history)[-10:]]
            if all(recent_memory[i] < recent_memory[i+1] for i in range(len(recent_memory)-1)):
                alerts.append(BottleneckAlert(
                    timestamp=timestamp,
                    severity='CRITICAL',
                    category='MEMORY',
                    metric_name='memory_leak',
                    current_value=metrics.memory_percent,
                    threshold=0.0,
                    description="Potential memory leak detected (continuous increase)",
                    suggested_action="Investigate memory usage, restart affected processes"
                ))
        
        # CPU thermal throttling risk
        if (metrics.temperature_celsius and metrics.temperature_celsius > 75 and 
            metrics.cpu_percent > 80):
            alerts.append(BottleneckAlert(
                timestamp=timestamp,
                severity='CRITICAL',
                category='THERMAL',
                metric_name='thermal_throttling_risk',
                current_value=metrics.temperature_celsius,
                threshold=75.0,
                description=f"Risk of thermal throttling: {metrics.temperature_celsius:.1f}°C with high CPU",
                suggested_action="Reduce CPU load immediately, check cooling"
            ))
        
        # ROS system instability
        if metrics.ros_nodes_count == 0 and metrics.ros_topics_count > 0:
            alerts.append(BottleneckAlert(
                timestamp=timestamp,
                severity='CRITICAL',
                category='ROS',
                metric_name='ros_inconsistency',
                current_value=0,
                threshold=1,
                description="ROS topics exist but no nodes detected",
                suggested_action="Check ROS master, restart ROS nodes"
            ))
        
        return alerts
    
    def log_metrics(self, metrics: SystemMetrics):
        """Log metrics to file"""
        try:
            with open(self.metrics_log, 'a') as f:
                f.write(f"{json.dumps(metrics.to_dict())}\\n")
        except Exception as e:
            logger.error(f"Failed to log metrics: {e}")
    
    def log_alert(self, alert: BottleneckAlert):
        """Log alert to file"""
        try:
            with open(self.alerts_log, 'a') as f:
                f.write(f"{json.dumps(alert.to_dict())}\\n")
        except Exception as e:
            logger.error(f"Failed to log alert: {e}")
    
    def monitor_loop(self):
        """Main monitoring loop"""
        logger.info("Starting monitoring loop")
        
        while self.monitoring_active:
            try:
                # Collect metrics
                metrics = self.collect_system_metrics()
                if metrics:
                    # Store metrics
                    self.metrics_history.append(metrics)
                    self.log_metrics(metrics)
                    
                    # Detect bottlenecks
                    alerts = self.detect_bottlenecks(metrics)
                    for alert in alerts:
                        self.alerts_history.append(alert)
                        self.log_alert(alert)
                        self.alert_queue.put(alert)
                        
                        # Log alert to console
                        logger.warning(f"🚨 {alert.severity} {alert.category}: {alert.description}")
                    
                    # Debug output
                    logger.debug(f"Metrics: CPU={metrics.cpu_percent:.1f}%, "
                               f"Mem={metrics.memory_percent:.1f}%, "
                               f"Temp={metrics.temperature_celsius}°C, "
                               f"ROS={metrics.ros_topics_count} topics")
                
                time.sleep(self.sampling_interval)
                
            except Exception as e:
                logger.error(f"Error in monitoring loop: {e}")
                time.sleep(self.sampling_interval)
    
    def alert_processor(self):
        """Process alerts in separate thread"""
        logger.info("Starting alert processor")
        
        while self.monitoring_active:
            try:
                # Wait for alerts with timeout
                alert = self.alert_queue.get(timeout=1.0)
                
                # Process alert (could send notifications, trigger actions, etc.)
                self.process_alert(alert)
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error processing alert: {e}")
    
    def process_alert(self, alert: BottleneckAlert):
        """Process individual alert"""
        timestamp_str = datetime.fromtimestamp(alert.timestamp).strftime('%H:%M:%S')
        
        print(f"\\n{'='*60}")
        print(f"🚨 BOTTLENECK ALERT - {timestamp_str}")
        print(f"{'='*60}")
        print(f"Severity: {alert.severity}")
        print(f"Category: {alert.category}")
        print(f"Metric: {alert.metric_name}")
        print(f"Current Value: {alert.current_value}")
        print(f"Threshold: {alert.threshold}")
        print(f"Description: {alert.description}")
        print(f"Suggested Action: {alert.suggested_action}")
        print(f"{'='*60}\\n")
        
        # Auto-mitigation for critical issues
        if alert.severity == 'CRITICAL':
            self.attempt_auto_mitigation(alert)
    
    def attempt_auto_mitigation(self, alert: BottleneckAlert):
        """Attempt automatic mitigation for critical alerts"""
        logger.info(f"Attempting auto-mitigation for: {alert.description}")
        
        if alert.category == 'MEMORY' and alert.current_value > 95:
            # Clear system caches
            success, output = self.ssh_command("sudo sync && sudo sysctl -w vm.drop_caches=3")
            if success:
                logger.info("Cleared system caches to free memory")
            else:
                logger.error(f"Failed to clear caches: {output}")
        
        elif alert.category == 'DISK' and alert.current_value > 95:
            # Clean temporary files
            success, output = self.ssh_command("sudo find /tmp -type f -atime +1 -delete 2>/dev/null || true")
            if success:
                logger.info("Cleaned temporary files")
        
        elif alert.category == 'THERMAL' and alert.current_value > 80:
            # Reduce CPU frequency
            success, output = self.ssh_command("echo powersave | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor")
            if success:
                logger.info("Set CPU governor to powersave mode")
    
    def start_monitoring(self):
        """Start the monitoring system"""
        if self.monitoring_active:
            logger.warning("Monitoring already active")
            return
        
        logger.info("Starting ultra monitoring system")
        self.monitoring_active = True
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        # Start alert processing thread
        self.alert_thread = threading.Thread(target=self.alert_processor, daemon=True)
        self.alert_thread.start()
        
        logger.info("Ultra monitoring system started")
    
    def stop_monitoring(self):
        """Stop the monitoring system"""
        if not self.monitoring_active:
            return
        
        logger.info("Stopping ultra monitoring system")
        self.monitoring_active = False
        
        # Wait for threads to finish
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
        if self.alert_thread:
            self.alert_thread.join(timeout=5)
        
        logger.info("Ultra monitoring system stopped")
    
    def get_summary_report(self) -> Dict[str, Any]:
        """Generate comprehensive monitoring summary"""
        if not self.metrics_history:
            return {"status": "No data available"}
        
        recent_metrics = list(self.metrics_history)[-10:]  # Last 10 samples
        latest = recent_metrics[-1]
        
        # Calculate averages
        avg_cpu = sum(m.cpu_percent for m in recent_metrics) / len(recent_metrics)
        avg_memory = sum(m.memory_percent for m in recent_metrics) / len(recent_metrics)
        avg_temp = sum(m.temperature_celsius for m in recent_metrics if m.temperature_celsius) / len([m for m in recent_metrics if m.temperature_celsius]) if any(m.temperature_celsius for m in recent_metrics) else None
        
        # Recent alerts
        recent_alerts = [a for a in self.alerts_history if time.time() - a.timestamp < 300]  # Last 5 minutes
        
        return {
            "timestamp": datetime.now().isoformat(),
            "status": "HEALTHY" if not recent_alerts else "ALERTS_ACTIVE",
            "current_metrics": latest.to_dict(),
            "averages": {
                "cpu_percent": round(avg_cpu, 1),
                "memory_percent": round(avg_memory, 1),
                "temperature_celsius": round(avg_temp, 1) if avg_temp else None
            },
            "recent_alerts": len(recent_alerts),
            "critical_alerts": len([a for a in recent_alerts if a.severity == 'CRITICAL']),
            "total_samples": len(self.metrics_history),
            "monitoring_duration_minutes": round((time.time() - self.metrics_history[0].timestamp) / 60, 1) if self.metrics_history else 0
        }


class ComponentHealthChecker:
    """
    Health checker for deployed components
    """
    
    def __init__(self, monitor: RealTimeMonitor):
        self.monitor = monitor
        
        # Component health check definitions
        self.health_checks = {
            'enhanced_vision_utils': self._check_vision_utils,
            'adaptive_line_detector': self._check_line_detector,
            'enhanced_lane_filter': self._check_lane_filter,
            'safety_monitoring': self._check_safety_monitoring,
            'performance_optimizer': self._check_performance_optimizer,
            'enhanced_navigation': self._check_navigation,
            'object_detection': self._check_object_detection,
            'apriltag_enhancements': self._check_apriltag
        }
    
    def check_all_components(self) -> Dict[str, ComponentHealth]:
        """Check health of all deployed components"""
        results = {}
        
        for component_name, check_func in self.health_checks.items():
            try:
                health = check_func()
                results[component_name] = health
                logger.debug(f"Health check {component_name}: {health.status}")
            except Exception as e:
                results[component_name] = ComponentHealth(
                    component_name=component_name,
                    status='FAILED',
                    last_check=time.time(),
                    metrics={},
                    errors=[str(e)],
                    warnings=[]
                )
                logger.error(f"Health check failed for {component_name}: {e}")
        
        return results
    
    def _check_vision_utils(self) -> ComponentHealth:
        """Check vision utils health"""
        success, output = self.monitor.ssh_command(
            "cd /tmp && python3 -c 'import advanced_vision_utils; print(\"OK\")' 2>&1"
        )
        
        status = 'HEALTHY' if success and 'OK' in output else 'FAILED'
        errors = [] if success else [output]
        
        return ComponentHealth(
            component_name='enhanced_vision_utils',
            status=status,
            last_check=time.time(),
            metrics={'import_test': success},
            errors=errors,
            warnings=[]
        )
    
    def _check_line_detector(self) -> ComponentHealth:
        """Check adaptive line detector health"""
        success, output = self.monitor.ssh_command(
            "test -d /tmp/enhanced_line_detector && echo 'EXISTS' || echo 'MISSING'"
        )
        
        status = 'HEALTHY' if 'EXISTS' in output else 'FAILED'
        
        return ComponentHealth(
            component_name='adaptive_line_detector',
            status=status,
            last_check=time.time(),
            metrics={'deployment_check': 'EXISTS' in output},
            errors=[] if status == 'HEALTHY' else ['Component directory not found'],
            warnings=[]
        )
    
    def _check_lane_filter(self) -> ComponentHealth:
        """Check enhanced lane filter health"""
        success, output = self.monitor.ssh_command(
            "test -d /tmp/enhanced_lane_filter && echo 'EXISTS' || echo 'MISSING'"
        )
        
        status = 'HEALTHY' if 'EXISTS' in output else 'FAILED'
        
        return ComponentHealth(
            component_name='enhanced_lane_filter',
            status=status,
            last_check=time.time(),
            metrics={'deployment_check': 'EXISTS' in output},
            errors=[] if status == 'HEALTHY' else ['Component directory not found'],
            warnings=[]
        )
    
    def _check_safety_monitoring(self) -> ComponentHealth:
        """Check safety monitoring health"""
        success, output = self.monitor.ssh_command(
            "test -f /tmp/safety_status_publisher.py && echo 'EXISTS' || echo 'MISSING'"
        )
        
        status = 'HEALTHY' if 'EXISTS' in output else 'FAILED'
        
        return ComponentHealth(
            component_name='safety_monitoring',
            status=status,
            last_check=time.time(),
            metrics={'deployment_check': 'EXISTS' in output},
            errors=[] if status == 'HEALTHY' else ['Component file not found'],
            warnings=[]
        )
    
    def _check_performance_optimizer(self) -> ComponentHealth:
        """Check performance optimizer health"""
        success, output = self.monitor.ssh_command(
            "cd /tmp && python3 -c 'import performance_optimizer; print(\"OK\")' 2>&1"
        )
        
        status = 'HEALTHY' if success and 'OK' in output else 'FAILED'
        errors = [] if success else [output]
        
        return ComponentHealth(
            component_name='performance_optimizer',
            status=status,
            last_check=time.time(),
            metrics={'import_test': success},
            errors=errors,
            warnings=[]
        )
    
    def _check_navigation(self) -> ComponentHealth:
        """Check enhanced navigation health"""
        success, output = self.monitor.ssh_command(
            "test -d /tmp/enhanced_navigation && echo 'EXISTS' || echo 'MISSING'"
        )
        
        status = 'HEALTHY' if 'EXISTS' in output else 'FAILED'
        
        return ComponentHealth(
            component_name='enhanced_navigation',
            status=status,
            last_check=time.time(),
            metrics={'deployment_check': 'EXISTS' in output},
            errors=[] if status == 'HEALTHY' else ['Component directory not found'],
            warnings=[]
        )
    
    def _check_object_detection(self) -> ComponentHealth:
        """Check object detection health"""
        success, output = self.monitor.ssh_command(
            "test -d /tmp/enhanced_vehicle_detection && echo 'EXISTS' || echo 'MISSING'"
        )
        
        status = 'HEALTHY' if 'EXISTS' in output else 'FAILED'
        
        return ComponentHealth(
            component_name='object_detection',
            status=status,
            last_check=time.time(),
            metrics={'deployment_check': 'EXISTS' in output},
            errors=[] if status == 'HEALTHY' else ['Component directory not found'],
            warnings=[]
        )
    
    def _check_apriltag(self) -> ComponentHealth:
        """Check AprilTag enhancements health"""
        success, output = self.monitor.ssh_command(
            "test -d /tmp/enhanced_apriltag && echo 'EXISTS' || echo 'MISSING'"
        )
        
        status = 'HEALTHY' if 'EXISTS' in output else 'FAILED'
        
        return ComponentHealth(
            component_name='apriltag_enhancements',
            status=status,
            last_check=time.time(),
            metrics={'deployment_check': 'EXISTS' in output},
            errors=[] if status == 'HEALTHY' else ['Component directory not found'],
            warnings=[]
        )


def main():
    """Main monitoring application"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Ultra Debugging and Monitoring System')
    parser.add_argument('--robot', default='blueduckie.local', help='Robot hostname')
    parser.add_argument('--user', default='duckie', help='SSH username')
    parser.add_argument('--duration', type=int, default=0, help='Monitoring duration in seconds (0 = infinite)')
    parser.add_argument('--interval', type=float, default=2.0, help='Sampling interval in seconds')
    parser.add_argument('--health-check', action='store_true', help='Run component health check')
    
    args = parser.parse_args()
    
    # Initialize monitor
    monitor = RealTimeMonitor(args.robot, args.user)
    monitor.sampling_interval = args.interval
    
    # Component health checker
    health_checker = ComponentHealthChecker(monitor)
    
    def signal_handler(signum, frame):
        print("\\nStopping monitoring...")
        monitor.stop_monitoring()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        if args.health_check:
            print("Running component health check...")
            health_results = health_checker.check_all_components()
            
            print("\\n" + "="*60)
            print("COMPONENT HEALTH CHECK RESULTS")
            print("="*60)
            
            for component, health in health_results.items():
                status_emoji = "✅" if health.status == "HEALTHY" else "❌"
                print(f"{status_emoji} {component}: {health.status}")
                if health.errors:
                    for error in health.errors:
                        print(f"   Error: {error}")
                if health.warnings:
                    for warning in health.warnings:
                        print(f"   Warning: {warning}")
            
            print("="*60)
        else:
            # Start monitoring
            monitor.start_monitoring()
            
            print(f"🚀 Ultra monitoring started for {args.robot}")
            print(f"📊 Sampling interval: {args.interval}s")
            print(f"📁 Logs: {monitor.log_dir}")
            print("Press Ctrl+C to stop\\n")
            
            # Monitoring loop
            start_time = time.time()
            last_summary = 0
            
            while True:
                current_time = time.time()
                
                # Print summary every 30 seconds
                if current_time - last_summary >= 30:
                    summary = monitor.get_summary_report()
                    print(f"\\n📊 Status: {summary['status']} | "
                          f"CPU: {summary['averages']['cpu_percent']}% | "
                          f"Memory: {summary['averages']['memory_percent']}% | "
                          f"Alerts: {summary['recent_alerts']}")
                    last_summary = current_time
                
                # Check duration
                if args.duration > 0 and (current_time - start_time) >= args.duration:
                    print(f"\\nMonitoring duration ({args.duration}s) completed")
                    break
                
                time.sleep(1)
    
    finally:
        monitor.stop_monitoring()
        
        # Final summary
        summary = monitor.get_summary_report()
        print("\\n" + "="*60)
        print("FINAL MONITORING SUMMARY")
        print("="*60)
        print(json.dumps(summary, indent=2))
        print("="*60)
        print(f"📁 Detailed logs available at: {monitor.log_dir}")


if __name__ == "__main__":
    main()

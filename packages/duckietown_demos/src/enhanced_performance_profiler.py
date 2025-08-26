#!/usr/bin/env python3
"""
Enhanced Performance Profiler for Advanced Autonomous Navigation

Integrates with easy_logs framework to provide comprehensive performance profiling,
bottleneck identification, and system optimization for Raspberry Pi deployment.

Author: Duckietown
"""

import rospy
import time
import psutil
import threading
import json
import os
from typing import Dict, List, Optional, Any, Tuple
from collections import deque, defaultdict
from dataclasses import dataclass, asdict
import numpy as np

# ROS messages
from std_msgs.msg import String, Float32, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Easy logs integration
try:
    from easy_logs import get_easy_logs_db2, logger as easy_logger
    from easy_logs.logs_structure import PhysicalLog
    EASY_LOGS_AVAILABLE = True
except ImportError:
    rospy.logwarn("Easy logs not available, using fallback logging")
    EASY_LOGS_AVAILABLE = False


@dataclass
class PerformanceProfile:
    """Comprehensive performance profile for a system component"""
    component_name: str
    timestamp: float
    cpu_usage_percent: float
    memory_usage_mb: float
    processing_time_ms: float
    fps: float
    queue_depth: int
    error_count: int
    latency_p50_ms: float
    latency_p95_ms: float
    latency_p99_ms: float
    throughput_ops_per_sec: float
    resource_efficiency: float  # 0-1 score
    bottleneck_indicators: Dict[str, float]
    optimization_opportunities: List[str]
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class SystemBottleneck:
    """Identified system bottleneck"""
    bottleneck_type: str  # 'cpu', 'memory', 'io', 'network', 'algorithm'
    severity: float  # 0-1, higher is more severe
    affected_components: List[str]
    root_cause: str
    impact_description: str
    recommended_actions: List[str]
    estimated_improvement: float  # Expected performance improvement %
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class EnhancedPerformanceProfiler:
    """
    Advanced performance profiler that integrates with easy_logs framework
    to provide comprehensive system analysis and optimization recommendations.
    """
    
    def __init__(self):
        self.node_name = "enhanced_performance_profiler"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Configuration
        self.profiling_frequency = rospy.get_param("~profiling_frequency", 1.0)  # Hz
        self.profile_history_size = rospy.get_param("~profile_history_size", 1000)
        self.bottleneck_detection_window = rospy.get_param("~bottleneck_detection_window", 30)  # seconds
        self.raspberry_pi_mode = rospy.get_param("~raspberry_pi_mode", True)
        
        # Performance thresholds for Raspberry Pi
        if self.raspberry_pi_mode:
            self.cpu_bottleneck_threshold = 85.0
            self.memory_bottleneck_threshold = 80.0
            self.latency_bottleneck_threshold = 300.0  # ms
            self.fps_bottleneck_threshold = 15.0
        else:
            self.cpu_bottleneck_threshold = 90.0
            self.memory_bottleneck_threshold = 85.0
            self.latency_bottleneck_threshold = 200.0  # ms
            self.fps_bottleneck_threshold = 20.0
        
        # Data storage
        self.performance_profiles: Dict[str, deque] = defaultdict(lambda: deque(maxlen=self.profile_history_size))
        self.system_metrics_history: deque = deque(maxlen=self.profile_history_size)
        self.bottleneck_history: deque = deque(maxlen=100)
        self.optimization_log: List[Dict] = []
        
        # Component registration
        self.registered_components: Dict[str, Dict] = {}
        self.component_priorities: Dict[str, int] = {}
        
        # Profiling state
        self.profiling_active = True
        self.profiling_lock = threading.Lock()
        self.current_bottlenecks: Dict[str, SystemBottleneck] = {}
        
        # Easy logs integration
        self.easy_logs_db = None
        if EASY_LOGS_AVAILABLE:
            try:
                self.easy_logs_db = get_easy_logs_db2(
                    do_not_use_cloud=True,
                    do_not_use_local=False,
                    ignore_cache=False
                )
                rospy.loginfo(f"[{self.node_name}] Easy logs integration enabled")
            except Exception as e:
                rospy.logwarn(f"[{self.node_name}] Easy logs integration failed: {e}")
                self.easy_logs_db = None
        
        # Performance statistics
        self.total_profiles_collected = 0
        self.bottlenecks_identified = 0
        self.optimizations_suggested = 0
        self.system_efficiency_score = 1.0
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] Enhanced Performance Profiler initialized")
        rospy.loginfo(f"[{self.node_name}] Profiling frequency: {self.profiling_frequency} Hz")
        rospy.loginfo(f"[{self.node_name}] Raspberry Pi mode: {self.raspberry_pi_mode}")
        rospy.loginfo(f"[{self.node_name}] CPU threshold: {self.cpu_bottleneck_threshold}%")
        rospy.loginfo(f"[{self.node_name}] Memory threshold: {self.memory_bottleneck_threshold}%")
        rospy.loginfo(f"[{self.node_name}] Latency threshold: {self.latency_bottleneck_threshold}ms")
    
    def _setup_subscribers(self):
        """Setup subscribers for component performance data"""
        rospy.loginfo(f"[{self.node_name}] Setting up performance data subscribers...")
        
        # Subscribe to component performance topics
        self.component_perf_sub = rospy.Subscriber(
            "~component_performance",
            String,  # JSON-encoded performance data
            self._component_performance_callback,
            queue_size=10
        )
        
        # Subscribe to system performance monitor
        self.system_perf_sub = rospy.Subscriber(
            "/system_performance_monitor/system_health",
            String,  # Fallback to String if custom messages not available
            self._system_performance_callback,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Performance subscribers initialized")
    
    def _setup_publishers(self):
        """Setup publishers for profiling results and recommendations"""
        rospy.loginfo(f"[{self.node_name}] Setting up profiling publishers...")
        
        # Performance profile reports
        self.profile_pub = rospy.Publisher(
            "~performance_profiles",
            String,  # JSON-encoded profile data
            queue_size=1
        )
        
        # Bottleneck alerts
        self.bottleneck_pub = rospy.Publisher(
            "~bottleneck_alerts",
            String,
            queue_size=10
        )
        
        # Optimization recommendations
        self.optimization_pub = rospy.Publisher(
            "~optimization_recommendations",
            String,
            queue_size=10
        )
        
        # System efficiency score
        self.efficiency_pub = rospy.Publisher(
            "~system_efficiency",
            Float32,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Profiling publishers initialized")
    
    def _setup_timers(self):
        """Setup periodic profiling timers"""
        # Main profiling timer
        self.profiling_timer = rospy.Timer(
            rospy.Duration(1.0 / self.profiling_frequency),
            self._profiling_callback
        )
        
        # Bottleneck analysis timer
        self.bottleneck_timer = rospy.Timer(
            rospy.Duration(5.0),  # Every 5 seconds
            self._bottleneck_analysis_callback
        )
        
        # Optimization recommendation timer
        self.optimization_timer = rospy.Timer(
            rospy.Duration(10.0),  # Every 10 seconds
            self._optimization_callback
        )
        
        # Easy logs integration timer
        if self.easy_logs_db:
            self.easy_logs_timer = rospy.Timer(
                rospy.Duration(30.0),  # Every 30 seconds
                self._easy_logs_callback
            )
        
        rospy.loginfo(f"[{self.node_name}] Profiling timers initialized")
    
    def register_component(self, component_name: str, priority: int = 5, 
                          performance_targets: Dict[str, float] = None) -> bool:
        """
        Register a component for performance profiling.
        
        Args:
            component_name: Name of the component
            priority: Priority level (1-10, higher = more critical)
            performance_targets: Target performance metrics
            
        Returns:
            bool: True if registration successful
        """
        with self.profiling_lock:
            if performance_targets is None:
                performance_targets = {
                    'target_fps': 20.0,
                    'max_latency_ms': 200.0,
                    'max_cpu_percent': 25.0,
                    'max_memory_mb': 512.0
                }
            
            self.registered_components[component_name] = {
                'priority': priority,
                'performance_targets': performance_targets,
                'registration_time': time.time(),
                'profile_count': 0,
                'last_profile': None,
                'bottleneck_count': 0,
                'optimization_count': 0
            }
            
            self.component_priorities[component_name] = priority
            
            rospy.loginfo(f"[{self.node_name}] Registered component '{component_name}' with priority {priority}")
            rospy.logdebug(f"[{self.node_name}] Performance targets: {performance_targets}")
            
            # Comprehensive logging
            rospy.logdebug(f"[{self.node_name}] Component registration event: {component_name}, "
                          f"Priority={priority}, Targets={performance_targets}")
            
            return True
    
    def _component_performance_callback(self, msg):
        """Handle component performance data"""
        try:
            # Parse JSON performance data
            perf_data = json.loads(msg.data)
            component_name = perf_data.get('component_name', 'unknown')
            
            if component_name not in self.registered_components:
                rospy.logwarn(f"[{self.node_name}] Performance data for unregistered component: {component_name}")
                return
            
            # Create performance profile
            profile = self._create_performance_profile(perf_data)
            
            with self.profiling_lock:
                self.performance_profiles[component_name].append(profile)
                self.registered_components[component_name]['profile_count'] += 1
                self.registered_components[component_name]['last_profile'] = time.time()
                self.total_profiles_collected += 1
            
            rospy.logdebug(f"[{self.node_name}] Performance profile collected for '{component_name}': "
                          f"FPS={profile.fps:.1f}, CPU={profile.cpu_usage_percent:.1f}%, "
                          f"Latency={profile.latency_p50_ms:.1f}ms")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Performance profile event: {component_name}, "
                          f"FPS={profile.fps:.1f}, Efficiency={profile.resource_efficiency:.2f}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing component performance data: {e}")
    
    def _system_performance_callback(self, msg):
        """Handle system-level performance data"""
        try:
            # Parse system performance data
            if hasattr(msg, 'data'):
                system_data = json.loads(msg.data)
            else:
                # Fallback for basic message types
                system_data = {'timestamp': time.time(), 'cpu_percent': 0, 'memory_percent': 0}
            
            with self.profiling_lock:
                self.system_metrics_history.append(system_data)
            
            rospy.logdebug(f"[{self.node_name}] System performance update: "
                          f"CPU={system_data.get('cpu_percent', 0):.1f}%, "
                          f"Memory={system_data.get('memory_percent', 0):.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing system performance data: {e}")
    
    def _create_performance_profile(self, perf_data: Dict) -> PerformanceProfile:
        """Create a comprehensive performance profile from raw data"""
        current_time = time.time()
        
        # Extract basic metrics
        component_name = perf_data.get('component_name', 'unknown')
        cpu_usage = perf_data.get('cpu_usage_percent', 0.0)
        memory_usage = perf_data.get('memory_usage_mb', 0.0)
        processing_time = perf_data.get('processing_time_ms', 0.0)
        fps = perf_data.get('fps', 0.0)
        queue_depth = perf_data.get('queue_depth', 0)
        error_count = perf_data.get('error_count', 0)
        
        # Calculate latency percentiles
        latencies = perf_data.get('latency_samples', [processing_time])
        if latencies:
            latency_p50 = np.percentile(latencies, 50)
            latency_p95 = np.percentile(latencies, 95)
            latency_p99 = np.percentile(latencies, 99)
        else:
            latency_p50 = latency_p95 = latency_p99 = processing_time
        
        # Calculate throughput
        throughput = fps if fps > 0 else (1000.0 / max(1.0, processing_time))
        
        # Calculate resource efficiency (0-1 score)
        targets = self.registered_components.get(component_name, {}).get('performance_targets', {})
        target_fps = targets.get('target_fps', 20.0)
        max_cpu = targets.get('max_cpu_percent', 25.0)
        max_memory = targets.get('max_memory_mb', 512.0)
        max_latency = targets.get('max_latency_ms', 200.0)
        
        fps_efficiency = min(1.0, fps / target_fps) if target_fps > 0 else 1.0
        cpu_efficiency = max(0.0, 1.0 - (cpu_usage / max_cpu)) if max_cpu > 0 else 1.0
        memory_efficiency = max(0.0, 1.0 - (memory_usage / max_memory)) if max_memory > 0 else 1.0
        latency_efficiency = max(0.0, 1.0 - (latency_p50 / max_latency)) if max_latency > 0 else 1.0
        
        resource_efficiency = np.mean([fps_efficiency, cpu_efficiency, memory_efficiency, latency_efficiency])
        
        # Identify bottleneck indicators
        bottleneck_indicators = {
            'cpu_pressure': min(1.0, cpu_usage / 100.0),
            'memory_pressure': min(1.0, memory_usage / 1024.0),  # Normalize to GB
            'latency_pressure': min(1.0, latency_p95 / 500.0),  # Normalize to 500ms
            'queue_pressure': min(1.0, queue_depth / 20.0),  # Normalize to 20 items
            'error_pressure': min(1.0, error_count / 10.0)  # Normalize to 10 errors
        }
        
        # Identify optimization opportunities
        optimization_opportunities = []
        if cpu_usage > max_cpu * 0.8:
            optimization_opportunities.append("reduce_cpu_usage")
        if memory_usage > max_memory * 0.8:
            optimization_opportunities.append("optimize_memory")
        if latency_p95 > max_latency * 0.8:
            optimization_opportunities.append("reduce_latency")
        if fps < target_fps * 0.8:
            optimization_opportunities.append("improve_throughput")
        if queue_depth > 5:
            optimization_opportunities.append("reduce_queue_depth")
        if error_count > 0:
            optimization_opportunities.append("fix_errors")
        
        return PerformanceProfile(
            component_name=component_name,
            timestamp=current_time,
            cpu_usage_percent=cpu_usage,
            memory_usage_mb=memory_usage,
            processing_time_ms=processing_time,
            fps=fps,
            queue_depth=queue_depth,
            error_count=error_count,
            latency_p50_ms=latency_p50,
            latency_p95_ms=latency_p95,
            latency_p99_ms=latency_p99,
            throughput_ops_per_sec=throughput,
            resource_efficiency=resource_efficiency,
            bottleneck_indicators=bottleneck_indicators,
            optimization_opportunities=optimization_opportunities
        )
    
    def _profiling_callback(self, event):
        """Main profiling callback - collect and analyze performance data"""
        if not self.profiling_active:
            return
        
        try:
            current_time = time.time()
            
            # Collect system-level metrics
            system_metrics = self._collect_system_metrics()
            
            with self.profiling_lock:
                self.system_metrics_history.append(system_metrics)
                
                # Update system efficiency score
                self._update_system_efficiency_score()
            
            # Publish system efficiency
            efficiency_msg = Float32()
            efficiency_msg.data = self.system_efficiency_score
            self.efficiency_pub.publish(efficiency_msg)
            
            rospy.logdebug(f"[{self.node_name}] Profiling cycle completed: "
                          f"Efficiency={self.system_efficiency_score:.2f}, "
                          f"Profiles={self.total_profiles_collected}")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Profiling event: System efficiency={self.system_efficiency_score:.2f}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Profiling callback error: {e}")
    
    def _collect_system_metrics(self) -> Dict[str, Any]:
        """Collect comprehensive system metrics"""
        try:
            current_time = time.time()
            
            # CPU metrics
            cpu_percent = psutil.cpu_percent(interval=0.1)
            cpu_freq = psutil.cpu_freq()
            cpu_count = psutil.cpu_count()
            
            # Memory metrics
            memory = psutil.virtual_memory()
            swap = psutil.swap_memory()
            
            # Disk metrics
            disk_usage = psutil.disk_usage('/')
            disk_io = psutil.disk_io_counters()
            
            # Network metrics
            network_io = psutil.net_io_counters()
            
            # Process metrics
            process = psutil.Process()
            process_cpu = process.cpu_percent()
            process_memory = process.memory_info()
            
            # Temperature (Raspberry Pi specific)
            temperature = None
            if self.raspberry_pi_mode:
                try:
                    with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                        temperature = float(f.read().strip()) / 1000.0  # Convert to Celsius
                except:
                    temperature = None
            
            metrics = {
                'timestamp': current_time,
                'cpu': {
                    'percent': cpu_percent,
                    'frequency_mhz': cpu_freq.current if cpu_freq else 0,
                    'count': cpu_count,
                    'process_percent': process_cpu
                },
                'memory': {
                    'total_mb': memory.total / 1024 / 1024,
                    'available_mb': memory.available / 1024 / 1024,
                    'percent': memory.percent,
                    'process_rss_mb': process_memory.rss / 1024 / 1024,
                    'process_vms_mb': process_memory.vms / 1024 / 1024,
                    'swap_percent': swap.percent
                },
                'disk': {
                    'total_gb': disk_usage.total / 1024 / 1024 / 1024,
                    'free_gb': disk_usage.free / 1024 / 1024 / 1024,
                    'percent': (disk_usage.used / disk_usage.total) * 100,
                    'read_mb': disk_io.read_bytes / 1024 / 1024 if disk_io else 0,
                    'write_mb': disk_io.write_bytes / 1024 / 1024 if disk_io else 0
                },
                'network': {
                    'bytes_sent': network_io.bytes_sent if network_io else 0,
                    'bytes_recv': network_io.bytes_recv if network_io else 0,
                    'packets_sent': network_io.packets_sent if network_io else 0,
                    'packets_recv': network_io.packets_recv if network_io else 0
                },
                'temperature_celsius': temperature
            }
            
            # Comprehensive logging
            rospy.logdebug(f"[{self.node_name}] System metrics collected: "
                          f"CPU={cpu_percent:.1f}%, Memory={memory.percent:.1f}%, "
                          f"Disk={metrics['disk']['percent']:.1f}%")
            
            if temperature:
                rospy.logdebug(f"[{self.node_name}] Temperature: {temperature:.1f}°C")
            
            return metrics
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error collecting system metrics: {e}")
            return {'timestamp': time.time(), 'error': str(e)}
    
    def _update_system_efficiency_score(self):
        """Update overall system efficiency score"""
        if not self.performance_profiles:
            self.system_efficiency_score = 1.0
            return
        
        # Calculate component efficiency scores
        component_scores = []
        for component_name, profiles in self.performance_profiles.items():
            if profiles:
                recent_profiles = list(profiles)[-5:]  # Last 5 profiles
                avg_efficiency = np.mean([p.resource_efficiency for p in recent_profiles])
                priority = self.component_priorities.get(component_name, 5)
                weighted_score = avg_efficiency * (priority / 10.0)  # Weight by priority
                component_scores.append(weighted_score)
        
        # Calculate system resource efficiency
        system_scores = []
        if self.system_metrics_history:
            recent_metrics = list(self.system_metrics_history)[-5:]
            
            for metrics in recent_metrics:
                cpu_score = max(0.0, 1.0 - (metrics.get('cpu', {}).get('percent', 0) / 100.0))
                memory_score = max(0.0, 1.0 - (metrics.get('memory', {}).get('percent', 0) / 100.0))
                
                # Temperature penalty for Raspberry Pi
                temp_score = 1.0
                if self.raspberry_pi_mode and metrics.get('temperature_celsius'):
                    temp = metrics['temperature_celsius']
                    if temp > 80:  # Critical temperature
                        temp_score = 0.5
                    elif temp > 70:  # Warning temperature
                        temp_score = 0.8
                
                system_score = np.mean([cpu_score, memory_score, temp_score])
                system_scores.append(system_score)
        
        # Combine component and system scores
        all_scores = component_scores + system_scores
        if all_scores:
            self.system_efficiency_score = np.mean(all_scores)
        else:
            self.system_efficiency_score = 1.0
        
        # Clamp to [0, 1]
        self.system_efficiency_score = max(0.0, min(1.0, self.system_efficiency_score))
    
    def _bottleneck_analysis_callback(self, event):
        """Analyze system for performance bottlenecks"""
        if not self.profiling_active:
            return
        
        try:
            current_time = time.time()
            window_start = current_time - self.bottleneck_detection_window
            
            # Analyze each component for bottlenecks
            detected_bottlenecks = []
            
            with self.profiling_lock:
                for component_name, profiles in self.performance_profiles.items():
                    if not profiles:
                        continue
                    
                    # Get profiles within analysis window
                    recent_profiles = [p for p in profiles if p.timestamp >= window_start]
                    if not recent_profiles:
                        continue
                    
                    bottlenecks = self._analyze_component_bottlenecks(component_name, recent_profiles)
                    detected_bottlenecks.extend(bottlenecks)
                
                # Analyze system-level bottlenecks
                system_bottlenecks = self._analyze_system_bottlenecks()
                detected_bottlenecks.extend(system_bottlenecks)
                
                # Update current bottlenecks
                self.current_bottlenecks.clear()
                for bottleneck in detected_bottlenecks:
                    self.current_bottlenecks[f"{bottleneck.bottleneck_type}_{bottleneck.affected_components[0]}"] = bottleneck
                    self.bottleneck_history.append(bottleneck)
                    self.bottlenecks_identified += 1
            
            # Publish bottleneck alerts
            if detected_bottlenecks:
                for bottleneck in detected_bottlenecks:
                    self._publish_bottleneck_alert(bottleneck)
                
                rospy.logwarn(f"[{self.node_name}] Detected {len(detected_bottlenecks)} performance bottlenecks")
                
                # Real-time monitoring
                rospy.logwarn(f"[{self.node_name}] Bottleneck detection event: {len(detected_bottlenecks)} bottlenecks found")
            else:
                rospy.logdebug(f"[{self.node_name}] No performance bottlenecks detected")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Bottleneck analysis error: {e}")
    
    def _analyze_component_bottlenecks(self, component_name: str, profiles: List[PerformanceProfile]) -> List[SystemBottleneck]:
        """Analyze a component for performance bottlenecks"""
        bottlenecks = []
        
        if not profiles:
            return bottlenecks
        
        # Calculate average metrics
        avg_cpu = np.mean([p.cpu_usage_percent for p in profiles])
        avg_memory = np.mean([p.memory_usage_mb for p in profiles])
        avg_latency = np.mean([p.latency_p95_ms for p in profiles])
        avg_fps = np.mean([p.fps for p in profiles])
        avg_queue = np.mean([p.queue_depth for p in profiles])
        avg_errors = np.mean([p.error_count for p in profiles])
        
        current_time = time.time()
        
        # CPU bottleneck
        if avg_cpu > self.cpu_bottleneck_threshold:
            severity = min(1.0, avg_cpu / 100.0)
            bottleneck = SystemBottleneck(
                bottleneck_type="cpu",
                severity=severity,
                affected_components=[component_name],
                root_cause=f"High CPU usage: {avg_cpu:.1f}%",
                impact_description=f"Component {component_name} is consuming excessive CPU resources",
                recommended_actions=[
                    "Reduce processing quality",
                    "Implement frame skipping",
                    "Optimize algorithms",
                    "Enable parallel processing"
                ],
                estimated_improvement=20.0,
                timestamp=current_time
            )
            bottlenecks.append(bottleneck)
        
        # Memory bottleneck
        if avg_memory > 512.0 and self.raspberry_pi_mode:  # 512MB threshold for RPi
            severity = min(1.0, avg_memory / 1024.0)
            bottleneck = SystemBottleneck(
                bottleneck_type="memory",
                severity=severity,
                affected_components=[component_name],
                root_cause=f"High memory usage: {avg_memory:.1f}MB",
                impact_description=f"Component {component_name} is consuming excessive memory",
                recommended_actions=[
                    "Implement memory pooling",
                    "Reduce buffer sizes",
                    "Enable garbage collection",
                    "Optimize data structures"
                ],
                estimated_improvement=15.0,
                timestamp=current_time
            )
            bottlenecks.append(bottleneck)
        
        # Latency bottleneck
        if avg_latency > self.latency_bottleneck_threshold:
            severity = min(1.0, avg_latency / 1000.0)  # Normalize to 1 second
            bottleneck = SystemBottleneck(
                bottleneck_type="latency",
                severity=severity,
                affected_components=[component_name],
                root_cause=f"High processing latency: {avg_latency:.1f}ms",
                impact_description=f"Component {component_name} has excessive processing delays",
                recommended_actions=[
                    "Optimize critical path algorithms",
                    "Implement asynchronous processing",
                    "Reduce input resolution",
                    "Enable hardware acceleration"
                ],
                estimated_improvement=25.0,
                timestamp=current_time
            )
            bottlenecks.append(bottleneck)
        
        # Throughput bottleneck
        if avg_fps < self.fps_bottleneck_threshold:
            severity = max(0.1, 1.0 - (avg_fps / self.fps_bottleneck_threshold))
            bottleneck = SystemBottleneck(
                bottleneck_type="throughput",
                severity=severity,
                affected_components=[component_name],
                root_cause=f"Low throughput: {avg_fps:.1f} FPS",
                impact_description=f"Component {component_name} is not meeting throughput requirements",
                recommended_actions=[
                    "Optimize processing pipeline",
                    "Implement multi-threading",
                    "Reduce computational complexity",
                    "Enable batch processing"
                ],
                estimated_improvement=30.0,
                timestamp=current_time
            )
            bottlenecks.append(bottleneck)
        
        # Queue bottleneck
        if avg_queue > 10:
            severity = min(1.0, avg_queue / 50.0)
            bottleneck = SystemBottleneck(
                bottleneck_type="queue",
                severity=severity,
                affected_components=[component_name],
                root_cause=f"High queue depth: {avg_queue:.1f}",
                impact_description=f"Component {component_name} has processing queue buildup",
                recommended_actions=[
                    "Increase processing rate",
                    "Implement queue management",
                    "Add load balancing",
                    "Enable queue prioritization"
                ],
                estimated_improvement=20.0,
                timestamp=current_time
            )
            bottlenecks.append(bottleneck)
        
        # Comprehensive logging for bottleneck identification
        if bottlenecks:
            rospy.logdebug(f"[{self.node_name}] Bottleneck identification for '{component_name}':")
            for bottleneck in bottlenecks:
                rospy.logdebug(f"[{self.node_name}] - {bottleneck.bottleneck_type}: {bottleneck.root_cause}")
                rospy.logdebug(f"[{self.node_name}] - Severity: {bottleneck.severity:.2f}")
                rospy.logdebug(f"[{self.node_name}] - Recommended actions: {bottleneck.recommended_actions}")
        
        return bottlenecks
    
    def _analyze_system_bottlenecks(self) -> List[SystemBottleneck]:
        """Analyze system-level bottlenecks"""
        bottlenecks = []
        
        if not self.system_metrics_history:
            return bottlenecks
        
        # Get recent system metrics
        recent_metrics = list(self.system_metrics_history)[-10:]  # Last 10 measurements
        current_time = time.time()
        
        # Calculate average system metrics
        avg_cpu = np.mean([m.get('cpu', {}).get('percent', 0) for m in recent_metrics])
        avg_memory = np.mean([m.get('memory', {}).get('percent', 0) for m in recent_metrics])
        avg_temp = None
        
        if self.raspberry_pi_mode:
            temps = [m.get('temperature_celsius') for m in recent_metrics if m.get('temperature_celsius')]
            if temps:
                avg_temp = np.mean(temps)
        
        # System CPU bottleneck
        if avg_cpu > self.cpu_bottleneck_threshold:
            severity = min(1.0, avg_cpu / 100.0)
            bottleneck = SystemBottleneck(
                bottleneck_type="system_cpu",
                severity=severity,
                affected_components=list(self.registered_components.keys()),
                root_cause=f"System CPU overload: {avg_cpu:.1f}%",
                impact_description="Overall system CPU usage is too high",
                recommended_actions=[
                    "Reduce component processing rates",
                    "Implement system-wide load balancing",
                    "Disable non-critical components",
                    "Optimize system configuration"
                ],
                estimated_improvement=25.0,
                timestamp=current_time
            )
            bottlenecks.append(bottleneck)
        
        # System memory bottleneck
        if avg_memory > self.memory_bottleneck_threshold:
            severity = min(1.0, avg_memory / 100.0)
            bottleneck = SystemBottleneck(
                bottleneck_type="system_memory",
                severity=severity,
                affected_components=list(self.registered_components.keys()),
                root_cause=f"System memory pressure: {avg_memory:.1f}%",
                impact_description="Overall system memory usage is too high",
                recommended_actions=[
                    "Implement memory cleanup",
                    "Reduce buffer sizes globally",
                    "Enable swap if available",
                    "Restart memory-intensive components"
                ],
                estimated_improvement=20.0,
                timestamp=current_time
            )
            bottlenecks.append(bottleneck)
        
        # Temperature bottleneck (Raspberry Pi)
        if avg_temp and avg_temp > 75.0:  # 75°C threshold
            severity = min(1.0, (avg_temp - 60.0) / 20.0)  # Scale from 60-80°C
            bottleneck = SystemBottleneck(
                bottleneck_type="thermal",
                severity=severity,
                affected_components=list(self.registered_components.keys()),
                root_cause=f"High system temperature: {avg_temp:.1f}°C",
                impact_description="System is overheating, may cause throttling",
                recommended_actions=[
                    "Reduce CPU-intensive operations",
                    "Implement thermal throttling",
                    "Check cooling system",
                    "Reduce processing quality"
                ],
                estimated_improvement=15.0,
                timestamp=current_time
            )
            bottlenecks.append(bottleneck)
        
        return bottlenecks
    
    def _publish_bottleneck_alert(self, bottleneck: SystemBottleneck):
        """Publish a bottleneck alert"""
        alert_data = {
            'type': 'BOTTLENECK_ALERT',
            'bottleneck': bottleneck.to_dict(),
            'timestamp': time.time()
        }
        
        alert_msg = String()
        alert_msg.data = json.dumps(alert_data)
        self.bottleneck_pub.publish(alert_msg)
        
        rospy.logwarn(f"[{self.node_name}] BOTTLENECK ALERT: {bottleneck.bottleneck_type} - {bottleneck.root_cause}")
        
        # Real-time monitoring
        rospy.logwarn(f"[{self.node_name}] Bottleneck alert event: Type={bottleneck.bottleneck_type}, "
                     f"Severity={bottleneck.severity:.2f}, Components={bottleneck.affected_components}")
    
    def _optimization_callback(self, event):
        """Generate optimization recommendations"""
        if not self.profiling_active:
            return
        
        try:
            current_time = time.time()
            recommendations = []
            
            with self.profiling_lock:
                # Generate recommendations based on current bottlenecks
                for bottleneck in self.current_bottlenecks.values():
                    for action in bottleneck.recommended_actions:
                        recommendation = {
                            'type': 'optimization_recommendation',
                            'action': action,
                            'bottleneck_type': bottleneck.bottleneck_type,
                            'affected_components': bottleneck.affected_components,
                            'estimated_improvement': bottleneck.estimated_improvement,
                            'priority': bottleneck.severity,
                            'timestamp': current_time
                        }
                        recommendations.append(recommendation)
                
                # Generate proactive recommendations
                proactive_recommendations = self._generate_proactive_recommendations()
                recommendations.extend(proactive_recommendations)
            
            # Publish recommendations
            if recommendations:
                # Sort by priority (severity)
                recommendations.sort(key=lambda x: x.get('priority', 0), reverse=True)
                
                for rec in recommendations[:5]:  # Top 5 recommendations
                    rec_msg = String()
                    rec_msg.data = json.dumps(rec)
                    self.optimization_pub.publish(rec_msg)
                    
                    self.optimizations_suggested += 1
                
                rospy.loginfo(f"[{self.node_name}] Generated {len(recommendations)} optimization recommendations")
                
                # Comprehensive logging
                rospy.logdebug(f"[{self.node_name}] Optimization recommendations generated:")
                for i, rec in enumerate(recommendations[:3]):  # Log top 3
                    rospy.logdebug(f"[{self.node_name}] {i+1}. {rec['action']} (Priority: {rec.get('priority', 0):.2f})")
                
                # Real-time monitoring
                rospy.loginfo(f"[{self.node_name}] Optimization event: {len(recommendations)} recommendations generated")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Optimization callback error: {e}")
    
    def _generate_proactive_recommendations(self) -> List[Dict]:
        """Generate proactive optimization recommendations"""
        recommendations = []
        current_time = time.time()
        
        # Analyze trends in performance data
        for component_name, profiles in self.performance_profiles.items():
            if len(profiles) < 10:  # Need sufficient data
                continue
            
            recent_profiles = list(profiles)[-10:]
            
            # Check for degrading performance trends
            fps_trend = np.polyfit(range(len(recent_profiles)), [p.fps for p in recent_profiles], 1)[0]
            latency_trend = np.polyfit(range(len(recent_profiles)), [p.latency_p50_ms for p in recent_profiles], 1)[0]
            efficiency_trend = np.polyfit(range(len(recent_profiles)), [p.resource_efficiency for p in recent_profiles], 1)[0]
            
            # Declining FPS trend
            if fps_trend < -0.5:  # FPS declining by 0.5 per measurement
                recommendations.append({
                    'type': 'proactive_recommendation',
                    'action': 'investigate_fps_decline',
                    'component': component_name,
                    'trend': fps_trend,
                    'description': f'FPS is declining for {component_name}',
                    'priority': 0.7,
                    'timestamp': current_time
                })
            
            # Increasing latency trend
            if latency_trend > 5.0:  # Latency increasing by 5ms per measurement
                recommendations.append({
                    'type': 'proactive_recommendation',
                    'action': 'optimize_latency',
                    'component': component_name,
                    'trend': latency_trend,
                    'description': f'Latency is increasing for {component_name}',
                    'priority': 0.8,
                    'timestamp': current_time
                })
            
            # Declining efficiency trend
            if efficiency_trend < -0.02:  # Efficiency declining by 2% per measurement
                recommendations.append({
                    'type': 'proactive_recommendation',
                    'action': 'improve_efficiency',
                    'component': component_name,
                    'trend': efficiency_trend,
                    'description': f'Resource efficiency is declining for {component_name}',
                    'priority': 0.6,
                    'timestamp': current_time
                })
        
        return recommendations
    
    def _easy_logs_callback(self, event):
        """Integrate with easy_logs framework for performance logging"""
        if not self.easy_logs_db:
            return
        
        try:
            current_time = time.time()
            
            # Create performance log entry
            log_entry = {
                'timestamp': current_time,
                'system_efficiency': self.system_efficiency_score,
                'total_profiles': self.total_profiles_collected,
                'bottlenecks_identified': self.bottlenecks_identified,
                'optimizations_suggested': self.optimizations_suggested,
                'component_count': len(self.registered_components),
                'active_bottlenecks': len(self.current_bottlenecks)
            }
            
            # Add component summaries
            component_summaries = {}
            with self.profiling_lock:
                for component_name, profiles in self.performance_profiles.items():
                    if profiles:
                        recent_profiles = list(profiles)[-5:]
                        avg_fps = np.mean([p.fps for p in recent_profiles])
                        avg_efficiency = np.mean([p.resource_efficiency for p in recent_profiles])
                        avg_latency = np.mean([p.latency_p50_ms for p in recent_profiles])
                        
                        component_summaries[component_name] = {
                            'average_fps': avg_fps,
                            'average_efficiency': avg_efficiency,
                            'average_latency_ms': avg_latency,
                            'profile_count': len(profiles)
                        }
            
            log_entry['components'] = component_summaries
            
            # Log to easy_logs (simplified integration)
            rospy.loginfo(f"[{self.node_name}] Easy logs integration: {json.dumps(log_entry, indent=2)}")
            
            # Comprehensive logging
            rospy.logdebug(f"[{self.node_name}] Easy logs integration event: "
                          f"Efficiency={self.system_efficiency_score:.2f}, "
                          f"Components={len(component_summaries)}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Easy logs integration error: {e}")
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get comprehensive performance summary"""
        with self.profiling_lock:
            current_time = time.time()
            
            # Component summaries
            component_summaries = {}
            for component_name, profiles in self.performance_profiles.items():
                if profiles:
                    recent_profiles = list(profiles)[-10:]
                    component_summaries[component_name] = {
                        'profile_count': len(profiles),
                        'average_fps': np.mean([p.fps for p in recent_profiles]),
                        'average_efficiency': np.mean([p.resource_efficiency for p in recent_profiles]),
                        'average_latency_ms': np.mean([p.latency_p50_ms for p in recent_profiles]),
                        'average_cpu_percent': np.mean([p.cpu_usage_percent for p in recent_profiles]),
                        'average_memory_mb': np.mean([p.memory_usage_mb for p in recent_profiles]),
                        'optimization_opportunities': len(recent_profiles[-1].optimization_opportunities) if recent_profiles else 0
                    }
            
            # System summary
            system_summary = {}
            if self.system_metrics_history:
                recent_metrics = list(self.system_metrics_history)[-10:]
                system_summary = {
                    'average_cpu_percent': np.mean([m.get('cpu', {}).get('percent', 0) for m in recent_metrics]),
                    'average_memory_percent': np.mean([m.get('memory', {}).get('percent', 0) for m in recent_metrics]),
                    'average_temperature_celsius': None
                }
                
                if self.raspberry_pi_mode:
                    temps = [m.get('temperature_celsius') for m in recent_metrics if m.get('temperature_celsius')]
                    if temps:
                        system_summary['average_temperature_celsius'] = np.mean(temps)
            
            # Bottleneck summary
            bottleneck_summary = {
                'active_bottlenecks': len(self.current_bottlenecks),
                'total_bottlenecks_identified': self.bottlenecks_identified,
                'bottleneck_types': list(set(b.bottleneck_type for b in self.current_bottlenecks.values()))
            }
            
            return {
                'timestamp': current_time,
                'system_efficiency_score': self.system_efficiency_score,
                'total_profiles_collected': self.total_profiles_collected,
                'optimizations_suggested': self.optimizations_suggested,
                'registered_components': len(self.registered_components),
                'component_summaries': component_summaries,
                'system_summary': system_summary,
                'bottleneck_summary': bottleneck_summary,
                'raspberry_pi_mode': self.raspberry_pi_mode
            }
    
    def shutdown(self):
        """Shutdown the performance profiler"""
        rospy.loginfo(f"[{self.node_name}] Enhanced Performance Profiler shutting down")
        
        self.profiling_active = False
        
        # Shutdown timers
        if hasattr(self, 'profiling_timer'):
            self.profiling_timer.shutdown()
        if hasattr(self, 'bottleneck_timer'):
            self.bottleneck_timer.shutdown()
        if hasattr(self, 'optimization_timer'):
            self.optimization_timer.shutdown()
        if hasattr(self, 'easy_logs_timer'):
            self.easy_logs_timer.shutdown()
        
        # Log final summary
        summary = self.get_performance_summary()
        rospy.loginfo(f"[{self.node_name}] Final performance summary:")
        rospy.loginfo(f"[{self.node_name}] System efficiency: {summary['system_efficiency_score']:.2f}")
        rospy.loginfo(f"[{self.node_name}] Total profiles: {summary['total_profiles_collected']}")
        rospy.loginfo(f"[{self.node_name}] Bottlenecks identified: {summary['bottleneck_summary']['total_bottlenecks_identified']}")
        rospy.loginfo(f"[{self.node_name}] Optimizations suggested: {summary['optimizations_suggested']}")
        rospy.loginfo(f"[{self.node_name}] Registered components: {summary['registered_components']}")
        
        rospy.loginfo(f"[{self.node_name}] Enhanced Performance Profiler shutdown complete")


def main():
    """Main function to run the enhanced performance profiler"""
    try:
        profiler = EnhancedPerformanceProfiler()
        rospy.loginfo("Enhanced Performance Profiler running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Enhanced Performance Profiler interrupted")
    except Exception as e:
        rospy.logerr(f"Enhanced Performance Profiler error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
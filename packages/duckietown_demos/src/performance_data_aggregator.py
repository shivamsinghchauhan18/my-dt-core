#!/usr/bin/env python3
"""
Performance Data Aggregator for Enhanced Autonomous Navigation

Aggregates performance data from all enhanced components and provides
unified performance metrics and resource monitoring data.

Author: Duckietown
"""

import rospy
import time
import threading
import json
import psutil
from typing import Dict, List, Optional, Any
from collections import deque, defaultdict
from dataclasses import dataclass, asdict
import numpy as np

# ROS messages
from std_msgs.msg import String, Float32, Header


@dataclass
class ComponentPerformanceData:
    """Performance data for a single component"""
    component_name: str
    timestamp: float
    fps: float
    avg_latency_ms: float
    max_latency_ms: float
    cpu_usage_percent: float
    memory_usage_mb: float
    queue_depth: int
    error_count: int
    processing_time_ms: float
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class AggregatedPerformanceData:
    """Aggregated performance data from all components"""
    timestamp: float
    system_fps: float
    total_cpu_usage: float
    total_memory_usage_mb: float
    average_latency_ms: float
    max_latency_ms: float
    total_queue_depth: int
    total_errors: int
    component_count: int
    system_efficiency: float
    resource_pressure: str
    components: Dict[str, ComponentPerformanceData]
    
    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data['components'] = {k: v.to_dict() for k, v in self.components.items()}
        return data


class PerformanceDataAggregator:
    """
    Aggregates performance data from all enhanced components and provides
    unified metrics for monitoring and optimization systems.
    """
    
    def __init__(self):
        self.node_name = "performance_data_aggregator"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Configuration
        self.aggregation_frequency = rospy.get_param("~aggregation_frequency", 1.0)  # Hz
        self.data_retention_seconds = rospy.get_param("~data_retention_seconds", 300)
        self.raspberry_pi_mode = rospy.get_param("~raspberry_pi_mode", True)
        self.enable_debug_logging = rospy.get_param("~enable_debug_logging", False)
        
        # Data storage
        self.component_data: Dict[str, deque] = defaultdict(lambda: deque(maxlen=100))
        self.aggregated_history: deque = deque(maxlen=300)  # 5 minutes at 1Hz
        self.system_metrics_history: deque = deque(maxlen=100)
        
        # Component registration
        self.registered_components: Dict[str, Dict] = {}
        self.component_last_update: Dict[str, float] = {}
        
        # Threading
        self.aggregation_active = True
        self.data_lock = threading.Lock()
        
        # Statistics
        self.total_data_points = 0
        self.aggregation_cycles = 0
        self.data_loss_count = 0
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] Performance Data Aggregator initialized")
        rospy.loginfo(f"[{self.node_name}] Aggregation frequency: {self.aggregation_frequency} Hz")
        rospy.loginfo(f"[{self.node_name}] Data retention: {self.data_retention_seconds} seconds")
        rospy.loginfo(f"[{self.node_name}] Raspberry Pi mode: {self.raspberry_pi_mode}")
    
    def _setup_subscribers(self):
        """Setup subscribers for component performance data"""
        rospy.loginfo(f"[{self.node_name}] Setting up performance data subscribers...")
        
        # YOLO detector performance
        self.yolo_sub = rospy.Subscriber(
            "~yolo_performance",
            String,
            lambda msg: self._component_performance_callback(msg, "yolo_detector"),
            queue_size=10
        )
        
        # Navigation performance
        self.navigation_sub = rospy.Subscriber(
            "~navigation_performance",
            String,
            lambda msg: self._component_performance_callback(msg, "navigation"),
            queue_size=10
        )
        
        # Lane detection performance
        self.lane_detection_sub = rospy.Subscriber(
            "~lane_detection_performance",
            String,
            lambda msg: self._component_performance_callback(msg, "lane_detection"),
            queue_size=10
        )
        
        # AprilTag detection performance
        self.apriltag_sub = rospy.Subscriber(
            "~apriltag_performance",
            String,
            lambda msg: self._component_performance_callback(msg, "apriltag_detection"),
            queue_size=10
        )
        
        # Integration coordinator performance
        self.integration_sub = rospy.Subscriber(
            "~integration_performance",
            String,
            lambda msg: self._component_performance_callback(msg, "integration_coordinator"),
            queue_size=10
        )
        
        # Generic component performance (for extensibility)
        self.generic_sub = rospy.Subscriber(
            "/performance_data/component_performance",
            String,
            self._generic_component_callback,
            queue_size=10
        )
        
        rospy.loginfo(f"[{self.node_name}] Performance data subscribers initialized")
    
    def _setup_publishers(self):
        """Setup publishers for aggregated performance data"""
        rospy.loginfo(f"[{self.node_name}] Setting up aggregated data publishers...")
        
        # Aggregated performance data
        self.aggregated_pub = rospy.Publisher(
            "/performance_data/aggregated",
            String,
            queue_size=1
        )
        
        # Resource metrics for quality adjuster
        self.resource_metrics_pub = rospy.Publisher(
            "/performance_data/resource_metrics",
            String,
            queue_size=1
        )
        
        # Component feedback for quality adjuster
        self.component_feedback_pub = rospy.Publisher(
            "/performance_data/component_feedback",
            String,
            queue_size=10
        )
        
        # Component metrics for alerting system
        self.component_metrics_pub = rospy.Publisher(
            "/performance_data/component_metrics",
            String,
            queue_size=10
        )
        
        # System efficiency score
        self.efficiency_pub = rospy.Publisher(
            "/performance_data/system_efficiency",
            Float32,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Aggregated data publishers initialized")
    
    def _setup_timers(self):
        """Setup periodic aggregation timers"""
        # Main aggregation timer
        self.aggregation_timer = rospy.Timer(
            rospy.Duration(1.0 / self.aggregation_frequency),
            self._aggregation_callback
        )
        
        # System metrics collection timer
        self.system_metrics_timer = rospy.Timer(
            rospy.Duration(2.0),  # Every 2 seconds
            self._system_metrics_callback
        )
        
        # Data cleanup timer
        self.cleanup_timer = rospy.Timer(
            rospy.Duration(60.0),  # Every minute
            self._cleanup_callback
        )
        
        rospy.loginfo(f"[{self.node_name}] Aggregation timers initialized")
    
    def _component_performance_callback(self, msg, component_name: str):
        """Handle component performance data"""
        try:
            # Parse performance data
            perf_data = json.loads(msg.data)
            current_time = time.time()
            
            # Create component performance data
            component_perf = ComponentPerformanceData(
                component_name=component_name,
                timestamp=current_time,
                fps=perf_data.get('fps', 0.0),
                avg_latency_ms=perf_data.get('avg_latency_ms', 0.0),
                max_latency_ms=perf_data.get('max_latency_ms', 0.0),
                cpu_usage_percent=perf_data.get('cpu_usage_percent', 0.0),
                memory_usage_mb=perf_data.get('memory_usage_mb', 0.0),
                queue_depth=perf_data.get('queue_depth', 0),
                error_count=perf_data.get('error_count', 0),
                processing_time_ms=perf_data.get('processing_time_ms', 0.0)
            )
            
            with self.data_lock:
                self.component_data[component_name].append(component_perf)
                self.component_last_update[component_name] = current_time
                self.total_data_points += 1
            
            # Register component if not already registered
            if component_name not in self.registered_components:
                self.registered_components[component_name] = {
                    'registration_time': current_time,
                    'data_points': 0,
                    'last_fps': 0.0,
                    'last_latency': 0.0
                }
            
            self.registered_components[component_name]['data_points'] += 1
            self.registered_components[component_name]['last_fps'] = component_perf.fps
            self.registered_components[component_name]['last_latency'] = component_perf.avg_latency_ms
            
            if self.enable_debug_logging:
                rospy.logdebug(f"[{self.node_name}] Performance data from '{component_name}': "
                              f"FPS={component_perf.fps:.1f}, Latency={component_perf.avg_latency_ms:.1f}ms")
            
            # Publish component feedback for quality adjuster
            self._publish_component_feedback(component_perf)
            
            # Publish component metrics for alerting system
            self._publish_component_metrics(component_perf)
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing performance data from '{component_name}': {e}")
            self.data_loss_count += 1
    
    def _generic_component_callback(self, msg):
        """Handle generic component performance data"""
        try:
            perf_data = json.loads(msg.data)
            component_name = perf_data.get('component_name', 'unknown')
            
            if component_name != 'unknown':
                self._component_performance_callback(msg, component_name)
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing generic component data: {e}")
    
    def _aggregation_callback(self, event):
        """Main aggregation callback"""
        if not self.aggregation_active:
            return
        
        try:
            current_time = time.time()
            
            # Collect recent component data
            recent_components = {}
            total_fps = 0.0
            total_cpu = 0.0
            total_memory = 0.0
            total_queue_depth = 0
            total_errors = 0
            latencies = []
            active_components = 0
            
            with self.data_lock:
                for component_name, data_queue in self.component_data.items():
                    if not data_queue:
                        continue
                    
                    # Get most recent data point
                    latest_data = data_queue[-1]
                    
                    # Check if data is recent (within last 10 seconds)
                    if (current_time - latest_data.timestamp) > 10.0:
                        continue
                    
                    recent_components[component_name] = latest_data
                    active_components += 1
                    
                    # Aggregate metrics
                    total_fps += latest_data.fps
                    total_cpu += latest_data.cpu_usage_percent
                    total_memory += latest_data.memory_usage_mb
                    total_queue_depth += latest_data.queue_depth
                    total_errors += latest_data.error_count
                    
                    if latest_data.avg_latency_ms > 0:
                        latencies.append(latest_data.avg_latency_ms)
            
            if active_components == 0:
                if self.enable_debug_logging:
                    rospy.logdebug(f"[{self.node_name}] No active components for aggregation")
                return
            
            # Calculate aggregated metrics
            system_fps = total_fps / active_components if active_components > 0 else 0.0
            average_latency = np.mean(latencies) if latencies else 0.0
            max_latency = np.max(latencies) if latencies else 0.0
            
            # Calculate system efficiency
            system_efficiency = self._calculate_system_efficiency(recent_components)
            
            # Determine resource pressure
            resource_pressure = self._determine_resource_pressure(total_cpu, total_memory, system_efficiency)
            
            # Create aggregated data
            aggregated_data = AggregatedPerformanceData(
                timestamp=current_time,
                system_fps=system_fps,
                total_cpu_usage=total_cpu,
                total_memory_usage_mb=total_memory,
                average_latency_ms=average_latency,
                max_latency_ms=max_latency,
                total_queue_depth=total_queue_depth,
                total_errors=total_errors,
                component_count=active_components,
                system_efficiency=system_efficiency,
                resource_pressure=resource_pressure,
                components=recent_components
            )
            
            # Store aggregated data
            with self.data_lock:
                self.aggregated_history.append(aggregated_data)
                self.aggregation_cycles += 1
            
            # Publish aggregated data
            self._publish_aggregated_data(aggregated_data)
            
            # Publish resource metrics
            self._publish_resource_metrics(aggregated_data)
            
            # Publish system efficiency
            efficiency_msg = Float32()
            efficiency_msg.data = system_efficiency
            self.efficiency_pub.publish(efficiency_msg)
            
            if self.enable_debug_logging:
                rospy.logdebug(f"[{self.node_name}] Aggregation completed: "
                              f"Components={active_components}, FPS={system_fps:.1f}, "
                              f"Efficiency={system_efficiency:.2f}")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Aggregation event: "
                          f"Components={active_components}, Efficiency={system_efficiency:.2f}, "
                          f"Pressure={resource_pressure}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Aggregation callback error: {e}")
    
    def _system_metrics_callback(self, event):
        """Collect system-level metrics"""
        try:
            current_time = time.time()
            
            # Get system metrics
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory = psutil.virtual_memory()
            
            # Temperature (Raspberry Pi)
            temperature = None
            if self.raspberry_pi_mode:
                try:
                    with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                        temperature = float(f.read().strip()) / 1000.0
                except:
                    temperature = None
            
            system_metrics = {
                'timestamp': current_time,
                'cpu_percent': cpu_percent,
                'memory_percent': memory.percent,
                'memory_available_mb': memory.available / 1024 / 1024,
                'temperature_celsius': temperature
            }
            
            with self.data_lock:
                self.system_metrics_history.append(system_metrics)
            
            if self.enable_debug_logging:
                rospy.logdebug(f"[{self.node_name}] System metrics: "
                              f"CPU={cpu_percent:.1f}%, Memory={memory.percent:.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] System metrics collection error: {e}")
    
    def _calculate_system_efficiency(self, components: Dict[str, ComponentPerformanceData]) -> float:
        """Calculate overall system efficiency score"""
        if not components:
            return 1.0
        
        efficiency_scores = []
        
        for component_name, data in components.items():
            # FPS efficiency (target varies by component)
            target_fps = self._get_target_fps(component_name)
            fps_efficiency = min(1.0, data.fps / target_fps) if target_fps > 0 else 1.0
            
            # Latency efficiency (lower is better)
            target_latency = self._get_target_latency(component_name)
            latency_efficiency = max(0.0, 1.0 - (data.avg_latency_ms / target_latency)) if target_latency > 0 else 1.0
            
            # CPU efficiency (lower usage is better for given performance)
            cpu_efficiency = max(0.0, 1.0 - (data.cpu_usage_percent / 100.0))
            
            # Memory efficiency
            memory_efficiency = max(0.0, 1.0 - (data.memory_usage_mb / 1024.0))  # Normalize to 1GB
            
            # Error efficiency (no errors is best)
            error_efficiency = 1.0 if data.error_count == 0 else max(0.0, 1.0 - (data.error_count / 10.0))
            
            # Component efficiency is weighted average
            component_efficiency = np.mean([
                fps_efficiency * 0.3,
                latency_efficiency * 0.3,
                cpu_efficiency * 0.2,
                memory_efficiency * 0.1,
                error_efficiency * 0.1
            ])
            
            efficiency_scores.append(component_efficiency)
        
        return np.mean(efficiency_scores)
    
    def _get_target_fps(self, component_name: str) -> float:
        """Get target FPS for a component"""
        targets = {
            'lane_detection': 20.0,
            'apriltag_detection': 15.0,
            'yolo_detector': 10.0,
            'navigation': 20.0,
            'integration_coordinator': 10.0
        }
        return targets.get(component_name, 15.0)
    
    def _get_target_latency(self, component_name: str) -> float:
        """Get target latency for a component"""
        targets = {
            'lane_detection': 150.0,
            'apriltag_detection': 200.0,
            'yolo_detector': 300.0,
            'navigation': 100.0,
            'integration_coordinator': 50.0
        }
        return targets.get(component_name, 200.0)
    
    def _determine_resource_pressure(self, total_cpu: float, total_memory: float, efficiency: float) -> str:
        """Determine overall resource pressure level"""
        # Get system metrics
        system_cpu = 0.0
        system_memory = 0.0
        
        if self.system_metrics_history:
            latest_system = self.system_metrics_history[-1]
            system_cpu = latest_system['cpu_percent']
            system_memory = latest_system['memory_percent']
        
        # Determine pressure based on multiple factors
        pressure_score = 0
        
        # CPU pressure
        if system_cpu > 90 or total_cpu > 200:  # 200% for multi-component
            pressure_score += 4
        elif system_cpu > 80 or total_cpu > 150:
            pressure_score += 3
        elif system_cpu > 70 or total_cpu > 100:
            pressure_score += 2
        elif system_cpu > 60 or total_cpu > 75:
            pressure_score += 1
        
        # Memory pressure
        if system_memory > 95:
            pressure_score += 4
        elif system_memory > 85:
            pressure_score += 3
        elif system_memory > 75:
            pressure_score += 2
        elif system_memory > 65:
            pressure_score += 1
        
        # Efficiency pressure
        if efficiency < 0.3:
            pressure_score += 3
        elif efficiency < 0.5:
            pressure_score += 2
        elif efficiency < 0.7:
            pressure_score += 1
        
        # Map pressure score to level
        if pressure_score >= 8:
            return "CRITICAL"
        elif pressure_score >= 6:
            return "HIGH"
        elif pressure_score >= 4:
            return "MEDIUM"
        elif pressure_score >= 2:
            return "LOW"
        else:
            return "NONE"
    
    def _publish_aggregated_data(self, data: AggregatedPerformanceData):
        """Publish aggregated performance data"""
        msg = String()
        msg.data = json.dumps(data.to_dict())
        self.aggregated_pub.publish(msg)
    
    def _publish_resource_metrics(self, data: AggregatedPerformanceData):
        """Publish resource metrics for quality adjuster"""
        # Get system metrics
        system_metrics = {}
        if self.system_metrics_history:
            system_metrics = self.system_metrics_history[-1]
        
        resource_data = {
            'timestamp': data.timestamp,
            'cpu_percent': system_metrics.get('cpu_percent', 0.0),
            'memory_percent': system_metrics.get('memory_percent', 0.0),
            'memory_available_mb': system_metrics.get('memory_available_mb', 1024.0),
            'temperature_celsius': system_metrics.get('temperature_celsius'),
            'processing_fps': data.system_fps,
            'target_fps': 20.0 if not self.raspberry_pi_mode else 15.0,
            'latency_ms': data.average_latency_ms,
            'queue_depth': data.total_queue_depth
        }
        
        msg = String()
        msg.data = json.dumps(resource_data)
        self.resource_metrics_pub.publish(msg)
    
    def _publish_component_feedback(self, component_data: ComponentPerformanceData):
        """Publish component feedback for quality adjuster"""
        feedback_data = {
            'component_name': component_data.component_name,
            'fps': component_data.fps,
            'latency_ms': component_data.avg_latency_ms,
            'cpu_usage': component_data.cpu_usage_percent,
            'memory_usage': component_data.memory_usage_mb,
            'timestamp': component_data.timestamp
        }
        
        msg = String()
        msg.data = json.dumps(feedback_data)
        self.component_feedback_pub.publish(msg)
    
    def _publish_component_metrics(self, component_data: ComponentPerformanceData):
        """Publish component metrics for alerting system"""
        metrics_data = {
            'component': component_data.component_name,
            'cpu_percent': component_data.cpu_usage_percent,
            'memory_mb': component_data.memory_usage_mb,
            'processing_fps': component_data.fps,
            'latency_ms': component_data.avg_latency_ms,
            'queue_depth': component_data.queue_depth,
            'error_count': component_data.error_count,
            'timestamp': component_data.timestamp
        }
        
        msg = String()
        msg.data = json.dumps(metrics_data)
        self.component_metrics_pub.publish(msg)
    
    def _cleanup_callback(self, event):
        """Cleanup old data"""
        try:
            current_time = time.time()
            retention_threshold = current_time - self.data_retention_seconds
            
            with self.data_lock:
                # Clean up component data
                for component_name, data_queue in self.component_data.items():
                    while data_queue and data_queue[0].timestamp < retention_threshold:
                        data_queue.popleft()
                
                # Clean up system metrics
                while (self.system_metrics_history and 
                       self.system_metrics_history[0]['timestamp'] < retention_threshold):
                    self.system_metrics_history.popleft()
            
            if self.enable_debug_logging:
                rospy.logdebug(f"[{self.node_name}] Data cleanup completed")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Cleanup callback error: {e}")
    
    def get_aggregation_statistics(self) -> Dict[str, Any]:
        """Get aggregation statistics"""
        with self.data_lock:
            current_time = time.time()
            
            # Component statistics
            component_stats = {}
            for component_name, info in self.registered_components.items():
                last_update = self.component_last_update.get(component_name, 0)
                component_stats[component_name] = {
                    'data_points': info['data_points'],
                    'last_fps': info['last_fps'],
                    'last_latency': info['last_latency'],
                    'last_update_age': current_time - last_update,
                    'active': (current_time - last_update) < 10.0
                }
            
            return {
                'timestamp': current_time,
                'total_data_points': self.total_data_points,
                'aggregation_cycles': self.aggregation_cycles,
                'data_loss_count': self.data_loss_count,
                'registered_components': len(self.registered_components),
                'active_components': sum(1 for stats in component_stats.values() if stats['active']),
                'component_statistics': component_stats,
                'aggregated_history_length': len(self.aggregated_history),
                'system_metrics_history_length': len(self.system_metrics_history)
            }
    
    def shutdown(self):
        """Shutdown the performance data aggregator"""
        rospy.loginfo(f"[{self.node_name}] Performance Data Aggregator shutting down")
        
        self.aggregation_active = False
        
        # Shutdown timers
        if hasattr(self, 'aggregation_timer'):
            self.aggregation_timer.shutdown()
        if hasattr(self, 'system_metrics_timer'):
            self.system_metrics_timer.shutdown()
        if hasattr(self, 'cleanup_timer'):
            self.cleanup_timer.shutdown()
        
        # Log final statistics
        stats = self.get_aggregation_statistics()
        rospy.loginfo(f"[{self.node_name}] Final aggregation statistics:")
        rospy.loginfo(f"[{self.node_name}] Total data points: {stats['total_data_points']}")
        rospy.loginfo(f"[{self.node_name}] Aggregation cycles: {stats['aggregation_cycles']}")
        rospy.loginfo(f"[{self.node_name}] Data loss count: {stats['data_loss_count']}")
        rospy.loginfo(f"[{self.node_name}] Registered components: {stats['registered_components']}")
        rospy.loginfo(f"[{self.node_name}] Active components: {stats['active_components']}")
        
        rospy.loginfo(f"[{self.node_name}] Performance Data Aggregator shutdown complete")


def main():
    """Main function to run the performance data aggregator"""
    try:
        aggregator = PerformanceDataAggregator()
        rospy.loginfo("Performance Data Aggregator running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Performance Data Aggregator interrupted")
    except Exception as e:
        rospy.logerr(f"Performance Data Aggregator error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
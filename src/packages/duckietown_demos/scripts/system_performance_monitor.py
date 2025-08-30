#!/usr/bin/env python3
"""
System Performance Monitor Node

Monitors system performance metrics, resource usage, and component health
for the enhanced autonomous system.

Author: Enhanced Autonomous Duckietown System
"""

import rospy
import psutil
import time
import threading
from collections import deque
from datetime import datetime
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import CompressedImage

# Try to import custom messages
try:
    from duckietown_enhanced_msgs.msg import PerformanceMetrics
except ImportError:
    rospy.logwarn("PerformanceMetrics message not available, using String")
    from std_msgs.msg import String as PerformanceMetrics


class SystemPerformanceMonitor:
    """
    Monitors and reports system performance metrics including:
    - CPU and memory usage
    - Frame rates and processing latencies
    - Component health and responsiveness
    - Resource optimization recommendations
    """
    
    def __init__(self):
        rospy.init_node('system_performance_monitor', anonymous=True)
        
        # Parameters
        self.monitoring_frequency = rospy.get_param('~monitoring_frequency', 1.0)
        self.performance_window_size = rospy.get_param('~performance_window_size', 30)
        self.enable_optimization_suggestions = rospy.get_param('~enable_optimization_suggestions', True)
        
        # Performance targets
        self.target_fps = rospy.get_param('~target_fps', 20.0)
        self.target_latency = rospy.get_param('~target_latency', 0.2)
        self.target_memory_usage = rospy.get_param('~target_memory_usage', 80.0)
        self.target_cpu_usage = rospy.get_param('~target_cpu_usage', 70.0)
        
        # Performance tracking
        self.fps_history = deque(maxlen=self.performance_window_size)
        self.latency_history = deque(maxlen=self.performance_window_size)
        self.cpu_history = deque(maxlen=self.performance_window_size)
        self.memory_history = deque(maxlen=self.performance_window_size)
        
        # Component timing
        self.component_timings = {}
        self.last_frame_time = None
        self.frame_count = 0
        
        # Publishers
        self.performance_pub = rospy.Publisher('~performance_metrics', PerformanceMetrics, queue_size=1)
        self.optimization_pub = rospy.Publisher('~optimization_suggestions', String, queue_size=1)
        self.fps_pub = rospy.Publisher('~current_fps', Float32, queue_size=1)
        self.latency_pub = rospy.Publisher('~current_latency', Float32, queue_size=1)
        
        # Subscribers for performance monitoring
        rospy.Subscriber('/*/camera_node/image/compressed', CompressedImage, self.image_callback)
        
        # Monitoring thread
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop)
        self.monitoring_thread.daemon = True
        self.shutdown_requested = False
        
        rospy.loginfo("System Performance Monitor initialized")
        
    def image_callback(self, msg):
        """Track image processing for FPS calculation."""
        current_time = rospy.Time.now()
        
        if self.last_frame_time is not None:
            frame_interval = (current_time - self.last_frame_time).to_sec()
            if frame_interval > 0:
                fps = 1.0 / frame_interval
                self.fps_history.append(fps)
                
                # Publish current FPS
                fps_msg = Float32()
                fps_msg.data = fps
                self.fps_pub.publish(fps_msg)
                
        self.last_frame_time = current_time
        self.frame_count += 1
        
    def collect_system_metrics(self):
        """Collect current system performance metrics."""
        try:
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=0.1)
            self.cpu_history.append(cpu_percent)
            
            # Memory usage
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            self.memory_history.append(memory_percent)
            
            # Disk usage
            disk = psutil.disk_usage('/')
            disk_percent = disk.percent
            
            # Network I/O
            network = psutil.net_io_counters()
            
            # Process count
            process_count = len(psutil.pids())
            
            return {
                'cpu_percent': cpu_percent,
                'memory_percent': memory_percent,
                'disk_percent': disk_percent,
                'process_count': process_count,
                'network_bytes_sent': network.bytes_sent,
                'network_bytes_recv': network.bytes_recv
            }
            
        except Exception as e:
            rospy.logerr(f"Failed to collect system metrics: {e}")
            return {}
            
    def calculate_performance_stats(self):
        """Calculate performance statistics from history."""
        stats = {}
        
        # FPS statistics
        if self.fps_history:
            stats['fps_current'] = self.fps_history[-1]
            stats['fps_average'] = sum(self.fps_history) / len(self.fps_history)
            stats['fps_min'] = min(self.fps_history)
            stats['fps_max'] = max(self.fps_history)
        else:
            stats['fps_current'] = 0.0
            stats['fps_average'] = 0.0
            stats['fps_min'] = 0.0
            stats['fps_max'] = 0.0
            
        # Latency statistics
        if self.latency_history:
            stats['latency_current'] = self.latency_history[-1]
            stats['latency_average'] = sum(self.latency_history) / len(self.latency_history)
            stats['latency_min'] = min(self.latency_history)
            stats['latency_max'] = max(self.latency_history)
        else:
            stats['latency_current'] = 0.0
            stats['latency_average'] = 0.0
            stats['latency_min'] = 0.0
            stats['latency_max'] = 0.0
            
        # CPU statistics
        if self.cpu_history:
            stats['cpu_average'] = sum(self.cpu_history) / len(self.cpu_history)
            stats['cpu_peak'] = max(self.cpu_history)
        else:
            stats['cpu_average'] = 0.0
            stats['cpu_peak'] = 0.0
            
        # Memory statistics
        if self.memory_history:
            stats['memory_average'] = sum(self.memory_history) / len(self.memory_history)
            stats['memory_peak'] = max(self.memory_history)
        else:
            stats['memory_average'] = 0.0
            stats['memory_peak'] = 0.0
            
        return stats
        
    def generate_optimization_suggestions(self, system_metrics, performance_stats):
        """Generate optimization suggestions based on performance data."""
        suggestions = []
        
        # FPS optimization
        if performance_stats['fps_average'] < self.target_fps * 0.8:
            suggestions.append("FPS below target: Consider reducing image resolution or processing complexity")
            
        # CPU optimization
        if system_metrics.get('cpu_percent', 0) > self.target_cpu_usage:
            suggestions.append("High CPU usage: Consider enabling frame skipping or reducing processing frequency")
            
        # Memory optimization
        if system_metrics.get('memory_percent', 0) > self.target_memory_usage:
            suggestions.append("High memory usage: Consider reducing buffer sizes or enabling garbage collection")
            
        # Latency optimization
        if performance_stats['latency_average'] > self.target_latency:
            suggestions.append("High latency: Consider optimizing algorithms or reducing processing pipeline depth")
            
        # Process optimization
        if system_metrics.get('process_count', 0) > 200:
            suggestions.append("High process count: Consider consolidating or reducing background processes")
            
        return suggestions
        
    def publish_performance_metrics(self, system_metrics, performance_stats):
        """Publish comprehensive performance metrics."""
        try:
            if hasattr(PerformanceMetrics, '_type'):  # Real PerformanceMetrics message
                msg = PerformanceMetrics()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "base_link"
                
                # Set performance fields (assuming they exist in the message)
                # msg.fps_current = performance_stats['fps_current']
                # msg.cpu_percent = system_metrics.get('cpu_percent', 0.0)
                # msg.memory_percent = system_metrics.get('memory_percent', 0.0)
                # ... (set other fields as defined in the message)
                
            else:  # Fallback to String
                msg = PerformanceMetrics()
                
                data_dict = {
                    'timestamp': rospy.Time.now().to_sec(),
                    'system_metrics': system_metrics,
                    'performance_stats': performance_stats,
                    'frame_count': self.frame_count
                }
                
                msg.data = str(data_dict)
                
            self.performance_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"Failed to publish performance metrics: {e}")
            
    def publish_optimization_suggestions(self, suggestions):
        """Publish optimization suggestions."""
        if suggestions and self.enable_optimization_suggestions:
            try:
                msg = String()
                msg.data = "; ".join(suggestions)
                self.optimization_pub.publish(msg)
                
                # Log suggestions periodically
                rospy.loginfo(f"Performance suggestions: {msg.data}")
                
            except Exception as e:
                rospy.logerr(f"Failed to publish optimization suggestions: {e}")
                
    def monitoring_loop(self):
        """Main monitoring loop."""
        rate = rospy.Rate(self.monitoring_frequency)
        
        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                # Collect system metrics
                system_metrics = self.collect_system_metrics()
                
                # Calculate performance statistics
                performance_stats = self.calculate_performance_stats()
                
                # Generate optimization suggestions
                suggestions = self.generate_optimization_suggestions(
                    system_metrics, performance_stats)
                
                # Publish metrics and suggestions
                self.publish_performance_metrics(system_metrics, performance_stats)
                self.publish_optimization_suggestions(suggestions)
                
                # Publish current latency (estimated from processing time)
                if performance_stats['fps_current'] > 0:
                    estimated_latency = 1.0 / performance_stats['fps_current']
                    self.latency_history.append(estimated_latency)
                    
                    latency_msg = Float32()
                    latency_msg.data = estimated_latency
                    self.latency_pub.publish(latency_msg)
                
            except Exception as e:
                rospy.logerr(f"Monitoring loop error: {e}")
                
            rate.sleep()
            
    def run(self):
        """Start the performance monitor."""
        self.monitoring_thread.start()
        rospy.loginfo("System Performance Monitor running")
        rospy.spin()
        
    def shutdown(self):
        """Shutdown the performance monitor."""
        rospy.loginfo("Shutting down System Performance Monitor")
        self.shutdown_requested = True
        if self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=2.0)


if __name__ == '__main__':
    try:
        node = SystemPerformanceMonitor()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"System Performance Monitor failed: {e}")

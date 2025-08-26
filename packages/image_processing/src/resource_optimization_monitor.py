#!/usr/bin/env python3
"""
Resource Optimization Monitor

ROS node wrapper for ResourceOptimizationUtils that provides continuous
monitoring and optimization of system resources.

Author: Duckietown
"""

import rospy
import time
import threading
import json
from typing import Dict, List, Optional, Any

# ROS messages
from std_msgs.msg import String, Float32, Bool

# Import the resource optimization utilities
from resource_optimization_utils import ResourceOptimizationUtils, ResourceUsage, OptimizationResult


class ResourceOptimizationMonitor:
    """
    ROS node wrapper for ResourceOptimizationUtils that provides continuous
    monitoring and optimization of system resources.
    """
    
    def __init__(self):
        self.node_name = "resource_optimization_monitor"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Configuration
        self.raspberry_pi_mode = rospy.get_param("~raspberry_pi_mode", True)
        self.optimization_interval = rospy.get_param("~optimization_interval", 30.0)
        self.memory_threshold_mb = rospy.get_param("~memory_threshold_mb", 1024)
        self.cpu_threshold_percent = rospy.get_param("~cpu_threshold_percent", 80.0)
        self.enable_aggressive_optimization = rospy.get_param("~enable_aggressive_optimization", True)
        self.enable_debug_logging = rospy.get_param("~enable_debug_logging", False)
        
        # Initialize resource optimization utilities
        self.resource_optimizer = ResourceOptimizationUtils()
        
        # Monitoring state
        self.monitoring_active = True
        self.last_optimization_time = 0.0
        
        # Statistics
        self.optimization_requests = 0
        self.successful_optimizations = 0
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] Resource Optimization Monitor initialized")
        rospy.loginfo(f"[{self.node_name}] Optimization interval: {self.optimization_interval}s")
        rospy.loginfo(f"[{self.node_name}] Memory threshold: {self.memory_threshold_mb}MB")
        rospy.loginfo(f"[{self.node_name}] CPU threshold: {self.cpu_threshold_percent}%")
    
    def _setup_subscribers(self):
        """Setup subscribers for optimization requests"""
        rospy.loginfo(f"[{self.node_name}] Setting up optimization subscribers...")
        
        # Manual optimization requests
        self.optimize_memory_sub = rospy.Subscriber(
            "~optimize_memory",
            Bool,
            self._optimize_memory_callback,
            queue_size=1
        )
        
        self.optimize_cpu_sub = rospy.Subscriber(
            "~optimize_cpu",
            Bool,
            self._optimize_cpu_callback,
            queue_size=1
        )
        
        # Performance data for triggered optimization
        self.performance_data_sub = rospy.Subscriber(
            "/performance_data/aggregated",
            String,
            self._performance_data_callback,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Optimization subscribers initialized")
    
    def _setup_publishers(self):
        """Setup publishers for optimization status and results"""
        rospy.loginfo(f"[{self.node_name}] Setting up optimization publishers...")
        
        # Resource status
        self.resource_status_pub = rospy.Publisher(
            "~resource_status",
            String,
            queue_size=1
        )
        
        # Optimization results
        self.optimization_results_pub = rospy.Publisher(
            "~optimization_results",
            String,
            queue_size=10
        )
        
        # Resource usage metrics
        self.resource_usage_pub = rospy.Publisher(
            "~resource_usage",
            String,
            queue_size=1
        )
        
        # Optimization statistics
        self.optimization_stats_pub = rospy.Publisher(
            "~optimization_statistics",
            String,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Optimization publishers initialized")
    
    def _setup_timers(self):
        """Setup periodic monitoring and optimization timers"""
        # Resource monitoring timer
        self.monitoring_timer = rospy.Timer(
            rospy.Duration(5.0),  # Every 5 seconds
            self._monitoring_callback
        )
        
        # Periodic optimization timer
        self.optimization_timer = rospy.Timer(
            rospy.Duration(self.optimization_interval),
            self._periodic_optimization_callback
        )
        
        # Statistics publishing timer
        self.stats_timer = rospy.Timer(
            rospy.Duration(60.0),  # Every minute
            self._statistics_callback
        )
        
        rospy.loginfo(f"[{self.node_name}] Monitoring timers initialized")
    
    def _optimize_memory_callback(self, msg):
        """Handle manual memory optimization requests"""
        if msg.data:
            rospy.loginfo(f"[{self.node_name}] Manual memory optimization requested")
            self._perform_memory_optimization(aggressive=True)
    
    def _optimize_cpu_callback(self, msg):
        """Handle manual CPU optimization requests"""
        if msg.data:
            rospy.loginfo(f"[{self.node_name}] Manual CPU optimization requested")
            self._perform_cpu_optimization()
    
    def _performance_data_callback(self, msg):
        """Handle performance data for triggered optimization"""
        try:
            perf_data = json.loads(msg.data)
            
            # Check if optimization is needed based on performance data
            resource_pressure = perf_data.get('resource_pressure', 'NONE')
            system_efficiency = perf_data.get('system_efficiency', 1.0)
            total_cpu = perf_data.get('total_cpu_usage', 0.0)
            total_memory = perf_data.get('total_memory_usage_mb', 0.0)
            
            # Trigger optimization if needed
            current_time = time.time()
            if (current_time - self.last_optimization_time) > 60.0:  # Minimum 1 minute between triggered optimizations
                
                if resource_pressure in ['HIGH', 'CRITICAL'] or system_efficiency < 0.5:
                    rospy.loginfo(f"[{self.node_name}] Performance-triggered optimization: "
                                 f"Pressure={resource_pressure}, Efficiency={system_efficiency:.2f}")
                    
                    if total_memory > self.memory_threshold_mb:
                        self._perform_memory_optimization(aggressive=(resource_pressure == 'CRITICAL'))
                    
                    if total_cpu > self.cpu_threshold_percent:
                        self._perform_cpu_optimization()
                    
                    self.last_optimization_time = current_time
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing performance data: {e}")
    
    def _monitoring_callback(self, event):
        """Periodic resource monitoring callback"""
        if not self.monitoring_active:
            return
        
        try:
            # Get current resource usage
            current_usage = self.resource_optimizer.get_current_resource_usage()
            
            # Publish resource usage
            usage_msg = String()
            usage_msg.data = json.dumps(current_usage.to_dict())
            self.resource_usage_pub.publish(usage_msg)
            
            # Publish resource status
            status_data = {
                'timestamp': current_usage.timestamp,
                'cpu_percent': current_usage.cpu_percent,
                'memory_percent': current_usage.memory_percent,
                'memory_rss_mb': current_usage.memory_rss_mb,
                'memory_available_mb': current_usage.memory_available_mb,
                'temperature_celsius': current_usage.temperature_celsius,
                'threads': current_usage.threads,
                'open_files': current_usage.open_files,
                'status': self._determine_resource_status(current_usage)
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.resource_status_pub.publish(status_msg)
            
            if self.enable_debug_logging:
                rospy.logdebug(f"[{self.node_name}] Resource monitoring: "
                              f"CPU={current_usage.cpu_percent:.1f}%, "
                              f"Memory={current_usage.memory_percent:.1f}%, "
                              f"RSS={current_usage.memory_rss_mb:.1f}MB")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Resource monitoring event: "
                          f"CPU={current_usage.cpu_percent:.1f}%, "
                          f"Memory={current_usage.memory_percent:.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Monitoring callback error: {e}")
    
    def _periodic_optimization_callback(self, event):
        """Periodic optimization callback"""
        if not self.monitoring_active:
            return
        
        try:
            # Perform comprehensive resource monitoring and optimization
            optimization_result = self.resource_optimizer.monitor_and_optimize()
            
            if optimization_result['optimizations_applied']:
                rospy.loginfo(f"[{self.node_name}] Periodic optimization completed: "
                             f"{len(optimization_result['optimizations_applied'])} optimizations applied")
                
                # Publish optimization results
                for result in optimization_result['optimizations_applied']:
                    result_msg = String()
                    result_msg.data = json.dumps(result)
                    self.optimization_results_pub.publish(result_msg)
                
                self.successful_optimizations += len(optimization_result['optimizations_applied'])
            
            if self.enable_debug_logging:
                rospy.logdebug(f"[{self.node_name}] Periodic optimization: "
                              f"{len(optimization_result['optimizations_applied'])} optimizations")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Periodic optimization error: {e}")
    
    def _statistics_callback(self, event):
        """Publish optimization statistics"""
        try:
            # Get optimization statistics from resource optimizer
            optimizer_stats = self.resource_optimizer.get_optimization_statistics()
            
            # Add monitor-specific statistics
            stats_data = {
                'timestamp': time.time(),
                'monitor_optimization_requests': self.optimization_requests,
                'monitor_successful_optimizations': self.successful_optimizations,
                'monitor_success_rate': self.successful_optimizations / max(1, self.optimization_requests),
                'optimizer_statistics': optimizer_stats
            }
            
            # Publish statistics
            stats_msg = String()
            stats_msg.data = json.dumps(stats_data)
            self.optimization_stats_pub.publish(stats_msg)
            
            if self.enable_debug_logging:
                rospy.logdebug(f"[{self.node_name}] Optimization statistics: "
                              f"Requests={self.optimization_requests}, "
                              f"Success={self.successful_optimizations}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Statistics callback error: {e}")
    
    def _perform_memory_optimization(self, aggressive: bool = False):
        """Perform memory optimization"""
        try:
            rospy.loginfo(f"[{self.node_name}] Performing memory optimization (aggressive={aggressive})")
            
            self.optimization_requests += 1
            result = self.resource_optimizer.optimize_memory_usage(aggressive=aggressive)
            
            if result.success:
                self.successful_optimizations += 1
                rospy.loginfo(f"[{self.node_name}] Memory optimization successful: "
                             f"{result.improvement_percent:.1f}% improvement")
            else:
                rospy.logwarn(f"[{self.node_name}] Memory optimization failed: {result.description}")
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result.to_dict())
            self.optimization_results_pub.publish(result_msg)
            
            # Comprehensive logging
            rospy.logdebug(f"[{self.node_name}] Memory optimization details:")
            rospy.logdebug(f"[{self.node_name}] - Success: {result.success}")
            rospy.logdebug(f"[{self.node_name}] - Improvement: {result.improvement_percent:.1f}%")
            rospy.logdebug(f"[{self.node_name}] - Duration: {result.duration_seconds:.2f}s")
            rospy.logdebug(f"[{self.node_name}] - Description: {result.description}")
            
            # Real-time monitoring
            rospy.loginfo(f"[{self.node_name}] Memory optimization event: "
                         f"Success={result.success}, Improvement={result.improvement_percent:.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Memory optimization error: {e}")
    
    def _perform_cpu_optimization(self):
        """Perform CPU optimization"""
        try:
            rospy.loginfo(f"[{self.node_name}] Performing CPU optimization")
            
            self.optimization_requests += 1
            result = self.resource_optimizer.optimize_cpu_usage()
            
            if result.success:
                self.successful_optimizations += 1
                rospy.loginfo(f"[{self.node_name}] CPU optimization successful: "
                             f"{result.improvement_percent:.1f}% improvement")
            else:
                rospy.logwarn(f"[{self.node_name}] CPU optimization failed: {result.description}")
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result.to_dict())
            self.optimization_results_pub.publish(result_msg)
            
            # Comprehensive logging
            rospy.logdebug(f"[{self.node_name}] CPU optimization details:")
            rospy.logdebug(f"[{self.node_name}] - Success: {result.success}")
            rospy.logdebug(f"[{self.node_name}] - Improvement: {result.improvement_percent:.1f}%")
            rospy.logdebug(f"[{self.node_name}] - Duration: {result.duration_seconds:.2f}s")
            rospy.logdebug(f"[{self.node_name}] - Description: {result.description}")
            
            # Real-time monitoring
            rospy.loginfo(f"[{self.node_name}] CPU optimization event: "
                         f"Success={result.success}, Improvement={result.improvement_percent:.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] CPU optimization error: {e}")
    
    def _determine_resource_status(self, usage: ResourceUsage) -> str:
        """Determine overall resource status"""
        status_score = 0
        
        # CPU status
        if usage.cpu_percent > 95:
            status_score += 4
        elif usage.cpu_percent > 85:
            status_score += 3
        elif usage.cpu_percent > 75:
            status_score += 2
        elif usage.cpu_percent > 65:
            status_score += 1
        
        # Memory status
        if usage.memory_percent > 95:
            status_score += 4
        elif usage.memory_percent > 85:
            status_score += 3
        elif usage.memory_percent > 75:
            status_score += 2
        elif usage.memory_percent > 65:
            status_score += 1
        
        # Temperature status (Raspberry Pi)
        if self.raspberry_pi_mode and usage.temperature_celsius:
            if usage.temperature_celsius > 85:
                status_score += 4
            elif usage.temperature_celsius > 80:
                status_score += 3
            elif usage.temperature_celsius > 75:
                status_score += 2
            elif usage.temperature_celsius > 70:
                status_score += 1
        
        # Map status score to level
        if status_score >= 8:
            return "CRITICAL"
        elif status_score >= 6:
            return "HIGH"
        elif status_score >= 4:
            return "MEDIUM"
        elif status_score >= 2:
            return "LOW"
        else:
            return "NORMAL"
    
    def shutdown(self):
        """Shutdown the resource optimization monitor"""
        rospy.loginfo(f"[{self.node_name}] Resource Optimization Monitor shutting down")
        
        self.monitoring_active = False
        
        # Shutdown timers
        if hasattr(self, 'monitoring_timer'):
            self.monitoring_timer.shutdown()
        if hasattr(self, 'optimization_timer'):
            self.optimization_timer.shutdown()
        if hasattr(self, 'stats_timer'):
            self.stats_timer.shutdown()
        
        # Shutdown resource optimizer
        if hasattr(self, 'resource_optimizer'):
            self.resource_optimizer.shutdown()
        
        # Log final statistics
        rospy.loginfo(f"[{self.node_name}] Final optimization statistics:")
        rospy.loginfo(f"[{self.node_name}] Optimization requests: {self.optimization_requests}")
        rospy.loginfo(f"[{self.node_name}] Successful optimizations: {self.successful_optimizations}")
        if self.optimization_requests > 0:
            success_rate = self.successful_optimizations / self.optimization_requests
            rospy.loginfo(f"[{self.node_name}] Success rate: {success_rate:.2%}")
        
        rospy.loginfo(f"[{self.node_name}] Resource Optimization Monitor shutdown complete")


def main():
    """Main function to run the resource optimization monitor"""
    try:
        monitor = ResourceOptimizationMonitor()
        rospy.loginfo("Resource Optimization Monitor running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Resource Optimization Monitor interrupted")
    except Exception as e:
        rospy.logerr(f"Resource Optimization Monitor error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

"""
Enhanced Lane Detection Pipeline Monitor

This module provides comprehensive monitoring and logging for the enhanced
lane detection pipeline, including component initialization, cross-component
communication, and overall system health monitoring.
"""

import time
import json
from collections import defaultdict, deque
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Any

try:
    import rospy
    from std_msgs.msg import String
    from duckietown_msgs.msg import SegmentList, LanePose
    from sensor_msgs.msg import CompressedImage
except ImportError:
    # Mock for testing without ROS
    rospy = None


@dataclass
class ComponentStatus:
    """Status information for a pipeline component."""
    name: str
    initialized: bool = False
    last_update: float = 0.0
    processing_time: float = 0.0
    error_count: int = 0
    performance_metrics: Dict[str, float] = None
    
    def __post_init__(self):
        if self.performance_metrics is None:
            self.performance_metrics = {}


@dataclass
class PipelineMetrics:
    """Overall pipeline performance metrics."""
    total_frames_processed: int = 0
    average_fps: float = 0.0
    end_to_end_latency: float = 0.0
    component_sync_score: float = 0.0
    data_flow_health: float = 0.0
    system_health_score: float = 0.0


class EnhancedLaneDetectionMonitor:
    """
    Comprehensive monitoring system for the enhanced lane detection pipeline.
    
    Monitors component initialization, performance, cross-component communication,
    data flow, and overall system health.
    """
    
    def __init__(self, monitoring_interval=1.0, history_size=100):
        self.monitoring_interval = monitoring_interval
        self.history_size = history_size
        
        # Component tracking
        self.components = {
            'line_detector': ComponentStatus('line_detector'),
            'adaptive_threshold': ComponentStatus('adaptive_threshold'),
            'temporal_filter': ComponentStatus('temporal_filter'),
            'lane_filter': ComponentStatus('lane_filter'),
            'curve_fitter': ComponentStatus('curve_fitter'),
            'lane_controller': ComponentStatus('lane_controller')
        }
        
        # Performance history
        self.performance_history = deque(maxlen=history_size)
        self.latency_history = deque(maxlen=history_size)
        self.fps_history = deque(maxlen=history_size)
        
        # Data flow tracking
        self.data_flow_timestamps = defaultdict(lambda: deque(maxlen=10))
        self.communication_health = defaultdict(float)
        
        # System health metrics
        self.system_errors = deque(maxlen=50)
        self.warning_counts = defaultdict(int)
        
        # Monitoring state
        self.monitoring_active = False
        self.last_monitoring_update = time.time()
        self.frame_count = 0
        
        # Initialize ROS monitoring if available
        if rospy:
            self._init_ros_monitoring()
    
    def _init_ros_monitoring(self):
        """Initialize ROS-based monitoring."""
        try:
            # Publishers for monitoring data
            self.status_pub = rospy.Publisher(
                '/enhanced_lane_detection/monitor/status',
                String, queue_size=1
            )
            
            self.metrics_pub = rospy.Publisher(
                '/enhanced_lane_detection/monitor/metrics',
                String, queue_size=1
            )
            
            # Subscribers for pipeline data flow monitoring
            self.image_sub = rospy.Subscriber(
                '/camera_node/image/compressed',
                CompressedImage,
                self._image_callback,
                queue_size=1
            )
            
            self.segments_sub = rospy.Subscriber(
                '/line_detector_node/segment_list',
                SegmentList,
                self._segments_callback,
                queue_size=1
            )
            
            self.lane_pose_sub = rospy.Subscriber(
                '/lane_filter_node/lane_pose',
                LanePose,
                self._lane_pose_callback,
                queue_size=1
            )
            
            rospy.loginfo("[EnhancedLaneDetectionMonitor] ROS monitoring initialized")
            
        except Exception as e:
            if rospy:
                rospy.logwarn(f"[EnhancedLaneDetectionMonitor] Failed to initialize ROS monitoring: {e}")
    
    def start_monitoring(self):
        """Start the monitoring system."""
        self.monitoring_active = True
        self.last_monitoring_update = time.time()
        
        if rospy:
            rospy.loginfo("[EnhancedLaneDetectionMonitor] Pipeline monitoring started")
        else:
            print("[EnhancedLaneDetectionMonitor] Pipeline monitoring started")
        
        # Log pipeline integration status
        self._log_pipeline_integration_status()
    
    def stop_monitoring(self):
        """Stop the monitoring system."""
        self.monitoring_active = False
        
        if rospy:
            rospy.loginfo("[EnhancedLaneDetectionMonitor] Pipeline monitoring stopped")
        else:
            print("[EnhancedLaneDetectionMonitor] Pipeline monitoring stopped")
    
    def update_component_status(self, component_name: str, **kwargs):
        """Update status for a specific component."""
        if component_name in self.components:
            component = self.components[component_name]
            
            for key, value in kwargs.items():
                if hasattr(component, key):
                    setattr(component, key, value)
            
            component.last_update = time.time()
            
            # Log component status update
            if rospy:
                rospy.logdebug(f"[EnhancedLaneDetectionMonitor] Updated {component_name} status: {kwargs}")
    
    def log_component_initialization(self, component_name: str, success: bool, 
                                   initialization_time: float = 0.0, config: Dict = None):
        """Log component initialization status."""
        if component_name in self.components:
            self.components[component_name].initialized = success
            self.components[component_name].processing_time = initialization_time
            
            if config:
                self.components[component_name].performance_metrics.update(config)
        
        status = "SUCCESS" if success else "FAILED"
        log_msg = f"[EnhancedLaneDetectionMonitor] Component initialization - " \
                  f"{component_name}: {status}"
        
        if initialization_time > 0:
            log_msg += f", Time: {initialization_time*1000:.2f}ms"
        
        if config:
            log_msg += f", Config: {config}"
        
        if rospy:
            if success:
                rospy.loginfo(log_msg)
            else:
                rospy.logerr(log_msg)
        else:
            print(log_msg)
    
    def log_cross_component_communication(self, source: str, target: str, 
                                        data_type: str, success: bool, 
                                        latency: float = 0.0):
        """Log cross-component communication events."""
        timestamp = time.time()
        communication_key = f"{source}->{target}"
        
        # Track communication timestamps
        self.data_flow_timestamps[communication_key].append(timestamp)
        
        # Update communication health
        if success:
            self.communication_health[communication_key] = min(1.0, 
                self.communication_health[communication_key] + 0.1)
        else:
            self.communication_health[communication_key] = max(0.0,
                self.communication_health[communication_key] - 0.2)
        
        # Log communication event
        status = "SUCCESS" if success else "FAILED"
        log_msg = f"[EnhancedLaneDetectionMonitor] Cross-component communication - " \
                  f"{source} -> {target}: {data_type}, {status}"
        
        if latency > 0:
            log_msg += f", Latency: {latency*1000:.2f}ms"
        
        if rospy:
            rospy.logdebug(log_msg)
    
    def log_data_flow(self, stage: str, data_size: int, processing_time: float):
        """Log data flow through pipeline stages."""
        timestamp = time.time()
        
        # Track data flow
        flow_key = f"data_flow_{stage}"
        self.data_flow_timestamps[flow_key].append(timestamp)
        
        # Update component processing time
        if stage in self.components:
            self.components[stage].processing_time = processing_time
        
        # Log data flow
        log_msg = f"[EnhancedLaneDetectionMonitor] Data flow - " \
                  f"Stage: {stage}, Size: {data_size}, " \
                  f"Processing time: {processing_time*1000:.2f}ms"
        
        if rospy:
            rospy.logdebug(log_msg)
    
    def log_system_health(self, health_score: float, details: Dict[str, Any] = None):
        """Log overall system health metrics."""
        current_time = time.time()
        
        # Update system health history
        health_data = {
            'timestamp': current_time,
            'health_score': health_score,
            'details': details or {}
        }
        self.performance_history.append(health_data)
        
        # Log system health
        log_msg = f"[EnhancedLaneDetectionMonitor] System health - " \
                  f"Score: {health_score:.3f}"
        
        if details:
            log_msg += f", Details: {details}"
        
        if rospy:
            rospy.loginfo(log_msg)
        else:
            print(log_msg)
    
    def _log_pipeline_integration_status(self):
        """Log comprehensive pipeline integration status."""
        integration_status = {
            'timestamp': time.time(),
            'components_initialized': sum(1 for c in self.components.values() if c.initialized),
            'total_components': len(self.components),
            'component_details': {}
        }
        
        for name, component in self.components.items():
            integration_status['component_details'][name] = {
                'initialized': component.initialized,
                'last_update': component.last_update,
                'error_count': component.error_count,
                'performance_metrics': component.performance_metrics
            }
        
        # Calculate integration health
        initialization_rate = integration_status['components_initialized'] / integration_status['total_components']
        
        log_msg = f"[EnhancedLaneDetectionMonitor] Pipeline Integration Status - " \
                  f"Initialized: {integration_status['components_initialized']}/{integration_status['total_components']} " \
                  f"({initialization_rate*100:.1f}%)"
        
        if rospy:
            rospy.loginfo(log_msg)
            
            # Publish detailed status
            status_json = json.dumps(integration_status, indent=2)
            self.status_pub.publish(String(data=status_json))
        else:
            print(log_msg)
            print(f"Integration details: {json.dumps(integration_status, indent=2)}")
    
    def get_pipeline_metrics(self) -> PipelineMetrics:
        """Get current pipeline performance metrics."""
        current_time = time.time()
        
        # Calculate FPS
        if len(self.fps_history) > 1:
            time_span = self.fps_history[-1] - self.fps_history[0]
            avg_fps = len(self.fps_history) / time_span if time_span > 0 else 0.0
        else:
            avg_fps = 0.0
        
        # Calculate average latency
        avg_latency = sum(self.latency_history) / len(self.latency_history) if self.latency_history else 0.0
        
        # Calculate component synchronization score
        sync_scores = []
        for component in self.components.values():
            if component.initialized and component.last_update > 0:
                time_since_update = current_time - component.last_update
                sync_score = max(0.0, 1.0 - time_since_update / 5.0)  # 5 second timeout
                sync_scores.append(sync_score)
        
        component_sync_score = sum(sync_scores) / len(sync_scores) if sync_scores else 0.0
        
        # Calculate data flow health
        flow_health_scores = list(self.communication_health.values())
        data_flow_health = sum(flow_health_scores) / len(flow_health_scores) if flow_health_scores else 0.0
        
        # Calculate overall system health
        system_health_score = (component_sync_score + data_flow_health) / 2.0
        
        return PipelineMetrics(
            total_frames_processed=self.frame_count,
            average_fps=avg_fps,
            end_to_end_latency=avg_latency,
            component_sync_score=component_sync_score,
            data_flow_health=data_flow_health,
            system_health_score=system_health_score
        )
    
    def _image_callback(self, msg):
        """Callback for image messages to track data flow."""
        self.frame_count += 1
        current_time = time.time()
        self.fps_history.append(current_time)
        
        # Log data flow
        self.log_data_flow('image_input', len(msg.data), 0.001)  # Minimal processing time
    
    def _segments_callback(self, msg):
        """Callback for segment messages to track data flow."""
        current_time = time.time()
        
        # Calculate latency from image to segments
        if self.fps_history:
            latency = current_time - self.fps_history[-1]
            self.latency_history.append(latency)
        
        # Log data flow
        self.log_data_flow('line_detection', len(msg.segments), 0.01)
        
        # Log cross-component communication
        self.log_cross_component_communication(
            'line_detector', 'lane_filter', 'SegmentList', True, 
            latency if self.fps_history else 0.0
        )
    
    def _lane_pose_callback(self, msg):
        """Callback for lane pose messages to track data flow."""
        current_time = time.time()
        
        # Log data flow
        self.log_data_flow('lane_filtering', 1, 0.005)
        
        # Log cross-component communication
        self.log_cross_component_communication(
            'lane_filter', 'lane_controller', 'LanePose', True
        )
    
    def periodic_monitoring_update(self):
        """Perform periodic monitoring updates."""
        if not self.monitoring_active:
            return
        
        current_time = time.time()
        
        # Check if it's time for periodic update
        if current_time - self.last_monitoring_update >= self.monitoring_interval:
            # Get current metrics
            metrics = self.get_pipeline_metrics()
            
            # Log periodic update
            log_msg = f"[EnhancedLaneDetectionMonitor] Periodic Update - " \
                      f"FPS: {metrics.average_fps:.1f}, " \
                      f"Latency: {metrics.end_to_end_latency*1000:.2f}ms, " \
                      f"Health: {metrics.system_health_score:.3f}"
            
            if rospy:
                rospy.loginfo(log_msg)
                
                # Publish metrics
                metrics_json = json.dumps(asdict(metrics), indent=2)
                self.metrics_pub.publish(String(data=metrics_json))
            else:
                print(log_msg)
            
            self.last_monitoring_update = current_time
    
    def generate_monitoring_report(self) -> Dict[str, Any]:
        """Generate comprehensive monitoring report."""
        current_metrics = self.get_pipeline_metrics()
        
        report = {
            'timestamp': time.time(),
            'monitoring_duration': time.time() - self.last_monitoring_update,
            'pipeline_metrics': asdict(current_metrics),
            'component_status': {name: asdict(comp) for name, comp in self.components.items()},
            'communication_health': dict(self.communication_health),
            'system_errors': list(self.system_errors),
            'warning_counts': dict(self.warning_counts)
        }
        
        return report


# Example usage and testing
if __name__ == '__main__':
    # Create monitor instance
    monitor = EnhancedLaneDetectionMonitor(monitoring_interval=2.0)
    
    # Start monitoring
    monitor.start_monitoring()
    
    # Simulate component initialization
    components = ['line_detector', 'adaptive_threshold', 'temporal_filter', 
                 'lane_filter', 'curve_fitter', 'lane_controller']
    
    for i, component in enumerate(components):
        time.sleep(0.1)  # Simulate initialization time
        success = True  # Simulate successful initialization
        init_time = 0.05 + i * 0.01  # Simulate varying initialization times
        config = {'param1': i, 'param2': f'value_{i}'}
        
        monitor.log_component_initialization(component, success, init_time, config)
    
    # Simulate some data flow and communication
    for frame in range(10):
        time.sleep(0.1)
        
        # Simulate image processing
        monitor.log_data_flow('image_input', 1024*768*3, 0.001)
        
        # Simulate line detection
        monitor.log_data_flow('line_detection', 50, 0.02)
        monitor.log_cross_component_communication(
            'line_detector', 'lane_filter', 'SegmentList', True, 0.001
        )
        
        # Simulate lane filtering
        monitor.log_data_flow('lane_filtering', 1, 0.01)
        monitor.log_cross_component_communication(
            'lane_filter', 'lane_controller', 'LanePose', True, 0.001
        )
        
        # Update system health
        health_score = 0.9 + 0.1 * (frame % 3) / 3  # Simulate varying health
        monitor.log_system_health(health_score, {'frame': frame})
        
        # Periodic monitoring update
        monitor.periodic_monitoring_update()
    
    # Generate final report
    report = monitor.generate_monitoring_report()
    print("\nFinal Monitoring Report:")
    print(json.dumps(report, indent=2))
    
    # Stop monitoring
    monitor.stop_monitoring()
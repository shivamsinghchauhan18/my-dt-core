#!/usr/bin/env python3
"""
Validate Integration Node

Validates the integration and health of the enhanced autonomous system.
Performs continuous checks on component connectivity, data flow, and system performance.

Author: Enhanced Autonomous Duckietown System
"""

import rospy
import time
import threading
from collections import defaultdict, deque
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

# Try to import custom messages
try:
    from duckietown_msgs.msg import (
        LanePose,
    )
    from duckietown_enhanced_msgs.msg import (
        ObjectDetectionArray,
        SafetyStatus,
        PerformanceMetrics,
    )
except ImportError:
    rospy.logwarn("Some custom messages not available for validation")
    from std_msgs.msg import String as LanePose
    from std_msgs.msg import String as ObjectDetectionArray
    from std_msgs.msg import String as SafetyStatus
    from std_msgs.msg import String as PerformanceMetrics


class IntegrationValidator:
    """
    Validates system integration by monitoring:
    - Topic connectivity and message flow
    - Component health and responsiveness
    - Data consistency and quality
    - Performance metrics compliance
    """
    
    def __init__(self):
        rospy.init_node('validate_integration', anonymous=True)
        
        # Parameters
        self.validation_frequency = rospy.get_param('~validation_frequency', 2.0)  # Hz
        self.message_timeout = rospy.get_param('~message_timeout', 5.0)  # seconds
        self.performance_check_interval = rospy.get_param('~performance_check_interval', 10.0)
        self.max_validation_failures = rospy.get_param('~max_validation_failures', 5)
        
        # Performance targets
        self.target_fps = rospy.get_param('~target_fps', 15.0)
        self.target_latency = rospy.get_param('~target_latency', 0.5)
        self.min_lane_pose_rate = rospy.get_param('~min_lane_pose_rate', 10.0)
        
        # Validation state
        self.validation_results = {}
        self.message_timestamps = defaultdict(lambda: None)
        self.message_counts = defaultdict(int)
        self.validation_failures = defaultdict(int)
        
        # Performance tracking
        self.fps_history = deque(maxlen=50)
        self.latency_history = deque(maxlen=50)
        self.lane_pose_rate_history = deque(maxlen=30)
        
        # Component health tracking
        self.component_health = {
            'camera': False,
            'lane_detection': False,
            'object_detection': False,
            'lane_control': False,
            'safety_system': False,
            'integration_coordinator': False
        }
        
        # Thread safety
        self.validation_lock = threading.Lock()
        
        # Publishers
        self.validation_results_pub = rospy.Publisher('~validation_results', String, queue_size=1)
        self.system_health_pub = rospy.Publisher('~system_health', String, queue_size=1)
        self.validation_status_pub = rospy.Publisher('~validation_status', Bool, queue_size=1)
        
        # Set up subscribers for validation
        self.setup_validation_subscribers()
        
        # Validation timer
        self.validation_timer = rospy.Timer(
            rospy.Duration(1.0 / self.validation_frequency),
            self.validation_callback
        )
        
        # Performance check timer
        self.performance_timer = rospy.Timer(
            rospy.Duration(self.performance_check_interval),
            self.performance_check_callback
        )
        
        rospy.loginfo("Integration Validator initialized")
        
    def setup_validation_subscribers(self):
        """Set up subscribers for validation monitoring."""
        vehicle_name = rospy.get_namespace().strip('/')
        
        # Camera validation
        rospy.Subscriber(f'{vehicle_name}/camera_node/image/compressed',
                        CompressedImage, lambda msg: self.message_received('camera', msg))
        
        # Lane detection validation
        rospy.Subscriber(f'{vehicle_name}/lane_filter_node/lane_pose',
                        LanePose, lambda msg: self.message_received('lane_detection', msg))
        
        # Object detection validation
        rospy.Subscriber(f'{vehicle_name}/enhanced_vehicle_detection_node/detections',
                        ObjectDetectionArray, lambda msg: self.message_received('object_detection', msg))
        
        # Control validation
        rospy.Subscriber(f'{vehicle_name}/lane_controller_node/car_cmd',
                        Twist, lambda msg: self.message_received('lane_control', msg))
        
        # Safety system validation
        rospy.Subscriber(f'{vehicle_name}/safety_status_publisher/safety_status',
                        SafetyStatus, lambda msg: self.message_received('safety_system', msg))
        
        # Integration coordinator validation
        rospy.Subscriber(f'{vehicle_name}/master_integration_coordinator/system_status',
                        String, lambda msg: self.message_received('integration_coordinator', msg))
        
        # Performance metrics validation
        rospy.Subscriber(f'{vehicle_name}/system_performance_monitor/performance_metrics',
                        PerformanceMetrics, lambda msg: self.message_received('performance_monitor', msg))
                        
    def message_received(self, component, msg):
        """Track message reception for validation."""
        current_time = rospy.Time.now()
        
        with self.validation_lock:
            self.message_timestamps[component] = current_time
            self.message_counts[component] += 1
            
            # Update component health
            if component in self.component_health:
                self.component_health[component] = True
                
            # Track specific metrics
            if component == 'camera':
                self.track_camera_performance(msg)
            elif component == 'lane_detection':
                self.track_lane_detection_performance(msg)
                
    def track_camera_performance(self, msg):
        """Track camera performance metrics."""
        try:
            # Estimate FPS based on message frequency
            current_time = rospy.Time.now()
            if hasattr(self, 'last_camera_time'):
                interval = (current_time - self.last_camera_time).to_sec()
                if interval > 0:
                    fps = 1.0 / interval
                    self.fps_history.append(fps)
                    
            self.last_camera_time = current_time
            
        except Exception as e:
            rospy.logwarn(f"Error tracking camera performance: {e}")
            
    def track_lane_detection_performance(self, msg):
        """Track lane detection performance metrics."""
        try:
            # Estimate lane pose update rate
            current_time = rospy.Time.now()
            if hasattr(self, 'last_lane_pose_time'):
                interval = (current_time - self.last_lane_pose_time).to_sec()
                if interval > 0:
                    rate = 1.0 / interval
                    self.lane_pose_rate_history.append(rate)
                    
            self.last_lane_pose_time = current_time
            
        except Exception as e:
            rospy.logwarn(f"Error tracking lane detection performance: {e}")
            
    def validate_message_flow(self):
        """Validate that all components are publishing messages."""
        current_time = rospy.Time.now()
        validation_results = {}
        
        with self.validation_lock:
            for component, last_time in self.message_timestamps.items():
                if last_time is None:
                    validation_results[component] = {
                        'status': 'FAIL',
                        'reason': 'No messages received',
                        'last_message': 'Never'
                    }
                    self.validation_failures[component] += 1
                else:
                    time_since_last = (current_time - last_time).to_sec()
                    if time_since_last > self.message_timeout:
                        validation_results[component] = {
                            'status': 'FAIL',
                            'reason': f'Message timeout ({time_since_last:.1f}s)',
                            'last_message': f'{time_since_last:.1f}s ago'
                        }
                        self.validation_failures[component] += 1
                        self.component_health[component] = False
                    else:
                        validation_results[component] = {
                            'status': 'PASS',
                            'reason': 'Messages flowing',
                            'last_message': f'{time_since_last:.1f}s ago',
                            'message_count': self.message_counts[component]
                        }
                        
        return validation_results
        
    def validate_performance_metrics(self):
        """Validate system performance against targets."""
        performance_results = {}
        
        with self.validation_lock:
            # FPS validation
            if self.fps_history:
                avg_fps = sum(self.fps_history) / len(self.fps_history)
                if avg_fps >= self.target_fps * 0.8:  # 80% of target
                    performance_results['fps'] = {
                        'status': 'PASS',
                        'value': avg_fps,
                        'target': self.target_fps
                    }
                else:
                    performance_results['fps'] = {
                        'status': 'FAIL',
                        'value': avg_fps,
                        'target': self.target_fps,
                        'reason': 'Below 80% of target FPS'
                    }
            else:
                performance_results['fps'] = {
                    'status': 'UNKNOWN',
                    'reason': 'No FPS data available'
                }
                
            # Lane pose rate validation
            if self.lane_pose_rate_history:
                avg_rate = sum(self.lane_pose_rate_history) / len(self.lane_pose_rate_history)
                if avg_rate >= self.min_lane_pose_rate * 0.8:
                    performance_results['lane_pose_rate'] = {
                        'status': 'PASS',
                        'value': avg_rate,
                        'target': self.min_lane_pose_rate
                    }
                else:
                    performance_results['lane_pose_rate'] = {
                        'status': 'FAIL',
                        'value': avg_rate,
                        'target': self.min_lane_pose_rate,
                        'reason': 'Lane pose rate too low'
                    }
            else:
                performance_results['lane_pose_rate'] = {
                    'status': 'UNKNOWN',
                    'reason': 'No lane pose rate data available'
                }
                
        return performance_results
        
    def validate_data_consistency(self):
        """Validate data consistency and quality."""
        consistency_results = {}
        
        # Check for component health correlation
        active_components = sum(1 for health in self.component_health.values() if health)
        total_components = len(self.component_health)
        
        health_ratio = active_components / total_components if total_components > 0 else 0
        
        if health_ratio >= 0.8:  # 80% of components healthy
            consistency_results['component_health'] = {
                'status': 'PASS',
                'active_components': active_components,
                'total_components': total_components,
                'health_ratio': health_ratio
            }
        else:
            consistency_results['component_health'] = {
                'status': 'FAIL',
                'active_components': active_components,
                'total_components': total_components,
                'health_ratio': health_ratio,
                'reason': 'Too many unhealthy components'
            }
            
        # Check for excessive validation failures
        total_failures = sum(self.validation_failures.values())
        if total_failures > self.max_validation_failures:
            consistency_results['validation_failures'] = {
                'status': 'FAIL',
                'total_failures': total_failures,
                'max_allowed': self.max_validation_failures,
                'reason': 'Too many validation failures'
            }
        else:
            consistency_results['validation_failures'] = {
                'status': 'PASS',
                'total_failures': total_failures,
                'max_allowed': self.max_validation_failures
            }
            
        return consistency_results
        
    def determine_overall_validation_status(self, message_flow, performance, consistency):
        """Determine overall validation status."""
        all_results = {**message_flow, **performance, **consistency}
        
        fail_count = sum(1 for result in all_results.values() 
                        if result.get('status') == 'FAIL')
        unknown_count = sum(1 for result in all_results.values() 
                           if result.get('status') == 'UNKNOWN')
        
        if fail_count == 0 and unknown_count <= 2:
            return 'PASS', 'All validation checks passed'
        elif fail_count <= 2:
            return 'WARNING', f'{fail_count} validation failures detected'
        else:
            return 'FAIL', f'{fail_count} critical validation failures'
            
    def publish_validation_results(self, message_flow, performance, consistency, overall_status):
        """Publish comprehensive validation results."""
        try:
            # Combine all results
            all_results = {
                'timestamp': rospy.Time.now().to_sec(),
                'overall_status': overall_status[0],
                'overall_reason': overall_status[1],
                'message_flow': message_flow,
                'performance': performance,
                'consistency': consistency,
                'component_health': dict(self.component_health)
            }
            
            # Publish detailed results
            results_msg = String()
            results_msg.data = str(all_results)
            self.validation_results_pub.publish(results_msg)
            
            # Publish system health summary
            health_msg = String()
            active_count = sum(1 for health in self.component_health.values() if health)
            health_msg.data = f"System Health: {active_count}/{len(self.component_health)} components active"
            self.system_health_pub.publish(health_msg)
            
            # Publish overall validation status
            status_msg = Bool()
            status_msg.data = (overall_status[0] == 'PASS')
            self.validation_status_pub.publish(status_msg)
            
        except Exception as e:
            rospy.logerr(f"Failed to publish validation results: {e}")
            
    def validation_callback(self, event):
        """Main validation callback."""
        try:
            # Perform all validation checks
            message_flow_results = self.validate_message_flow()
            performance_results = self.validate_performance_metrics()
            consistency_results = self.validate_data_consistency()
            
            # Determine overall status
            overall_status = self.determine_overall_validation_status(
                message_flow_results, performance_results, consistency_results)
            
            # Publish results
            self.publish_validation_results(
                message_flow_results, performance_results, 
                consistency_results, overall_status)
            
            # Log critical issues
            if overall_status[0] == 'FAIL':
                rospy.logwarn(f"Validation FAILED: {overall_status[1]}")
                
        except Exception as e:
            rospy.logerr(f"Validation callback error: {e}")
            
    def performance_check_callback(self, event):
        """Periodic performance check callback."""
        try:
            with self.validation_lock:
                # Log performance statistics
                if self.fps_history:
                    avg_fps = sum(self.fps_history) / len(self.fps_history)
                    rospy.loginfo(f"Average FPS: {avg_fps:.1f} (Target: {self.target_fps})")
                    
                if self.lane_pose_rate_history:
                    avg_rate = sum(self.lane_pose_rate_history) / len(self.lane_pose_rate_history)
                    rospy.loginfo(f"Lane pose rate: {avg_rate:.1f} Hz (Min: {self.min_lane_pose_rate})")
                    
                # Log component health
                active_components = [comp for comp, health in self.component_health.items() if health]
                rospy.loginfo(f"Active components: {', '.join(active_components)}")
                
        except Exception as e:
            rospy.logwarn(f"Performance check error: {e}")
            
    def shutdown(self):
        """Shutdown the integration validator."""
        rospy.loginfo("Shutting down Integration Validator")
        
        if hasattr(self, 'validation_timer'):
            self.validation_timer.shutdown()
        if hasattr(self, 'performance_timer'):
            self.performance_timer.shutdown()
            
        # Final validation report
        rospy.loginfo("Final validation summary:")
        with self.validation_lock:
            for component, count in self.message_counts.items():
                rospy.loginfo(f"  {component}: {count} messages received")


if __name__ == '__main__':
    try:
        node = IntegrationValidator()
        rospy.on_shutdown(node.shutdown)
        rospy.loginfo("Integration Validator running")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Integration Validator failed: {e}")

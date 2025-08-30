#!/usr/bin/env python3
"""
Integration Data Logger Node

Logs comprehensive system data for analysis, debugging, and performance optimization.
Handles data from perception, control, safety, and coordination components.

Author: Enhanced Autonomous Duckietown System
"""

import rospy
import os
import json
import pickle
import threading
from datetime import datetime
from collections import deque
from std_msgs.msg import String, Float32, Bool
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
        RiskAssessmentStatus,
        SystemStatus,
        CoordinationLog,
    )
except ImportError:
    rospy.logwarn("Some custom messages not available for logging")
    from std_msgs.msg import String as LanePose
    from std_msgs.msg import String as ObjectDetectionArray
    from std_msgs.msg import String as SafetyStatus
    from std_msgs.msg import String as PerformanceMetrics
    RiskAssessmentStatus = String
    SystemStatus = String
    CoordinationLog = String


class IntegrationDataLogger:
    """
    Comprehensive data logger for the enhanced autonomous system.
    
    Features:
    - Multi-topic data logging
    - Configurable logging rates
    - Data compression and rotation
    - Real-time logging statistics
    """
    
    def __init__(self):
        rospy.init_node('integration_data_logger', anonymous=True)
        
        # Parameters
        self.log_directory = rospy.get_param('~log_directory', '/tmp/enhanced_autonomous_logs')
        self.log_rate = rospy.get_param('~log_rate', 10.0)  # Hz
        self.max_log_size = rospy.get_param('~max_log_size', 100 * 1024 * 1024)  # 100MB
        self.max_log_files = rospy.get_param('~max_log_files', 10)
        self.enable_compression = rospy.get_param('~enable_compression', True)
        
        # Logging configuration
        self.log_perception = rospy.get_param('~log_perception', True)
        self.log_control = rospy.get_param('~log_control', True)
        self.log_safety = rospy.get_param('~log_safety', True)
        self.log_performance = rospy.get_param('~log_performance', True)
        self.log_coordination = rospy.get_param('~log_coordination', True)
        
        # Create log directory
        os.makedirs(self.log_directory, exist_ok=True)
        
        # Data buffers
        self.perception_buffer = deque(maxlen=1000)
        self.control_buffer = deque(maxlen=1000)
        self.safety_buffer = deque(maxlen=1000)
        self.performance_buffer = deque(maxlen=1000)
        self.coordination_buffer = deque(maxlen=1000)
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Log files
        self.current_log_file = None
        self.log_file_size = 0
        self.log_file_count = 0
        
        # Statistics
        self.messages_logged = 0
        self.start_time = rospy.Time.now()
        
        # Publishers for logging statistics
        self.log_stats_pub = rospy.Publisher('~log_statistics', String, queue_size=1)
        
        # Subscribers
        self.setup_subscribers()
        
        # Logging timer
        self.log_timer = rospy.Timer(
            rospy.Duration(1.0 / self.log_rate),
            self.log_callback
        )
        
        # Initialize log file
        self.create_new_log_file()
        
        rospy.loginfo("Integration Data Logger initialized")
        rospy.loginfo(f"Logging to: {self.log_directory}")
        
    def setup_subscribers(self):
        """Set up subscribers for data logging."""
        vehicle_name = rospy.get_namespace().strip('/')
        
        if self.log_perception:
            rospy.Subscriber(f'{vehicle_name}/lane_filter_node/lane_pose',
                           LanePose, self.lane_pose_callback)
            rospy.Subscriber(f'{vehicle_name}/enhanced_vehicle_detection_node/detections',
                           ObjectDetectionArray, self.objects_callback)
            
        if self.log_control:
            rospy.Subscriber(f'{vehicle_name}/car_cmd_switch_node/cmd',
                           Twist, self.control_callback)
            rospy.Subscriber(f'{vehicle_name}/lane_controller_node/car_cmd',
                           Twist, self.lane_control_callback)
            
        if self.log_safety:
            rospy.Subscriber(f'{vehicle_name}/safety_status_publisher/safety_status',
                           SafetyStatus, self.safety_callback)
            
        if self.log_performance:
            rospy.Subscriber(f'{vehicle_name}/system_performance_monitor/performance_metrics',
                           PerformanceMetrics, self.performance_callback)
            
        if self.log_coordination:
            rospy.Subscriber(f'{vehicle_name}/master_integration_coordinator/system_status',
                           String, self.coordination_callback)
                           
    def lane_pose_callback(self, msg):
        """Log lane pose data."""
        data = {
            'timestamp': rospy.Time.now().to_sec(),
            'type': 'lane_pose',
            'data': self.message_to_dict(msg)
        }
        
        with self.data_lock:
            self.perception_buffer.append(data)
            
    def objects_callback(self, msg):
        """Log object detection data."""
        data = {
            'timestamp': rospy.Time.now().to_sec(),
            'type': 'object_detections',
            'data': self.message_to_dict(msg)
        }
        
        with self.data_lock:
            self.perception_buffer.append(data)
            
    def control_callback(self, msg):
        """Log control command data."""
        data = {
            'timestamp': rospy.Time.now().to_sec(),
            'type': 'control_cmd',
            'data': self.message_to_dict(msg)
        }
        
        with self.data_lock:
            self.control_buffer.append(data)
            
    def lane_control_callback(self, msg):
        """Log lane control command data."""
        data = {
            'timestamp': rospy.Time.now().to_sec(),
            'type': 'lane_control_cmd',
            'data': self.message_to_dict(msg)
        }
        
        with self.data_lock:
            self.control_buffer.append(data)
            
    def safety_callback(self, msg):
        """Log safety status data."""
        data = {
            'timestamp': rospy.Time.now().to_sec(),
            'type': 'safety_status',
            'data': self.message_to_dict(msg)
        }
        
        with self.data_lock:
            self.safety_buffer.append(data)
            
    def performance_callback(self, msg):
        """Log performance metrics data."""
        data = {
            'timestamp': rospy.Time.now().to_sec(),
            'type': 'performance_metrics',
            'data': self.message_to_dict(msg)
        }
        
        with self.data_lock:
            self.performance_buffer.append(data)
            
    def coordination_callback(self, msg):
        """Log coordination data."""
        data = {
            'timestamp': rospy.Time.now().to_sec(),
            'type': 'coordination_status',
            'data': self.message_to_dict(msg)
        }
        
        with self.data_lock:
            self.coordination_buffer.append(data)
            
    def message_to_dict(self, msg):
        """Convert ROS message to dictionary for logging."""
        try:
            # Standard ROS Python messages expose __slots__ and _slot_types
            if hasattr(msg, '__slots__') and hasattr(msg, '_slot_types'):
                result = {}
                for field_name, field_type in zip(msg.__slots__, msg._slot_types):
                    field_value = getattr(msg, field_name)

                    # Handle arrays/sequences
                    if isinstance(field_value, (list, tuple)):
                        converted = []
                        for item in field_value:
                            if hasattr(item, '__slots__') and hasattr(item, '_slot_types'):
                                converted.append(self.message_to_dict(item))
                            else:
                                converted.append(item)
                        result[field_name] = converted
                    else:
                        # Nested ROS message
                        if hasattr(field_value, '__slots__') and hasattr(field_value, '_slot_types'):
                            result[field_name] = self.message_to_dict(field_value)
                        else:
                            # Basic types
                            result[field_name] = field_value
                return result
            else:
                # Fallback for basic message types or mocks
                if hasattr(msg, 'data'):
                    return {'data': msg.data}
                elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                    return {
                        'linear': {'x': getattr(msg.linear, 'x', None), 'y': getattr(msg.linear, 'y', None), 'z': getattr(msg.linear, 'z', None)},
                        'angular': {'x': getattr(msg.angular, 'x', None), 'y': getattr(msg.angular, 'y', None), 'z': getattr(msg.angular, 'z', None)}
                    }
                else:
                    return str(msg)

        except Exception as e:
            rospy.logwarn(f"Failed to convert message to dict: {e}")
            return str(msg)
            
    def create_new_log_file(self):
        """Create a new log file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"enhanced_autonomous_log_{timestamp}_{self.log_file_count:03d}.json"
        filepath = os.path.join(self.log_directory, filename)
        
        try:
            if self.current_log_file:
                self.current_log_file.close()
                
            self.current_log_file = open(filepath, 'w')
            self.log_file_size = 0
            self.log_file_count += 1
            
            # Write log header
            header = {
                'log_start_time': rospy.Time.now().to_sec(),
                'node_name': rospy.get_name(),
                'log_configuration': {
                    'log_rate': self.log_rate,
                    'log_perception': self.log_perception,
                    'log_control': self.log_control,
                    'log_safety': self.log_safety,
                    'log_performance': self.log_performance,
                    'log_coordination': self.log_coordination
                }
            }
            
            self.write_log_entry(header)
            rospy.loginfo(f"Created new log file: {filename}")
            
        except Exception as e:
            rospy.logerr(f"Failed to create log file: {e}")
            
    def write_log_entry(self, data):
        """Write a log entry to the current file."""
        try:
            if self.current_log_file:
                log_line = json.dumps(data) + '\n'
                self.current_log_file.write(log_line)
                self.current_log_file.flush()
                
                self.log_file_size += len(log_line.encode('utf-8'))
                self.messages_logged += 1
                
                # Check if file rotation is needed
                if self.log_file_size > self.max_log_size:
                    self.rotate_log_files()
                    
        except Exception as e:
            rospy.logerr(f"Failed to write log entry: {e}")
            
    def rotate_log_files(self):
        """Rotate log files when size limit is reached."""
        rospy.loginfo("Rotating log files")
        
        # Clean up old files if we exceed the limit
        self.cleanup_old_logs()
        
        # Create new log file
        self.create_new_log_file()
        
    def cleanup_old_logs(self):
        """Clean up old log files to maintain file count limit."""
        try:
            log_files = [f for f in os.listdir(self.log_directory) 
                        if f.startswith('enhanced_autonomous_log_') and f.endswith('.json')]
            log_files.sort()
            
            while len(log_files) >= self.max_log_files:
                old_file = os.path.join(self.log_directory, log_files[0])
                os.remove(old_file)
                rospy.loginfo(f"Removed old log file: {log_files[0]}")
                log_files.pop(0)
                
        except Exception as e:
            rospy.logerr(f"Failed to cleanup old logs: {e}")
            
    def log_callback(self, event):
        """Main logging callback."""
        try:
            with self.data_lock:
                # Collect data from all buffers
                log_data = {
                    'timestamp': rospy.Time.now().to_sec(),
                    'perception_data': list(self.perception_buffer),
                    'control_data': list(self.control_buffer),
                    'safety_data': list(self.safety_buffer),
                    'performance_data': list(self.performance_buffer),
                    'coordination_data': list(self.coordination_buffer)
                }
                
                # Clear buffers
                self.perception_buffer.clear()
                self.control_buffer.clear()
                self.safety_buffer.clear()
                self.performance_buffer.clear()
                self.coordination_buffer.clear()
                
            # Write log data
            if any([log_data['perception_data'], log_data['control_data'], 
                   log_data['safety_data'], log_data['performance_data'],
                   log_data['coordination_data']]):
                self.write_log_entry(log_data)
                
            # Publish logging statistics
            self.publish_log_statistics()
            
        except Exception as e:
            rospy.logerr(f"Log callback error: {e}")
            
    def publish_log_statistics(self):
        """Publish logging statistics."""
        try:
            uptime = (rospy.Time.now() - self.start_time).to_sec()
            
            stats = {
                'messages_logged': self.messages_logged,
                'log_rate_actual': self.messages_logged / uptime if uptime > 0 else 0.0,
                'current_log_file_size': self.log_file_size,
                'log_file_count': self.log_file_count,
                'uptime_seconds': uptime
            }
            
            stats_msg = String()
            stats_msg.data = json.dumps(stats)
            self.log_stats_pub.publish(stats_msg)
            
        except Exception as e:
            rospy.logwarn(f"Failed to publish log statistics: {e}")
            
    def shutdown(self):
        """Shutdown the data logger."""
        rospy.loginfo("Shutting down Integration Data Logger")
        
        if hasattr(self, 'log_timer'):
            self.log_timer.shutdown()
            
        # Final log write
        with self.data_lock:
            if any([self.perception_buffer, self.control_buffer, 
                   self.safety_buffer, self.performance_buffer,
                   self.coordination_buffer]):
                final_data = {
                    'timestamp': rospy.Time.now().to_sec(),
                    'log_end': True,
                    'final_statistics': {
                        'total_messages_logged': self.messages_logged,
                        'total_uptime': (rospy.Time.now() - self.start_time).to_sec()
                    }
                }
                self.write_log_entry(final_data)
                
        # Close log file
        if self.current_log_file:
            self.current_log_file.close()
            
        rospy.loginfo(f"Data logging completed. Total messages: {self.messages_logged}")


if __name__ == '__main__':
    try:
        node = IntegrationDataLogger()
        rospy.on_shutdown(node.shutdown)
        rospy.loginfo("Integration Data Logger running")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Integration Data Logger failed: {e}")

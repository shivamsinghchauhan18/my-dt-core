#!/usr/bin/env python3
"""
Safety Status Publisher Node

Monitors system hardware, sensors, and algorithms to publish comprehensive safety status.
Integrates with the enhanced safety monitoring system.

Author: Enhanced Autonomous Duckietown System
"""

import rospy
import psutil
import time
import threading
from datetime import datetime
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

# Try to import custom messages, fallback to basic types
try:
    from duckietown_enhanced_msgs.msg import SafetyStatus
except ImportError:
    rospy.logwarn("SafetyStatus message not available, using String")
    from std_msgs.msg import String as SafetyStatus


class SafetyStatusPublisher:
    """
    Publishes comprehensive safety status including hardware health,
    sensor monitoring, and algorithm performance.
    """
    
    def __init__(self):
        rospy.init_node('safety_status_publisher', anonymous=True)
        
        # Parameters
        self.publish_rate = rospy.get_param('~publish_rate', 5.0)
        self.enable_hardware_monitoring = rospy.get_param('~enable_hardware_monitoring', True)
        self.enable_sensor_monitoring = rospy.get_param('~enable_sensor_monitoring', True)
        self.enable_algorithm_monitoring = rospy.get_param('~enable_algorithm_monitoring', True)
        
        # Safety thresholds
        self.cpu_warning_threshold = rospy.get_param('~cpu_warning_threshold', 80.0)
        self.cpu_critical_threshold = rospy.get_param('~cpu_critical_threshold', 95.0)
        self.memory_warning_threshold = rospy.get_param('~memory_warning_threshold', 80.0)
        self.memory_critical_threshold = rospy.get_param('~memory_critical_threshold', 95.0)
        self.sensor_timeout_threshold = rospy.get_param('~sensor_timeout_threshold', 2.0)
        
        # Publishers
        self.safety_status_pub = rospy.Publisher('~safety_status', SafetyStatus, queue_size=1)
        self.hardware_status_pub = rospy.Publisher('~hardware_status', String, queue_size=1)
        self.sensor_status_pub = rospy.Publisher('~sensor_status', String, queue_size=1)
        
        # Subscribers for sensor monitoring
        self.last_camera_time = None
        self.last_cmd_time = None
        
        rospy.Subscriber(f'{rospy.get_namespace()}camera_node/image/compressed', 
                        CompressedImage, self.camera_callback)
        rospy.Subscriber(f'{rospy.get_namespace()}car_cmd_switch_node/cmd', 
                        Twist, self.cmd_callback)
        
        # Safety status tracking
        self.current_safety_level = 0  # SAFE
        self.hardware_health = 0  # HEALTH_OK
        self.sensor_health = 0  # SENSOR_OK
        
        # Monitoring thread
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop)
        self.monitoring_thread.daemon = True
        self.shutdown_requested = False
        
        rospy.loginfo("Safety Status Publisher initialized")
        
    def camera_callback(self, msg):
        """Update camera sensor timestamp."""
        self.last_camera_time = rospy.Time.now()
        
    def cmd_callback(self, msg):
        """Update command timestamp."""
        self.last_cmd_time = rospy.Time.now()
        
    def check_hardware_health(self):
        """Check system hardware health."""
        try:
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=1)
            
            # Memory usage
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            
            # Temperature (if available)
            cpu_temp = 0.0
            try:
                temps = psutil.sensors_temperatures()
                if temps:
                    cpu_temp = list(temps.values())[0][0].current
            except:
                pass
            
            # Determine hardware health level
            if cpu_percent > self.cpu_critical_threshold or memory_percent > self.memory_critical_threshold:
                self.hardware_health = 2  # HEALTH_CRITICAL
            elif cpu_percent > self.cpu_warning_threshold or memory_percent > self.memory_warning_threshold:
                self.hardware_health = 1  # HEALTH_WARNING
            else:
                self.hardware_health = 0  # HEALTH_OK
                
            # Publish hardware status
            hardware_msg = String()
            hardware_msg.data = f"CPU: {cpu_percent:.1f}%, Memory: {memory_percent:.1f}%, Temp: {cpu_temp:.1f}°C"
            self.hardware_status_pub.publish(hardware_msg)
            
            return cpu_percent, memory_percent, cpu_temp
            
        except Exception as e:
            rospy.logerr(f"Hardware monitoring error: {e}")
            self.hardware_health = 2  # HEALTH_CRITICAL
            return 0.0, 0.0, 0.0
            
    def check_sensor_health(self):
        """Check sensor health and timeouts."""
        current_time = rospy.Time.now()
        sensor_issues = []
        
        # Check camera timeout
        if self.last_camera_time is None:
            sensor_issues.append("Camera: No data")
        elif (current_time - self.last_camera_time).to_sec() > self.sensor_timeout_threshold:
            sensor_issues.append(f"Camera: Timeout ({(current_time - self.last_camera_time).to_sec():.1f}s)")
            
        # Check command timeout
        if self.last_cmd_time is None:
            sensor_issues.append("Commands: No data")
        elif (current_time - self.last_cmd_time).to_sec() > self.sensor_timeout_threshold:
            sensor_issues.append(f"Commands: Timeout ({(current_time - self.last_cmd_time).to_sec():.1f}s)")
            
        # Determine sensor health
        if len(sensor_issues) > 1:
            self.sensor_health = 2  # SENSOR_CRITICAL
        elif len(sensor_issues) > 0:
            self.sensor_health = 1  # SENSOR_WARNING
        else:
            self.sensor_health = 0  # SENSOR_OK
            
        # Publish sensor status
        sensor_msg = String()
        if sensor_issues:
            sensor_msg.data = "; ".join(sensor_issues)
        else:
            sensor_msg.data = "All sensors OK"
        self.sensor_status_pub.publish(sensor_msg)
        
        return sensor_issues
        
    def determine_overall_safety_level(self, cpu_percent, memory_percent, sensor_issues):
        """Determine overall safety level."""
        # Emergency conditions
        if (cpu_percent > self.cpu_critical_threshold or 
            memory_percent > self.memory_critical_threshold or 
            len(sensor_issues) > 1):
            return 3  # EMERGENCY
            
        # Critical conditions
        if (self.hardware_health == 2 or self.sensor_health == 2):
            return 2  # CRITICAL
            
        # Warning conditions
        if (self.hardware_health == 1 or self.sensor_health == 1):
            return 1  # WARNING
            
        return 0  # SAFE
        
    def publish_safety_status(self, cpu_percent, memory_percent, cpu_temp, sensor_issues):
        """Publish comprehensive safety status."""
        try:
            # Create safety status message
            if hasattr(SafetyStatus, '_type'):  # Real SafetyStatus message
                msg = SafetyStatus()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "base_link"
                
                msg.safety_level = self.current_safety_level
                msg.hardware_health = self.hardware_health
                msg.cpu_temperature = cpu_temp
                msg.memory_usage = memory_percent
                # Set other fields as needed
                
            else:  # Fallback to String
                msg = SafetyStatus()
                status_dict = {
                    'timestamp': rospy.Time.now().to_sec(),
                    'safety_level': self.current_safety_level,
                    'hardware_health': self.hardware_health,
                    'sensor_health': self.sensor_health,
                    'cpu_percent': cpu_percent,
                    'memory_percent': memory_percent,
                    'cpu_temperature': cpu_temp,
                    'sensor_issues': sensor_issues
                }
                msg.data = str(status_dict)
                
            self.safety_status_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"Failed to publish safety status: {e}")
            
    def monitoring_loop(self):
        """Main monitoring loop."""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown() and not self.shutdown_requested:
            try:
                # Check hardware health
                cpu_percent, memory_percent, cpu_temp = self.check_hardware_health()
                
                # Check sensor health
                sensor_issues = self.check_sensor_health()
                
                # Determine overall safety level
                self.current_safety_level = self.determine_overall_safety_level(
                    cpu_percent, memory_percent, sensor_issues)
                
                # Publish safety status
                self.publish_safety_status(cpu_percent, memory_percent, cpu_temp, sensor_issues)
                
                # Log critical issues
                if self.current_safety_level >= 2:  # CRITICAL or EMERGENCY
                    rospy.logwarn(f"Safety Level: {self.current_safety_level}, "
                                f"Hardware: {self.hardware_health}, "
                                f"Sensors: {self.sensor_health}")
                    
            except Exception as e:
                rospy.logerr(f"Monitoring loop error: {e}")
                
            rate.sleep()
            
    def run(self):
        """Start the safety status publisher."""
        self.monitoring_thread.start()
        rospy.loginfo("Safety Status Publisher running")
        rospy.spin()
        
    def shutdown(self):
        """Shutdown the safety status publisher."""
        rospy.loginfo("Shutting down Safety Status Publisher")
        self.shutdown_requested = True
        if self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=2.0)


if __name__ == '__main__':
    try:
        node = SafetyStatusPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Safety Status Publisher failed: {e}")

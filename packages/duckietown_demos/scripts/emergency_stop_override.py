#!/usr/bin/env python3
"""
Emergency Stop Override Node

Provides emergency stop functionality with multiple trigger sources
and fail-safe mechanisms for the enhanced autonomous system.

Author: Enhanced Autonomous Duckietown System
"""

import rospy
import threading
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Try to import custom messages
try:
    from duckietown_msgs.msg import (
        SafetyStatus,
        Twist2DStamped,
        BoolStamped
    )
except ImportError:
    rospy.logwarn("Some custom messages not available, using fallbacks")
    from std_msgs.msg import String as SafetyStatus
    from geometry_msgs.msg import Twist as Twist2DStamped
    from std_msgs.msg import Bool as BoolStamped


class EmergencyStopOverride:
    """
    Emergency stop system with multiple trigger sources:
    - Manual emergency button
    - Safety system triggers
    - System health monitors
    - Remote emergency commands
    """
    
    def __init__(self):
        rospy.init_node('emergency_stop_override', anonymous=True)
        
        # Parameters
        self.enable_emergency_stop = rospy.get_param('~enable_emergency_stop', True)
        self.emergency_timeout = rospy.get_param('~emergency_timeout', 0.2)  # seconds
        self.auto_recovery_enabled = rospy.get_param('~auto_recovery_enabled', False)
        self.recovery_delay = rospy.get_param('~recovery_delay', 5.0)  # seconds
        self.max_recovery_attempts = rospy.get_param('~max_recovery_attempts', 3)
        
        # Emergency triggers
        self.manual_emergency_active = False
        self.safety_emergency_active = False
        self.system_emergency_active = False
        self.remote_emergency_active = False
        
        # Recovery tracking
        self.emergency_start_time = None
        self.recovery_attempts = 0
        self.last_recovery_time = None
        
        # Thread safety
        self.emergency_lock = threading.Lock()
        
        # Emergency state
        self.emergency_active = False
        self.emergency_reason = ""
        
    # Publishers
    self.emergency_cmd_pub = rospy.Publisher('~emergency_car_cmd', Twist2DStamped, queue_size=1)
    self.emergency_status_pub = rospy.Publisher('~emergency_status', String, queue_size=1)
    self.emergency_active_pub = rospy.Publisher('~emergency_active', BoolStamped, queue_size=1)

    # Command override publishers
    # Primary: match master launch remap (~emergency_cmd -> wheels_driver_node/emergency_stop)
    self.override_cmd_pub = rospy.Publisher('~emergency_cmd', Twist2DStamped, queue_size=1)
    # Secondary: direct to car_cmd_switch (kept for backward compatibility)
    self.override_cmd_direct_pub = rospy.Publisher('car_cmd_switch_node/emergency_cmd', Twist2DStamped, queue_size=1)

    # Subscribers for emergency triggers
    rospy.Subscriber('~manual_emergency', Bool, self.manual_emergency_callback)
    rospy.Subscriber('~safety_emergency', SafetyStatus, self.safety_emergency_callback)
    rospy.Subscriber('~system_emergency', String, self.system_emergency_callback)
    rospy.Subscriber('~remote_emergency', Bool, self.remote_emergency_callback)
    # Additional channels used by master launch remaps
    rospy.Subscriber('~navigation_emergency', String, self.system_emergency_callback)
    rospy.Subscriber('~coordination_emergency', String, self.system_emergency_callback)
    rospy.Subscriber('~integration_emergency', String, self.system_emergency_callback)

    # Joystick emergency trigger (if available)
    rospy.Subscriber('/*/joy', Joy, self.joy_callback)

    # Monitor normal car commands to detect anomalies (within current namespace)
    rospy.Subscriber('car_cmd_switch_node/cmd', Twist, self.car_cmd_callback)

    # Emergency monitoring timer
        self.emergency_timer = rospy.Timer(
            rospy.Duration(0.1),  # 10Hz emergency monitoring
            self.emergency_monitor_callback
        )
        
        # Recovery timer
        self.recovery_timer = None
        
        rospy.loginfo("Emergency Stop Override initialized")
        
    def manual_emergency_callback(self, msg):
        """Handle manual emergency button press."""
        with self.emergency_lock:
            if hasattr(msg, 'data'):
                self.manual_emergency_active = msg.data
                if msg.data:
                    self.trigger_emergency("Manual emergency button pressed")
                    
    def safety_emergency_callback(self, msg):
        """Handle safety system emergency triggers."""
        with self.emergency_lock:
            try:
                if hasattr(msg, 'safety_level'):
                    # Real SafetyStatus message
                    if msg.safety_level >= 3:  # EMERGENCY level
                        self.safety_emergency_active = True
                        self.trigger_emergency(f"Safety system emergency: level {msg.safety_level}")
                    else:
                        self.safety_emergency_active = False
                elif hasattr(msg, 'data'):
                    # Fallback to string parsing
                    if 'emergency' in str(msg.data).lower():
                        self.safety_emergency_active = True
                        self.trigger_emergency("Safety system emergency detected")
                    else:
                        self.safety_emergency_active = False
                        
            except Exception as e:
                rospy.logwarn(f"Error processing safety emergency: {e}")
                
    def system_emergency_callback(self, msg):
        """Handle system health emergency triggers."""
        with self.emergency_lock:
            if hasattr(msg, 'data'):
                message = str(msg.data).lower()
                if any(keyword in message for keyword in ['emergency', 'critical', 'failure']):
                    self.system_emergency_active = True
                    self.trigger_emergency(f"System emergency: {msg.data}")
                else:
                    self.system_emergency_active = False
                    
    def remote_emergency_callback(self, msg):
        """Handle remote emergency commands."""
        with self.emergency_lock:
            if hasattr(msg, 'data'):
                self.remote_emergency_active = msg.data
                if msg.data:
                    self.trigger_emergency("Remote emergency command received")
                    
    def joy_callback(self, msg):
        """Handle joystick emergency triggers."""
        try:
            # Check for emergency button combination (buttons 6+7 on most joysticks)
            if len(msg.buttons) > 7 and msg.buttons[6] and msg.buttons[7]:
                with self.emergency_lock:
                    self.manual_emergency_active = True
                    self.trigger_emergency("Joystick emergency buttons pressed")
                    
        except Exception as e:
            rospy.logwarn(f"Error processing joystick emergency: {e}")
            
    def car_cmd_callback(self, msg):
        """Monitor car commands for anomalies."""
        try:
            # Check for dangerous commands
            if hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                linear_speed = abs(msg.linear.x)
                angular_speed = abs(msg.angular.z)
                
                # Trigger emergency if commands are dangerously high
                if linear_speed > 2.0 or angular_speed > 10.0:  # Reasonable safety limits
                    with self.emergency_lock:
                        self.system_emergency_active = True
                        self.trigger_emergency(
                            f"Dangerous command detected: v={linear_speed:.2f}, ω={angular_speed:.2f}")
                            
        except Exception as e:
            rospy.logwarn(f"Error monitoring car commands: {e}")
            
    def trigger_emergency(self, reason):
        """Trigger emergency stop with given reason."""
        if not self.emergency_active:
            self.emergency_active = True
            self.emergency_reason = reason
            self.emergency_start_time = rospy.Time.now()
            
            rospy.logwarn(f"EMERGENCY STOP TRIGGERED: {reason}")
            
            # Immediately publish stop command
            self.publish_emergency_stop()
            
    def clear_emergency(self, reason):
        """Clear emergency stop condition."""
        if self.emergency_active:
            self.emergency_active = False
            self.emergency_reason = ""
            
            rospy.loginfo(f"Emergency stop cleared: {reason}")
            
    def check_emergency_conditions(self):
        """Check if any emergency conditions are active."""
        with self.emergency_lock:
            any_emergency = (
                self.manual_emergency_active or
                self.safety_emergency_active or
                self.system_emergency_active or
                self.remote_emergency_active
            )
            
            if any_emergency and not self.emergency_active:
                # Determine the specific trigger
                triggers = []
                if self.manual_emergency_active:
                    triggers.append("manual")
                if self.safety_emergency_active:
                    triggers.append("safety")
                if self.system_emergency_active:
                    triggers.append("system")
                if self.remote_emergency_active:
                    triggers.append("remote")
                    
                self.trigger_emergency(f"Emergency triggers: {', '.join(triggers)}")
                
            elif not any_emergency and self.emergency_active:
                # Check if auto-recovery is enabled
                if self.auto_recovery_enabled:
                    self.attempt_auto_recovery()
                    
    def attempt_auto_recovery(self):
        """Attempt automatic recovery from emergency state."""
        current_time = rospy.Time.now()
        
        # Check recovery delay
        if (self.last_recovery_time is not None and 
            (current_time - self.last_recovery_time).to_sec() < self.recovery_delay):
            return
            
        # Check recovery attempt limit
        if self.recovery_attempts >= self.max_recovery_attempts:
            rospy.logwarn("Maximum recovery attempts reached, manual intervention required")
            return
            
        # Attempt recovery
        self.recovery_attempts += 1
        self.last_recovery_time = current_time
        
        rospy.loginfo(f"Attempting auto-recovery (attempt {self.recovery_attempts})")
        self.clear_emergency("Auto-recovery attempt")
        
    def publish_emergency_stop(self):
        """Publish emergency stop command."""
        try:
            # Create stop command
            if hasattr(Twist2DStamped, 'header'):
                # Real Twist2DStamped message
                stop_cmd = Twist2DStamped()
                stop_cmd.header.stamp = rospy.Time.now()
                stop_cmd.v = 0.0
                stop_cmd.omega = 0.0
            else:
                # Fallback to basic Twist
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.linear.z = 0.0
                stop_cmd.angular.x = 0.0
                stop_cmd.angular.y = 0.0
                stop_cmd.angular.z = 0.0
                
            # Publish emergency command
            self.emergency_cmd_pub.publish(stop_cmd)
            self.override_cmd_pub.publish(stop_cmd)
            # Backward compatible direct channel if using Twist2DStamped
            if hasattr(stop_cmd, 'v'):
                self.override_cmd_direct_pub.publish(stop_cmd)
            
        except Exception as e:
            rospy.logerr(f"Failed to publish emergency stop: {e}")
            
    def publish_emergency_status(self):
        """Publish current emergency status."""
        try:
            # Emergency status string
            status_msg = String()
            if self.emergency_active:
                elapsed_time = (rospy.Time.now() - self.emergency_start_time).to_sec()
                status_msg.data = f"EMERGENCY ACTIVE: {self.emergency_reason} (Duration: {elapsed_time:.1f}s)"
            else:
                status_msg.data = "Emergency system ready"
                
            self.emergency_status_pub.publish(status_msg)
            
            # Emergency active flag
            active_msg = BoolStamped()
            if hasattr(active_msg, 'header'):
                active_msg.header.stamp = rospy.Time.now()
                active_msg.data = self.emergency_active
            else:
                # Fallback to basic Bool
                active_msg.data = self.emergency_active
                
            self.emergency_active_pub.publish(active_msg)
            
        except Exception as e:
            rospy.logwarn(f"Failed to publish emergency status: {e}")
            
    def emergency_monitor_callback(self, event):
        """Main emergency monitoring callback."""
        try:
            # Check emergency conditions
            self.check_emergency_conditions()
            
            # Publish emergency stop if active
            if self.emergency_active:
                self.publish_emergency_stop()
                
            # Publish status
            self.publish_emergency_status()
            
        except Exception as e:
            rospy.logerr(f"Emergency monitor callback error: {e}")
            
    def shutdown(self):
        """Shutdown the emergency stop override."""
        rospy.loginfo("Shutting down Emergency Stop Override")
        
        # Clear any active emergency
        with self.emergency_lock:
            if self.emergency_active:
                self.clear_emergency("System shutdown")
                
        # Shutdown timers
        if hasattr(self, 'emergency_timer'):
            self.emergency_timer.shutdown()
        if self.recovery_timer:
            self.recovery_timer.shutdown()


if __name__ == '__main__':
    try:
        node = EmergencyStopOverride()
        rospy.on_shutdown(node.shutdown)
        rospy.loginfo("Emergency Stop Override running")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Emergency Stop Override failed: {e}")

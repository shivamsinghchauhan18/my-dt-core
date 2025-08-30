#!/usr/bin/env python3
"""
Emergency Stop Override for Enhanced Autonomous Navigation

Monitors emergency conditions from all sources and provides immediate
motor override capabilities for safety-critical situations.

Author: Duckietown
"""

import rospy
import time
from threading import Lock
from typing import Dict, List, Optional

# ROS messages
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, WheelsCmdStamped


class EmergencyStopOverride:
    """
    Emergency stop override system for autonomous navigation.
    
    Monitors multiple emergency sources and provides immediate motor override
    capabilities to ensure safe operation under all conditions.
    
    Features:
    - Multi-source emergency monitoring
    - Immediate motor command override
    - Emergency state logging and reporting
    - Recovery condition monitoring
    - Comprehensive safety validation
    """
    
    def __init__(self):
        """Initialize the emergency stop override system."""
        self.node_name = "emergency_stop_override"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Thread safety
        self.lock = Lock()
        
        # Configuration
        self.override_timeout = rospy.get_param("~override_timeout", 10.0)  # seconds
        self.recovery_delay = rospy.get_param("~recovery_delay", 2.0)  # seconds
        self.enable_logging = rospy.get_param("~enable_logging", True)
        
        # Emergency state
        self.emergency_active = False
        self.emergency_sources = {
            'navigation': {'active': False, 'last_update': None, 'message': ''},
            'safety': {'active': False, 'last_update': None, 'message': ''},
            'manual': {'active': False, 'last_update': None, 'message': ''},
            'system': {'active': False, 'last_update': None, 'message': ''}
        }
        
        # Emergency history
        self.emergency_history = []
        self.emergency_start_time = None
        self.last_emergency_log = None
        
        # Recovery state
        self.recovery_start_time = None
        self.recovery_conditions_met = False
        
        # Statistics
        self.stats = {
            'total_emergencies': 0,
            'navigation_emergencies': 0,
            'safety_emergencies': 0,
            'manual_emergencies': 0,
            'system_emergencies': 0,
            'total_override_time': 0.0,
            'start_time': time.time()
        }
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] Emergency Stop Override initialized")
        rospy.loginfo(f"[{self.node_name}] Override timeout: {self.override_timeout}s")
        rospy.loginfo(f"[{self.node_name}] Recovery delay: {self.recovery_delay}s")
    
    def _setup_subscribers(self):
        """Setup subscribers for emergency signals from all sources."""
        rospy.loginfo(f"[{self.node_name}] Setting up emergency signal subscribers...")
        
        # Navigation emergency signals
        self.nav_emergency_sub = rospy.Subscriber(
            "~navigation_emergency",
            BoolStamped,
            self._navigation_emergency_callback,
            queue_size=1
        )
        
        # Safety system emergency signals
        self.safety_emergency_sub = rospy.Subscriber(
            "~safety_emergency",
            BoolStamped,
            self._safety_emergency_callback,
            queue_size=1
        )
        
        # Manual emergency signals (e.g., from joystick)
        self.manual_emergency_sub = rospy.Subscriber(
            "~manual_emergency",
            BoolStamped,
            self._manual_emergency_callback,
            queue_size=1
        )
        
        # System-level emergency signals
        self.system_emergency_sub = rospy.Subscriber(
            "/emergency_stop",
            Bool,
            self._system_emergency_callback,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Emergency signal subscribers initialized")
    
    def _setup_publishers(self):
        """Setup publishers for emergency override commands."""
        rospy.loginfo(f"[{self.node_name}] Setting up emergency override publishers...")
        
        # Emergency motor command override
        self.emergency_cmd_pub = rospy.Publisher(
            "~emergency_cmd",
            WheelsCmdStamped,
            queue_size=1
        )
        
        # Emergency status reporting
        self.emergency_status_pub = rospy.Publisher(
            "~emergency_status",
            BoolStamped,
            queue_size=1
        )
        
        # Emergency log messages
        self.emergency_log_pub = rospy.Publisher(
            "~emergency_log",
            String,
            queue_size=10
        )
        
        rospy.loginfo(f"[{self.node_name}] Emergency override publishers initialized")
    
    def _setup_timers(self):
        """Setup periodic timers for emergency monitoring."""
        # High-frequency emergency monitoring
        self.monitoring_timer = rospy.Timer(
            rospy.Duration(0.1),  # 10 Hz
            self._emergency_monitoring_callback
        )
        
        # Emergency status reporting
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0),  # 1 Hz
            self._status_reporting_callback
        )
        
        # Statistics reporting
        self.stats_timer = rospy.Timer(
            rospy.Duration(30.0),  # Every 30 seconds
            self._statistics_callback
        )
        
        rospy.loginfo(f"[{self.node_name}] Emergency monitoring timers initialized")
    
    def _navigation_emergency_callback(self, msg):
        """Handle emergency signals from navigation system."""
        with self.lock:
            current_time = time.time()
            
            self.emergency_sources['navigation']['active'] = msg.data
            self.emergency_sources['navigation']['last_update'] = current_time
            self.emergency_sources['navigation']['message'] = "Navigation emergency triggered"
            
            if msg.data:
                self.stats['navigation_emergencies'] += 1
                self._log_emergency_event("NAVIGATION_EMERGENCY", "Navigation system triggered emergency stop")
                rospy.logwarn(f"[{self.node_name}] NAVIGATION EMERGENCY ACTIVATED!")
            else:
                rospy.loginfo(f"[{self.node_name}] Navigation emergency cleared")
            
            self._update_emergency_state()
    
    def _safety_emergency_callback(self, msg):
        """Handle emergency signals from safety monitoring system."""
        with self.lock:
            current_time = time.time()
            
            self.emergency_sources['safety']['active'] = msg.data
            self.emergency_sources['safety']['last_update'] = current_time
            self.emergency_sources['safety']['message'] = "Safety system emergency triggered"
            
            if msg.data:
                self.stats['safety_emergencies'] += 1
                self._log_emergency_event("SAFETY_EMERGENCY", "Safety monitoring system triggered emergency stop")
                rospy.logwarn(f"[{self.node_name}] SAFETY EMERGENCY ACTIVATED!")
            else:
                rospy.loginfo(f"[{self.node_name}] Safety emergency cleared")
            
            self._update_emergency_state()
    
    def _manual_emergency_callback(self, msg):
        """Handle manual emergency signals (e.g., from joystick)."""
        with self.lock:
            current_time = time.time()
            
            self.emergency_sources['manual']['active'] = msg.data
            self.emergency_sources['manual']['last_update'] = current_time
            self.emergency_sources['manual']['message'] = "Manual emergency stop activated"
            
            if msg.data:
                self.stats['manual_emergencies'] += 1
                self._log_emergency_event("MANUAL_EMERGENCY", "Manual emergency stop activated")
                rospy.logwarn(f"[{self.node_name}] MANUAL EMERGENCY ACTIVATED!")
            else:
                rospy.loginfo(f"[{self.node_name}] Manual emergency cleared")
            
            self._update_emergency_state()
    
    def _system_emergency_callback(self, msg):
        """Handle system-level emergency signals."""
        with self.lock:
            current_time = time.time()
            
            self.emergency_sources['system']['active'] = msg.data
            self.emergency_sources['system']['last_update'] = current_time
            self.emergency_sources['system']['message'] = "System-level emergency triggered"
            
            if msg.data:
                self.stats['system_emergencies'] += 1
                self._log_emergency_event("SYSTEM_EMERGENCY", "System-level emergency stop triggered")
                rospy.logwarn(f"[{self.node_name}] SYSTEM EMERGENCY ACTIVATED!")
            else:
                rospy.loginfo(f"[{self.node_name}] System emergency cleared")
            
            self._update_emergency_state()
    
    def _update_emergency_state(self):
        """Update overall emergency state based on all sources."""
        current_time = time.time()
        
        # Check if any emergency source is active
        any_emergency_active = any(source['active'] for source in self.emergency_sources.values())
        
        # Handle emergency activation
        if any_emergency_active and not self.emergency_active:
            self.emergency_active = True
            self.emergency_start_time = current_time
            self.recovery_start_time = None
            self.recovery_conditions_met = False
            self.stats['total_emergencies'] += 1
            
            rospy.logerr(f"[{self.node_name}] *** EMERGENCY STOP ACTIVATED ***")
            self._log_emergency_event("EMERGENCY_ACTIVATED", "Emergency stop system activated")
            
        # Handle emergency deactivation
        elif not any_emergency_active and self.emergency_active:
            if self.recovery_start_time is None:
                self.recovery_start_time = current_time
                rospy.loginfo(f"[{self.node_name}] Emergency conditions cleared, starting recovery delay...")
            
            # Check if recovery delay has passed
            elif current_time - self.recovery_start_time >= self.recovery_delay:
                self.emergency_active = False
                
                # Update statistics
                if self.emergency_start_time:
                    emergency_duration = current_time - self.emergency_start_time
                    self.stats['total_override_time'] += emergency_duration
                
                rospy.loginfo(f"[{self.node_name}] Emergency stop deactivated after recovery delay")
                self._log_emergency_event("EMERGENCY_DEACTIVATED", "Emergency stop system deactivated")
                
                self.emergency_start_time = None
                self.recovery_start_time = None
    
    def _emergency_monitoring_callback(self, event):
        """High-frequency emergency monitoring and command override."""
        if not self.emergency_active:
            return
        
        current_time = time.time()
        
        with self.lock:
            # Generate emergency stop command
            emergency_cmd = WheelsCmdStamped()
            emergency_cmd.header.stamp = rospy.Time.now()
            emergency_cmd.vel_left = 0.0
            emergency_cmd.vel_right = 0.0
            
            # Publish emergency command
            self.emergency_cmd_pub.publish(emergency_cmd)
            
            # Check for emergency timeout
            if (self.emergency_start_time and 
                current_time - self.emergency_start_time > self.override_timeout):
                
                rospy.logwarn(f"[{self.node_name}] Emergency stop timeout reached ({self.override_timeout}s)")
                self._log_emergency_event("EMERGENCY_TIMEOUT", 
                                        f"Emergency stop active for {self.override_timeout}s")
            
            # Log emergency status periodically
            if (self.enable_logging and 
                (self.last_emergency_log is None or 
                 current_time - self.last_emergency_log > 5.0)):
                
                active_sources = [name for name, source in self.emergency_sources.items() 
                                if source['active']]
                
                rospy.logwarn(f"[{self.node_name}] Emergency active - Sources: {active_sources}")
                self.last_emergency_log = current_time
    
    def _status_reporting_callback(self, event):
        """Periodic emergency status reporting."""
        with self.lock:
            # Publish emergency status
            status_msg = BoolStamped()
            status_msg.header.stamp = rospy.Time.now()
            status_msg.data = self.emergency_active
            self.emergency_status_pub.publish(status_msg)
            
            # Check for stale emergency sources
            current_time = time.time()
            stale_threshold = 5.0  # seconds
            
            for source_name, source_data in self.emergency_sources.items():
                if (source_data['last_update'] and 
                    current_time - source_data['last_update'] > stale_threshold and
                    source_data['active']):
                    
                    rospy.logwarn(f"[{self.node_name}] Emergency source '{source_name}' is stale "
                                f"({current_time - source_data['last_update']:.1f}s)")
    
    def _statistics_callback(self, event):
        """Report emergency system statistics."""
        current_time = time.time()
        runtime = current_time - self.stats['start_time']
        
        rospy.loginfo(f"[{self.node_name}] Emergency System Statistics (Runtime: {runtime:.1f}s):")
        rospy.loginfo(f"[{self.node_name}]   Total emergencies: {self.stats['total_emergencies']}")
        rospy.loginfo(f"[{self.node_name}]   Navigation emergencies: {self.stats['navigation_emergencies']}")
        rospy.loginfo(f"[{self.node_name}]   Safety emergencies: {self.stats['safety_emergencies']}")
        rospy.loginfo(f"[{self.node_name}]   Manual emergencies: {self.stats['manual_emergencies']}")
        rospy.loginfo(f"[{self.node_name}]   System emergencies: {self.stats['system_emergencies']}")
        rospy.loginfo(f"[{self.node_name}]   Total override time: {self.stats['total_override_time']:.1f}s")
        rospy.loginfo(f"[{self.node_name}]   Current status: {'EMERGENCY ACTIVE' if self.emergency_active else 'Normal'}")
        
        if self.emergency_active:
            active_sources = [name for name, source in self.emergency_sources.items() 
                            if source['active']]
            rospy.loginfo(f"[{self.node_name}]   Active emergency sources: {active_sources}")
    
    def _log_emergency_event(self, event_type: str, message: str):
        """Log an emergency event with timestamp."""
        current_time = time.time()
        
        event_data = {
            'timestamp': current_time,
            'event_type': event_type,
            'message': message,
            'active_sources': {name: source['active'] for name, source in self.emergency_sources.items()}
        }
        
        self.emergency_history.append(event_data)
        
        # Keep only recent history
        if len(self.emergency_history) > 100:
            self.emergency_history = self.emergency_history[-50:]
        
        # Publish log message
        log_msg = String()
        log_msg.data = f"[{event_type}] {message}"
        self.emergency_log_pub.publish(log_msg)
        
        rospy.loginfo(f"[{self.node_name}] Emergency event logged: {event_type} - {message}")
    
    def get_emergency_status(self) -> Dict:
        """Get current emergency status information."""
        with self.lock:
            current_time = time.time()
            
            status = {
                'emergency_active': self.emergency_active,
                'emergency_duration': (current_time - self.emergency_start_time 
                                     if self.emergency_start_time else 0.0),
                'recovery_time_remaining': (self.recovery_delay - (current_time - self.recovery_start_time)
                                          if self.recovery_start_time else 0.0),
                'active_sources': {name: source['active'] for name, source in self.emergency_sources.items()},
                'source_messages': {name: source['message'] for name, source in self.emergency_sources.items()},
                'statistics': self.stats.copy(),
                'recent_events': self.emergency_history[-10:] if self.emergency_history else []
            }
            
            return status
    
    def shutdown(self):
        """Clean shutdown of the emergency stop override system."""
        rospy.loginfo(f"[{self.node_name}] Shutting down emergency stop override...")
        
        with self.lock:
            # If emergency is active, send final stop command
            if self.emergency_active:
                emergency_cmd = WheelsCmdStamped()
                emergency_cmd.header.stamp = rospy.Time.now()
                emergency_cmd.vel_left = 0.0
                emergency_cmd.vel_right = 0.0
                self.emergency_cmd_pub.publish(emergency_cmd)
                
                rospy.logwarn(f"[{self.node_name}] Final emergency stop command sent during shutdown")
            
            # Log final statistics
            current_time = time.time()
            runtime = current_time - self.stats['start_time']
            
            rospy.loginfo(f"[{self.node_name}] Final Emergency System Statistics:")
            rospy.loginfo(f"[{self.node_name}]   Runtime: {runtime:.1f}s")
            rospy.loginfo(f"[{self.node_name}]   Total emergencies: {self.stats['total_emergencies']}")
            rospy.loginfo(f"[{self.node_name}]   Total override time: {self.stats['total_override_time']:.1f}s")
            
            if self.emergency_history:
                rospy.loginfo(f"[{self.node_name}]   Emergency events logged: {len(self.emergency_history)}")
        
        rospy.loginfo(f"[{self.node_name}] Emergency stop override shutdown complete")


def main():
    """Main function to run the emergency stop override."""
    try:
        override_system = EmergencyStopOverride()
        rospy.loginfo("Emergency Stop Override running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Emergency Stop Override interrupted")
    except Exception as e:
        rospy.logerr(f"Emergency Stop Override error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'override_system' in locals():
            override_system.shutdown()


if __name__ == '__main__':
    main()
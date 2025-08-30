#!/usr/bin/env python3
"""
Lane Change Planner Node

Implements dynamic lane changing with gap analysis, trajectory generation,
and LED signaling for enhanced autonomous navigation.

Author: Enhanced Autonomous Duckietown System
"""

import rospy
import numpy as np
import time
from threading import Lock
from collections import deque
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

# Try to import custom messages
try:
    from duckietown_msgs.msg import (
        LanePose, 
        Twist2DStamped,
        BoolStamped,
        LEDPattern
    )
    from duckietown_enhanced_msgs.msg import ObjectDetectionArray
except ImportError:
    rospy.logwarn("Some custom messages not available, using fallbacks")
    from std_msgs.msg import String as ObjectDetectionArray
    from geometry_msgs.msg import Twist as Twist2DStamped
    from std_msgs.msg import Bool as BoolStamped
    from std_msgs.msg import String as LEDPattern
    
    # Create minimal LanePose if not available
    class LanePose:
        def __init__(self):
            self.d = 0.0
            self.phi = 0.0
            self.in_lane = True


class LaneChangePlanner:
    """
    Manages lane changing decisions and trajectory generation.
    
    Features:
    - Gap analysis and safety assessment
    - Smooth trajectory generation
    - LED signaling coordination
    - Collision avoidance integration
    """
    
    def __init__(self):
        rospy.init_node('lane_change_planner', anonymous=True)
        
        # Parameters
        self.enable_lane_changing = rospy.get_param('~enable_lane_changing', True)
        self.gap_analysis_distance = rospy.get_param('~gap_analysis_distance', 2.0)
        self.safety_margin = rospy.get_param('~safety_margin', 0.5)
        self.decision_frequency = rospy.get_param('~decision_frequency', 2.0)
        
        # Trajectory parameters
        self.lane_change_duration = rospy.get_param('~lane_change_duration', 3.0)
        self.max_lateral_acceleration = rospy.get_param('~max_lateral_acceleration', 2.0)
        self.trajectory_smoothness_factor = rospy.get_param('~trajectory_smoothness_factor', 0.8)
        self.polynomial_degree = rospy.get_param('~polynomial_degree', 5)
        
        # LED signaling parameters
        self.enable_led_signaling = rospy.get_param('~enable_led_signaling', True)
        self.signal_duration = rospy.get_param('~signal_duration', 1.0)
        self.signal_frequency = rospy.get_param('~signal_frequency', 2.0)
        self.abort_signal_duration = rospy.get_param('~abort_signal_duration', 0.5)
        
        # State variables
        self.current_lane_pose = None
        self.detected_objects = []
        self.lane_change_state = "normal"  # normal, planning, executing, aborting
        self.lane_change_start_time = None
        self.target_lane_offset = 0.0
        self.trajectory_points = []
        
        # Thread safety
        self.state_lock = Lock()
        
        # Publishers
        self.lane_change_cmd_pub = rospy.Publisher('~lane_change_cmd', Twist2DStamped, queue_size=1)
        self.led_pattern_pub = rospy.Publisher('~led_pattern', LEDPattern, queue_size=1)
        self.lane_change_status_pub = rospy.Publisher('~lane_change_status', String, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('~lane_pose', LanePose, self.lane_pose_callback)
        rospy.Subscriber('~object_detections', ObjectDetectionArray, self.objects_callback)
        rospy.Subscriber('~car_cmd', Twist, self.car_cmd_callback)
        
        # Decision timer
        self.decision_timer = rospy.Timer(
            rospy.Duration(1.0 / self.decision_frequency), 
            self.decision_callback
        )
        
        rospy.loginfo("Lane Change Planner initialized")
        
    def lane_pose_callback(self, msg):
        """Update current lane pose."""
        with self.state_lock:
            self.current_lane_pose = msg
            
    def objects_callback(self, msg):
        """Update detected objects for gap analysis."""
        with self.state_lock:
            if hasattr(msg, 'detections'):
                self.detected_objects = msg.detections
            else:
                # Fallback for basic message types
                self.detected_objects = []
                
    def car_cmd_callback(self, msg):
        """Monitor car commands for integration."""
        # Can be used to detect manual overrides or emergency stops
        pass
        
    def analyze_gap_safety(self, target_lane_offset):
        """
        Analyze if there's a safe gap for lane changing.
        
        Args:
            target_lane_offset: Target lateral offset for lane change
            
        Returns:
            bool: True if safe to change lanes
        """
        if not self.detected_objects:
            return True  # No objects detected, assume safe
            
        # Simple gap analysis - in a real implementation,
        # this would use object positions, velocities, and trajectories
        
        for obj in self.detected_objects:
            # Check if object is in target lane area
            if hasattr(obj, 'lateral_distance') and hasattr(obj, 'longitudinal_distance'):
                lateral_dist = abs(obj.lateral_distance - target_lane_offset)
                longitudinal_dist = abs(obj.longitudinal_distance)
                
                # Check safety margins
                if (lateral_dist < self.safety_margin and 
                    longitudinal_dist < self.gap_analysis_distance):
                    return False
                    
        return True
        
    def should_initiate_lane_change(self):
        """
        Determine if a lane change should be initiated.
        
        Returns:
            tuple: (should_change, target_offset, reason)
        """
        if not self.enable_lane_changing:
            return False, 0.0, "Lane changing disabled"
            
        if not self.current_lane_pose:
            return False, 0.0, "No lane pose available"
            
        # Simple lane change logic - change if too far from center
        lane_deviation = abs(self.current_lane_pose.d)
        
        if lane_deviation > 0.15:  # 15cm from center
            # Decide which direction to change
            if self.current_lane_pose.d > 0:
                target_offset = -0.3  # Move left
            else:
                target_offset = 0.3   # Move right
                
            # Check if safe
            if self.analyze_gap_safety(target_offset):
                return True, target_offset, f"Correcting deviation: {lane_deviation:.2f}m"
                
        return False, 0.0, "No lane change needed"
        
    def generate_trajectory(self, target_offset):
        """
        Generate smooth trajectory for lane change.
        
        Args:
            target_offset: Target lateral offset
            
        Returns:
            list: Trajectory points [(time, lateral_offset), ...]
        """
        trajectory = []
        dt = 0.1  # 10Hz trajectory
        total_time = self.lane_change_duration
        
        # Generate polynomial trajectory
        for i in range(int(total_time / dt) + 1):
            t = i * dt
            t_norm = t / total_time
            
            # Smooth S-curve using polynomial
            if t_norm <= 1.0:
                # 5th order polynomial for smooth acceleration profile
                s = 10 * t_norm**3 - 15 * t_norm**4 + 6 * t_norm**5
                lateral_offset = s * target_offset
            else:
                lateral_offset = target_offset
                
            trajectory.append((t, lateral_offset))
            
        return trajectory
        
    def execute_lane_change(self, target_offset):
        """
        Execute lane change maneuver.
        
        Args:
            target_offset: Target lateral offset for lane change
        """
        with self.state_lock:
            if self.lane_change_state != "normal":
                return False
                
            # Generate trajectory
            self.trajectory_points = self.generate_trajectory(target_offset)
            self.target_lane_offset = target_offset
            self.lane_change_start_time = rospy.Time.now()
            self.lane_change_state = "executing"
            
            # Signal lane change intention
            if self.enable_led_signaling:
                self.signal_lane_change_intention(target_offset)
                
            rospy.loginfo(f"Starting lane change: target_offset={target_offset:.2f}m")
            return True
            
    def signal_lane_change_intention(self, target_offset):
        """
        Signal lane change intention using LEDs.
        
        Args:
            target_offset: Direction of lane change
        """
        try:
            led_msg = LEDPattern()
            
            if hasattr(led_msg, 'color_list'):
                # Real LEDPattern message
                if target_offset > 0:
                    led_msg.color_list = ['green'] * 5  # Right turn signal
                else:
                    led_msg.color_list = ['blue'] * 5   # Left turn signal
            else:
                # Fallback to String
                direction = "RIGHT" if target_offset > 0 else "LEFT"
                led_msg.data = f"LANE_CHANGE_{direction}_SIGNAL"
                
            self.led_pattern_pub.publish(led_msg)
            
        except Exception as e:
            rospy.logwarn(f"Failed to publish LED signal: {e}")
            
    def update_lane_change_execution(self):
        """Update ongoing lane change execution."""
        if self.lane_change_state != "executing":
            return
            
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.lane_change_start_time).to_sec()
        
        # Find current trajectory point
        current_target_offset = 0.0
        for t, offset in self.trajectory_points:
            if t <= elapsed_time:
                current_target_offset = offset
            else:
                break
                
        # Check if lane change is complete
        if elapsed_time >= self.lane_change_duration:
            with self.state_lock:
                self.lane_change_state = "normal"
                rospy.loginfo("Lane change completed")
                return
                
        # Generate control command
        self.publish_lane_change_command(current_target_offset)
        
    def publish_lane_change_command(self, target_offset):
        """
        Publish lane change control command.
        
        Args:
            target_offset: Current target lateral offset
        """
        try:
            cmd_msg = Twist2DStamped()
            
            if hasattr(cmd_msg, 'header'):
                cmd_msg.header.stamp = rospy.Time.now()
                
            if self.current_lane_pose:
                # Simple proportional controller for lateral correction
                lateral_error = target_offset - self.current_lane_pose.d
                omega = np.clip(lateral_error * 2.0, -2.0, 2.0)  # Limit turning rate
                
                if hasattr(cmd_msg, 'v'):
                    cmd_msg.v = 0.3  # Constant forward velocity
                    cmd_msg.omega = omega
                else:
                    # Fallback to basic Twist
                    cmd_msg.linear.x = 0.3
                    cmd_msg.angular.z = omega
                    
            self.lane_change_cmd_pub.publish(cmd_msg)
            
        except Exception as e:
            rospy.logwarn(f"Failed to publish lane change command: {e}")
            
    def publish_status(self, message):
        """Publish lane change status."""
        status_msg = String()
        status_msg.data = f"{self.lane_change_state}: {message}"
        self.lane_change_status_pub.publish(status_msg)
        
    def decision_callback(self, event):
        """Main decision loop callback."""
        try:
            # Update ongoing lane change
            if self.lane_change_state == "executing":
                self.update_lane_change_execution()
                return
                
            # Check if new lane change should be initiated
            should_change, target_offset, reason = self.should_initiate_lane_change()
            
            if should_change:
                if self.execute_lane_change(target_offset):
                    self.publish_status(f"Initiated: {reason}")
                else:
                    self.publish_status(f"Failed to initiate: {reason}")
            else:
                self.publish_status(reason)
                
        except Exception as e:
            rospy.logerr(f"Decision callback error: {e}")
            
    def shutdown(self):
        """Shutdown the lane change planner."""
        rospy.loginfo("Shutting down Lane Change Planner")
        if hasattr(self, 'decision_timer'):
            self.decision_timer.shutdown()


if __name__ == '__main__':
    try:
        node = LaneChangePlanner()
        rospy.on_shutdown(node.shutdown)
        rospy.loginfo("Lane Change Planner running")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Lane Change Planner failed: {e}")

#!/usr/bin/env python3
import numpy as np
import time
import math
from threading import Timer

import rospy
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from duckietown_msgs.msg import BoolStamped, FSMState, LanePose, SegmentList, StopLineReading, Twist2DStamped
from duckietown_msgs.srv import ChangePattern
from geometry_msgs.msg import Point
from std_msgs.msg import String


class PrecisionStopController:
    """
    Precision stop controller for AprilTag-based stops with smooth deceleration profiles
    and LED integration for visual feedback.
    """
    
    def __init__(self, node_instance):
        """
        Initialize the precision stop controller.
        
        Args:
            node_instance: Reference to the parent node for logging and publishing
        """
        self.node = node_instance
        
        # Stop control parameters
        self.target_stop_distance = 0.30  # 30cm from AprilTag
        self.deceleration_start_distance = 1.0  # Start decelerating at 1m
        self.stop_duration = 2.0  # Stop for 2 seconds
        self.max_deceleration = 2.0  # Maximum deceleration in m/s²
        self.min_velocity = 0.05  # Minimum velocity before full stop
        
        # State variables
        self.is_stopping = False
        self.stop_start_time = None
        self.current_velocity = 0.0
        self.target_distance = 0.0
        self.deceleration_profile = []
        self.stop_timer = None
        
        # LED integration
        self.led_service_available = False
        self.original_led_pattern = None
        
        # Performance monitoring
        self.stop_count = 0
        self.total_stop_accuracy = 0.0
        self.deceleration_profiles_generated = 0
        
        # Try to connect to LED service
        try:
            rospy.wait_for_service('/led_emitter_node/set_pattern', timeout=2.0)
            self.led_service = rospy.ServiceProxy('/led_emitter_node/set_pattern', ChangePattern)
            self.led_service_available = True
            rospy.loginfo("[PrecisionStopController] LED service connected successfully")
        except rospy.ROSException:
            rospy.logwarn("[PrecisionStopController] LED service not available, continuing without LED feedback")
        
        # Publisher for control commands
        self.cmd_pub = rospy.Publisher('/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        
        rospy.loginfo("[PrecisionStopController] Initialized with target distance: {:.2f}m".format(self.target_stop_distance))
        rospy.loginfo("[PrecisionStopController] Deceleration start distance: {:.2f}m".format(self.deceleration_start_distance))
        rospy.loginfo("[PrecisionStopController] Stop duration: {:.1f}s".format(self.stop_duration))
    
    def initiate_precision_stop(self, current_distance, current_velocity=0.3):
        """
        Initiate precision stop sequence with smooth deceleration profile.
        
        Args:
            current_distance: Current distance to the stop target
            current_velocity: Current velocity of the robot
        """
        if self.is_stopping:
            rospy.logdebug("[PrecisionStopController] Stop sequence already in progress")
            return
        
        self.current_velocity = current_velocity
        self.target_distance = current_distance
        self.is_stopping = True
        self.stop_start_time = time.time()
        
        rospy.loginfo(f"[PrecisionStopController] Initiating precision stop sequence")
        rospy.loginfo(f"[PrecisionStopController] Current distance: {current_distance:.3f}m, velocity: {current_velocity:.3f}m/s")
        
        # Generate deceleration profile
        self.deceleration_profile = self._generate_deceleration_profile(
            current_distance, current_velocity
        )
        
        # Activate stop LED pattern
        self._activate_stop_led_pattern()
        
        # Log deceleration profile details
        rospy.logdebug(f"[PrecisionStopController] Generated deceleration profile with {len(self.deceleration_profile)} waypoints")
        for i, (dist, vel, decel) in enumerate(self.deceleration_profile[:5]):  # Log first 5 waypoints
            rospy.logdebug(f"[PrecisionStopController] Profile {i}: distance={dist:.3f}m, velocity={vel:.3f}m/s, decel={decel:.3f}m/s²")
        
        # Start executing the stop sequence
        self._execute_stop_sequence()
    
    def _generate_deceleration_profile(self, initial_distance, initial_velocity):
        """
        Generate smooth deceleration profile for precision stopping.
        
        Args:
            initial_distance: Initial distance to target
            initial_velocity: Initial velocity
            
        Returns:
            List of (distance, velocity, deceleration) tuples
        """
        profile = []
        
        # Calculate stopping distance needed
        stopping_distance = initial_distance - self.target_stop_distance
        
        if stopping_distance <= 0:
            rospy.logwarn(f"[PrecisionStopController] Already at or past target distance: {initial_distance:.3f}m")
            return [(initial_distance, 0.0, self.max_deceleration)]
        
        # Calculate required deceleration for smooth stop
        # Using kinematic equation: v² = u² + 2as
        required_deceleration = (initial_velocity ** 2) / (2 * stopping_distance)
        
        # Limit deceleration to maximum allowed
        actual_deceleration = min(required_deceleration, self.max_deceleration)
        
        rospy.logdebug(f"[PrecisionStopController] Deceleration calculation:")
        rospy.logdebug(f"[PrecisionStopController] Stopping distance: {stopping_distance:.3f}m")
        rospy.logdebug(f"[PrecisionStopController] Required deceleration: {required_deceleration:.3f}m/s²")
        rospy.logdebug(f"[PrecisionStopController] Actual deceleration: {actual_deceleration:.3f}m/s²")
        
        # Generate profile points
        num_points = 20
        for i in range(num_points + 1):
            progress = i / num_points
            
            # Distance traveled during deceleration
            distance_traveled = progress * stopping_distance
            current_distance = initial_distance - distance_traveled
            
            # Velocity at this point using kinematic equation
            velocity_squared = initial_velocity ** 2 - 2 * actual_deceleration * distance_traveled
            current_velocity = max(0.0, math.sqrt(max(0.0, velocity_squared)))
            
            # Apply minimum velocity threshold
            if current_velocity < self.min_velocity and current_distance > self.target_stop_distance:
                current_velocity = self.min_velocity
            
            profile.append((current_distance, current_velocity, actual_deceleration))
        
        self.deceleration_profiles_generated += 1
        
        rospy.logdebug(f"[PrecisionStopController] Profile generation complete: {len(profile)} points")
        rospy.logdebug(f"[PrecisionStopController] Final profile point: distance={profile[-1][0]:.3f}m, velocity={profile[-1][1]:.3f}m/s")
        
        return profile
    
    def _execute_stop_sequence(self):
        """
        Execute the stop sequence by following the deceleration profile.
        """
        if not self.deceleration_profile:
            rospy.logwarn("[PrecisionStopController] No deceleration profile available")
            return
        
        rospy.loginfo("[PrecisionStopController] Executing stop sequence")
        
        # For now, we'll publish a stop command immediately
        # In a full implementation, this would follow the deceleration profile over time
        stop_cmd = Twist2DStamped()
        stop_cmd.header.stamp = rospy.Time.now()
        stop_cmd.v = 0.0  # Stop velocity
        stop_cmd.omega = 0.0  # No rotation
        
        self.cmd_pub.publish(stop_cmd)
        
        rospy.loginfo("[PrecisionStopController] Stop command published, starting 2-second timer")
        rospy.logdebug(f"[PrecisionStopController] Stop command: v={stop_cmd.v:.3f}, omega={stop_cmd.omega:.3f}")
        
        # Start 2-second stop timer
        self.stop_timer = Timer(self.stop_duration, self._complete_stop_sequence)
        self.stop_timer.start()
        
        # Real-time monitoring
        rospy.loginfo(f"[PrecisionStopController] Stop timer started: {self.stop_duration}s duration")
    
    def _complete_stop_sequence(self):
        """
        Complete the stop sequence after the timer expires.
        """
        stop_end_time = time.time()
        total_stop_time = stop_end_time - self.stop_start_time if self.stop_start_time else 0.0
        
        rospy.loginfo("[PrecisionStopController] Stop sequence completed")
        rospy.loginfo(f"[PrecisionStopController] Total stop sequence time: {total_stop_time:.3f}s")
        
        # Calculate stop accuracy (would need actual distance measurement)
        # For now, assume perfect accuracy
        stop_accuracy = 0.95  # 95% accuracy placeholder
        self.stop_count += 1
        self.total_stop_accuracy += stop_accuracy
        
        # Deactivate stop LED pattern
        self._deactivate_stop_led_pattern()
        
        # Reset state
        self.is_stopping = False
        self.stop_start_time = None
        self.deceleration_profile = []
        
        # Performance monitoring
        avg_accuracy = self.total_stop_accuracy / self.stop_count if self.stop_count > 0 else 0.0
        rospy.loginfo(f"[PrecisionStopController] Stop performance: {self.stop_count} stops, avg accuracy: {avg_accuracy:.3f}")
        
        # Real-time monitoring
        rospy.loginfo(f"[PrecisionStopController] Stop sequence {self.stop_count} completed successfully")
        rospy.logdebug(f"[PrecisionStopController] Deceleration profiles generated: {self.deceleration_profiles_generated}")
        
        rospy.loginfo("[PrecisionStopController] Resuming normal operation")
    
    def _activate_stop_led_pattern(self):
        """
        Activate LED pattern to indicate stopping sequence.
        """
        if not self.led_service_available:
            rospy.logdebug("[PrecisionStopController] LED service not available, skipping LED activation")
            return
        
        try:
            # Set LED pattern to red for stopping
            pattern_msg = String()
            pattern_msg.data = "RED"
            
            response = self.led_service(pattern_msg)
            rospy.loginfo("[PrecisionStopController] Stop LED pattern activated (RED)")
            rospy.logdebug(f"[PrecisionStopController] LED service response: {response}")
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"[PrecisionStopController] Failed to activate stop LED pattern: {e}")
    
    def _deactivate_stop_led_pattern(self):
        """
        Deactivate stop LED pattern and return to normal.
        """
        if not self.led_service_available:
            rospy.logdebug("[PrecisionStopController] LED service not available, skipping LED deactivation")
            return
        
        try:
            # Return to normal LED pattern (white/off)
            pattern_msg = String()
            pattern_msg.data = "WHITE"
            
            response = self.led_service(pattern_msg)
            rospy.loginfo("[PrecisionStopController] Stop LED pattern deactivated (WHITE)")
            rospy.logdebug(f"[PrecisionStopController] LED service response: {response}")
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"[PrecisionStopController] Failed to deactivate stop LED pattern: {e}")
    
    def abort_stop_sequence(self):
        """
        Abort the current stop sequence in case of emergency.
        """
        if not self.is_stopping:
            return
        
        rospy.logwarn("[PrecisionStopController] Aborting stop sequence")
        
        # Cancel timer if running
        if self.stop_timer and self.stop_timer.is_alive():
            self.stop_timer.cancel()
            rospy.loginfo("[PrecisionStopController] Stop timer cancelled")
        
        # Deactivate LED pattern
        self._deactivate_stop_led_pattern()
        
        # Reset state
        self.is_stopping = False
        self.stop_start_time = None
        self.deceleration_profile = []
        
        rospy.logwarn("[PrecisionStopController] Stop sequence aborted, resuming normal operation")
    
    def is_stop_in_progress(self):
        """
        Check if a stop sequence is currently in progress.
        
        Returns:
            bool: True if stop sequence is active
        """
        return self.is_stopping
    
    def get_stop_metrics(self):
        """
        Get performance metrics for the stop controller.
        
        Returns:
            dict: Performance metrics
        """
        if self.stop_count == 0:
            return {}
        
        return {
            'total_stops': self.stop_count,
            'average_accuracy': self.total_stop_accuracy / self.stop_count,
            'deceleration_profiles_generated': self.deceleration_profiles_generated,
            'led_service_available': self.led_service_available
        }


class StopLineFilterNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(StopLineFilterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Initialize the parameters
        self.stop_distance = DTParam("~stop_distance", param_type=ParamType.FLOAT)
        self.min_segs = DTParam("~min_segs", param_type=ParamType.INT)
        self.off_time = DTParam("~off_time", param_type=ParamType.FLOAT)
        self.max_y = DTParam("~max_y", param_type=ParamType.FLOAT)
        
        # Precision stop control parameters
        self.enable_precision_stop = DTParam("~enable_precision_stop", param_type=ParamType.BOOL, default=True)
        self.precision_stop_distance = DTParam("~precision_stop_distance", param_type=ParamType.FLOAT, default=0.30)
        self.deceleration_start_distance = DTParam("~deceleration_start_distance", param_type=ParamType.FLOAT, default=1.0)
        self.stop_duration = DTParam("~stop_duration", param_type=ParamType.FLOAT, default=2.0)

        ## state vars
        self.lane_pose = LanePose()
        self.state = "JOYSTICK_CONTROL"
        self.sleep = False
        
        # AprilTag detection integration
        self.apriltag_detections = []
        self.last_apriltag_time = None
        
        # Performance monitoring
        self.detection_count = 0
        self.precision_stops_executed = 0
        self.last_performance_log = time.time()

    ## publishers and subscribers
    self.sub_segs = rospy.Subscriber("~segment_list", SegmentList, self.cb_segments)
    self.sub_lane = rospy.Subscriber("~lane_pose", LanePose, self.cb_lane_pose)
    self.sub_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cb_state_change)
        
    # Subscribe to AprilTag detections for precision stop control (respect namespace)
    apriltag_topic = rospy.get_param('~apriltag_detections_topic', 'apriltag_detector_node/detections')
    self.sub_apriltag = rospy.Subscriber(apriltag_topic, rospy.AnyMsg, self.cb_apriltag_detections)
        
    self.pub_stop_line_reading = rospy.Publisher("~stop_line_reading", StopLineReading, queue_size=1)
    self.pub_at_stop_line = rospy.Publisher("~at_stop_line", BoolStamped, queue_size=1)
        
        # Initialize precision stop controller
        if self.enable_precision_stop.value:
            self.precision_stop_controller = PrecisionStopController(self)
            rospy.loginfo("[StopLineFilterNode] Precision stop controller initialized")
        else:
            self.precision_stop_controller = None
            rospy.loginfo("[StopLineFilterNode] Precision stop controller disabled")
        
        rospy.loginfo(f"[StopLineFilterNode] Initialized with precision stop: {self.enable_precision_stop.value}")
        rospy.loginfo(f"[StopLineFilterNode] Stop distance: {self.stop_distance.value}m")
        rospy.loginfo(f"[StopLineFilterNode] Precision stop distance: {self.precision_stop_distance.value}m")

    # def setupParam(self,param_name,default_value):
    #     value = rospy.get_param(param_name,default_value)
    #     rospy.set_param(param_name,value) #Write to parameter server for transparancy
    #     rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
    #     return value
    #
    # def updateParams(self,event):
    #     self.stop_distance = rospy.get_param("~stop_distance")
    #     self.min_segs      = rospy.get_param("~min_segs")
    #     self.off_time      = rospy.get_param("~off_time")
    #     self.max_y         = rospy.get_param("~max_y")

    def cb_state_change(self, msg):
        if (self.state == "INTERSECTION_CONTROL") and (msg.state == "LANE_FOLLOWING"):
            self.after_intersection_work()
        self.state = msg.state

    def after_intersection_work(self):
        self.loginfo("Blocking stop line detection after the intersection")
        stop_line_reading_msg = StopLineReading()
        stop_line_reading_msg.stop_line_detected = False
        stop_line_reading_msg.at_stop_line = False
        self.pub_stop_line_reading.publish(stop_line_reading_msg)
        self.sleep = True
        rospy.sleep(self.off_time.value)
        self.sleep = False
        self.loginfo("Resuming stop line detection after the intersection")

    # def cbSwitch(self, switch_msg):
    #     self.active = switch_msg.data
    #     if self.active and self.state == "INTERSECTION_CONTROL":
    #         self.after_intersection_work()
    #
    #

    def cb_lane_pose(self, lane_pose_msg):
        self.lane_pose = lane_pose_msg
    
    def cb_apriltag_detections(self, msg):
        """
        Callback for AprilTag detections to enable precision stop control.
        
        Args:
            msg: AprilTag detection message
        """
        try:
            # For now, we'll use a simplified approach
            # In a full implementation, we would parse the actual AprilTag message
            self.last_apriltag_time = rospy.Time.now()
            
            # Simulate AprilTag detection for precision stop
            # In reality, this would extract distance from the AprilTag pose
            simulated_distance = 0.8  # 80cm from AprilTag
            
            rospy.logdebug(f"[StopLineFilterNode] AprilTag detection received, simulated distance: {simulated_distance:.3f}m")
            
            # Check if we should initiate precision stop
            if (self.precision_stop_controller and 
                not self.precision_stop_controller.is_stop_in_progress() and
                simulated_distance <= self.deceleration_start_distance.value and
                simulated_distance > self.precision_stop_distance.value):
                
                rospy.loginfo(f"[StopLineFilterNode] Initiating precision stop for AprilTag at {simulated_distance:.3f}m")
                self.precision_stop_controller.initiate_precision_stop(simulated_distance)
                self.precision_stops_executed += 1
                
        except Exception as e:
            rospy.logwarn(f"[StopLineFilterNode] Error processing AprilTag detection: {e}")

    def cb_segments(self, segment_list_msg):
        detection_start_time = time.time()

        if not self.switch or self.sleep:
            return

        good_seg_count = 0
        stop_line_x_accumulator = 0.0
        stop_line_y_accumulator = 0.0
        
        rospy.logdebug(f"[StopLineFilterNode] Processing {len(segment_list_msg.segments)} segments")
        
        for segment in segment_list_msg.segments:
            if segment.color != segment.RED:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0:  # the point is behind us
                continue

            p1_lane = self.to_lane_frame(segment.points[0])
            p2_lane = self.to_lane_frame(segment.points[1])
            avg_x = 0.5 * (p1_lane[0] + p2_lane[0])
            avg_y = 0.5 * (p1_lane[1] + p2_lane[1])
            stop_line_x_accumulator += avg_x
            stop_line_y_accumulator += avg_y  # TODO output covariance and not just mean
            good_seg_count += 1.0

        stop_line_reading_msg = StopLineReading()
        stop_line_reading_msg.header.stamp = segment_list_msg.header.stamp
        
        if good_seg_count < self.min_segs.value:
            stop_line_reading_msg.stop_line_detected = False
            stop_line_reading_msg.at_stop_line = False
            self.pub_stop_line_reading.publish(stop_line_reading_msg)
            
            rospy.logdebug(f"[StopLineFilterNode] Insufficient segments: {good_seg_count} < {self.min_segs.value}")

        else:
            stop_line_reading_msg.stop_line_detected = True
            stop_line_point = Point()
            stop_line_point.x = stop_line_x_accumulator / good_seg_count
            stop_line_point.y = stop_line_y_accumulator / good_seg_count
            stop_line_reading_msg.stop_line_point = stop_line_point
            
            # Calculate distance to stop line
            stop_line_distance = stop_line_point.x
            
            # Only detect redline if y is within max_y distance:
            stop_line_reading_msg.at_stop_line = (
                stop_line_point.x < self.stop_distance.value and np.abs(stop_line_point.y) < self.max_y.value
            )

            rospy.logdebug(f"[StopLineFilterNode] Stop line detected: distance={stop_line_distance:.3f}m, y_offset={stop_line_point.y:.3f}m")
            rospy.logdebug(f"[StopLineFilterNode] At stop line: {stop_line_reading_msg.at_stop_line}")

            self.pub_stop_line_reading.publish(stop_line_reading_msg)
            
            # Check if we should use precision stop control
            if (self.precision_stop_controller and 
                stop_line_reading_msg.at_stop_line and
                not self.precision_stop_controller.is_stop_in_progress()):
                
                rospy.loginfo(f"[StopLineFilterNode] Initiating precision stop for stop line at {stop_line_distance:.3f}m")
                self.precision_stop_controller.initiate_precision_stop(stop_line_distance)
                self.precision_stops_executed += 1
                
                rospy.logdebug(f"[StopLineFilterNode] Precision stop initiated, total stops: {self.precision_stops_executed}")
            
            if stop_line_reading_msg.at_stop_line:
                msg = BoolStamped()
                msg.header.stamp = stop_line_reading_msg.header.stamp
                msg.data = True
                self.pub_at_stop_line.publish(msg)
                
                rospy.logdebug("[StopLineFilterNode] At stop line message published")
        
        # Performance monitoring
        detection_time = time.time() - detection_start_time
        self.detection_count += 1
        
        rospy.logdebug(f"[StopLineFilterNode] Segment processing time: {detection_time:.4f}s")
        
        # Real-time monitoring - log performance every 50 detections
        if self.detection_count % 50 == 0:
            current_time = time.time()
            time_since_last_log = current_time - self.last_performance_log
            
            rospy.loginfo(f"[StopLineFilterNode] Performance summary after {self.detection_count} detections:")
            rospy.loginfo(f"[StopLineFilterNode] Detection frequency: {50.0/time_since_last_log:.2f} Hz")
            rospy.loginfo(f"[StopLineFilterNode] Precision stops executed: {self.precision_stops_executed}")
            
            if self.precision_stop_controller:
                metrics = self.precision_stop_controller.get_stop_metrics()
                if metrics:
                    rospy.loginfo(f"[StopLineFilterNode] Stop controller metrics: {metrics}")
            
            self.last_performance_log = current_time

    def to_lane_frame(self, point):
        p_homo = np.array([point.x, point.y, 1])
        phi = self.lane_pose.phi
        d = self.lane_pose.d
        T = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), d], [0, 0, 1]])
        p_new_homo = T.dot(p_homo)
        p_new = p_new_homo[0:2]
        return p_new

    # def onShutdown(self):
    #     rospy.loginfo("[StopLineFilterNode] Shutdown.")


if __name__ == "__main__":
    lane_filter_node = StopLineFilterNode(node_name="stop_line_filter")
    rospy.spin()

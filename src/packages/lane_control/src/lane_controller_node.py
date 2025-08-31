#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    WheelsCmdStamped,
    BoolStamped,
    FSMState,
    StopLineReading,
)

from lane_controller.controller import LaneController


class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocities, by processing the estimate error in
    lateral deviationa and heading.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:
        ~v_bar (:obj:`float`): Nominal velocity in m/s
        ~k_d (:obj:`float`): Proportional term for lateral deviation
        ~k_theta (:obj:`float`): Proportional term for heading deviation
        ~k_Id (:obj:`float`): integral term for lateral deviation
        ~k_Iphi (:obj:`float`): integral term for lateral deviation
        ~d_thres (:obj:`float`): Maximum value for lateral error
        ~theta_thres (:obj:`float`): Maximum value for heading error
        ~d_offset (:obj:`float`): Goal offset from center of the lane
        ~integral_bounds (:obj:`dict`): Bounds for integral term
        ~d_resolution (:obj:`float`): Resolution of lateral position estimate
        ~phi_resolution (:obj:`float`): Resolution of heading estimate
        ~omega_ff (:obj:`float`): Feedforward part of controller
        ~verbose (:obj:`bool`): Verbosity level (0,1,2)
        ~stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
        ~intersection_navigation_pose (:obj:`LanePose`): The lane pose estimate from intersection navigation
        ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Confirmation that the control action was executed
        ~stop_line_reading (:obj:`StopLineReading`): Distance from stopline, to reduce speed
        ~obstacle_distance_reading (:obj:`stop_line_reading`): Distancefrom obstacle virtual stopline, to reduce speed
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Add the node parameters to the parameters dictionary
        # TODO: MAKE TO WORK WITH NEW DTROS PARAMETERS
        self.params = dict()
        self.params["~v_bar"] = DTParam("~v_bar", param_type=ParamType.FLOAT, min_value=0.0, max_value=5.0)
        self.params["~k_d"] = DTParam("~k_d", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_theta"] = DTParam(
            "~k_theta", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        self.params["~k_Id"] = DTParam("~k_Id", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_Iphi"] = DTParam(
            "~k_Iphi", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        #self.params["~theta_thres"] = rospy.get_param("~theta_thres", None)
        #Breaking up the self.params["~theta_thres"] parameter for more finer tuning of phi
        self.params["~theta_thres_min"] = DTParam("~theta_thres_min", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)  #SUGGESTION mandatorizing the use of DTParam inplace of rospy.get_param for parameters in the entire dt-core repository as it allows active tuning while Robot is in action.
        self.params["~theta_thres_max"] = DTParam("~theta_thres_max", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0) 
        self.params["~d_thres"] = rospy.get_param("~d_thres", None)
        self.params["~d_offset"] = rospy.get_param("~d_offset", None)
        self.params["~integral_bounds"] = rospy.get_param("~integral_bounds", None)
        self.params["~d_resolution"] = rospy.get_param("~d_resolution", None)
        self.params["~phi_resolution"] = rospy.get_param("~phi_resolution", None)
        self.params["~omega_ff"] = rospy.get_param("~omega_ff", None)
        self.params["~verbose"] = rospy.get_param("~verbose", None)
        self.params["~stop_line_slowdown"] = rospy.get_param("~stop_line_slowdown", None)
        
        # MPC parameters
        self.params["~use_mpc"] = rospy.get_param("~use_mpc", False)
        self.params["~mpc_prediction_horizon"] = rospy.get_param("~mpc_prediction_horizon", 10)
        self.params["~mpc_control_horizon"] = rospy.get_param("~mpc_control_horizon", 5)
        self.params["~mpc_dt"] = rospy.get_param("~mpc_dt", 0.1)
        self.params["~mpc_max_velocity"] = rospy.get_param("~mpc_max_velocity", 0.5)
        self.params["~mpc_max_angular_velocity"] = rospy.get_param("~mpc_max_angular_velocity", 2.0)
        self.params["~mpc_velocity_weight"] = rospy.get_param("~mpc_velocity_weight", 1.0)
        self.params["~mpc_angular_weight"] = rospy.get_param("~mpc_angular_weight", 1.0)
        self.params["~mpc_lateral_weight"] = rospy.get_param("~mpc_lateral_weight", 10.0)
        self.params["~mpc_heading_weight"] = rospy.get_param("~mpc_heading_weight", 5.0)
        self.params["~mpc_smoothness_weight"] = rospy.get_param("~mpc_smoothness_weight", 1.0)
        self.params["~mpc_adaptive_horizon"] = rospy.get_param("~mpc_adaptive_horizon", True)
        
        # Enhanced vehicle model parameters
        self.params["~use_enhanced_vehicle_model"] = rospy.get_param("~use_enhanced_vehicle_model", True)
        self.params["~vehicle_wheelbase"] = rospy.get_param("~vehicle_wheelbase", 0.1)
        self.params["~vehicle_wheel_radius"] = rospy.get_param("~vehicle_wheel_radius", 0.0318)
        self.params["~vehicle_width"] = rospy.get_param("~vehicle_width", 0.13)
        self.params["~vehicle_length"] = rospy.get_param("~vehicle_length", 0.18)
        self.params["~vehicle_mass"] = rospy.get_param("~vehicle_mass", 1.2)
        self.params["~vehicle_max_linear_velocity"] = rospy.get_param("~vehicle_max_linear_velocity", 0.5)
        self.params["~vehicle_max_angular_velocity"] = rospy.get_param("~vehicle_max_angular_velocity", 2.0)
        self.params["~vehicle_max_linear_acceleration"] = rospy.get_param("~vehicle_max_linear_acceleration", 1.0)
        self.params["~vehicle_max_angular_acceleration"] = rospy.get_param("~vehicle_max_angular_acceleration", 3.0)
        self.params["~vehicle_moment_of_inertia"] = rospy.get_param("~vehicle_moment_of_inertia", 0.01)
        self.params["~vehicle_friction_coefficient"] = rospy.get_param("~vehicle_friction_coefficient", 0.8)
        self.params["~vehicle_drag_coefficient"] = rospy.get_param("~vehicle_drag_coefficient", 0.1)
        self.params["~vehicle_motor_time_constant"] = rospy.get_param("~vehicle_motor_time_constant", 0.1)
        self.params["~vehicle_motor_deadband"] = rospy.get_param("~vehicle_motor_deadband", 0.05)
        self.params["~vehicle_motor_saturation"] = rospy.get_param("~vehicle_motor_saturation", 1.0)
        
        # Adaptive gain scheduling parameters
        self.params["~use_adaptive_gains"] = rospy.get_param("~use_adaptive_gains", True)
        self.params["~adaptive_low_speed_threshold"] = rospy.get_param("~adaptive_low_speed_threshold", 0.1)
        self.params["~adaptive_high_speed_threshold"] = rospy.get_param("~adaptive_high_speed_threshold", 0.3)
        self.params["~adaptive_low_speed_k_d_scale"] = rospy.get_param("~adaptive_low_speed_k_d_scale", 1.5)
        self.params["~adaptive_low_speed_k_theta_scale"] = rospy.get_param("~adaptive_low_speed_k_theta_scale", 1.2)
        self.params["~adaptive_high_speed_k_d_scale"] = rospy.get_param("~adaptive_high_speed_k_d_scale", 0.8)
        self.params["~adaptive_high_speed_k_theta_scale"] = rospy.get_param("~adaptive_high_speed_k_theta_scale", 0.9)
        self.params["~adaptive_small_error_threshold"] = rospy.get_param("~adaptive_small_error_threshold", 0.05)
        self.params["~adaptive_large_error_threshold"] = rospy.get_param("~adaptive_large_error_threshold", 0.15)
        self.params["~adaptive_small_error_scale"] = rospy.get_param("~adaptive_small_error_scale", 0.8)
        self.params["~adaptive_large_error_scale"] = rospy.get_param("~adaptive_large_error_scale", 1.3)
        self.params["~adaptive_integral_windup_threshold"] = rospy.get_param("~adaptive_integral_windup_threshold", 0.2)
        self.params["~adaptive_integral_scale_factor"] = rospy.get_param("~adaptive_integral_scale_factor", 0.5)
        self.params["~adaptive_adaptation_rate"] = rospy.get_param("~adaptive_adaptation_rate", 0.1)
        self.params["~adaptive_min_gain_scale"] = rospy.get_param("~adaptive_min_gain_scale", 0.3)
        self.params["~adaptive_max_gain_scale"] = rospy.get_param("~adaptive_max_gain_scale", 2.0)

        # Need to create controller object before updating parameters, otherwise it will fail
        self.controller = LaneController(self.params)
        # self.updateParameters() # TODO: This needs be replaced by the new DTROS callback when it is implemented

        # Initialize variables
        self.fsm_state = None
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.last_s = None
        self.stop_line_distance = None
        self.stop_line_detected = False
        self.at_stop_line = False
        self.obstacle_stop_line_distance = None
        self.obstacle_stop_line_detected = False
        self.at_obstacle_stop_line = False

        self.current_pose_source = "lane_filter"

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber(
            "~lane_pose", LanePose, self.cbAllPoses, "lane_filter", queue_size=1
        )
        self.sub_intersection_navigation_pose = rospy.Subscriber(
            "~intersection_navigation_pose",
            LanePose,
            self.cbAllPoses,
            "intersection_navigation",
            queue_size=1,
        )
        self.sub_wheels_cmd_executed = rospy.Subscriber(
            "~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmdExecuted, queue_size=1
        )
        self.sub_stop_line = rospy.Subscriber(
            "~stop_line_reading", StopLineReading, self.cbStopLineReading, queue_size=1
        )
        self.sub_obstacle_stop_line = rospy.Subscriber(
            "~obstacle_distance_reading", StopLineReading, self.cbObstacleStopLineReading, queue_size=1
        )

        # Performance monitoring
        self.control_performance_log_time = 0.0
        self.trajectory_predictions = []
        self.vehicle_diagnostics_log_time = 0.0
        self.gain_scheduling_log_time = 0.0
        
        self.log("Initialized! MPC enabled: %s" % self.params.get("~use_mpc", False))

    # Lightweight logger to avoid dependency on DTROS.log availability across versions
    def log(self, msg, level="info"):
        try:
            lvl = (level or "").lower()
            if lvl == "debug":
                rospy.logdebug(msg)
            elif lvl in ("warn", "warning"):
                rospy.logwarn(msg)
            elif lvl == "error":
                rospy.logerr(msg)
            else:
                rospy.loginfo(msg)
        except Exception:
            # Fallback to info if anything goes wrong
            rospy.loginfo(msg)

    def cbObstacleStopLineReading(self, msg):
        """
        Callback storing the current obstacle distance, if detected.

        Args:
            msg(:obj:`StopLineReading`): Message containing information about the virtual obstacle stopline.
        """
        self.obstacle_stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2)
        self.obstacle_stop_line_detected = msg.stop_line_detected
        self.at_stop_line = msg.at_stop_line

    def cbStopLineReading(self, msg):
        """Callback storing current distance to the next stopline, if one is detected.

        Args:
            msg (:obj:`StopLineReading`): Message containing information about the next stop line.
        """
        self.stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2)
        self.stop_line_detected = msg.stop_line_detected
        self.at_obstacle_stop_line = msg.at_stop_line

    def cbMode(self, fsm_state_msg):

        self.fsm_state = fsm_state_msg.state  # String of current FSM state

        if self.fsm_state == "INTERSECTION_CONTROL":
            self.current_pose_source = "intersection_navigation"
        else:
            self.current_pose_source = "lane_filter"

        if self.params["~verbose"] == 2:
            self.log("Pose source: %s" % self.current_pose_source)

    def cbAllPoses(self, input_pose_msg, pose_source):
        """Callback receiving pose messages from multiple topics.

        If the source of the message corresponds with the current wanted pose source, it computes a control command.

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
            pose_source (:obj:`String`): Source of the message, specified in the subscriber.
        """

        if pose_source == self.current_pose_source:
            self.pose_msg_dict[pose_source] = input_pose_msg

            self.pose_msg = input_pose_msg

            self.getControlAction(self.pose_msg)
        
        # Log pose source changes
        if self.params["~verbose"] >= 1:
            self.log("Using pose from: %s, d=%.4f, phi=%.4f" % 
                    (pose_source, input_pose_msg.d, input_pose_msg.phi))

    def cbWheelsCmdExecuted(self, msg_wheels_cmd):
        """Callback that reports if the requested control action was executed.

        Args:
            msg_wheels_cmd (:obj:`WheelsCmdStamped`): Executed wheel commands
        """
        self.wheels_cmd_executed = msg_wheels_cmd

    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)

    def getControlAction(self, pose_msg):
        """Callback that receives a pose message and updates the related control command.

        Using a controller object, computes the control action using the current pose estimate.

        Args:
            pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
        """
        current_s = rospy.Time.now().to_sec()
        dt = None
        if self.last_s is not None:
            dt = current_s - self.last_s

        if self.at_stop_line or self.at_obstacle_stop_line:
            v = 0
            omega = 0
        else:

            # Compute errors
            d_err = pose_msg.d - self.params["~d_offset"]
            phi_err = pose_msg.phi

            # We cap the error if it grows too large
            if np.abs(d_err) > self.params["~d_thres"]:
                self.log("d_err too large, thresholding it!", "error")
                d_err = np.sign(d_err) * self.params["~d_thres"]
            
            if phi_err > self.params["~theta_thres_max"].value or phi_err < self.params["~theta_thres_min"].value:
                self.log("phi_err too large/small, thresholding it!", "error")
                phi_err = np.maximum(self.params["~theta_thres_min"].value, np.minimum(phi_err, self.params["~theta_thres_max"].value))

            wheels_cmd_exec = [self.wheels_cmd_executed.vel_left, self.wheels_cmd_executed.vel_right]
            if self.obstacle_stop_line_detected:
                v, omega = self.controller.compute_control_action(
                    d_err, phi_err, dt, wheels_cmd_exec, self.obstacle_stop_line_distance
                )
                # TODO: This is a temporarily fix to avoid vehicle image detection latency caused unable to stop in time.
                v = v * 0.25
                omega = omega * 0.25

            else:
                v, omega = self.controller.compute_control_action(
                    d_err, phi_err, dt, wheels_cmd_exec, self.stop_line_distance
                )

            # For feedforward action (i.e. during intersection navigation)
            omega += self.params["~omega_ff"]

        # Initialize car control msg, add header from input message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        # Add commands to car message
        car_control_msg.v = v
        car_control_msg.omega = omega

        self.publishCmd(car_control_msg)
        self.last_s = current_s
        
        # Comprehensive logging for predictive control
        if self.params["~verbose"] >= 2:
            self.log("Control computed: v=%.3f, omega=%.3f, d_err=%.4f, phi_err=%.4f, dt=%.3f" %
                    (v, omega, d_err, phi_err, dt if dt else 0.0))
        
        # Log MPC performance metrics periodically
        current_time = rospy.Time.now().to_sec()
        if current_time - self.control_performance_log_time > 10.0:  # Every 10 seconds
            if hasattr(self.controller, 'get_mpc_performance_metrics'):
                metrics = self.controller.get_mpc_performance_metrics()
                if metrics:
                    self.log("MPC Performance: avg_time=%.2fms, convergence=%.1f%%, controls=%d" %
                            (metrics.get('avg_optimization_time', 0) * 1000,
                             metrics.get('convergence_rate', 0) * 100,
                             metrics.get('total_controls', 0)))
            self.control_performance_log_time = current_time
        
        # Log vehicle model diagnostics periodically
        if current_time - self.vehicle_diagnostics_log_time > 15.0:  # Every 15 seconds
            if hasattr(self.controller, 'get_vehicle_diagnostics'):
                diagnostics = self.controller.get_vehicle_diagnostics()
                if diagnostics:
                    self.log("Vehicle Model: valid=%s, calibrated=%s, violations=%d, success_rate=%.1f%%" %
                            (diagnostics.get('parameters_valid', False),
                             diagnostics.get('calibration_loaded', False),
                             diagnostics.get('recent_constraint_violations', 0),
                             diagnostics.get('prediction_success_rate', 1.0) * 100))
                    
                    # Log detailed vehicle parameters
                    if self.params["~verbose"] >= 2:
                        self.log("Vehicle Params: wheelbase=%.3fm, wheel_radius=%.4fm, gains=[%.3f,%.3f], trims=[%.3f,%.3f]" %
                                (diagnostics.get('wheelbase', 0.1),
                                 diagnostics.get('wheel_radius', 0.0318),
                                 diagnostics.get('motor_gains', [1.0, 1.0])[0],
                                 diagnostics.get('motor_gains', [1.0, 1.0])[1],
                                 diagnostics.get('motor_trims', [0.0, 0.0])[0],
                                 diagnostics.get('motor_trims', [0.0, 0.0])[1]))
            self.vehicle_diagnostics_log_time = current_time
        
        # Log gain scheduling diagnostics periodically
        if current_time - self.gain_scheduling_log_time > 20.0:  # Every 20 seconds
            if hasattr(self.controller, 'get_gain_scheduling_diagnostics'):
                gain_diagnostics = self.controller.get_gain_scheduling_diagnostics()
                if gain_diagnostics:
                    self.log("Gain Scheduling: speed=%.3fm/s, d_err=%.4fm, phi_err=%.4frad, adaptation=%s" %
                            (gain_diagnostics.get('current_speed', 0.0),
                             gain_diagnostics.get('current_lateral_error', 0.0),
                             gain_diagnostics.get('current_heading_error', 0.0),
                             gain_diagnostics.get('adaptation_active', False)))
                    
                    # Log detailed gain information
                    if self.params["~verbose"] >= 2:
                        scheduled_gains = gain_diagnostics.get('scheduled_gains', {})
                        speed_scaling = gain_diagnostics.get('speed_scaling', {})
                        error_scaling = gain_diagnostics.get('error_scaling', {})
                        self.log("Scheduled Gains: k_d=%.3f, k_theta=%.3f, k_Id=%.3f, k_Iphi=%.3f" %
                                (scheduled_gains.get('k_d', 0.0),
                                 scheduled_gains.get('k_theta', 0.0),
                                 scheduled_gains.get('k_Id', 0.0),
                                 scheduled_gains.get('k_Iphi', 0.0)))
                        self.log("Gain Scaling: speed=[%.3f,%.3f], error=[%.3f,%.3f]" %
                                (speed_scaling.get('k_d', 1.0),
                                 speed_scaling.get('k_theta', 1.0),
                                 error_scaling.get('k_d', 1.0),
                                 error_scaling.get('k_theta', 1.0)))
            self.gain_scheduling_log_time = current_time

    def cbParametersChanged(self):
        """Updates parameters in the controller object."""

        self.controller.update_parameters(self.params)


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name="lane_controller_node")
    # Keep it spinning
    rospy.spin()

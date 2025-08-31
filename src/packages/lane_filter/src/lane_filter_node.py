#!/usr/bin/env python3
import json
import time
from collections import deque

import numpy as np
from scipy.optimize import curve_fit
from scipy.interpolate import UnivariateSpline

import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import FSMState, LanePose, SegmentList, Twist2DStamped
from duckietown_msgs.srv import SetBool, SetBoolResponse
from lane_filter import LaneFilterHistogram
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PolynomialCurveFitter:
    """
    Polynomial curve fitting for lane trajectory prediction.
    
    This class fits polynomial curves to detected lane segments and provides
    trajectory prediction capabilities for predictive lane following.
    """
    
    def __init__(self, polynomial_degree=1, min_points=8, extrapolation_distance=1.0, 
                 smoothing_factor=0.1, history_size=10):
        # Prefer a more stable fit by default: linear with more points
        self.polynomial_degree = polynomial_degree
        self.min_points = min_points
        self.extrapolation_distance = extrapolation_distance
        self.smoothing_factor = smoothing_factor
        self.history_size = history_size
        
        # Curve fitting history for temporal smoothing
        self.curve_history = {
            'left': deque(maxlen=history_size),
            'right': deque(maxlen=history_size),
            'center': deque(maxlen=history_size)
        }
        
        # Performance metrics
        self.fitting_errors = {}
        self.prediction_accuracy = {}
        self.last_coefficients = {}
        
    def extract_lane_points(self, segments, lane_type='center'):
        """
        Extract lane points from segments for curve fitting.
        
        Args:
            segments: List of segment objects
            lane_type: Type of lane ('left', 'right', 'center')
            
        Returns:
            tuple: (x_points, y_points) arrays for curve fitting
        """
        start_time = time.time()
        
        x_points = []
        y_points = []
        
        for segment in segments:
            # Extract segment endpoints
            x1, y1 = segment.pixels_normalized[0].x, segment.pixels_normalized[0].y
            x2, y2 = segment.pixels_normalized[1].x, segment.pixels_normalized[1].y
            
            # Add both endpoints
            x_points.extend([x1, x2])
            y_points.extend([y1, y2])
        
        # Convert to numpy arrays and sort by x coordinate
        if len(x_points) >= self.min_points:
            points = np.array(list(zip(x_points, y_points)))
            # Sort by x coordinate for proper curve fitting
            points = points[np.argsort(points[:, 0])]
            x_points, y_points = points[:, 0], points[:, 1]
        else:
            x_points, y_points = np.array([]), np.array([])
        
        processing_time = time.time() - start_time
        
        # Log point extraction details
        rospy.logdebug(f"[PolynomialCurveFitter] Extracted {len(x_points)} points for {lane_type} lane, "
                      f"Processing time: {processing_time*1000:.2f}ms")
        
        return x_points, y_points
    
    def fit_polynomial_curve(self, x_points, y_points, lane_type='center'):
        """
        Fit a polynomial curve to the lane points.
        
        Args:
            x_points: X coordinates of lane points
            y_points: Y coordinates of lane points  
            lane_type: Type of lane for logging
            
        Returns:
            dict: Curve fitting results including coefficients and metrics
        """
        start_time = time.time()

        if len(x_points) < self.min_points:
            rospy.logdebug(f"[PolynomialCurveFitter] Insufficient points ({len(x_points)}) for {lane_type} curve fitting")
            return {
                'success': False,
                'coefficients': None,
                'fitting_error': float('inf'),
                'r_squared': 0.0,
                'curvature': 0.0,
                'processing_time': time.time() - start_time
            }

        try:
            # Fit polynomial using least squares
            coefficients = np.polyfit(x_points, y_points, self.polynomial_degree)

            # Calculate fitting error (RMSE)
            y_pred = np.polyval(coefficients, x_points)
            fitting_error = np.sqrt(np.mean((y_points - y_pred) ** 2))

            # Calculate R-squared
            ss_res = np.sum((y_points - y_pred) ** 2)
            ss_tot = np.sum((y_points - np.mean(y_points)) ** 2)
            r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

            # Calculate curvature at the center point
            if self.polynomial_degree >= 2:
                # For polynomial y = ax^2 + bx + c, curvature = |2a| / (1 + (2ax + b)^2)^(3/2)
                x_center = np.mean(x_points)
                a, b = coefficients[0], coefficients[1]
                dy_dx = 2 * a * x_center + b
                d2y_dx2 = 2 * a
                curvature = abs(d2y_dx2) / (1 + dy_dx**2)**(3/2)
            else:
                curvature = 0.0

            # Store in history for temporal smoothing
            curve_data = {
                'coefficients': coefficients,
                'fitting_error': fitting_error,
                'r_squared': r_squared,
                'curvature': curvature,
                'timestamp': time.time()
            }
            self.curve_history[lane_type].append(curve_data)

            # Update metrics
            self.fitting_errors[lane_type] = fitting_error
            self.last_coefficients[lane_type] = coefficients

            processing_time = time.time() - start_time

            # Log curve fitting details
            rospy.logdebug(
                f"[PolynomialCurveFitter] {lane_type} curve fitting - "
                f"Coefficients: {coefficients}, "
                f"RMSE: {fitting_error:.4f}, "
                f"R²: {r_squared:.4f}, "
                f"Curvature: {curvature:.4f}, "
                f"Processing time: {processing_time*1000:.2f}ms"
            )

            return {
                'success': True,
                'coefficients': coefficients,
                'fitting_error': fitting_error,
                'r_squared': r_squared,
                'curvature': curvature,
                'processing_time': processing_time
            }
        except np.linalg.LinAlgError as e:
            # Be quieter to avoid log spam; try to fall back to last good coefficients
            last = self.last_coefficients.get(lane_type)
            if last is not None:
                y_pred = np.polyval(last, x_points) if len(x_points) else np.array([])
                fitting_error = float(np.sqrt(np.mean((y_points - y_pred) ** 2))) if len(x_points) else float('inf')
                return {
                    'success': True,
                    'coefficients': last,
                    'fitting_error': fitting_error,
                    'r_squared': 0.0,
                    'curvature': 0.0,
                    'processing_time': time.time() - start_time
                }
            rospy.logdebug(f"[PolynomialCurveFitter] Curve fitting failed for {lane_type}: {e}")
            return {
                'success': False,
                'coefficients': None,
                'fitting_error': float('inf'),
                'r_squared': 0.0,
                'curvature': 0.0,
                'processing_time': time.time() - start_time
            }
    
    def get_smoothed_coefficients(self, lane_type='center'):
        """
        Get temporally smoothed curve coefficients.
        
        Args:
            lane_type: Type of lane
            
        Returns:
            numpy.ndarray: Smoothed polynomial coefficients
        """
        if lane_type not in self.curve_history or len(self.curve_history[lane_type]) == 0:
            return None
        
        history = list(self.curve_history[lane_type])
        
        # Weight recent measurements more heavily
        weights = np.exp(-np.arange(len(history)) * 0.1)[::-1]  # More weight to recent
        weights /= np.sum(weights)
        
        # Calculate weighted average of coefficients
        coefficients_array = np.array([h['coefficients'] for h in history])
        smoothed_coefficients = np.average(coefficients_array, axis=0, weights=weights)
        
        rospy.logdebug(f"[PolynomialCurveFitter] Smoothed {lane_type} coefficients: {smoothed_coefficients}")
        
        return smoothed_coefficients
    
    def predict_trajectory(self, coefficients, x_start, x_end, num_points=50):
        """
        Predict trajectory points using fitted curve.
        
        Args:
            coefficients: Polynomial coefficients
            x_start: Starting x coordinate
            x_end: Ending x coordinate
            num_points: Number of prediction points
            
        Returns:
            tuple: (x_pred, y_pred) trajectory points
        """
        start_time = time.time()
        
        if coefficients is None:
            return np.array([]), np.array([])
        
        # Generate prediction points
        x_pred = np.linspace(x_start, x_end, num_points)
        y_pred = np.polyval(coefficients, x_pred)
        
        processing_time = time.time() - start_time
        
        # Log trajectory prediction
        rospy.logdebug(f"[PolynomialCurveFitter] Trajectory prediction - "
                      f"Range: [{x_start:.3f}, {x_end:.3f}], "
                      f"Points: {num_points}, "
                      f"Processing time: {processing_time*1000:.2f}ms")
        
        return x_pred, y_pred
    
    def extrapolate_curve(self, coefficients, current_x, extrapolation_distance=None):
        """
        Extrapolate curve for predictive lane following.
        
        Args:
            coefficients: Polynomial coefficients
            current_x: Current x position
            extrapolation_distance: Distance to extrapolate ahead
            
        Returns:
            dict: Extrapolation results including predicted points and curvature
        """
        start_time = time.time()
        
        if coefficients is None:
            return {
                'success': False,
                'predicted_points': (np.array([]), np.array([])),
                'predicted_curvature': 0.0,
                'predicted_heading': 0.0
            }
        
        if extrapolation_distance is None:
            extrapolation_distance = self.extrapolation_distance
        
        # Extrapolate ahead
        x_future = current_x + extrapolation_distance
        y_future = np.polyval(coefficients, x_future)
        
        # Calculate predicted heading (derivative at future point)
        if self.polynomial_degree >= 1:
            derivative_coeffs = np.polyder(coefficients)
            predicted_heading = np.arctan(np.polyval(derivative_coeffs, x_future))
        else:
            predicted_heading = 0.0
        
        # Calculate predicted curvature
        if self.polynomial_degree >= 2:
            a = coefficients[0]
            b = coefficients[1]
            dy_dx = 2 * a * x_future + b
            d2y_dx2 = 2 * a
            predicted_curvature = abs(d2y_dx2) / (1 + dy_dx**2)**(3/2)
        else:
            predicted_curvature = 0.0
        
        # Generate trajectory points
        x_traj = np.linspace(current_x, x_future, 20)
        y_traj = np.polyval(coefficients, x_traj)
        
        processing_time = time.time() - start_time
        
        # Log extrapolation results
        rospy.logdebug(f"[PolynomialCurveFitter] Curve extrapolation - "
                      f"Current: ({current_x:.3f}), "
                      f"Future: ({x_future:.3f}, {y_future:.3f}), "
                      f"Heading: {predicted_heading:.3f} rad, "
                      f"Curvature: {predicted_curvature:.4f}, "
                      f"Processing time: {processing_time*1000:.2f}ms")
        
        return {
            'success': True,
            'predicted_points': (x_traj, y_traj),
            'predicted_position': (x_future, y_future),
            'predicted_curvature': predicted_curvature,
            'predicted_heading': predicted_heading,
            'processing_time': processing_time
        }
    
    def get_curve_metrics(self):
        """
        Get current curve fitting metrics.
        
        Returns:
            dict: Comprehensive metrics for all lane types
        """
        metrics = {
            'fitting_errors': dict(self.fitting_errors),
            'prediction_accuracy': dict(self.prediction_accuracy),
            'history_sizes': {lane: len(history) for lane, history in self.curve_history.items()},
            'last_coefficients': dict(self.last_coefficients)
        }
        
        # Calculate average metrics across history
        for lane_type, history in self.curve_history.items():
            if history:
                recent_history = list(history)[-5:]  # Last 5 measurements
                avg_error = np.mean([h['fitting_error'] for h in recent_history])
                avg_r_squared = np.mean([h['r_squared'] for h in recent_history])
                avg_curvature = np.mean([h['curvature'] for h in recent_history])
                
                metrics[f'{lane_type}_avg_error'] = avg_error
                metrics[f'{lane_type}_avg_r_squared'] = avg_r_squared
                metrics[f'{lane_type}_avg_curvature'] = avg_curvature
        
        return metrics


class LaneFilterNode(DTROS):
    """Generates an estimate of the lane pose.

    Creates a `lane_filter` to get estimates on `d` and `phi`, the lateral and heading deviation from the
    center of the lane.
    It gets the segments extracted by the line_detector as input and output the lane pose estimate.


    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~filter (:obj:`list`): A list of parameters for the lane pose estimation filter
        ~debug (:obj:`bool`): A parameter to enable/disable the publishing of debug topics and images

    Subscribers:
        ~segment_list (:obj:`SegmentList`): The detected line segments from the line detector
        ~car_cmd (:obj:`Twist2DStamped`): The car commands executed. Used for the predict step of the filter
        ~change_params (:obj:`String`): A topic to temporarily changes filter parameters for a finite time
        only
        ~switch (:obj:``BoolStamped): A topic to turn on and off the node. WARNING : to be replaced with a
        service call to the provided mother node switch service
        ~fsm_mode (:obj:`FSMState`): A topic to change the state of the node. WARNING : currently not
        implemented

    Publishers:
        ~lane_pose (:obj:`LanePose`): The computed lane pose estimate
        ~belief_img (:obj:`Image`): A debug image that shows the filter's internal state
        ~seglist_filtered (:obj:``SegmentList): a debug topic to send the filtered list of segments that
        are considered as valid

    """

    filter: LaneFilterHistogram
    bridge: CvBridge

    def __init__(self, node_name):
        super(LaneFilterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self._filter = rospy.get_param("~lane_filter_histogram_configuration", None)
        self._debug = rospy.get_param("~debug", False)
        
        # Enhanced curve fitting parameters
        self._curve_fitting_enabled = rospy.get_param("~curve_fitting_enabled", True)
        self._polynomial_degree = rospy.get_param("~polynomial_degree", 2)
        self._min_curve_points = rospy.get_param("~min_curve_points", 5)
        self._extrapolation_distance = rospy.get_param("~extrapolation_distance", 1.0)
        self._curve_smoothing_factor = rospy.get_param("~curve_smoothing_factor", 0.1)
        self._curve_history_size = rospy.get_param("~curve_history_size", 10)

        # Create the filter
        self.filter = LaneFilterHistogram(**self._filter)

        # Initialize polynomial curve fitter
        if self._curve_fitting_enabled:
            self.curve_fitter = PolynomialCurveFitter(
                polynomial_degree=self._polynomial_degree,
                min_points=self._min_curve_points,
                extrapolation_distance=self._extrapolation_distance,
                smoothing_factor=self._curve_smoothing_factor,
                history_size=self._curve_history_size
            )
            rospy.loginfo("[LaneFilterNode] Polynomial curve fitting enabled")
        else:
            self.curve_fitter = None

        # Creating cvBridge
        self.bridge = CvBridge()

        self.t_last_update = rospy.get_time()
        self.currentVelocity = None

        self.latencyArray = []
        
        # Performance monitoring for curve fitting
        self.curve_fitting_times = []
        self.frame_count = 0
        self.last_performance_log = time.time()

        # Subscribers
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.cbProcessSegments, queue_size=1)

        self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.updateVelocity)

        self.sub_change_params = rospy.Subscriber("~change_params", String, self.cbTemporaryChangeParams)

        # Publishers
        self.pub_lane_pose = rospy.Publisher(
            "~lane_pose", LanePose, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

        self.pub_belief_img = rospy.Publisher(
            "~belief_img", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.pub_seglist_filtered = rospy.Publisher(
            "~seglist_filtered", SegmentList, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

    # FSM
    # self.sub_switch = rospy.Subscriber(
    #     "~switch", BoolStamped, self.cbSwitch, queue_size=1)
    self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)

    # Service to switch node on/off (used by FSM)
    self._active = True
    self._srv_switch = rospy.Service("~switch", SetBool, self._cb_switch)

    rospy.loginfo("[LaneFilterNode] Initialized.")

    def cbTemporaryChangeParams(self, msg):
        """Callback that changes temporarily the filter's parameters.

        Args:
            msg (:obj:`String`): list of the new parameters

        """
        # This weird callback changes parameters only temporarily - used in the unicorn intersection.
        # comment from 03/2020
        data = json.loads(msg.data)
        params = data["params"]
        reset_time = data["time"]
        # Set all paramters which need to be updated
        for param_name in list(params.keys()):
            param_val = params[param_name]
            params[param_name] = eval("self.filter." + str(param_name))  # FIXME: really?
            exec("self.filter." + str(param_name) + "=" + str(param_val))  # FIXME: really?

        # Sleep for reset time
        rospy.sleep(reset_time)

        # Reset parameters to old values
        for param_name in list(params.keys()):
            param_val = params[param_name]

            exec("self.filter." + str(param_name) + "=" + str(param_val))  # FIXME: really?

    #    def nbSwitch(self, switch_msg):
    #        """Callback to turn on/off the node
    #
    #        Args:
    #            switch_msg (:obj:`BoolStamped`): message containing the on or off command
    #
    #        """
    #        # All calls to this message should be replaced directly by the srvSwitch
    #        request = SetBool()
    #        request.data = switch_msg.data
    #        eelf.nub_switch(request)

    def cbProcessSegments(self, segment_list_msg):
        """Callback to process the segments with enhanced curve fitting

        Args:
            segment_list_msg (:obj:`SegmentList`): message containing list of processed segments

        """
        # If disabled via FSM switch, do not process
        if not getattr(self, "_active", True):
            return

        # Get actual timestamp for latency measurement
        timestamp_before_processing = rospy.Time.now()
        frame_start_time = time.time()
        self.frame_count += 1

        # Log frame processing start
        rospy.logdebug(f"[LaneFilterNode] Frame {self.frame_count} - "
                      f"Processing {len(segment_list_msg.segments)} segments at {rospy.Time.now()}")

        # Step 1: predict
        current_time = rospy.get_time()
        if self.currentVelocity:
            dt = current_time - self.t_last_update
            self.filter.predict(dt=dt, v=self.currentVelocity.v, w=self.currentVelocity.omega)

        self.t_last_update = current_time

        # Step 2: update
        self.filter.update(segment_list_msg.segments)

        # Step 3: Enhanced curve fitting and trajectory prediction
        curve_fitting_results = {}
        if self.curve_fitter:
            curve_fitting_start = time.time()
            
            # Separate segments by color/type for individual curve fitting
            white_segments = [s for s in segment_list_msg.segments if hasattr(s, 'color') and s.color == s.WHITE]
            yellow_segments = [s for s in segment_list_msg.segments if hasattr(s, 'color') and s.color == s.YELLOW]
            all_segments = segment_list_msg.segments
            
            # Fit curves for different lane types
            lane_types = [
                ('center', all_segments),
                ('left', white_segments),
                ('right', yellow_segments)
            ]
            
            for lane_type, segments in lane_types:
                if segments:
                    # Extract points and fit curve
                    x_points, y_points = self.curve_fitter.extract_lane_points(segments, lane_type)
                    curve_result = self.curve_fitter.fit_polynomial_curve(x_points, y_points, lane_type)
                    curve_fitting_results[lane_type] = curve_result
                    
                    # Log curve fitting results
                    if curve_result['success']:
                        rospy.logdebug(f"[LaneFilterNode] {lane_type} curve fitting successful - "
                                      f"RMSE: {curve_result['fitting_error']:.4f}, "
                                      f"R²: {curve_result['r_squared']:.4f}")
                    else:
                        rospy.logdebug(f"[LaneFilterNode] {lane_type} curve fitting failed")
            
            curve_fitting_time = time.time() - curve_fitting_start
            self.curve_fitting_times.append(curve_fitting_time)
            
            # Keep only recent timing data
            if len(self.curve_fitting_times) > 50:
                self.curve_fitting_times.pop(0)

        # Step 4: build messages and publish things
        [d_max, phi_max] = self.filter.getEstimate()

        # Enhanced lane pose with curve information
        if self.curve_fitter and 'center' in curve_fitting_results and curve_fitting_results['center']['success']:
            # Get smoothed coefficients for better stability
            smoothed_coeffs = self.curve_fitter.get_smoothed_coefficients('center')
            
            if smoothed_coeffs is not None:
                # Predict future trajectory
                current_x = 0.5  # Assume center of image as current position
                extrapolation_result = self.curve_fitter.extrapolate_curve(
                    smoothed_coeffs, current_x, self._extrapolation_distance
                )
                
                if extrapolation_result['success']:
                    # Enhance phi estimate with curve prediction
                    predicted_heading = extrapolation_result['predicted_heading']
                    predicted_curvature = extrapolation_result['predicted_curvature']
                    
                    # Blend traditional estimate with curve-based prediction
                    curve_weight = 0.3  # Weight for curve-based estimate
                    phi_max = (1 - curve_weight) * phi_max + curve_weight * predicted_heading
                    
                    rospy.logdebug(f"[LaneFilterNode] Enhanced pose estimation - "
                                  f"Original phi: {phi_max:.3f}, "
                                  f"Predicted heading: {predicted_heading:.3f}, "
                                  f"Predicted curvature: {predicted_curvature:.4f}")

        # Getting the highest belief value from the belief matrix
        max_val = self.filter.getMax()
        # Comparing it to a minimum belief threshold to make sure we are certain enough of our estimate
        in_lane = max_val > self.filter.min_max

        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = in_lane
        # XXX: is it always NORMAL?
        lanePose.status = lanePose.NORMAL

        self.pub_lane_pose.publish(lanePose)
        
        # Calculate total processing time
        total_processing_time = time.time() - frame_start_time
        
        # Log comprehensive performance metrics
        rospy.logdebug(f"[LaneFilterNode] Frame {self.frame_count} Performance - "
                      f"Total: {total_processing_time*1000:.2f}ms, "
                      f"Curve fitting: {curve_fitting_time*1000:.2f}ms, "
                      f"Lane pose: d={d_max:.3f}, phi={phi_max:.3f}")
        
        # Real-time monitoring
        current_time_monitor = time.time()
        if self.frame_count % 30 == 0:  # Every 30 frames
            time_since_last_log = current_time_monitor - self.last_performance_log
            avg_fps = 30.0 / time_since_last_log if time_since_last_log > 0 else 0
            avg_curve_fitting_time = np.mean(self.curve_fitting_times) if self.curve_fitting_times else 0
            
            performance_summary = f"[LaneFilterNode] Performance Summary (30 frames) - " \
                                f"Avg FPS: {avg_fps:.1f}, " \
                                f"Avg curve fitting: {avg_curve_fitting_time*1000:.2f}ms"
            
            if self.curve_fitter:
                curve_metrics = self.curve_fitter.get_curve_metrics()
                if 'center_avg_error' in curve_metrics:
                    performance_summary += f", Avg curve error: {curve_metrics['center_avg_error']:.4f}"
                if 'center_avg_r_squared' in curve_metrics:
                    performance_summary += f", Avg R²: {curve_metrics['center_avg_r_squared']:.3f}"
            
            rospy.loginfo(performance_summary)
            self.last_performance_log = current_time_monitor

        self.debugOutput(segment_list_msg, d_max, phi_max, timestamp_before_processing, curve_fitting_results)

    # --- Services ---
    def _cb_switch(self, req: SetBool):
        """Enable/disable this node.

        Args:
            req (duckietown_msgs/SetBool): req.data True to enable, False to disable

        Returns:
            duckietown_msgs/SetBoolResponse
        """
        self._active = bool(req.data)
        status = "enabled" if self._active else "disabled"
        rospy.loginfo(f"[LaneFilterNode] Switch: {status}")
        return SetBoolResponse(success=True, message=f"LaneFilterNode {status}")

    # --- Services ---
    def _cb_switch(self, req: SetBool):
        """Enable/disable this node.

        Args:
            req (duckietown_msgs/SetBool): req.data True to enable, False to disable

        Returns:
            duckietown_msgs/SetBoolResponse
        """
        self._active = bool(req.data)
        status = "enabled" if self._active else "disabled"
        rospy.loginfo(f"[LaneFilterNode] Switch: {status}")
        return SetBoolResponse(success=True, message=f"LaneFilterNode {status}")

    def debugOutput(self, segment_list_msg, d_max, phi_max, timestamp_before_processing, curve_fitting_results=None):
        """Creates and publishes debug messages with enhanced curve fitting information

        Args:
            segment_list_msg (:obj:`SegmentList`): message containing list of filtered segments
            d_max (:obj:`float`): best estimate for d
            phi_max (:obj:``float): best estimate for phi
            timestamp_before_processing (:obj:`float`): timestamp dating from before the processing
            curve_fitting_results (:obj:`dict`): Results from curve fitting process

        """
        if self._debug:
            # Latency of Estimation including curvature estimation
            estimation_latency_stamp = rospy.Time.now() - timestamp_before_processing
            estimation_latency = estimation_latency_stamp.secs + estimation_latency_stamp.nsecs / 1e9
            self.latencyArray.append(estimation_latency)

            if len(self.latencyArray) >= 20:
                self.latencyArray.pop(0)

            # Log comprehensive latency information
            rospy.logdebug(f"[LaneFilterNode] Estimation latency: {estimation_latency*1000:.2f}ms, "
                          f"Mean latency: {np.mean(self.latencyArray)*1000:.2f}ms")

            # Get the segments that agree with the best estimate and publish them
            inlier_segments = self.filter.get_inlier_segments(segment_list_msg.segments, d_max, phi_max)
            inlier_segments_msg = SegmentList()
            inlier_segments_msg.header = segment_list_msg.header
            inlier_segments_msg.segments = inlier_segments
            self.pub_seglist_filtered.publish(inlier_segments_msg)

            # Create belief image and publish it
            belief_img = self.bridge.cv2_to_imgmsg(
                np.array(255 * self.filter.belief).astype("uint8"), "mono8"
            )
            belief_img.header.stamp = segment_list_msg.header.stamp
            self.pub_belief_img.publish(belief_img)
            
            # Log curve fitting debug information
            if curve_fitting_results and self.curve_fitter:
                for lane_type, result in curve_fitting_results.items():
                    if result['success']:
                        rospy.logdebug(f"[LaneFilterNode] {lane_type} curve debug - "
                                      f"Coefficients: {result['coefficients']}, "
                                      f"Fitting error: {result['fitting_error']:.4f}, "
                                      f"R²: {result['r_squared']:.4f}, "
                                      f"Curvature: {result['curvature']:.4f}")
                        
                        # Log trajectory prediction if available
                        smoothed_coeffs = self.curve_fitter.get_smoothed_coefficients(lane_type)
                        if smoothed_coeffs is not None:
                            current_x = 0.5
                            extrapolation = self.curve_fitter.extrapolate_curve(smoothed_coeffs, current_x)
                            if extrapolation['success']:
                                rospy.logdebug(f"[LaneFilterNode] {lane_type} trajectory prediction - "
                                              f"Future position: {extrapolation['predicted_position']}, "
                                              f"Predicted heading: {extrapolation['predicted_heading']:.3f} rad, "
                                              f"Predicted curvature: {extrapolation['predicted_curvature']:.4f}")
                
                # Log overall curve fitting metrics
                curve_metrics = self.curve_fitter.get_curve_metrics()
                rospy.logdebug(f"[LaneFilterNode] Curve fitting metrics - "
                              f"Fitting errors: {curve_metrics['fitting_errors']}, "
                              f"History sizes: {curve_metrics['history_sizes']}")

    def cbMode(self, msg):
        return  # TODO adjust self.active

    def updateVelocity(self, twist_msg):
        self.currentVelocity = twist_msg


if __name__ == "__main__":
    lane_filter_node = LaneFilterNode(node_name="lane_filter_node")
    rospy.spin()

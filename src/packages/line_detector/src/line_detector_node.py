#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
import time
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Segment, SegmentList, AntiInstagramThresholds
from duckietown_msgs.srv import SetBool, SetBoolResponse
from line_detector import LineDetector, ColorRange, plotSegments, plotMaps
from image_processing.anti_instagram import AntiInstagram

from duckietown.dtros import DTROS, NodeType, TopicType


class AdaptiveThresholdDetector:
    """
    Adaptive threshold detector that automatically adjusts detection parameters based on lighting conditions.
    
    This class monitors image brightness and contrast to dynamically adjust Canny edge detection thresholds
    and other line detection parameters for optimal performance across varying lighting conditions.
    """
    
    def __init__(self, base_canny_low=80, base_canny_high=200, adaptation_rate=0.1):
        self.base_canny_low = base_canny_low
        self.base_canny_high = base_canny_high
        self.adaptation_rate = adaptation_rate
        
        # Current adaptive parameters
        self.current_canny_low = base_canny_low
        self.current_canny_high = base_canny_high
        
        # Lighting condition tracking
        self.brightness_history = deque(maxlen=10)
        self.contrast_history = deque(maxlen=10)
        
        # Performance metrics
        self.last_detection_count = 0
        self.detection_confidence = 0.0
        
    def analyze_lighting_conditions(self, image):
        """
        Analyze current lighting conditions from the image.
        
        Args:
            image: Input BGR image
            
        Returns:
            dict: Lighting conditions including brightness, contrast, and quality metrics
        """
        start_time = time.time()
        
        # Convert to grayscale for analysis
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Calculate brightness (mean intensity)
        brightness = np.mean(gray)
        
        # Calculate contrast (standard deviation)
        contrast = np.std(gray)
        
        # Calculate histogram for more detailed analysis
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        hist_normalized = hist.flatten() / hist.sum()
        
        # Calculate entropy as a measure of information content
        entropy = -np.sum(hist_normalized * np.log2(hist_normalized + 1e-10))
        
        # Update history
        self.brightness_history.append(brightness)
        self.contrast_history.append(contrast)
        
        processing_time = time.time() - start_time
        
        lighting_conditions = {
            'brightness': brightness,
            'contrast': contrast,
            'entropy': entropy,
            'brightness_stability': np.std(self.brightness_history) if len(self.brightness_history) > 1 else 0,
            'contrast_stability': np.std(self.contrast_history) if len(self.contrast_history) > 1 else 0,
            'processing_time': processing_time
        }
        
        # Log detailed lighting analysis
        rospy.logdebug(f"[AdaptiveThreshold] Lighting Analysis - "
                      f"Brightness: {brightness:.2f}, Contrast: {contrast:.2f}, "
                      f"Entropy: {entropy:.2f}, Processing time: {processing_time*1000:.2f}ms")
        
        return lighting_conditions
    
    def adapt_thresholds(self, lighting_conditions, detection_count):
        """
        Adapt detection thresholds based on lighting conditions and detection performance.
        
        Args:
            lighting_conditions: Dictionary with lighting analysis results
            detection_count: Number of lines detected in the current frame
        """
        start_time = time.time()
        
        brightness = lighting_conditions['brightness']
        contrast = lighting_conditions['contrast']
        
        # Determine lighting category
        if brightness < 80:
            lighting_category = "dark"
        elif brightness > 180:
            lighting_category = "bright"
        else:
            lighting_category = "normal"
        
        # Adaptive threshold calculation
        old_canny_low = self.current_canny_low
        old_canny_high = self.current_canny_high
        
        if lighting_category == "dark":
            # Lower thresholds for dark conditions
            target_low = max(30, self.base_canny_low - 30)
            target_high = max(100, self.base_canny_high - 50)
        elif lighting_category == "bright":
            # Higher thresholds for bright conditions
            target_low = min(120, self.base_canny_low + 20)
            target_high = min(300, self.base_canny_high + 50)
        else:
            # Normal conditions - use base values with contrast adjustment
            contrast_factor = max(0.5, min(2.0, contrast / 50.0))
            target_low = self.base_canny_low * contrast_factor
            target_high = self.base_canny_high * contrast_factor
        
        # Smooth adaptation using exponential moving average
        self.current_canny_low = (1 - self.adaptation_rate) * self.current_canny_low + \
                                self.adaptation_rate * target_low
        self.current_canny_high = (1 - self.adaptation_rate) * self.current_canny_high + \
                                 self.adaptation_rate * target_high
        
        # Update detection confidence based on detection count stability
        if detection_count > 0:
            detection_ratio = min(1.0, detection_count / 20.0)  # Normalize to expected range
            self.detection_confidence = 0.9 * self.detection_confidence + 0.1 * detection_ratio
        else:
            self.detection_confidence *= 0.95  # Decay confidence if no detections
        
        self.last_detection_count = detection_count
        
        processing_time = time.time() - start_time
        
        # Log threshold adaptation details
        rospy.logdebug(f"[AdaptiveThreshold] Threshold Adaptation - "
                      f"Category: {lighting_category}, "
                      f"Canny: {old_canny_low:.1f}->{self.current_canny_low:.1f}, "
                      f"{old_canny_high:.1f}->{self.current_canny_high:.1f}, "
                      f"Confidence: {self.detection_confidence:.3f}, "
                      f"Detections: {detection_count}, "
                      f"Processing time: {processing_time*1000:.2f}ms")
    
    def get_adapted_parameters(self):
        """
        Get current adapted parameters for line detection.
        
        Returns:
            dict: Current detection parameters
        """
        return {
            'canny_thresholds': [int(self.current_canny_low), int(self.current_canny_high)],
            'detection_confidence': self.detection_confidence,
            'brightness_avg': np.mean(self.brightness_history) if self.brightness_history else 0,
            'contrast_avg': np.mean(self.contrast_history) if self.contrast_history else 0
        }


class TemporalConsistencyFilter:
    """
    Temporal consistency filter for stable line detection across frames.
    
    This class maintains a history of detected lines and filters out inconsistent detections
    to provide more stable and reliable line detection results.
    """
    
    def __init__(self, history_size=5, consistency_threshold=0.7):
        self.history_size = history_size
        self.consistency_threshold = consistency_threshold
        
        # Detection history for each color
        self.detection_history = {
            'WHITE': deque(maxlen=history_size),
            'YELLOW': deque(maxlen=history_size),
            'RED': deque(maxlen=history_size)
        }
        
        # Temporal metrics
        self.temporal_confidence = {}
        self.stability_scores = {}
        
    def add_detections(self, color, detections):
        """
        Add new detections to the temporal history.
        
        Args:
            color: Color name (WHITE, YELLOW, RED)
            detections: Detection results from LineDetector
        """
        start_time = time.time()
        
        if color not in self.detection_history:
            self.detection_history[color] = deque(maxlen=self.history_size)
        
        # Store detection metadata
        detection_data = {
            'timestamp': time.time(),
            'line_count': len(detections.lines) if hasattr(detections, 'lines') else 0,
            'lines': detections.lines if hasattr(detections, 'lines') else [],
            'normals': detections.normals if hasattr(detections, 'normals') else [],
            'centers': detections.centers if hasattr(detections, 'centers') else []
        }
        
        self.detection_history[color].append(detection_data)
        
        processing_time = time.time() - start_time
        
        # Log temporal filter addition
        rospy.logdebug(f"[TemporalFilter] Added {color} detections - "
                      f"Lines: {detection_data['line_count']}, "
                      f"History size: {len(self.detection_history[color])}, "
                      f"Processing time: {processing_time*1000:.2f}ms")
    
    def get_filtered_detections(self, color):
        """
        Get temporally filtered detections for a specific color.
        
        Args:
            color: Color name to filter
            
        Returns:
            dict: Filtered detection results with confidence metrics
        """
        start_time = time.time()
        
        if color not in self.detection_history or len(self.detection_history[color]) == 0:
            return {
                'lines': [],
                'normals': [],
                'centers': [],
                'temporal_confidence': 0.0,
                'stability_score': 0.0
            }
        
        history = list(self.detection_history[color])
        
        # Calculate temporal consistency metrics
        line_counts = [d['line_count'] for d in history]
        avg_line_count = np.mean(line_counts)
        line_count_stability = 1.0 - (np.std(line_counts) / (avg_line_count + 1e-6))
        
        # Calculate temporal confidence based on detection consistency
        recent_frames = min(3, len(history))
        recent_detections = sum(1 for d in history[-recent_frames:] if d['line_count'] > 0)
        temporal_confidence = recent_detections / recent_frames if recent_frames > 0 else 0.0
        
        # Use most recent detections if confidence is high enough
        if temporal_confidence >= self.consistency_threshold:
            # Find the most recent frame with detections
            latest_with_detections = None
            for detection_data in reversed(history):
                if detection_data['line_count'] > 0:
                    latest_with_detections = detection_data
                    break
            
            if latest_with_detections:
                filtered_result = {
                    'lines': latest_with_detections['lines'],
                    'normals': latest_with_detections['normals'],
                    'centers': latest_with_detections['centers'],
                    'temporal_confidence': temporal_confidence,
                    'stability_score': line_count_stability
                }
            else:
                # No detections found even with high confidence
                filtered_result = {
                    'lines': [],
                    'normals': [],
                    'centers': [],
                    'temporal_confidence': temporal_confidence,
                    'stability_score': line_count_stability
                }
        else:
            # Use averaged/filtered results for low confidence
            filtered_result = {
                'lines': [],
                'normals': [],
                'centers': [],
                'temporal_confidence': temporal_confidence,
                'stability_score': line_count_stability
            }
        
        # Store metrics for monitoring
        self.temporal_confidence[color] = temporal_confidence
        self.stability_scores[color] = line_count_stability
        
        processing_time = time.time() - start_time
        
        # Log temporal filtering results
        rospy.logdebug(f"[TemporalFilter] Filtered {color} - "
                      f"Confidence: {temporal_confidence:.3f}, "
                      f"Stability: {line_count_stability:.3f}, "
                      f"Lines: {len(filtered_result['lines'])}, "
                      f"Processing time: {processing_time*1000:.2f}ms")
        
        return filtered_result
    
    def get_temporal_metrics(self):
        """
        Get current temporal filtering metrics.
        
        Returns:
            dict: Temporal metrics for all colors
        """
        return {
            'temporal_confidence': dict(self.temporal_confidence),
            'stability_scores': dict(self.stability_scores),
            'history_sizes': {color: len(history) for color, history in self.detection_history.items()}
        }


class LineDetectorNode(DTROS):
    """
    The ``LineDetectorNode`` is responsible for detecting the line white, yellow and red line segment in an image and
    is used for lane localization.

    Upon receiving an image, this node reduces its resolution, cuts off the top part so that only the
    road-containing part of the image is left, extracts the white, red, and yellow segments and publishes them.
    The main functionality of this node is implemented in the :py:class:`line_detector.LineDetector` class.

    The performance of this node can be very sensitive to its configuration parameters. Therefore, it also provides a
    number of debug topics which can be used for fine-tuning these parameters. These configuration parameters can be
    changed dynamically while the node is running via ``rosparam set`` commands.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~line_detector_parameters (:obj:`dict`): A dictionary with the parameters for the detector. The full list can be found in :py:class:`line_detector.LineDetector`.
        ~colors (:obj:`dict`): A dictionary of colors and color ranges to be detected in the image. The keys (color names) should match the ones in the Segment message definition, otherwise an exception will be thrown! See the ``config`` directory in the node code for the default ranges.
        ~img_size (:obj:`list` of ``int``): The desired downsized resolution of the image. Lower resolution would result in faster detection but lower performance, default is ``[120,160]``
        ~top_cutoff (:obj:`int`): The number of rows to be removed from the top of the image _after_ resizing, default is 40

    Subscriber:
        ~camera_node/image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): The camera images
        ~anti_instagram_node/thresholds(:obj:`duckietown_msgs.msg.AntiInstagramThresholds`): The thresholds to do color correction

    Publishers:
        ~segment_list (:obj:`duckietown_msgs.msg.SegmentList`): A list of the detected segments. Each segment is an :obj:`duckietown_msgs.msg.Segment` message
        ~debug/segments/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the segments drawn on the input image
        ~debug/edges/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the Canny edges drawn on the input image
        ~debug/maps/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug topic with the regions falling in each color range drawn on the input image
        ~debug/ranges_HS (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the input image and the color ranges, Hue-Saturation projection
        ~debug/ranges_SV (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the input image and the color ranges, Saturation-Value projection
        ~debug/ranges_HV (:obj:`sensor_msgs.msg.Image`): Debug topic with a histogram of the colors in the input image and the color ranges, Hue-Value projection

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LineDetectorNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Define parameters
        self._line_detector_parameters = rospy.get_param("~line_detector_parameters", None)
        self._colors = rospy.get_param("~colors", None)
        self._img_size = rospy.get_param("~img_size", None)
        self._top_cutoff = rospy.get_param("~top_cutoff", None)
        
        # Enhanced detection parameters
        self._adaptive_threshold_enabled = rospy.get_param("~adaptive_threshold_enabled", True)
        self._temporal_filtering_enabled = rospy.get_param("~temporal_filtering_enabled", True)
        self._adaptation_rate = rospy.get_param("~adaptation_rate", 0.1)
        self._temporal_history_size = rospy.get_param("~temporal_history_size", 5)
        self._consistency_threshold = rospy.get_param("~consistency_threshold", 0.7)

        self.bridge = CvBridge()

        # The thresholds to be used for AntiInstagram color correction
        self.ai_thresholds_received = False
        self.anti_instagram_thresholds = dict()
        self.ai = AntiInstagram()

        # This holds the colormaps for the debug/ranges images after they are computed once
        self.colormaps = dict()

        # Create a new LineDetector object with the parameters from the Parameter Server / config file
        self.detector = LineDetector(**self._line_detector_parameters)
        # Update the color ranges objects
        self.color_ranges = {color: ColorRange.fromDict(d) for color, d in list(self._colors.items())}
        
        # Initialize enhanced detection components
        if self._adaptive_threshold_enabled:
            base_canny = self._line_detector_parameters.get('canny_thresholds', [80, 200])
            self.adaptive_detector = AdaptiveThresholdDetector(
                base_canny_low=base_canny[0],
                base_canny_high=base_canny[1],
                adaptation_rate=self._adaptation_rate
            )
            rospy.loginfo("[LineDetectorNode] Adaptive threshold detection enabled")
        else:
            self.adaptive_detector = None
            
        if self._temporal_filtering_enabled:
            self.temporal_filter = TemporalConsistencyFilter(
                history_size=self._temporal_history_size,
                consistency_threshold=self._consistency_threshold
            )
            rospy.loginfo("[LineDetectorNode] Temporal consistency filtering enabled")
        else:
            self.temporal_filter = None
            
        # Performance monitoring
        self.frame_count = 0
        self.total_processing_time = 0.0
        self.last_performance_log = time.time()

    # Active flag controlled by FSM via ~switch service
        self._active = True

        # Publishers
        self.pub_lines = rospy.Publisher(
            "~segment_list", SegmentList, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )
        self.pub_d_segments = rospy.Publisher(
            "~debug/segments/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_edges = rospy.Publisher(
            "~debug/edges/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_maps = rospy.Publisher(
            "~debug/maps/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        # these are not compressed because compression adds undesired blur
        self.pub_d_ranges_HS = rospy.Publisher(
            "~debug/ranges_HS", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_ranges_SV = rospy.Publisher(
            "~debug/ranges_SV", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )
        self.pub_d_ranges_HV = rospy.Publisher(
            "~debug/ranges_HV", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        # Subscribers
        self.sub_image = rospy.Subscriber(
            "~image/compressed", CompressedImage, self.image_cb, buff_size=10000000, queue_size=1
        )

        self.sub_thresholds = rospy.Subscriber(
            "~thresholds", AntiInstagramThresholds, self.thresholds_cb, queue_size=1
        )

        # Service to enable/disable node
        self._srv_switch = rospy.Service("~switch", SetBool, self._cb_switch)

    def thresholds_cb(self, thresh_msg):
        self.anti_instagram_thresholds["lower"] = thresh_msg.low
        self.anti_instagram_thresholds["higher"] = thresh_msg.high
        self.ai_thresholds_received = True

    # --- Services ---
    def _cb_switch(self, req: SetBool):
        self._active = bool(req.data)
        status = "enabled" if self._active else "disabled"
        self.loginfo(f"Switch: {status}")
        return SetBoolResponse(success=True, message=f"LineDetectorNode {status}")

    def image_cb(self, image_msg):
        """
        Processes the incoming image messages with enhanced adaptive detection.

        Performs the following steps for each incoming image:

        #. Performs color correction
        #. Resizes the image to the ``~img_size`` resolution
        #. Removes the top ``~top_cutoff`` rows in order to remove the part of the image that doesn't include the road
        #. Analyzes lighting conditions and adapts detection parameters (if enabled)
        #. Extracts the line segments in the image using enhanced :py:class:`line_detector.LineDetector`
        #. Applies temporal consistency filtering (if enabled)
        #. Converts the coordinates of detected segments to normalized ones
        #. Creates and publishes the resultant :obj:`duckietown_msgs.msg.SegmentList` message
        #. Creates and publishes debug images if there is a subscriber to the respective topics
        #. Logs comprehensive performance and detection metrics

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): The receive image message

        """
        if not self._active:
            return
        frame_start_time = time.time()
        self.frame_count += 1

        # Decode from compressed image with OpenCV
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr(f"Could not decode image: {e}")
            return

        # Log image processing start
        rospy.logdebug(f"[LineDetectorNode] Frame {self.frame_count} - "
                      f"Processing image {image.shape[1]}x{image.shape[0]} at {rospy.Time.now()}")

        # Perform color correction
        color_correction_start = time.time()
        if self.ai_thresholds_received:
            image = self.ai.apply_color_balance(
                self.anti_instagram_thresholds["lower"], self.anti_instagram_thresholds["higher"], image
            )
        color_correction_time = time.time() - color_correction_start

        # Resize the image to the desired dimensions
        resize_start_time = time.time()
        height_original, width_original = image.shape[0:2]
        img_size = (self._img_size[1], self._img_size[0])
        if img_size[0] != width_original or img_size[1] != height_original:
            image = cv2.resize(image, img_size, interpolation=cv2.INTER_NEAREST)
        image = image[self._top_cutoff :, :, :]
        resize_time = time.time() - resize_start_time

        # Enhanced adaptive detection
        lighting_conditions = None
        if self.adaptive_detector:
            # Analyze lighting conditions
            lighting_analysis_start = time.time()
            lighting_conditions = self.adaptive_detector.analyze_lighting_conditions(image)
            lighting_analysis_time = time.time() - lighting_analysis_start
            
            rospy.logdebug(f"[LineDetectorNode] Lighting analysis time: {lighting_analysis_time*1000:.2f}ms")

        # Set image and adapt parameters if needed
        detection_start_time = time.time()
        self.detector.setImage(image)
        
        # Apply adaptive thresholds if enabled
        if self.adaptive_detector and lighting_conditions:
            adapted_params = self.adaptive_detector.get_adapted_parameters()
            # Update detector with adapted parameters
            self.detector.canny_thresholds = adapted_params['canny_thresholds']
            
            rospy.logdebug(f"[LineDetectorNode] Adapted Canny thresholds: {adapted_params['canny_thresholds']}, "
                          f"Detection confidence: {adapted_params['detection_confidence']:.3f}")

        # Extract the line segments for every color
        detections = {}
        total_detection_count = 0
        
        for color, ranges in list(self.color_ranges.items()):
            color_detection_start = time.time()
            detection_result = self.detector.detectLines(ranges)
            color_detection_time = time.time() - color_detection_start
            
            line_count = len(detection_result.lines) if hasattr(detection_result, 'lines') else 0
            total_detection_count += line_count
            
            # Apply temporal filtering if enabled
            if self.temporal_filter:
                self.temporal_filter.add_detections(color, detection_result)
                filtered_result = self.temporal_filter.get_filtered_detections(color)
                
                # Create a detection object with filtered results
                from line_detector.detections import Detections
                detection_result = Detections(
                    lines=np.array(filtered_result['lines']) if filtered_result['lines'] else np.array([]),
                    normals=np.array(filtered_result['normals']) if filtered_result['normals'] else np.array([]),
                    map=detection_result.map if hasattr(detection_result, 'map') else np.array([]),
                    centers=np.array(filtered_result['centers']) if filtered_result['centers'] else np.array([])
                )
                
                rospy.logdebug(f"[LineDetectorNode] {color} detection - "
                              f"Raw: {line_count}, Filtered: {len(filtered_result['lines'])}, "
                              f"Temporal confidence: {filtered_result['temporal_confidence']:.3f}, "
                              f"Processing time: {color_detection_time*1000:.2f}ms")
            else:
                rospy.logdebug(f"[LineDetectorNode] {color} detection - "
                              f"Lines: {line_count}, "
                              f"Processing time: {color_detection_time*1000:.2f}ms")
            
            detections[color] = detection_result
        
        detection_time = time.time() - detection_start_time
        
        # Update adaptive thresholds based on detection performance
        if self.adaptive_detector and lighting_conditions:
            adaptation_start_time = time.time()
            self.adaptive_detector.adapt_thresholds(lighting_conditions, total_detection_count)
            adaptation_time = time.time() - adaptation_start_time
            
            rospy.logdebug(f"[LineDetectorNode] Threshold adaptation time: {adaptation_time*1000:.2f}ms")

        # Construct a SegmentList
        segment_construction_start = time.time()
        segment_list = SegmentList()
        segment_list.header.stamp = image_msg.header.stamp

        # Remove the offset in coordinates coming from the removing of the top part and
        arr_cutoff = np.array([0, self._top_cutoff, 0, self._top_cutoff])
        arr_ratio = np.array(
            [
                1.0 / self._img_size[1],
                1.0 / self._img_size[0],
                1.0 / self._img_size[1],
                1.0 / self._img_size[0],
            ]
        )

        # Fill in the segment_list with all the detected segments
        total_segments = 0
        for color, det in list(detections.items()):
            # Get the ID for the color from the Segment msg definition
            # Throw and exception otherwise
            if len(det.lines) > 0 and len(det.normals) > 0:
                try:
                    color_id = getattr(Segment, color)
                    lines_normalized = (det.lines + arr_cutoff) * arr_ratio
                    segments = self._to_segment_msg(lines_normalized, det.normals, color_id)
                    segment_list.segments.extend(segments)
                    total_segments += len(segments)
                except AttributeError:
                    self.logerr(f"Color name {color} is not defined in the Segment message")

        segment_construction_time = time.time() - segment_construction_start

        # Publish the message
        publish_start_time = time.time()
        self.pub_lines.publish(segment_list)
        publish_time = time.time() - publish_start_time

        # Debug image publishing with timing
        debug_start_time = time.time()
        debug_images_published = 0
        
        # If there are any subscribers to the debug topics, generate a debug image and publish it
        if self.pub_d_segments.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in list(detections.items())}
            debug_img = plotSegments(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_segments.publish(debug_image_msg)
            debug_images_published += 1

        if self.pub_d_edges.get_num_connections() > 0:
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(self.detector.canny_edges)
            debug_image_msg.header = image_msg.header
            self.pub_d_edges.publish(debug_image_msg)
            debug_images_published += 1

        if self.pub_d_maps.get_num_connections() > 0:
            colorrange_detections = {self.color_ranges[c]: det for c, det in list(detections.items())}
            debug_img = plotMaps(image, colorrange_detections)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_maps.publish(debug_image_msg)
            debug_images_published += 1

        for channels in ["HS", "SV", "HV"]:
            publisher = getattr(self, f"pub_d_ranges_{channels}")
            if publisher.get_num_connections() > 0:
                debug_img = self._plot_ranges_histogram(channels)
                debug_image_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                debug_image_msg.header = image_msg.header
                publisher.publish(debug_image_msg)
                debug_images_published += 1
        
        debug_time = time.time() - debug_start_time
        
        # Calculate total frame processing time
        total_frame_time = time.time() - frame_start_time
        self.total_processing_time += total_frame_time
        
        # Comprehensive performance logging
        rospy.logdebug(f"[LineDetectorNode] Frame {self.frame_count} Performance - "
                      f"Total: {total_frame_time*1000:.2f}ms, "
                      f"Color correction: {color_correction_time*1000:.2f}ms, "
                      f"Resize: {resize_time*1000:.2f}ms, "
                      f"Detection: {detection_time*1000:.2f}ms, "
                      f"Segment construction: {segment_construction_time*1000:.2f}ms, "
                      f"Publish: {publish_time*1000:.2f}ms, "
                      f"Debug: {debug_time*1000:.2f}ms")
        
        # Real-time monitoring - log every frame with key metrics
        current_time = time.time()
        fps = 1.0 / total_frame_time if total_frame_time > 0 else 0
        avg_processing_time = self.total_processing_time / self.frame_count
        
        rospy.logdebug(f"[LineDetectorNode] Real-time Monitoring - "
                      f"Frame: {self.frame_count}, "
                      f"FPS: {fps:.1f}, "
                      f"Avg processing: {avg_processing_time*1000:.2f}ms, "
                      f"Total segments: {total_segments}, "
                      f"Debug images: {debug_images_published}")
        
        # Periodic performance summary (every 30 frames)
        if self.frame_count % 30 == 0:
            time_since_last_log = current_time - self.last_performance_log
            avg_fps = 30.0 / time_since_last_log if time_since_last_log > 0 else 0
            
            performance_summary = f"[LineDetectorNode] Performance Summary (30 frames) - " \
                                f"Avg FPS: {avg_fps:.1f}, " \
                                f"Avg processing time: {avg_processing_time*1000:.2f}ms, " \
                                f"Total segments detected: {total_segments}"
            
            if self.adaptive_detector:
                adapted_params = self.adaptive_detector.get_adapted_parameters()
                performance_summary += f", Adaptive confidence: {adapted_params['detection_confidence']:.3f}"
                
            if self.temporal_filter:
                temporal_metrics = self.temporal_filter.get_temporal_metrics()
                avg_temporal_confidence = np.mean(list(temporal_metrics['temporal_confidence'].values())) \
                    if temporal_metrics['temporal_confidence'] else 0
                performance_summary += f", Temporal confidence: {avg_temporal_confidence:.3f}"
            
            rospy.loginfo(performance_summary)
            self.last_performance_log = current_time

    @staticmethod
    def _to_segment_msg(lines, normals, color):
        """
        Converts line detections to a list of Segment messages.

        Converts the resultant line segments and normals from the line detection to a list of Segment messages.

        Args:
            lines (:obj:`numpy array`): An ``Nx4`` array where each row represents a line.
            normals (:obj:`numpy array`): An ``Nx2`` array where each row represents the normal of a line.
            color (:obj:`str`): Color name string, should be one of the pre-defined in the Segment message definition.

        Returns:
            :obj:`list` of :obj:`duckietown_msgs.msg.Segment`: List of Segment messages

        """
        segment_msg_list = []
        for x1, y1, x2, y2, norm_x, norm_y in np.hstack((lines, normals)):
            segment = Segment()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2
            segment.normal.x = norm_x
            segment.normal.y = norm_y
            segment_msg_list.append(segment)
        return segment_msg_list

    def _plot_ranges_histogram(self, channels):
        """Utility method for plotting color histograms and color ranges.

        Args:
            channels (:obj:`str`): The desired two channels, should be one of ``['HS','SV','HV']``

        Returns:
            :obj:`numpy array`: The resultant plot image

        """
        channel_to_axis = {"H": 0, "S": 1, "V": 2}
        axis_to_range = {0: 180, 1: 256, 2: 256}

        # Get which is the third channel that will not be shown in this plot
        missing_channel = "HSV".replace(channels[0], "").replace(channels[1], "")

        hsv_im = self.detector.hsv
        # Get the pixels as a list (flatten the horizontal and vertical dimensions)
        hsv_im = hsv_im.reshape((-1, 3))

        channel_idx = [channel_to_axis[channels[0]], channel_to_axis[channels[1]]]

        # Get only the relevant channels
        x_bins = np.arange(0, axis_to_range[channel_idx[1]] + 1, 2)
        y_bins = np.arange(0, axis_to_range[channel_idx[0]] + 1, 2)
        h, _, _ = np.histogram2d(
            x=hsv_im[:, channel_idx[0]], y=hsv_im[:, channel_idx[1]], bins=[y_bins, x_bins]
        )
        # Log-normalized histogram
        np.log(h, out=h, where=(h != 0))
        h = (255 * h / np.max(h)).astype(np.uint8)

        # Make a color map, for the missing channel, just take the middle of the range
        if channels not in self.colormaps:
            colormap_1, colormap_0 = np.meshgrid(x_bins[:-1], y_bins[:-1])
            colormap_2 = np.ones_like(colormap_0) * (axis_to_range[channel_to_axis[missing_channel]] / 2)

            channel_to_map = {channels[0]: colormap_0, channels[1]: colormap_1, missing_channel: colormap_2}

            self.colormaps[channels] = np.stack(
                [channel_to_map["H"], channel_to_map["S"], channel_to_map["V"]], axis=-1
            ).astype(np.uint8)
            self.colormaps[channels] = cv2.cvtColor(self.colormaps[channels], cv2.COLOR_HSV2BGR)

        # resulting histogram image as a blend of the two images
        im = cv2.cvtColor(h[:, :, None], cv2.COLOR_GRAY2BGR)
        im = cv2.addWeighted(im, 0.5, self.colormaps[channels], 1 - 0.5, 0.0)

        # now plot the color ranges on top
        for _, color_range in list(self.color_ranges.items()):
            # convert HSV color to BGR
            c = color_range.representative
            c = np.uint8([[[c[0], c[1], c[2]]]])
            color = cv2.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int)
            for i in range(len(color_range.low)):
                cv2.rectangle(
                    im,
                    pt1=(
                        (color_range.high[i, channel_idx[1]] / 2).astype(np.uint8),
                        (color_range.high[i, channel_idx[0]] / 2).astype(np.uint8),
                    ),
                    pt2=(
                        (color_range.low[i, channel_idx[1]] / 2).astype(np.uint8),
                        (color_range.low[i, channel_idx[0]] / 2).astype(np.uint8),
                    ),
                    color=color,
                    lineType=cv2.LINE_4,
                )
        # ---
        return im


if __name__ == "__main__":
    # Initialize the node
    line_detector_node = LineDetectorNode(node_name="line_detector_node")
    # Keep it spinning to keep the node alive
    rospy.spin()

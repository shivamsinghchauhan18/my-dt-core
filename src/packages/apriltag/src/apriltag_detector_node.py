#!/usr/bin/env python3

import cv2
import rospy
import tf
import numpy as np
import time
import math

from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from turbojpeg import TurboJPEG, TJPF_GRAY
from image_geometry import PinholeCameraModel
from dt_apriltags import Detector

from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType

from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Transform, Vector3, Quaternion


class MultiResolutionDetector:
    """
    Multi-resolution AprilTag detector with configurable scale factors and confidence filtering.
    Provides enhanced detection robustness through multi-scale analysis.
    """
    
    def __init__(self, base_detector, scale_factors=None, confidence_threshold=0.1):
        """
        Initialize multi-resolution detector.
        
        Args:
            base_detector: Base AprilTag detector instance
            scale_factors: List of scale factors for multi-resolution detection
            confidence_threshold: Minimum confidence threshold for detections
        """
        self.base_detector = base_detector
        self.scale_factors = scale_factors or [0.5, 1.0, 1.5, 2.0]
        self.confidence_threshold = confidence_threshold
        
        # Logging and monitoring
        self.detection_count = 0
        self.total_processing_time = 0.0
        self.scale_performance = {scale: {'detections': 0, 'time': 0.0} for scale in self.scale_factors}
        
        rospy.loginfo(f"[MultiResolutionDetector] Initialized with scale factors: {self.scale_factors}")
        rospy.loginfo(f"[MultiResolutionDetector] Confidence threshold: {self.confidence_threshold}")
    
    def detect_multi_scale(self, image, camera_params, tag_size):
        """
        Perform multi-scale AprilTag detection with comprehensive logging.
        
        Args:
            image: Input grayscale image
            camera_params: Camera calibration parameters (fx, fy, cx, cy)
            tag_size: Physical size of AprilTag in meters
            
        Returns:
            List of filtered AprilTag detections with confidence scores
        """
        start_time = time.time()
        all_detections = []
        scale_results = {}
        
        rospy.logdebug(f"[MultiResolutionDetector] Starting multi-scale detection at {start_time}")
        rospy.logdebug(f"[MultiResolutionDetector] Image shape: {image.shape}")
        rospy.logdebug(f"[MultiResolutionDetector] Camera params: fx={camera_params[0]:.2f}, fy={camera_params[1]:.2f}, cx={camera_params[2]:.2f}, cy={camera_params[3]:.2f}")
        
        for scale_factor in self.scale_factors:
            scale_start_time = time.time()
            
            # Scale image
            if scale_factor != 1.0:
                new_width = int(image.shape[1] * scale_factor)
                new_height = int(image.shape[0] * scale_factor)
                scaled_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
                
                # Adjust camera parameters for scaled image
                scaled_camera_params = (
                    camera_params[0] * scale_factor,  # fx
                    camera_params[1] * scale_factor,  # fy
                    camera_params[2] * scale_factor,  # cx
                    camera_params[3] * scale_factor   # cy
                )
            else:
                scaled_image = image
                scaled_camera_params = camera_params
            
            rospy.logdebug(f"[MultiResolutionDetector] Scale {scale_factor}: Image scaled to {scaled_image.shape}")
            rospy.logdebug(f"[MultiResolutionDetector] Scale {scale_factor}: Adjusted camera params: fx={scaled_camera_params[0]:.2f}, fy={scaled_camera_params[1]:.2f}")
            
            # Detect tags at this scale
            try:
                detections = self.base_detector.detect(scaled_image, True, scaled_camera_params, tag_size)
                
                # Adjust detection coordinates back to original scale
                for detection in detections:
                    if scale_factor != 1.0:
                        # Scale back corner coordinates
                        detection.corners = detection.corners / scale_factor
                        detection.center = detection.center / scale_factor
                        
                        # Scale back homography
                        H = detection.homography.copy()
                        H[0, 2] /= scale_factor  # tx
                        H[1, 2] /= scale_factor  # ty
                        H[2, 0] *= scale_factor  # h20
                        H[2, 1] *= scale_factor  # h21
                        detection.homography = H
                    
                    # Add scale factor information
                    detection.scale_factor = scale_factor
                    detection.detection_confidence = detection.decision_margin
                
                scale_time = time.time() - scale_start_time
                self.scale_performance[scale_factor]['detections'] += len(detections)
                self.scale_performance[scale_factor]['time'] += scale_time
                
                scale_results[scale_factor] = {
                    'detections': len(detections),
                    'processing_time': scale_time,
                    'confidence_scores': [d.decision_margin for d in detections]
                }
                
                rospy.logdebug(f"[MultiResolutionDetector] Scale {scale_factor}: Found {len(detections)} detections in {scale_time:.4f}s")
                for i, det in enumerate(detections):
                    rospy.logdebug(f"[MultiResolutionDetector] Scale {scale_factor}: Detection {i}: ID={det.tag_id}, confidence={det.decision_margin:.4f}")
                
                all_detections.extend(detections)
                
            except Exception as e:
                rospy.logwarn(f"[MultiResolutionDetector] Scale {scale_factor}: Detection failed: {e}")
                scale_results[scale_factor] = {
                    'detections': 0,
                    'processing_time': time.time() - scale_start_time,
                    'error': str(e)
                }
        
        # Filter and merge detections
        filtered_detections = self._filter_detections(all_detections)
        
        total_time = time.time() - start_time
        self.detection_count += 1
        self.total_processing_time += total_time
        
        # Comprehensive logging
        rospy.logdebug(f"[MultiResolutionDetector] Total processing time: {total_time:.4f}s")
        rospy.logdebug(f"[MultiResolutionDetector] Raw detections: {len(all_detections)}, Filtered: {len(filtered_detections)}")
        rospy.logdebug(f"[MultiResolutionDetector] Scale results: {scale_results}")
        
        # Real-time monitoring logs
        if self.detection_count % 30 == 0:  # Log every 30 detections
            avg_time = self.total_processing_time / self.detection_count
            rospy.loginfo(f"[MultiResolutionDetector] Performance summary after {self.detection_count} detections:")
            rospy.loginfo(f"[MultiResolutionDetector] Average processing time: {avg_time:.4f}s")
            for scale, perf in self.scale_performance.items():
                if perf['detections'] > 0:
                    avg_scale_time = perf['time'] / max(1, perf['detections'])
                    rospy.loginfo(f"[MultiResolutionDetector] Scale {scale}: {perf['detections']} detections, avg time: {avg_scale_time:.4f}s")
        
        return filtered_detections
    
    def _filter_detections(self, detections):
        """
        Filter detections based on confidence threshold and remove duplicates.
        
        Args:
            detections: List of raw detections from all scales
            
        Returns:
            List of filtered and deduplicated detections
        """
        if not detections:
            return []
        
        # Filter by confidence threshold
        confident_detections = [d for d in detections if d.decision_margin >= self.confidence_threshold]
        
        rospy.logdebug(f"[MultiResolutionDetector] Confidence filtering: {len(detections)} -> {len(confident_detections)} detections")
        
        if not confident_detections:
            return []
        
        # Group detections by tag ID
        tag_groups = {}
        for detection in confident_detections:
            tag_id = detection.tag_id
            if tag_id not in tag_groups:
                tag_groups[tag_id] = []
            tag_groups[tag_id].append(detection)
        
        # Select best detection for each tag ID
        filtered_detections = []
        for tag_id, group in tag_groups.items():
            if len(group) == 1:
                filtered_detections.append(group[0])
                rospy.logdebug(f"[MultiResolutionDetector] Tag {tag_id}: Single detection, confidence={group[0].decision_margin:.4f}")
            else:
                # Select detection with highest confidence
                best_detection = max(group, key=lambda d: d.decision_margin)
                filtered_detections.append(best_detection)
                
                confidences = [d.decision_margin for d in group]
                scales = [getattr(d, 'scale_factor', 1.0) for d in group]
                rospy.logdebug(f"[MultiResolutionDetector] Tag {tag_id}: {len(group)} detections, selected best confidence={best_detection.decision_margin:.4f}")
                rospy.logdebug(f"[MultiResolutionDetector] Tag {tag_id}: All confidences={confidences}, scales={scales}")
        
        return filtered_detections
    
    def estimate_distance_and_angle(self, detection, camera_params, tag_size):
        """
        Estimate precise distance and approach angle using tag geometry.
        
        Args:
            detection: AprilTag detection object
            camera_params: Camera calibration parameters
            tag_size: Physical size of AprilTag in meters
            
        Returns:
            Tuple of (distance, approach_angle, estimation_quality)
        """
        try:
            # Extract pose information
            pose_t = detection.pose_t.flatten()
            pose_R = detection.pose_R
            
            # Calculate distance from translation vector
            distance = np.linalg.norm(pose_t)
            
            # Calculate approach angle from rotation matrix
            # Extract yaw angle (rotation around Z-axis)
            approach_angle = math.atan2(pose_R[1, 0], pose_R[0, 0])
            
            # Estimate quality based on detection confidence and pose error
            pose_error = getattr(detection, 'pose_err', 0.0)
            confidence = detection.decision_margin
            
            # Quality metric combining confidence and pose error
            estimation_quality = confidence / (1.0 + pose_error)
            
            rospy.logdebug(f"[MultiResolutionDetector] Tag {detection.tag_id}: Distance={distance:.3f}m, Angle={math.degrees(approach_angle):.1f}°")
            rospy.logdebug(f"[MultiResolutionDetector] Tag {detection.tag_id}: Pose error={pose_error:.4f}, Quality={estimation_quality:.4f}")
            
            return distance, approach_angle, estimation_quality
            
        except Exception as e:
            rospy.logwarn(f"[MultiResolutionDetector] Distance estimation failed for tag {detection.tag_id}: {e}")
            return 0.0, 0.0, 0.0
    
    def get_detection_metrics(self):
        """
        Get comprehensive detection performance metrics.
        
        Returns:
            Dictionary containing performance metrics
        """
        if self.detection_count == 0:
            return {}
        
        avg_time = self.total_processing_time / self.detection_count
        
        metrics = {
            'total_detections': self.detection_count,
            'average_processing_time': avg_time,
            'scale_performance': {}
        }
        
        for scale, perf in self.scale_performance.items():
            if perf['detections'] > 0:
                metrics['scale_performance'][scale] = {
                    'detections': perf['detections'],
                    'average_time': perf['time'] / perf['detections']
                }
        
        return metrics


class AprilTagDetector(DTROS):
    def __init__(self):
        super(AprilTagDetector, self).__init__(
            node_name="apriltag_detector_node", node_type=NodeType.PERCEPTION
        )
        # get static parameters
        self.family = rospy.get_param("~family", "tag36h11")
        self.ndetectors = rospy.get_param("~ndetectors", 1)
        self.nthreads = rospy.get_param("~nthreads", 1)
        self.quad_decimate = rospy.get_param("~quad_decimate", 1.0)
        self.quad_sigma = rospy.get_param("~quad_sigma", 0.0)
        self.refine_edges = rospy.get_param("~refine_edges", 1)
        self.decode_sharpening = rospy.get_param("~decode_sharpening", 0.25)
        self.tag_size = rospy.get_param("~tag_size", 0.065)
        self.rectify_alpha = rospy.get_param("~rectify_alpha", 0.0)
        
        # Multi-resolution detection parameters
        self.enable_multi_resolution = rospy.get_param("~enable_multi_resolution", True)
        self.scale_factors = rospy.get_param("~scale_factors", [0.5, 1.0, 1.5, 2.0])
        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.1)
        self.distance_estimation_enabled = rospy.get_param("~distance_estimation_enabled", True)
        
        # dynamic parameter
        self.detection_freq = DTParam(
            "~detection_freq", default=-1, param_type=ParamType.INT, min_value=-1, max_value=30
        )
        self._detection_reminder = DTReminder(frequency=self.detection_freq.value)
        # camera info
        self._camera_parameters = None
        self._mapx, self._mapy = None, None
        # create detector object
        self._detectors = [
            Detector(
                families=self.family,
                nthreads=self.nthreads,
                quad_decimate=self.quad_decimate,
                quad_sigma=self.quad_sigma,
                refine_edges=self.refine_edges,
                decode_sharpening=self.decode_sharpening,
            )
            for _ in range(self.ndetectors)
        ]
        
        # Create multi-resolution detectors
        self._multi_res_detectors = []
        if self.enable_multi_resolution:
            for detector in self._detectors:
                multi_res_detector = MultiResolutionDetector(
                    detector, 
                    scale_factors=self.scale_factors,
                    confidence_threshold=self.confidence_threshold
                )
                self._multi_res_detectors.append(multi_res_detector)
            rospy.loginfo(f"[AprilTagDetector] Multi-resolution detection enabled with scales: {self.scale_factors}")
        else:
            rospy.loginfo("[AprilTagDetector] Multi-resolution detection disabled, using standard detection")
        
        self._renderer_busy = False
        # create a CV bridge object
        self._jpeg = TurboJPEG()
        # create subscribers
        self._img_sub = rospy.Subscriber(
            "~image", CompressedImage, self._img_cb, queue_size=1, buff_size="20MB"
        )
        self._cinfo_sub = rospy.Subscriber("~camera_info", CameraInfo, self._cinfo_cb, queue_size=1)
        # create publisher
        self._tag_pub = rospy.Publisher(
            "~detections",
            AprilTagDetectionArray,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION,
            dt_help="Tag detections",
        )
        self._img_pub = rospy.Publisher(
            "~detections/image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="Camera image with tag publishs superimposed",
        )
        # create thread pool
        self._workers = ThreadPoolExecutor(self.ndetectors)
        self._tasks = [None] * self.ndetectors
        # create TF broadcaster
        self._tf_bcaster = tf.TransformBroadcaster()
        
        # Performance monitoring
        self._detection_count = 0
        self._total_processing_time = 0.0
        self._last_performance_log = time.time()

    def on_shutdown(self):
        self.loginfo("Shutting down workers pool")
        self._workers.shutdown()

    def _cinfo_cb(self, msg):
        # create mapx and mapy
        H, W = msg.height, msg.width
        # create new camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        # find optimal rectified pinhole camera
        with self.profiler("/cb/camera_info/get_optimal_new_camera_matrix"):
            rect_K, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_model.K, self.camera_model.D, (W, H), self.rectify_alpha
            )
            # store new camera parameters
            self._camera_parameters = (rect_K[0, 0], rect_K[1, 1], rect_K[0, 2], rect_K[1, 2])
        # create rectification map
        with self.profiler("/cb/camera_info/init_undistort_rectify_map"):
            self._mapx, self._mapy = cv2.initUndistortRectifyMap(
                self.camera_model.K, self.camera_model.D, None, rect_K, (W, H), cv2.CV_32FC1
            )
        # once we got the camera info, we can stop the subscriber
        self.loginfo("Camera info message received. Unsubscribing from camera_info topic.")
        # noinspection PyBroadException
        try:
            self._cinfo_sub.shutdown()
        except BaseException:
            pass

    def _detect(self, detector_id, msg):
        detection_start_time = time.time()
        
        # turn image message into grayscale image
        with self.profiler("/cb/image/decode"):
            img = self._jpeg.decode(msg.data, pixel_format=TJPF_GRAY)
            
        rospy.logdebug(f"[AprilTagDetector] Image decoded: {img.shape}, timestamp: {msg.header.stamp}")
        
        # run input image through the rectification map
        with self.profiler("/cb/image/rectify"):
            img = cv2.remap(img, self._mapx, self._mapy, cv2.INTER_NEAREST)
            
        rospy.logdebug(f"[AprilTagDetector] Image rectified: {img.shape}")
        
        # detect tags using multi-resolution or standard detection
        with self.profiler("/cb/image/detection"):
            if self.enable_multi_resolution and detector_id < len(self._multi_res_detectors):
                tags = self._multi_res_detectors[detector_id].detect_multi_scale(
                    img, self._camera_parameters, self.tag_size
                )
                rospy.logdebug(f"[AprilTagDetector] Multi-resolution detection completed: {len(tags)} tags found")
            else:
                tags = self._detectors[detector_id].detect(img, True, self._camera_parameters, self.tag_size)
                rospy.logdebug(f"[AprilTagDetector] Standard detection completed: {len(tags)} tags found")
        
        # pack detections into a message
        tags_msg = AprilTagDetectionArray()
        tags_msg.header.stamp = msg.header.stamp
        tags_msg.header.frame_id = msg.header.frame_id
        
        for i, tag in enumerate(tags):
            # turn rotation matrix into quaternion
            q = _matrix_to_quaternion(tag.pose_R)
            p = tag.pose_t.T[0]
            
            # Calculate distance and approach angle if enabled
            if self.distance_estimation_enabled and self.enable_multi_resolution and detector_id < len(self._multi_res_detectors):
                distance, approach_angle, estimation_quality = self._multi_res_detectors[detector_id].estimate_distance_and_angle(
                    tag, self._camera_parameters, self.tag_size
                )
                rospy.logdebug(f"[AprilTagDetector] Tag {tag.tag_id}: Distance={distance:.3f}m, Angle={math.degrees(approach_angle):.1f}°, Quality={estimation_quality:.3f}")
            else:
                distance = np.linalg.norm(p)
                approach_angle = 0.0
                estimation_quality = tag.decision_margin
            
            # create single tag detection object
            detection = AprilTagDetection(
                transform=Transform(
                    translation=Vector3(x=p[0], y=p[1], z=p[2]),
                    rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                ),
                tag_id=tag.tag_id,
                tag_family=str(tag.tag_family),
                hamming=tag.hamming,
                decision_margin=tag.decision_margin,
                homography=tag.homography.flatten().astype(np.float32).tolist(),
                center=tag.center.tolist(),
                corners=tag.corners.flatten().tolist(),
                pose_error=tag.pose_err,
            )
            
            # add detection to array
            tags_msg.detections.append(detection)
            
            # publish tf
            self._tf_bcaster.sendTransform(
                p.tolist(),
                q.tolist(),
                msg.header.stamp,
                "tag/{:s}".format(str(tag.tag_id)),
                msg.header.frame_id,
            )
            
            rospy.logdebug(f"[AprilTagDetector] Detection {i}: Tag ID={tag.tag_id}, Confidence={tag.decision_margin:.4f}, Distance={distance:.3f}m")
        
        # publish detections
        self._tag_pub.publish(tags_msg)
        
        # update healthy frequency metadata
        self._tag_pub.set_healthy_freq(self._img_sub.get_frequency())
        self._img_pub.set_healthy_freq(self._img_sub.get_frequency())
        
        # Performance monitoring and logging
        detection_time = time.time() - detection_start_time
        self._detection_count += 1
        self._total_processing_time += detection_time
        
        rospy.logdebug(f"[AprilTagDetector] Detection processing time: {detection_time:.4f}s")
        
        # Real-time monitoring - log performance every 30 detections
        if self._detection_count % 30 == 0:
            avg_time = self._total_processing_time / self._detection_count
            current_time = time.time()
            time_since_last_log = current_time - self._last_performance_log
            
            rospy.loginfo(f"[AprilTagDetector] Performance summary after {self._detection_count} detections:")
            rospy.loginfo(f"[AprilTagDetector] Average processing time: {avg_time:.4f}s")
            rospy.loginfo(f"[AprilTagDetector] Detection frequency: {30.0/time_since_last_log:.2f} Hz")
            
            if self.enable_multi_resolution and self._multi_res_detectors:
                metrics = self._multi_res_detectors[0].get_detection_metrics()
                if metrics:
                    rospy.loginfo(f"[AprilTagDetector] Multi-resolution metrics: {metrics}")
            
            self._last_performance_log = current_time
        
        # render visualization (if needed)
        if self._img_pub.anybody_listening() and not self._renderer_busy:
            self._renderer_busy = True
            Thread(target=self._render_detections, args=(msg, img, tags)).start()

    def _img_cb(self, msg):
        # make sure we have received camera info
        if self._camera_parameters is None:
            return
        # make sure we have a rectification map available
        if self._mapx is None or self._mapy is None:
            return
        # make sure somebody wants this
        if (not self._img_pub.anybody_listening()) and (not self._tag_pub.anybody_listening()):
            return
        # make sure this is a good time to detect (always keep this as last check)
        if not self._detection_reminder.is_time(frequency=self.detection_freq.value):
            return
        # make sure we are still running
        if self.is_shutdown:
            return
        # ---
        # find the first available worker (if any)
        for i in range(self.ndetectors):
            if self._tasks[i] is None or self._tasks[i].done():
                # submit this image to the pool
                self._tasks[i] = self._workers.submit(self._detect, i, msg)
                break

    def _render_detections(self, msg, img, detections):
        with self.profiler("/publishs_image"):
            # get a color buffer from the BW image
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            # draw each tag
            for detection in detections:
                for idx in range(len(detection.corners)):
                    cv2.line(
                        img,
                        tuple(detection.corners[idx - 1, :].astype(int)),
                        tuple(detection.corners[idx, :].astype(int)),
                        (0, 255, 0),
                    )
                # draw the tag ID
                cv2.putText(
                    img,
                    str(detection.tag_id),
                    org=(detection.corners[0, 0].astype(int) + 10, detection.corners[0, 1].astype(int) + 10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255),
                )
            # pack image into a message
            img_msg = CompressedImage()
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = msg.header.frame_id
            img_msg.format = "jpeg"
            img_msg.data = self._jpeg.encode(img)
        # ---
        self._img_pub.publish(img_msg)
        self._renderer_busy = False


def _matrix_to_quaternion(r):
    T = np.array(((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 1)), dtype=np.float64)
    T[0:3, 0:3] = r
    return tf.transformations.quaternion_from_matrix(T)


if __name__ == "__main__":
    node = AprilTagDetector()
    # spin forever
    rospy.spin()

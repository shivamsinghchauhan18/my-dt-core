#!/usr/bin/env python3

import os
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from duckietown_msgs.msg import ObjectDetectionArray
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from datetime import datetime

from optimized_yolo_detector import OptimizedYOLODetector


class EnhancedVehicleDetectionNode(DTROS):
    """
    Enhanced vehicle detection node using YOLOv5 for comprehensive object detection.
    
    This node extends the basic vehicle detection capabilities with advanced deep learning-based
    object detection, supporting multiple Duckietown object classes with real-time performance
    monitoring and comprehensive logging.
    
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~model_path (:obj:`str`): Path to YOLO model file
        ~confidence_threshold (:obj:`float`): Minimum confidence for detections
        ~nms_threshold (:obj:`float`): Non-maximum suppression threshold
        ~device (:obj:`str`): Device to run inference on ('cpu', 'cuda', 'auto')
        ~tensorrt_enabled (:obj:`bool`): Whether to use TensorRT optimization
        ~max_detections (:obj:`int`): Maximum number of detections per frame
        ~process_frequency (:obj:`float`): Frequency at which to process images

    Subscriber:
        ~image (:obj:`sensor_msgs.msg.CompressedImage`): Input image

    Publishers:
        ~detections (:obj:`duckietown_msgs.msg.ObjectDetectionArray`): Detected objects
        ~debug/detection_image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Debug image with detections
        ~performance_status (:obj:`std_msgs.msg.String`): Performance monitoring information
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(EnhancedVehicleDetectionNode, self).__init__(
            node_name=node_name, 
            node_type=NodeType.PERCEPTION
        )

        # Initialize parameters
        self.model_path = DTParam("~model_path", param_type=ParamType.STRING, default="yolov5s.pt")
        self.confidence_threshold = DTParam("~confidence_threshold", param_type=ParamType.FLOAT, default=0.6)
        self.nms_threshold = DTParam("~nms_threshold", param_type=ParamType.FLOAT, default=0.45)
        self.device = DTParam("~device", param_type=ParamType.STRING, default="auto")
        self.tensorrt_enabled = DTParam("~tensorrt_enabled", param_type=ParamType.BOOL, default=True)
        self.max_detections = DTParam("~max_detections", param_type=ParamType.INT, default=100)
        self.process_frequency = DTParam("~process_frequency", param_type=ParamType.FLOAT, default=10.0)

        # Initialize components
        self.bridge = CvBridge()
        self.last_stamp = rospy.Time.now()
        self.frame_count = 0
        
        # Calculate processing duration
        self.publish_duration = rospy.Duration.from_sec(1.0 / self.process_frequency.value)
        
        rospy.loginfo(f"[Enhanced Vehicle Detection] Initializing with parameters:")
        rospy.loginfo(f"  - model_path: {self.model_path.value}")
        rospy.loginfo(f"  - confidence_threshold: {self.confidence_threshold.value}")
        rospy.loginfo(f"  - nms_threshold: {self.nms_threshold.value}")
        rospy.loginfo(f"  - device: {self.device.value}")
        rospy.loginfo(f"  - tensorrt_enabled: {self.tensorrt_enabled.value}")
        rospy.loginfo(f"  - max_detections: {self.max_detections.value}")
        rospy.loginfo(f"  - process_frequency: {self.process_frequency.value} Hz")

        # Initialize YOLO detector
        try:
            rospy.loginfo("[Enhanced Vehicle Detection] Initializing YOLO detector...")
            self.yolo_detector = OptimizedYOLODetector(
                model_path=self.model_path.value,
                confidence_threshold=self.confidence_threshold.value,
                nms_threshold=self.nms_threshold.value,
                device=self.device.value,
                tensorrt_enabled=self.tensorrt_enabled.value,
                max_detections=self.max_detections.value
            )
            rospy.loginfo("[Enhanced Vehicle Detection] YOLO detector initialized successfully")
        except Exception as e:
            rospy.logerr(f"[Enhanced Vehicle Detection] Failed to initialize YOLO detector: {e}")
            raise

        # ROS Subscribers
        self.sub_image = rospy.Subscriber(
            "~image", 
            CompressedImage, 
            self.cb_image, 
            queue_size=1
        )

        # ROS Publishers
        self.pub_detections = rospy.Publisher(
            "~detections", 
            ObjectDetectionArray, 
            queue_size=1
        )
        
        self.pub_debug_image = rospy.Publisher(
            "~debug/detection_image/compressed", 
            CompressedImage, 
            queue_size=1
        )
        
        self.pub_performance = rospy.Publisher(
            "~performance_status", 
            String, 
            queue_size=1
        )

        # Performance monitoring timer
        self.performance_timer = rospy.Timer(
            rospy.Duration(5.0),  # Every 5 seconds
            self.cb_performance_monitor
        )

        rospy.loginfo("[Enhanced Vehicle Detection] Node initialization completed successfully")

    def cb_image(self, image_msg):
        """
        Callback for processing images with YOLO object detection.
        
        Processes the image only if sufficient time has passed since processing the previous
        image (relative to the chosen processing frequency).

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): Input image
        """
        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now

        self.frame_count += 1
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        rospy.logdebug(f"[Enhanced Vehicle Detection] Processing frame {self.frame_count} at {timestamp}")

        try:
            # Convert ROS image to OpenCV format
            image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            rospy.logdebug(f"[Enhanced Vehicle Detection] Image converted: {image_cv.shape}")

            # Perform YOLO detection
            detection_array = self.yolo_detector.detect_objects(image_cv, image_msg.header.stamp)
            
            # Update header information
            detection_array.header.frame_id = image_msg.header.frame_id
            
            # Publish detection results
            self.pub_detections.publish(detection_array)
            
            # Log detection summary
            num_objects = detection_array.total_objects
            avg_confidence = detection_array.average_confidence
            high_risk_count = detection_array.high_risk_objects_count
            processing_time = detection_array.processing_time
            fps = detection_array.inference_fps
            
            rospy.loginfo(f"[Enhanced Vehicle Detection] Frame {self.frame_count}: "
                         f"{num_objects} objects detected, avg_conf={avg_confidence:.3f}, "
                         f"high_risk={high_risk_count}, fps={fps:.1f}, "
                         f"processing_time={processing_time:.4f}s")

            # Generate and publish debug image if there are subscribers
            if self.pub_debug_image.get_num_connections() > 0:
                debug_image = self._create_debug_image(image_cv, detection_array)
                debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_image)
                debug_msg.header = image_msg.header
                self.pub_debug_image.publish(debug_msg)
                rospy.logdebug(f"[Enhanced Vehicle Detection] Debug image published for frame {self.frame_count}")

        except Exception as e:
            rospy.logerr(f"[Enhanced Vehicle Detection] Error processing frame {self.frame_count}: {e}")
            # Publish empty detection array on error
            empty_array = ObjectDetectionArray()
            empty_array.header = image_msg.header
            empty_array.detections = []
            empty_array.total_objects = 0
            empty_array.processing_time = 0.0
            empty_array.inference_fps = 0.0
            empty_array.average_confidence = 0.0
            empty_array.high_risk_objects_count = 0
            self.pub_detections.publish(empty_array)

    def _create_debug_image(self, image, detection_array):
        """
        Create debug visualization image with bounding boxes and labels.
        
        Args:
            image: Original OpenCV image
            detection_array: ObjectDetectionArray with detections
            
        Returns:
            OpenCV image with visualization overlays
        """
        import cv2
        debug_image = image.copy()
        
        # Color mapping for risk levels
        risk_colors = {
            0: (0, 255, 0),    # Low - Green
            1: (0, 255, 255),  # Medium - Yellow
            2: (0, 165, 255),  # High - Orange
            3: (0, 0, 255)     # Critical - Red
        }
        
        for detection in detection_array.detections:
            # Extract bounding box coordinates
            x1 = int(detection.top_left.x)
            y1 = int(detection.top_left.y)
            x2 = int(detection.bottom_right.x)
            y2 = int(detection.bottom_right.y)
            
            # Get color based on risk level
            color = risk_colors.get(detection.risk_level, (255, 255, 255))
            
            # Draw bounding box
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
            
            # Create label text
            label = f"{detection.class_name} {detection.confidence:.2f}"
            distance_text = f"{detection.distance:.1f}m"
            risk_text = f"Risk:{detection.risk_level}"
            
            # Calculate text size and position
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            
            (label_w, label_h), _ = cv2.getTextSize(label, font, font_scale, thickness)
            (dist_w, dist_h), _ = cv2.getTextSize(distance_text, font, font_scale, thickness)
            (risk_w, risk_h), _ = cv2.getTextSize(risk_text, font, font_scale, thickness)
            
            # Draw background rectangles for text
            cv2.rectangle(debug_image, (x1, y1 - label_h - 5), (x1 + label_w, y1), color, -1)
            cv2.rectangle(debug_image, (x1, y2), (x1 + max(dist_w, risk_w), y2 + dist_h + risk_h + 10), color, -1)
            
            # Draw text
            cv2.putText(debug_image, label, (x1, y1 - 5), font, font_scale, (0, 0, 0), thickness)
            cv2.putText(debug_image, distance_text, (x1, y2 + dist_h + 5), font, font_scale, (0, 0, 0), thickness)
            cv2.putText(debug_image, risk_text, (x1, y2 + dist_h + risk_h + 10), font, font_scale, (0, 0, 0), thickness)
        
        # Add frame information
        frame_info = f"Frame: {self.frame_count} | Objects: {detection_array.total_objects} | FPS: {detection_array.inference_fps:.1f}"
        cv2.putText(debug_image, frame_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return debug_image

    def cb_performance_monitor(self, event):
        """
        Timer callback for performance monitoring and reporting.
        
        Args:
            event: ROS timer event
        """
        try:
            # Get performance metrics from YOLO detector
            metrics = self.yolo_detector.get_performance_metrics()
            
            # Create performance status message
            timestamp = datetime.now().strftime("%H:%M:%S")
            status_msg = (
                f"[{timestamp}] Enhanced Vehicle Detection Performance: "
                f"frames_processed={self.frame_count}, "
                f"current_fps={metrics.fps:.1f}, "
                f"inference_time={metrics.inference_time:.4f}s, "
                f"total_time={metrics.total_time:.4f}s, "
                f"memory_usage={metrics.memory_usage_mb:.1f}MB, "
                f"cpu_usage={metrics.cpu_usage:.1f}%, "
                f"gpu_utilization={metrics.gpu_utilization:.1f}%"
            )
            
            # Publish performance status
            performance_msg = String()
            performance_msg.data = status_msg
            self.pub_performance.publish(performance_msg)
            
            # Log performance information
            rospy.loginfo(f"[Enhanced Vehicle Detection] {status_msg}")
            
            # Log detailed breakdown
            rospy.logdebug(f"[Enhanced Vehicle Detection] Performance breakdown: "
                          f"preprocess={metrics.preprocessing_time:.4f}s, "
                          f"inference={metrics.inference_time:.4f}s, "
                          f"postprocess={metrics.postprocessing_time:.4f}s")
            
        except Exception as e:
            rospy.logwarn(f"[Enhanced Vehicle Detection] Performance monitoring error: {e}")

    def on_shutdown(self):
        """Clean shutdown callback."""
        rospy.loginfo("[Enhanced Vehicle Detection] Shutting down node...")
        
        # Stop performance monitoring
        if hasattr(self, 'performance_timer'):
            self.performance_timer.shutdown()
        
        # Shutdown YOLO detector
        if hasattr(self, 'yolo_detector'):
            self.yolo_detector.shutdown()
        
        rospy.loginfo("[Enhanced Vehicle Detection] Node shutdown complete")


if __name__ == "__main__":
    try:
        enhanced_vehicle_detection_node = EnhancedVehicleDetectionNode("enhanced_vehicle_detection")
        rospy.on_shutdown(enhanced_vehicle_detection_node.on_shutdown)
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"[Enhanced Vehicle Detection] Node failed to start: {e}")
        raise
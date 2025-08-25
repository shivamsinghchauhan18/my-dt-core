#!/usr/bin/env python3

import time
import logging
from typing import List, Dict, Tuple, Optional
import numpy as np
import cv2
import torch
from ultralytics import YOLO
import psutil
import threading
from dataclasses import dataclass
from datetime import datetime

import rospy
from geometry_msgs.msg import Point32, Vector3
from duckietown_msgs.msg import ObjectDetection, ObjectDetectionArray


@dataclass
class PerformanceMetrics:
    """Performance metrics for YOLO detection"""
    inference_time: float
    preprocessing_time: float
    postprocessing_time: float
    total_time: float
    fps: float
    memory_usage_mb: float
    gpu_utilization: float
    cpu_usage: float


class OptimizedYOLODetector:
    """
    Optimized YOLO-based object detector with TensorRT support for Duckietown environments.
    
    This class provides real-time object detection with custom Duckietown object classes,
    performance monitoring, and comprehensive logging capabilities.
    """
    
    # Duckietown-specific object classes
    DUCKIETOWN_CLASSES = {
        0: 'duckie',
        1: 'duckiebot',
        2: 'traffic_cone',
        3: 'stop_sign',
        4: 'intersection_sign',
        5: 'pedestrian',
        6: 'vehicle',
        7: 'barrier'
    }
    
    def __init__(self, 
                 model_path: str = 'yolov5s.pt',
                 confidence_threshold: float = 0.6,
                 nms_threshold: float = 0.45,
                 device: str = 'auto',
                 tensorrt_enabled: bool = True,
                 max_detections: int = 100):
        """
        Initialize the OptimizedYOLODetector.
        
        Args:
            model_path: Path to YOLO model file
            confidence_threshold: Minimum confidence for detections
            nms_threshold: Non-maximum suppression threshold
            device: Device to run inference on ('cpu', 'cuda', 'auto')
            tensorrt_enabled: Whether to use TensorRT optimization
            max_detections: Maximum number of detections per frame
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_detections = max_detections
        self.tensorrt_enabled = tensorrt_enabled
        
        # Performance monitoring
        self.performance_metrics = PerformanceMetrics(0, 0, 0, 0, 0, 0, 0, 0)
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.detection_history = []
        
        # Logging setup
        self.logger = logging.getLogger(f'{__name__}.OptimizedYOLODetector')
        self.logger.setLevel(logging.DEBUG)
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Initialize device
        self.device = self._setup_device(device)
        rospy.loginfo(f"[YOLO] Device setup complete: {self.device}")
        
        # Load and optimize model
        self.model = self._load_model()
        rospy.loginfo(f"[YOLO] Model loading complete: {self.model_path}")
        
        # Performance monitoring thread
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._performance_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        rospy.loginfo("[YOLO] OptimizedYOLODetector initialization complete")
    
    def _setup_device(self, device: str) -> str:
        """Setup and validate the compute device."""
        start_time = time.time()
        
        if device == 'auto':
            if torch.cuda.is_available():
                device = 'cuda'
                gpu_name = torch.cuda.get_device_name(0)
                gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
                rospy.loginfo(f"[YOLO] Auto-selected CUDA device: {gpu_name} ({gpu_memory:.1f}GB)")
            else:
                device = 'cpu'
                rospy.logwarn("[YOLO] CUDA not available, falling back to CPU")
        
        setup_time = time.time() - start_time
        rospy.logdebug(f"[YOLO] Device setup time: {setup_time:.3f}s, Selected device: {device}")
        
        return device
    
    def _load_model(self) -> YOLO:
        """Load and optimize the YOLO model."""
        load_start = time.time()
        
        try:
            rospy.loginfo(f"[YOLO] Loading model from: {self.model_path}")
            model = YOLO(self.model_path)
            
            # Move model to device
            model.to(self.device)
            rospy.loginfo(f"[YOLO] Model moved to device: {self.device}")
            
            # TensorRT optimization
            if self.tensorrt_enabled and self.device == 'cuda':
                try:
                    rospy.loginfo("[YOLO] Attempting TensorRT optimization...")
                    # Export to TensorRT format
                    tensorrt_path = self.model_path.replace('.pt', '_tensorrt.engine')
                    model.export(format='engine', device=self.device)
                    rospy.loginfo(f"[YOLO] TensorRT optimization successful: {tensorrt_path}")
                except Exception as e:
                    rospy.logwarn(f"[YOLO] TensorRT optimization failed: {e}")
                    rospy.loginfo("[YOLO] Continuing with standard PyTorch model")
            
            # Warm up the model
            rospy.loginfo("[YOLO] Warming up model...")
            dummy_input = torch.randn(1, 3, 640, 640).to(self.device)
            with torch.no_grad():
                _ = model.predict(dummy_input, verbose=False)
            
            load_time = time.time() - load_start
            rospy.loginfo(f"[YOLO] Model loading and optimization complete in {load_time:.3f}s")
            
            return model
            
        except Exception as e:
            rospy.logerr(f"[YOLO] Failed to load model: {e}")
            raise
    
    def detect_objects(self, image: np.ndarray, timestamp: rospy.Time) -> ObjectDetectionArray:
        """
        Detect objects in the input image.
        
        Args:
            image: Input image as numpy array (BGR format)
            timestamp: ROS timestamp for the detection
            
        Returns:
            ObjectDetectionArray message with detected objects
        """
        detection_start = time.time()
        
        with self.lock:
            self.frame_count += 1
            frame_id = self.frame_count
        
        rospy.logdebug(f"[YOLO] Starting detection for frame {frame_id} at {timestamp}")
        
        # Preprocessing
        preprocess_start = time.time()
        processed_image = self._preprocess_image(image)
        preprocess_time = time.time() - preprocess_start
        
        rospy.logdebug(f"[YOLO] Frame {frame_id} preprocessing time: {preprocess_time:.4f}s")
        
        # Inference
        inference_start = time.time()
        try:
            results = self.model.predict(
                processed_image,
                conf=self.confidence_threshold,
                iou=self.nms_threshold,
                max_det=self.max_detections,
                verbose=False
            )
            inference_time = time.time() - inference_start
            
            rospy.logdebug(f"[YOLO] Frame {frame_id} inference time: {inference_time:.4f}s")
            
        except Exception as e:
            rospy.logerr(f"[YOLO] Inference failed for frame {frame_id}: {e}")
            return self._create_empty_detection_array(timestamp)
        
        # Postprocessing
        postprocess_start = time.time()
        detection_array = self._postprocess_results(results, image.shape, timestamp, frame_id)
        postprocess_time = time.time() - postprocess_start
        
        total_time = time.time() - detection_start
        fps = 1.0 / total_time if total_time > 0 else 0.0
        
        # Update performance metrics
        self._update_performance_metrics(
            inference_time, preprocess_time, postprocess_time, total_time, fps
        )
        
        # Log detection summary
        num_detections = len(detection_array.detections)
        avg_confidence = detection_array.average_confidence
        high_risk_count = detection_array.high_risk_objects_count
        
        rospy.loginfo(f"[YOLO] Frame {frame_id}: {num_detections} objects detected, "
                     f"avg_conf={avg_confidence:.3f}, high_risk={high_risk_count}, "
                     f"fps={fps:.1f}, total_time={total_time:.4f}s")
        
        # Log individual detections
        for i, detection in enumerate(detection_array.detections):
            rospy.logdebug(f"[YOLO] Frame {frame_id} Detection {i}: "
                          f"class={detection.class_name}, conf={detection.confidence:.3f}, "
                          f"bbox=({detection.top_left.x:.1f},{detection.top_left.y:.1f})-"
                          f"({detection.bottom_right.x:.1f},{detection.bottom_right.y:.1f}), "
                          f"dist={detection.distance:.2f}m, risk={detection.risk_level}")
        
        return detection_array
    
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for YOLO inference."""
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return rgb_image
    
    def _postprocess_results(self, results, image_shape: Tuple[int, int, int], 
                           timestamp: rospy.Time, frame_id: int) -> ObjectDetectionArray:
        """Convert YOLO results to ROS ObjectDetectionArray message."""
        detection_array = ObjectDetectionArray()
        detection_array.header.stamp = timestamp
        detection_array.header.frame_id = f"camera_frame_{frame_id}"
        
        detections = []
        total_confidence = 0.0
        high_risk_count = 0
        
        if results and len(results) > 0:
            result = results[0]  # First (and only) result
            
            if result.boxes is not None:
                boxes = result.boxes.cpu().numpy()
                
                for i, box in enumerate(boxes):
                    detection = ObjectDetection()
                    detection.header = detection_array.header
                    
                    # Extract box data
                    x1, y1, x2, y2 = box.xyxy[0]
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    
                    # Set bounding box
                    detection.top_left = Point32(x=float(x1), y=float(y1), z=0.0)
                    detection.bottom_right = Point32(x=float(x2), y=float(y2), z=0.0)
                    
                    # Set classification
                    detection.class_name = self.DUCKIETOWN_CLASSES.get(class_id, f'unknown_{class_id}')
                    detection.confidence = confidence
                    
                    # Estimate distance (simple heuristic based on bounding box size)
                    bbox_area = (x2 - x1) * (y2 - y1)
                    image_area = image_shape[0] * image_shape[1]
                    relative_size = bbox_area / image_area
                    detection.distance = self._estimate_distance(relative_size, detection.class_name)
                    
                    # Set relative velocity (placeholder - would need tracking for real values)
                    detection.relative_velocity = Vector3(x=0.0, y=0.0, z=0.0)
                    
                    # Assess risk level
                    detection.risk_level = self._assess_risk_level(
                        detection.distance, detection.class_name, confidence
                    )
                    
                    # Set metadata
                    detection.processing_time = self.performance_metrics.total_time
                    detection.is_tracked = False  # Would be True if using tracking
                    
                    detections.append(detection)
                    total_confidence += confidence
                    
                    if detection.risk_level >= ObjectDetection.RISK_HIGH:
                        high_risk_count += 1
                    
                    rospy.logdebug(f"[YOLO] Processed detection {i}: {detection.class_name} "
                                  f"conf={confidence:.3f} dist={detection.distance:.2f}m "
                                  f"risk={detection.risk_level}")
        
        detection_array.detections = detections
        detection_array.total_objects = len(detections)
        detection_array.processing_time = self.performance_metrics.total_time
        detection_array.inference_fps = self.performance_metrics.fps
        detection_array.average_confidence = total_confidence / len(detections) if detections else 0.0
        detection_array.high_risk_objects_count = high_risk_count
        
        return detection_array
    
    def _estimate_distance(self, relative_size: float, class_name: str) -> float:
        """Estimate distance to object based on relative size in image."""
        # Simple heuristic distance estimation
        # In a real implementation, this would use camera calibration and known object sizes
        
        # Typical object sizes in meters (rough estimates)
        object_sizes = {
            'duckie': 0.1,
            'duckiebot': 0.3,
            'traffic_cone': 0.2,
            'stop_sign': 0.15,
            'intersection_sign': 0.15,
            'pedestrian': 1.7,
            'vehicle': 2.0,
            'barrier': 1.0
        }
        
        typical_size = object_sizes.get(class_name, 0.5)
        
        # Distance estimation based on relative size
        # Larger relative size = closer object
        if relative_size > 0:
            estimated_distance = typical_size / (relative_size ** 0.5) * 2.0
            return max(0.1, min(10.0, estimated_distance))  # Clamp between 0.1m and 10m
        else:
            return 5.0  # Default distance
    
    def _assess_risk_level(self, distance: float, class_name: str, confidence: float) -> int:
        """Assess collision risk level based on object properties."""
        # Risk assessment based on distance, object type, and confidence
        
        # Distance-based risk
        if distance < 0.5:
            distance_risk = ObjectDetection.RISK_CRITICAL
        elif distance < 1.0:
            distance_risk = ObjectDetection.RISK_HIGH
        elif distance < 2.0:
            distance_risk = ObjectDetection.RISK_MEDIUM
        else:
            distance_risk = ObjectDetection.RISK_LOW
        
        # Object type risk multiplier
        high_risk_objects = ['duckiebot', 'vehicle', 'pedestrian', 'barrier']
        medium_risk_objects = ['traffic_cone', 'stop_sign']
        
        if class_name in high_risk_objects:
            type_risk_multiplier = 1.0
        elif class_name in medium_risk_objects:
            type_risk_multiplier = 0.7
        else:
            type_risk_multiplier = 0.5
        
        # Confidence-based adjustment
        confidence_multiplier = confidence  # Lower confidence = lower effective risk
        
        # Calculate final risk
        final_risk_score = distance_risk * type_risk_multiplier * confidence_multiplier
        
        if final_risk_score >= 2.5:
            return ObjectDetection.RISK_CRITICAL
        elif final_risk_score >= 1.5:
            return ObjectDetection.RISK_HIGH
        elif final_risk_score >= 0.8:
            return ObjectDetection.RISK_MEDIUM
        else:
            return ObjectDetection.RISK_LOW
    
    def _create_empty_detection_array(self, timestamp: rospy.Time) -> ObjectDetectionArray:
        """Create an empty detection array for error cases."""
        detection_array = ObjectDetectionArray()
        detection_array.header.stamp = timestamp
        detection_array.header.frame_id = "camera_frame"
        detection_array.detections = []
        detection_array.total_objects = 0
        detection_array.processing_time = 0.0
        detection_array.inference_fps = 0.0
        detection_array.average_confidence = 0.0
        detection_array.high_risk_objects_count = 0
        return detection_array
    
    def _update_performance_metrics(self, inference_time: float, preprocess_time: float,
                                  postprocess_time: float, total_time: float, fps: float):
        """Update performance metrics with thread safety."""
        with self.lock:
            self.total_inference_time += inference_time
            
            # Update current metrics
            self.performance_metrics.inference_time = inference_time
            self.performance_metrics.preprocessing_time = preprocess_time
            self.performance_metrics.postprocessing_time = postprocess_time
            self.performance_metrics.total_time = total_time
            self.performance_metrics.fps = fps
            
            # Update system metrics
            process = psutil.Process()
            self.performance_metrics.memory_usage_mb = process.memory_info().rss / 1024 / 1024
            self.performance_metrics.cpu_usage = process.cpu_percent()
            
            # GPU utilization (if available)
            if torch.cuda.is_available():
                self.performance_metrics.gpu_utilization = torch.cuda.utilization()
            else:
                self.performance_metrics.gpu_utilization = 0.0
    
    def _performance_monitor(self):
        """Background thread for performance monitoring and logging."""
        while self.monitoring_active:
            try:
                time.sleep(5.0)  # Log every 5 seconds
                
                with self.lock:
                    metrics = self.performance_metrics
                    frame_count = self.frame_count
                    avg_inference_time = self.total_inference_time / max(1, frame_count)
                
                timestamp = datetime.now().strftime("%H:%M:%S")
                rospy.loginfo(f"[YOLO] Performance Monitor [{timestamp}]: "
                             f"frames={frame_count}, avg_fps={metrics.fps:.1f}, "
                             f"avg_inference={avg_inference_time:.4f}s, "
                             f"memory={metrics.memory_usage_mb:.1f}MB, "
                             f"cpu={metrics.cpu_usage:.1f}%, "
                             f"gpu={metrics.gpu_utilization:.1f}%")
                
            except Exception as e:
                rospy.logwarn(f"[YOLO] Performance monitoring error: {e}")
    
    def get_performance_metrics(self) -> PerformanceMetrics:
        """Get current performance metrics."""
        with self.lock:
            return self.performance_metrics
    
    def shutdown(self):
        """Clean shutdown of the detector."""
        rospy.loginfo("[YOLO] Shutting down OptimizedYOLODetector...")
        self.monitoring_active = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=1.0)
        rospy.loginfo("[YOLO] OptimizedYOLODetector shutdown complete")
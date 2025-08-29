#!/usr/bin/env python3
"""
Utility functions for working with Duckietown messages.
"""

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Vector3
from duckietown_enhanced_msgs.msg import (
    ObjectDetection, ObjectDetectionArray, SafetyStatus, 
    AdvancedLanePose, LaneCurves, LaneCurve
)
from packages.image_processing.src.advanced_vision_utils import (
    RiskLevel, SafetyLevel, HealthStatus, BoundingBox, 
    VehicleState, ObjectDetection as ObjectDetectionData,
    SafetyStatus as SafetyStatusData
)


def create_header(frame_id: str = "base_link") -> Header:
    """Create a standard ROS header with current timestamp."""
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header


def bounding_box_to_points(bbox: BoundingBox) -> tuple:
    """Convert BoundingBox to ROS Point32 messages."""
    top_left = Point32()
    top_left.x = bbox.x
    top_left.y = bbox.y
    top_left.z = 0.0
    
    bottom_right = Point32()
    bottom_right.x = bbox.x + bbox.width
    bottom_right.y = bbox.y + bbox.height
    bottom_right.z = 0.0
    
    return top_left, bottom_right


def vector3d_to_ros(vector) -> Vector3:
    """Convert Vector3D to ROS Vector3 message."""
    ros_vector = Vector3()
    ros_vector.x = vector.x
    ros_vector.y = vector.y
    ros_vector.z = vector.z
    return ros_vector


def object_detection_to_msg(detection: ObjectDetectionData, frame_id: str = "camera") -> ObjectDetection:
    """Convert ObjectDetectionData to ROS ObjectDetection message."""
    msg = ObjectDetection()
    msg.header = create_header(frame_id)
    
    msg.class_name = detection.class_name
    msg.confidence = detection.confidence
    
    # Convert bounding box
    msg.top_left, msg.bottom_right = bounding_box_to_points(detection.bounding_box)
    
    msg.distance = detection.distance
    msg.relative_velocity = vector3d_to_ros(detection.relative_velocity)
    msg.risk_level = detection.risk_level.value
    msg.processing_time = detection.processing_time
    msg.is_tracked = detection.is_tracked
    
    return msg


def object_detection_array_to_msg(detections: list, frame_id: str = "camera") -> ObjectDetectionArray:
    """Convert list of ObjectDetectionData to ROS ObjectDetectionArray message."""
    msg = ObjectDetectionArray()
    msg.header = create_header(frame_id)
    
    # Convert individual detections
    msg.detections = [object_detection_to_msg(det, frame_id) for det in detections]
    
    # Calculate summary information
    msg.total_objects = len(detections)
    msg.processing_time = sum(det.processing_time for det in detections)
    msg.average_confidence = sum(det.confidence for det in detections) / len(detections) if detections else 0.0
    msg.high_risk_objects_count = sum(1 for det in detections if det.is_high_risk())
    
    # Estimate FPS (simplified)
    msg.inference_fps = 1.0 / msg.processing_time if msg.processing_time > 0 else 0.0
    
    return msg


def safety_status_to_msg(status: SafetyStatusData, frame_id: str = "base_link") -> SafetyStatus:
    """Convert SafetyStatusData to ROS SafetyStatus message."""
    msg = SafetyStatus()
    msg.header = create_header(frame_id)
    
    msg.safety_level = status.overall_level.value
    msg.hardware_health = status.hardware_health.value
    
    # Convert sensor status (simplified - take worst status)
    if status.sensor_status:
        worst_sensor_status = max(status.sensor_status.values(), key=lambda x: x.value)
        msg.camera_status = worst_sensor_status.value
        msg.imu_status = worst_sensor_status.value
        msg.encoder_status = worst_sensor_status.value
    else:
        msg.camera_status = HealthStatus.OK.value
        msg.imu_status = HealthStatus.OK.value
        msg.encoder_status = HealthStatus.OK.value
    
    # Algorithm performance
    msg.lane_detection_confidence = status.algorithm_performance.get('lane_detection', 1.0)
    msg.object_detection_fps = status.algorithm_performance.get('object_detection_fps', 30.0)
    msg.control_loop_frequency = status.algorithm_performance.get('control_loop_frequency', 50.0)
    
    msg.emergency_stop_active = status.emergency_stop_active
    msg.emergency_reason = status.active_warnings[0] if status.active_warnings else ""
    
    msg.active_warnings = status.active_warnings
    msg.active_alerts = []  # Could be separated from warnings in the future
    
    msg.system_health_score = status.system_health_score
    
    # Timestamps
    msg.last_safety_check = rospy.Time.now()
    msg.last_hardware_check = rospy.Time.now()
    
    return msg


def create_advanced_lane_pose(d: float, phi: float, curvature: float = 0.0, 
                            confidence: float = 1.0, frame_id: str = "base_link") -> AdvancedLanePose:
    """Create an AdvancedLanePose message with basic parameters."""
    msg = AdvancedLanePose()
    msg.header = create_header(frame_id)
    
    # Basic pose
    msg.d = d
    msg.phi = phi
    
    # Curve information
    msg.curvature = curvature
    msg.curve_radius = 1.0 / curvature if abs(curvature) > 1e-6 else float('inf')
    msg.curve_confidence = confidence
    
    # Quality metrics
    msg.detection_confidence = confidence
    msg.left_lane_confidence = confidence
    msg.right_lane_confidence = confidence
    
    # Temporal tracking
    msg.frames_tracked = 1
    msg.temporal_consistency = 1.0
    
    # Processing metadata
    msg.processing_time = 0.01  # Default 10ms
    msg.detection_method = "advanced_lane_detection"
    
    return msg


def create_lane_curve(coefficients: list, lane_type: int = 2, 
                     confidence: float = 1.0, frame_id: str = "camera") -> LaneCurve:
    """Create a LaneCurve message from polynomial coefficients."""
    msg = LaneCurve()
    msg.header = create_header(frame_id)
    
    # Ensure we have 4 coefficients (pad with zeros if needed)
    coeffs = list(coefficients) + [0.0] * (4 - len(coefficients))
    msg.a = coeffs[0] if len(coeffs) > 0 else 0.0
    msg.b = coeffs[1] if len(coeffs) > 1 else 0.0
    msg.c = coeffs[2] if len(coeffs) > 2 else 0.0
    msg.d = coeffs[3] if len(coeffs) > 3 else 0.0
    
    # Calculate curvature at x=0 (simplified)
    msg.curvature = abs(2 * msg.b) if abs(msg.b) > 1e-6 else 0.0
    msg.curve_radius = 1.0 / msg.curvature if msg.curvature > 1e-6 else float('inf')
    
    msg.confidence = confidence
    msg.lane_type = lane_type
    
    # Default values for other fields
    msg.arc_length = 10.0  # Default 10m
    msg.fit_quality = confidence
    msg.num_points = 50  # Default
    msg.start_x = 0.0
    msg.end_x = 10.0
    msg.max_deviation = 0.1  # 10cm
    
    return msg
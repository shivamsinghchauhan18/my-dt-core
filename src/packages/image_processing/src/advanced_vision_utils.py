#!/usr/bin/env python3
"""
Advanced vision utility classes for the Duckietown autonomous system.
Provides shared algorithms and data structures for computer vision tasks.
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional, Union
from dataclasses import dataclass
from enum import Enum
import time


class RiskLevel(Enum):
    """Risk level enumeration for object detection and safety assessment."""
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    CRITICAL = 3


class SafetyLevel(Enum):
    """Overall system safety level enumeration."""
    SAFE = 0
    WARNING = 1
    CRITICAL = 2
    EMERGENCY = 3


class HealthStatus(Enum):
    """Component health status enumeration."""
    OK = 0
    WARNING = 1
    CRITICAL = 2
    FAILED = 3


@dataclass
class Point3D:
    """3D point representation."""
    x: float
    y: float
    z: float


@dataclass
class Vector3D:
    """3D vector representation."""
    x: float
    y: float
    z: float


@dataclass
class BoundingBox:
    """2D bounding box representation."""
    x: float
    y: float
    width: float
    height: float
    
    @property
    def center(self) -> Tuple[float, float]:
        """Get the center point of the bounding box."""
        return (self.x + self.width / 2, self.y + self.height / 2)
    
    @property
    def area(self) -> float:
        """Get the area of the bounding box."""
        return self.width * self.height


@dataclass
class VehicleState:
    """Complete vehicle state representation."""
    position: Point3D
    velocity: Vector3D
    acceleration: Vector3D
    orientation: float  # Yaw angle in radians
    timestamp: float
    
    @classmethod
    def from_current_time(cls, position: Point3D, velocity: Vector3D, 
                         acceleration: Vector3D, orientation: float) -> 'VehicleState':
        """Create VehicleState with current timestamp."""
        return cls(position, velocity, acceleration, orientation, time.time())


@dataclass
class ObjectDetection:
    """Object detection result with risk assessment."""
    class_name: str
    confidence: float
    bounding_box: BoundingBox
    distance: float
    relative_velocity: Vector3D
    risk_level: RiskLevel
    processing_time: float
    is_tracked: bool = False
    
    def is_high_risk(self) -> bool:
        """Check if object poses high or critical risk."""
        return self.risk_level in [RiskLevel.HIGH, RiskLevel.CRITICAL]


@dataclass
class SafetyStatus:
    """Comprehensive safety status representation."""
    overall_level: SafetyLevel
    hardware_health: HealthStatus
    sensor_status: dict  # sensor_name -> HealthStatus
    algorithm_performance: dict  # algorithm_name -> float (0-1)
    emergency_stop_active: bool
    active_warnings: List[str]
    system_health_score: float  # 0-100
    timestamp: float
    
    @classmethod
    def create_safe_status(cls) -> 'SafetyStatus':
        """Create a default safe status."""
        return cls(
            overall_level=SafetyLevel.SAFE,
            hardware_health=HealthStatus.OK,
            sensor_status={},
            algorithm_performance={},
            emergency_stop_active=False,
            active_warnings=[],
            system_health_score=100.0,
            timestamp=time.time()
        )


class AdaptiveThresholdDetector:
    """Adaptive threshold detection for varying lighting conditions."""
    
    def __init__(self, initial_threshold: int = 127, adaptation_rate: float = 0.1):
        self.threshold = initial_threshold
        self.adaptation_rate = adaptation_rate
        self.brightness_history = []
        self.max_history = 10
    
    def detect_edges(self, image: np.ndarray) -> np.ndarray:
        """
        Detect edges using adaptive thresholding.
        
        Args:
            image: Input grayscale image
            
        Returns:
            Binary edge image
        """
        # Calculate image brightness
        brightness = np.mean(image)
        self.brightness_history.append(brightness)
        
        # Keep history limited
        if len(self.brightness_history) > self.max_history:
            self.brightness_history.pop(0)
        
        # Adapt threshold based on brightness history
        avg_brightness = np.mean(self.brightness_history)
        target_threshold = int(avg_brightness * 0.8)  # 80% of average brightness
        
        # Smooth threshold adaptation
        self.threshold = int(self.threshold * (1 - self.adaptation_rate) + 
                           target_threshold * self.adaptation_rate)
        
        # Apply adaptive threshold
        _, binary = cv2.threshold(image, self.threshold, 255, cv2.THRESH_BINARY)
        
        # Apply Canny edge detection for better results
        edges = cv2.Canny(binary, self.threshold // 2, self.threshold)
        
        return edges
    
    def get_current_threshold(self) -> int:
        """Get the current adaptive threshold value."""
        return self.threshold


class TemporalConsistencyFilter:
    """Filter for maintaining temporal consistency across frames."""
    
    def __init__(self, consistency_threshold: float = 0.7, max_frames: int = 5):
        self.consistency_threshold = consistency_threshold
        self.max_frames = max_frames
        self.detection_history = []
    
    def add_detection(self, detection: any) -> bool:
        """
        Add a detection and check for temporal consistency.
        
        Args:
            detection: Detection result to add
            
        Returns:
            True if detection is temporally consistent
        """
        self.detection_history.append(detection)
        
        # Keep history limited
        if len(self.detection_history) > self.max_frames:
            self.detection_history.pop(0)
        
        # Check consistency (simplified - in practice would compare detection properties)
        if len(self.detection_history) < 2:
            return True
        
        # Calculate consistency score (placeholder implementation)
        consistency_score = self._calculate_consistency()
        
        return consistency_score >= self.consistency_threshold
    
    def _calculate_consistency(self) -> float:
        """Calculate consistency score across recent detections."""
        if len(self.detection_history) < 2:
            return 1.0
        
        # Simplified consistency calculation
        # In practice, this would compare detection properties like position, size, etc.
        return 0.8  # Placeholder value
    
    def get_stable_detection(self) -> Optional[any]:
        """Get the most stable detection from recent history."""
        if not self.detection_history:
            return None
        
        # Return most recent detection if consistent
        if self._calculate_consistency() >= self.consistency_threshold:
            return self.detection_history[-1]
        
        return None


class PolynomialCurveFitter:
    """Polynomial curve fitting for lane trajectory prediction."""
    
    def __init__(self, degree: int = 3, min_points: int = 10):
        self.degree = degree
        self.min_points = min_points
    
    def fit_curve(self, points: List[Tuple[float, float]]) -> Optional[np.ndarray]:
        """
        Fit a polynomial curve to the given points.
        
        Args:
            points: List of (x, y) coordinate tuples
            
        Returns:
            Polynomial coefficients or None if fitting fails
        """
        if len(points) < self.min_points:
            return None
        
        try:
            # Convert points to numpy arrays
            x_coords = np.array([p[0] for p in points])
            y_coords = np.array([p[1] for p in points])
            
            # Fit polynomial
            coefficients = np.polyfit(x_coords, y_coords, self.degree)
            
            return coefficients
            
        except np.linalg.LinAlgError:
            # Fitting failed
            return None
    
    def evaluate_curve(self, coefficients: np.ndarray, x_values: np.ndarray) -> np.ndarray:
        """
        Evaluate the polynomial curve at given x values.
        
        Args:
            coefficients: Polynomial coefficients
            x_values: X values to evaluate
            
        Returns:
            Corresponding Y values
        """
        return np.polyval(coefficients, x_values)
    
    def calculate_curvature(self, coefficients: np.ndarray, x: float) -> float:
        """
        Calculate curvature at a specific point.
        
        Args:
            coefficients: Polynomial coefficients
            x: X coordinate to calculate curvature at
            
        Returns:
            Curvature value (1/radius)
        """
        # Calculate first and second derivatives
        first_deriv = np.polyder(coefficients)
        second_deriv = np.polyder(first_deriv)
        
        # Evaluate derivatives at x
        dy_dx = np.polyval(first_deriv, x)
        d2y_dx2 = np.polyval(second_deriv, x)
        
        # Calculate curvature: κ = |d²y/dx²| / (1 + (dy/dx)²)^(3/2)
        curvature = abs(d2y_dx2) / (1 + dy_dx**2)**(3/2)
        
        return curvature


class ROIManager:
    """Dynamic Region of Interest management for adaptive processing."""
    
    def __init__(self, image_width: int, image_height: int):
        self.image_width = image_width
        self.image_height = image_height
        self.default_roi = (0, image_height // 2, image_width, image_height // 2)
        self.current_roi = self.default_roi
    
    def update_roi(self, vehicle_state: VehicleState) -> Tuple[int, int, int, int]:
        """
        Update ROI based on vehicle state.
        
        Args:
            vehicle_state: Current vehicle state
            
        Returns:
            ROI as (x, y, width, height)
        """
        # Adjust ROI based on vehicle speed
        speed = np.sqrt(vehicle_state.velocity.x**2 + vehicle_state.velocity.y**2)
        
        # Higher speed -> look further ahead (lower y start)
        speed_factor = min(speed / 2.0, 1.0)  # Normalize to 0-1
        
        y_start = int(self.image_height * (0.3 + 0.2 * speed_factor))
        roi_height = self.image_height - y_start
        
        self.current_roi = (0, y_start, self.image_width, roi_height)
        
        return self.current_roi
    
    def get_roi_mask(self) -> np.ndarray:
        """
        Get binary mask for current ROI.
        
        Returns:
            Binary mask with ROI region set to 255
        """
        mask = np.zeros((self.image_height, self.image_width), dtype=np.uint8)
        x, y, w, h = self.current_roi
        mask[y:y+h, x:x+w] = 255
        
        return mask
    
    def apply_roi(self, image: np.ndarray) -> np.ndarray:
        """
        Apply ROI to image.
        
        Args:
            image: Input image
            
        Returns:
            Image with ROI applied (other regions set to 0)
        """
        mask = self.get_roi_mask()
        
        if len(image.shape) == 3:
            # Color image
            result = image.copy()
            result[mask == 0] = 0
        else:
            # Grayscale image
            result = cv2.bitwise_and(image, mask)
        
        return result


class PerformanceMonitor:
    """Monitor and track algorithm performance metrics."""
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.processing_times = []
        self.fps_history = []
        self.last_timestamp = time.time()
    
    def start_timing(self) -> float:
        """Start timing an operation."""
        return time.time()
    
    def end_timing(self, start_time: float) -> float:
        """
        End timing and record the duration.
        
        Args:
            start_time: Start time from start_timing()
            
        Returns:
            Processing time in seconds
        """
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        
        # Keep history limited
        if len(self.processing_times) > self.window_size:
            self.processing_times.pop(0)
        
        # Calculate FPS
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_timestamp) if current_time > self.last_timestamp else 0
        self.fps_history.append(fps)
        self.last_timestamp = current_time
        
        if len(self.fps_history) > self.window_size:
            self.fps_history.pop(0)
        
        return processing_time
    
    def get_average_processing_time(self) -> float:
        """Get average processing time."""
        return np.mean(self.processing_times) if self.processing_times else 0.0
    
    def get_average_fps(self) -> float:
        """Get average FPS."""
        return np.mean(self.fps_history) if self.fps_history else 0.0
    
    def get_performance_metrics(self) -> dict:
        """Get comprehensive performance metrics."""
        return {
            'avg_processing_time': self.get_average_processing_time(),
            'avg_fps': self.get_average_fps(),
            'min_processing_time': min(self.processing_times) if self.processing_times else 0,
            'max_processing_time': max(self.processing_times) if self.processing_times else 0,
            'samples_count': len(self.processing_times)
        }
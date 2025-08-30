#!/usr/bin/env python3
"""
Dynamic Quality Adjuster for Image Processing Pipeline

Automatically adjusts image processing quality based on available system resources
to maintain optimal performance on resource-constrained devices like Raspberry Pi.

Author: Duckietown
"""

import rospy
import time
import threading
import json
from typing import Dict, List, Optional, Any, Tuple, Callable
from dataclasses import dataclass, asdict
from enum import Enum
import numpy as np

# ROS messages
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image, CompressedImage


class QualityLevel(Enum):
    """Image processing quality levels"""
    ULTRA_LOW = 0.2
    LOW = 0.4
    MEDIUM = 0.6
    HIGH = 0.8
    ULTRA_HIGH = 1.0


class ResourcePressure(Enum):
    """System resource pressure levels"""
    NONE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4


@dataclass
class QualitySettings:
    """Image processing quality settings"""
    resolution_scale: float  # 0.1 to 1.0
    compression_quality: int  # 1 to 100
    frame_skip_rate: int  # 0 to 10
    roi_reduction: float  # 0.0 to 0.5
    blur_kernel_size: int  # 1, 3, 5, 7
    detection_threshold: float  # 0.1 to 0.9
    max_detections: int  # 1 to 100
    enable_temporal_filtering: bool
    enable_multi_scale: bool
    enable_gpu_acceleration: bool
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ResourceMetrics:
    """Current system resource metrics"""
    cpu_percent: float
    memory_percent: float
    memory_available_mb: float
    temperature_celsius: Optional[float]
    processing_fps: float
    target_fps: float
    latency_ms: float
    queue_depth: int
    timestamp: float
    
    def get_pressure_level(self, raspberry_pi_mode: bool = True) -> ResourcePressure:
        """Calculate overall resource pressure level"""
        if raspberry_pi_mode:
            cpu_threshold = [50, 65, 80, 90]
            memory_threshold = [60, 75, 85, 95]
            temp_threshold = [60, 70, 75, 80]
        else:
            cpu_threshold = [60, 75, 85, 95]
            memory_threshold = [70, 80, 90, 95]
            temp_threshold = [70, 80, 85, 90]
        
        pressure_scores = []
        
        # CPU pressure
        for i, threshold in enumerate(cpu_threshold):
            if self.cpu_percent <= threshold:
                pressure_scores.append(i)
                break
        else:
            pressure_scores.append(len(cpu_threshold))
        
        # Memory pressure
        for i, threshold in enumerate(memory_threshold):
            if self.memory_percent <= threshold:
                pressure_scores.append(i)
                break
        else:
            pressure_scores.append(len(memory_threshold))
        
        # Temperature pressure (if available)
        if self.temperature_celsius:
            for i, threshold in enumerate(temp_threshold):
                if self.temperature_celsius <= threshold:
                    pressure_scores.append(i)
                    break
            else:
                pressure_scores.append(len(temp_threshold))
        
        # FPS pressure
        fps_ratio = self.processing_fps / max(1.0, self.target_fps)
        if fps_ratio >= 0.9:
            pressure_scores.append(0)
        elif fps_ratio >= 0.7:
            pressure_scores.append(1)
        elif fps_ratio >= 0.5:
            pressure_scores.append(2)
        elif fps_ratio >= 0.3:
            pressure_scores.append(3)
        else:
            pressure_scores.append(4)
        
        # Return maximum pressure level
        max_pressure = max(pressure_scores)
        return ResourcePressure(min(max_pressure, 4))


class DynamicQualityAdjuster:
    """
    Dynamically adjusts image processing quality based on system resources.
    Integrates with existing image processing pipeline to maintain performance.
    """
    
    def __init__(self):
        self.node_name = "dynamic_quality_adjuster"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Configuration
        self.adjustment_frequency = rospy.get_param("~adjustment_frequency", 2.0)  # Hz
        self.raspberry_pi_mode = rospy.get_param("~raspberry_pi_mode", True)
        self.target_fps = rospy.get_param("~target_fps", 20.0)
        self.min_fps = rospy.get_param("~min_fps", 10.0)
        self.max_fps = rospy.get_param("~max_fps", 30.0)
        
        # Quality adjustment parameters
        self.aggressive_adjustment = rospy.get_param("~aggressive_adjustment", True)
        self.adjustment_smoothing = rospy.get_param("~adjustment_smoothing", 0.7)  # 0-1
        self.quality_hysteresis = rospy.get_param("~quality_hysteresis", 0.1)  # Prevent oscillation
        
        # Current state
        self.current_quality_level = QualityLevel.HIGH
        self.current_settings = self._get_default_quality_settings(self.current_quality_level)
        self.last_adjustment_time = 0.0
        self.adjustment_history = []
        self.resource_history = []
        
        # Component callbacks
        self.quality_callbacks: Dict[str, Callable] = {}
        self.registered_components: Dict[str, Dict] = {}
        
        # Statistics
        self.total_adjustments = 0
        self.quality_improvements = 0
        self.quality_reductions = 0
        self.performance_gains = 0.0
        
        # Threading
        self.adjustment_lock = threading.Lock()
        self.adjustment_active = True
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] Dynamic Quality Adjuster initialized")
        rospy.loginfo(f"[{self.node_name}] Target FPS: {self.target_fps}, Min FPS: {self.min_fps}")
        rospy.loginfo(f"[{self.node_name}] Raspberry Pi mode: {self.raspberry_pi_mode}")
        rospy.loginfo(f"[{self.node_name}] Aggressive adjustment: {self.aggressive_adjustment}")
        rospy.loginfo(f"[{self.node_name}] Current quality level: {self.current_quality_level.name}")
    
    def _setup_subscribers(self):
        """Setup subscribers for resource monitoring"""
        rospy.loginfo(f"[{self.node_name}] Setting up resource monitoring subscribers...")
        
        # System performance data
        self.resource_sub = rospy.Subscriber(
            "~resource_metrics",
            String,  # JSON-encoded resource data
            self._resource_metrics_callback,
            queue_size=1
        )
        
        # Performance profiler data
        self.profiler_sub = rospy.Subscriber(
            "/enhanced_performance_profiler/system_efficiency",
            Float32,
            self._system_efficiency_callback,
            queue_size=1
        )
        
        # Component performance feedback
        self.component_feedback_sub = rospy.Subscriber(
            "~component_feedback",
            String,  # JSON-encoded feedback
            self._component_feedback_callback,
            queue_size=10
        )
        
        rospy.loginfo(f"[{self.node_name}] Resource monitoring subscribers initialized")
    
    def _setup_publishers(self):
        """Setup publishers for quality adjustments"""
        rospy.loginfo(f"[{self.node_name}] Setting up quality adjustment publishers...")
        
        # Quality settings updates
        self.quality_pub = rospy.Publisher(
            "~quality_settings",
            String,  # JSON-encoded quality settings
            queue_size=1
        )
        
        # Quality level changes
        self.quality_level_pub = rospy.Publisher(
            "~quality_level",
            String,
            queue_size=1
        )
        
        # Adjustment notifications
        self.adjustment_pub = rospy.Publisher(
            "~quality_adjustments",
            String,
            queue_size=10
        )
        
        # Performance impact reports
        self.impact_pub = rospy.Publisher(
            "~performance_impact",
            String,
            queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Quality adjustment publishers initialized")
    
    def _setup_timers(self):
        """Setup periodic adjustment timers"""
        # Main quality adjustment timer
        self.adjustment_timer = rospy.Timer(
            rospy.Duration(1.0 / self.adjustment_frequency),
            self._adjustment_callback
        )
        
        # Performance monitoring timer
        self.monitoring_timer = rospy.Timer(
            rospy.Duration(5.0),  # Every 5 seconds
            self._monitoring_callback
        )
        
        rospy.loginfo(f"[{self.node_name}] Quality adjustment timers initialized")
    
    def register_component(self, component_name: str, quality_callback: Callable,
                          priority: int = 5, quality_targets: Dict[str, float] = None) -> bool:
        """
        Register a component for dynamic quality adjustment.
        
        Args:
            component_name: Name of the component
            quality_callback: Callback function to apply quality settings
            priority: Priority level (1-10, higher = more important)
            quality_targets: Target performance metrics
            
        Returns:
            bool: True if registration successful
        """
        with self.adjustment_lock:
            if quality_targets is None:
                quality_targets = {
                    'target_fps': self.target_fps,
                    'max_latency_ms': 200.0,
                    'min_quality': 0.3
                }
            
            self.registered_components[component_name] = {
                'priority': priority,
                'quality_targets': quality_targets,
                'registration_time': time.time(),
                'adjustment_count': 0,
                'last_adjustment': None,
                'current_quality': 1.0,
                'performance_history': []
            }
            
            self.quality_callbacks[component_name] = quality_callback
            
            rospy.loginfo(f"[{self.node_name}] Registered component '{component_name}' with priority {priority}")
            rospy.logdebug(f"[{self.node_name}] Quality targets: {quality_targets}")
            
            # Comprehensive logging
            rospy.logdebug(f"[{self.node_name}] Component registration event: {component_name}, "
                          f"Priority={priority}, Targets={quality_targets}")
            
            return True
    
    def _resource_metrics_callback(self, msg):
        """Handle resource metrics updates"""
        try:
            # Parse resource metrics
            metrics_data = json.loads(msg.data)
            
            metrics = ResourceMetrics(
                cpu_percent=metrics_data.get('cpu_percent', 0.0),
                memory_percent=metrics_data.get('memory_percent', 0.0),
                memory_available_mb=metrics_data.get('memory_available_mb', 1024.0),
                temperature_celsius=metrics_data.get('temperature_celsius'),
                processing_fps=metrics_data.get('processing_fps', self.target_fps),
                target_fps=self.target_fps,
                latency_ms=metrics_data.get('latency_ms', 100.0),
                queue_depth=metrics_data.get('queue_depth', 0),
                timestamp=time.time()
            )
            
            with self.adjustment_lock:
                self.resource_history.append(metrics)
                if len(self.resource_history) > 100:  # Keep last 100 measurements
                    self.resource_history.pop(0)
            
            rospy.logdebug(f"[{self.node_name}] Resource metrics update: "
                          f"CPU={metrics.cpu_percent:.1f}%, Memory={metrics.memory_percent:.1f}%, "
                          f"FPS={metrics.processing_fps:.1f}")
            
            # Real-time monitoring
            pressure = metrics.get_pressure_level(self.raspberry_pi_mode)
            rospy.logdebug(f"[{self.node_name}] Resource pressure event: {pressure.name}, "
                          f"CPU={metrics.cpu_percent:.1f}%, Memory={metrics.memory_percent:.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing resource metrics: {e}")
    
    def _system_efficiency_callback(self, msg):
        """Handle system efficiency updates from performance profiler"""
        efficiency = msg.data
        
        # Use efficiency score to influence quality adjustments
        if efficiency < 0.6:  # Low efficiency
            rospy.logdebug(f"[{self.node_name}] Low system efficiency detected: {efficiency:.2f}")
        elif efficiency > 0.9:  # High efficiency
            rospy.logdebug(f"[{self.node_name}] High system efficiency detected: {efficiency:.2f}")
        
        rospy.logdebug(f"[{self.node_name}] System efficiency update: {efficiency:.2f}")
    
    def _component_feedback_callback(self, msg):
        """Handle component performance feedback"""
        try:
            feedback_data = json.loads(msg.data)
            component_name = feedback_data.get('component_name', 'unknown')
            
            if component_name in self.registered_components:
                performance = {
                    'fps': feedback_data.get('fps', 0.0),
                    'latency_ms': feedback_data.get('latency_ms', 0.0),
                    'cpu_usage': feedback_data.get('cpu_usage', 0.0),
                    'memory_usage': feedback_data.get('memory_usage', 0.0),
                    'timestamp': time.time()
                }
                
                with self.adjustment_lock:
                    self.registered_components[component_name]['performance_history'].append(performance)
                    # Keep last 20 performance measurements
                    if len(self.registered_components[component_name]['performance_history']) > 20:
                        self.registered_components[component_name]['performance_history'].pop(0)
                
                rospy.logdebug(f"[{self.node_name}] Component feedback from '{component_name}': "
                              f"FPS={performance['fps']:.1f}, Latency={performance['latency_ms']:.1f}ms")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing component feedback: {e}")
    
    def _adjustment_callback(self, event):
        """Main quality adjustment callback"""
        if not self.adjustment_active:
            return
        
        try:
            current_time = time.time()
            
            # Get current resource state
            if not self.resource_history:
                return
            
            latest_metrics = self.resource_history[-1]
            pressure_level = latest_metrics.get_pressure_level(self.raspberry_pi_mode)
            
            # Determine if adjustment is needed
            adjustment_needed, target_quality = self._assess_quality_adjustment(latest_metrics, pressure_level)
            
            if adjustment_needed:
                success = self._apply_quality_adjustment(target_quality, latest_metrics)
                
                if success:
                    self.total_adjustments += 1
                    self.last_adjustment_time = current_time
                    
                    # Track improvement vs reduction
                    if target_quality.value > self.current_quality_level.value:
                        self.quality_improvements += 1
                        rospy.loginfo(f"[{self.node_name}] Quality improved: {self.current_quality_level.name} -> {target_quality.name}")
                    else:
                        self.quality_reductions += 1
                        rospy.loginfo(f"[{self.node_name}] Quality reduced: {self.current_quality_level.name} -> {target_quality.name}")
                    
                    self.current_quality_level = target_quality
                    
                    # Comprehensive logging
                    rospy.logdebug(f"[{self.node_name}] Quality adjustment applied:")
                    rospy.logdebug(f"[{self.node_name}] - New level: {target_quality.name}")
                    rospy.logdebug(f"[{self.node_name}] - Pressure: {pressure_level.name}")
                    rospy.logdebug(f"[{self.node_name}] - CPU: {latest_metrics.cpu_percent:.1f}%")
                    rospy.logdebug(f"[{self.node_name}] - Memory: {latest_metrics.memory_percent:.1f}%")
                    rospy.logdebug(f"[{self.node_name}] - FPS: {latest_metrics.processing_fps:.1f}")
                    
                    # Real-time monitoring
                    rospy.loginfo(f"[{self.node_name}] Quality adjustment event: {target_quality.name}, "
                                 f"Pressure={pressure_level.name}, Total={self.total_adjustments}")
                else:
                    rospy.logwarn(f"[{self.node_name}] Failed to apply quality adjustment")
            else:
                rospy.logdebug(f"[{self.node_name}] No quality adjustment needed: "
                              f"Level={self.current_quality_level.name}, Pressure={pressure_level.name}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Quality adjustment callback error: {e}")
    
    def _assess_quality_adjustment(self, metrics: ResourceMetrics, pressure: ResourcePressure) -> Tuple[bool, QualityLevel]:
        """Assess if quality adjustment is needed and determine target level"""
        current_time = time.time()
        
        # Prevent too frequent adjustments
        if current_time - self.last_adjustment_time < 2.0:  # Minimum 2 seconds between adjustments
            return False, self.current_quality_level
        
        # Determine target quality based on pressure level
        if pressure == ResourcePressure.CRITICAL:
            target_quality = QualityLevel.ULTRA_LOW
        elif pressure == ResourcePressure.HIGH:
            target_quality = QualityLevel.LOW
        elif pressure == ResourcePressure.MEDIUM:
            target_quality = QualityLevel.MEDIUM
        elif pressure == ResourcePressure.LOW:
            target_quality = QualityLevel.HIGH
        else:  # ResourcePressure.NONE
            target_quality = QualityLevel.ULTRA_HIGH
        
        # Apply hysteresis to prevent oscillation
        current_value = self.current_quality_level.value
        target_value = target_quality.value
        
        if abs(target_value - current_value) < self.quality_hysteresis:
            return False, self.current_quality_level
        
        # Consider FPS performance
        fps_ratio = metrics.processing_fps / metrics.target_fps
        if fps_ratio < 0.7 and target_value > current_value:
            # Don't increase quality if FPS is already low
            return False, self.current_quality_level
        elif fps_ratio > 1.2 and target_value < current_value:
            # Don't decrease quality if FPS is high
            target_quality = min(QualityLevel.ULTRA_HIGH, 
                               QualityLevel(min(1.0, current_value + 0.2)))
        
        # Apply smoothing for gradual adjustments
        if self.adjustment_smoothing > 0:
            smoothed_value = (current_value * self.adjustment_smoothing + 
                            target_value * (1 - self.adjustment_smoothing))
            
            # Find closest quality level
            for level in QualityLevel:
                if abs(level.value - smoothed_value) < 0.1:
                    target_quality = level
                    break
        
        # Check if adjustment is significant enough
        if abs(target_quality.value - current_value) < 0.15:
            return False, self.current_quality_level
        
        return True, target_quality
    
    def _apply_quality_adjustment(self, target_quality: QualityLevel, metrics: ResourceMetrics) -> bool:
        """Apply quality adjustment to all registered components"""
        try:
            # Generate quality settings for target level
            new_settings = self._get_default_quality_settings(target_quality)
            
            # Customize settings based on current metrics
            new_settings = self._customize_quality_settings(new_settings, metrics)
            
            # Apply settings to all registered components
            successful_applications = 0
            
            with self.adjustment_lock:
                for component_name, callback in self.quality_callbacks.items():
                    try:
                        # Apply component-specific customizations
                        component_settings = self._customize_for_component(new_settings, component_name)
                        
                        # Call component's quality adjustment callback
                        callback(component_settings.to_dict())
                        
                        # Update component state
                        self.registered_components[component_name]['adjustment_count'] += 1
                        self.registered_components[component_name]['last_adjustment'] = time.time()
                        self.registered_components[component_name]['current_quality'] = target_quality.value
                        
                        successful_applications += 1
                        
                        rospy.logdebug(f"[{self.node_name}] Applied quality settings to '{component_name}'")
                        
                    except Exception as e:
                        rospy.logerr(f"[{self.node_name}] Failed to apply quality settings to '{component_name}': {e}")
            
            if successful_applications > 0:
                # Update current settings
                self.current_settings = new_settings
                
                # Publish quality settings update
                self._publish_quality_update(target_quality, new_settings)
                
                # Record adjustment in history
                adjustment_record = {
                    'timestamp': time.time(),
                    'from_level': self.current_quality_level.name,
                    'to_level': target_quality.name,
                    'pressure_level': metrics.get_pressure_level(self.raspberry_pi_mode).name,
                    'cpu_percent': metrics.cpu_percent,
                    'memory_percent': metrics.memory_percent,
                    'fps': metrics.processing_fps,
                    'successful_applications': successful_applications
                }
                
                self.adjustment_history.append(adjustment_record)
                if len(self.adjustment_history) > 50:  # Keep last 50 adjustments
                    self.adjustment_history.pop(0)
                
                rospy.loginfo(f"[{self.node_name}] Quality adjustment successful: "
                             f"{successful_applications}/{len(self.quality_callbacks)} components updated")
                
                return True
            else:
                rospy.logerr(f"[{self.node_name}] No components successfully updated")
                return False
                
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error applying quality adjustment: {e}")
            return False
    
    def _get_default_quality_settings(self, quality_level: QualityLevel) -> QualitySettings:
        """Get default quality settings for a given quality level"""
        if quality_level == QualityLevel.ULTRA_LOW:
            return QualitySettings(
                resolution_scale=0.3,
                compression_quality=30,
                frame_skip_rate=3,
                roi_reduction=0.4,
                blur_kernel_size=7,
                detection_threshold=0.7,
                max_detections=10,
                enable_temporal_filtering=False,
                enable_multi_scale=False,
                enable_gpu_acceleration=False
            )
        elif quality_level == QualityLevel.LOW:
            return QualitySettings(
                resolution_scale=0.5,
                compression_quality=50,
                frame_skip_rate=2,
                roi_reduction=0.3,
                blur_kernel_size=5,
                detection_threshold=0.6,
                max_detections=20,
                enable_temporal_filtering=False,
                enable_multi_scale=False,
                enable_gpu_acceleration=False
            )
        elif quality_level == QualityLevel.MEDIUM:
            return QualitySettings(
                resolution_scale=0.7,
                compression_quality=70,
                frame_skip_rate=1,
                roi_reduction=0.2,
                blur_kernel_size=3,
                detection_threshold=0.5,
                max_detections=30,
                enable_temporal_filtering=True,
                enable_multi_scale=False,
                enable_gpu_acceleration=True
            )
        elif quality_level == QualityLevel.HIGH:
            return QualitySettings(
                resolution_scale=0.9,
                compression_quality=85,
                frame_skip_rate=0,
                roi_reduction=0.1,
                blur_kernel_size=3,
                detection_threshold=0.4,
                max_detections=50,
                enable_temporal_filtering=True,
                enable_multi_scale=True,
                enable_gpu_acceleration=True
            )
        else:  # QualityLevel.ULTRA_HIGH
            return QualitySettings(
                resolution_scale=1.0,
                compression_quality=95,
                frame_skip_rate=0,
                roi_reduction=0.0,
                blur_kernel_size=1,
                detection_threshold=0.3,
                max_detections=100,
                enable_temporal_filtering=True,
                enable_multi_scale=True,
                enable_gpu_acceleration=True
            )
    
    def _customize_quality_settings(self, settings: QualitySettings, metrics: ResourceMetrics) -> QualitySettings:
        """Customize quality settings based on current system metrics"""
        # Temperature-based adjustments
        if metrics.temperature_celsius and metrics.temperature_celsius > 75:
            # Reduce processing load for thermal management
            settings.resolution_scale *= 0.8
            settings.frame_skip_rate = min(5, settings.frame_skip_rate + 1)
            settings.enable_gpu_acceleration = False
            
            rospy.logdebug(f"[{self.node_name}] Thermal adjustment applied: Temp={metrics.temperature_celsius:.1f}°C")
        
        # Memory-based adjustments
        if metrics.memory_percent > 85:
            # Reduce memory usage
            settings.max_detections = max(5, int(settings.max_detections * 0.7))
            settings.enable_temporal_filtering = False
            
            rospy.logdebug(f"[{self.node_name}] Memory adjustment applied: Memory={metrics.memory_percent:.1f}%")
        
        # FPS-based adjustments
        fps_ratio = metrics.processing_fps / metrics.target_fps
        if fps_ratio < 0.5:
            # Aggressive performance optimization
            settings.resolution_scale *= 0.7
            settings.frame_skip_rate = min(5, settings.frame_skip_rate + 2)
            settings.roi_reduction = min(0.5, settings.roi_reduction + 0.2)
            
            rospy.logdebug(f"[{self.node_name}] FPS adjustment applied: FPS={metrics.processing_fps:.1f}")
        
        # Queue depth adjustments
        if metrics.queue_depth > 10:
            # Reduce processing complexity
            settings.detection_threshold = min(0.9, settings.detection_threshold + 0.1)
            settings.max_detections = max(5, int(settings.max_detections * 0.8))
            
            rospy.logdebug(f"[{self.node_name}] Queue adjustment applied: Queue={metrics.queue_depth}")
        
        return settings
    
    def _customize_for_component(self, settings: QualitySettings, component_name: str) -> QualitySettings:
        """Customize quality settings for a specific component"""
        if component_name not in self.registered_components:
            return settings
        
        component_info = self.registered_components[component_name]
        priority = component_info['priority']
        
        # High priority components get better quality
        if priority >= 8:
            settings.resolution_scale = min(1.0, settings.resolution_scale * 1.2)
            settings.compression_quality = min(100, int(settings.compression_quality * 1.1))
            settings.detection_threshold = max(0.1, settings.detection_threshold * 0.9)
        elif priority <= 3:
            settings.resolution_scale *= 0.8
            settings.compression_quality = max(10, int(settings.compression_quality * 0.9))
            settings.detection_threshold = min(0.9, settings.detection_threshold * 1.1)
        
        # Component-specific optimizations
        if 'yolo' in component_name.lower():
            # YOLO-specific optimizations
            if settings.resolution_scale < 0.5:
                settings.enable_multi_scale = False
        elif 'lane' in component_name.lower():
            # Lane detection specific optimizations
            settings.roi_reduction = min(0.3, settings.roi_reduction)  # Keep more of the image
        elif 'apriltag' in component_name.lower():
            # AprilTag specific optimizations
            settings.blur_kernel_size = max(1, settings.blur_kernel_size - 2)  # Less blur for tags
        
        return settings
    
    def _publish_quality_update(self, quality_level: QualityLevel, settings: QualitySettings):
        """Publish quality update notifications"""
        # Publish quality level change
        level_msg = String()
        level_msg.data = quality_level.name
        self.quality_level_pub.publish(level_msg)
        
        # Publish detailed quality settings
        settings_msg = String()
        settings_msg.data = json.dumps(settings.to_dict())
        self.quality_pub.publish(settings_msg)
        
        # Publish adjustment notification
        adjustment_data = {
            'timestamp': time.time(),
            'quality_level': quality_level.name,
            'quality_value': quality_level.value,
            'settings': settings.to_dict(),
            'total_adjustments': self.total_adjustments
        }
        
        adjustment_msg = String()
        adjustment_msg.data = json.dumps(adjustment_data)
        self.adjustment_pub.publish(adjustment_msg)
        
        rospy.logdebug(f"[{self.node_name}] Published quality update: {quality_level.name}")
    
    def _monitoring_callback(self, event):
        """Periodic monitoring and performance impact assessment"""
        try:
            current_time = time.time()
            
            # Calculate performance impact of recent adjustments
            impact_data = self._calculate_performance_impact()
            
            if impact_data:
                # Publish performance impact report
                impact_msg = String()
                impact_msg.data = json.dumps(impact_data)
                self.impact_pub.publish(impact_msg)
                
                rospy.logdebug(f"[{self.node_name}] Performance impact: "
                              f"FPS change={impact_data.get('fps_change', 0):.1f}, "
                              f"CPU change={impact_data.get('cpu_change', 0):.1f}%")
            
            # Log monitoring summary
            rospy.logdebug(f"[{self.node_name}] Monitoring summary: "
                          f"Level={self.current_quality_level.name}, "
                          f"Adjustments={self.total_adjustments}, "
                          f"Components={len(self.registered_components)}")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Monitoring event: Quality={self.current_quality_level.name}, "
                          f"Total adjustments={self.total_adjustments}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Monitoring callback error: {e}")
    
    def _calculate_performance_impact(self) -> Optional[Dict[str, Any]]:
        """Calculate performance impact of recent quality adjustments"""
        if len(self.adjustment_history) < 2 or len(self.resource_history) < 10:
            return None
        
        try:
            # Get recent adjustment
            recent_adjustment = self.adjustment_history[-1]
            adjustment_time = recent_adjustment['timestamp']
            
            # Get metrics before and after adjustment
            before_metrics = [m for m in self.resource_history if m.timestamp < adjustment_time]
            after_metrics = [m for m in self.resource_history if m.timestamp > adjustment_time]
            
            if len(before_metrics) < 3 or len(after_metrics) < 3:
                return None
            
            # Calculate average metrics before and after
            before_avg = {
                'cpu': np.mean([m.cpu_percent for m in before_metrics[-5:]]),
                'memory': np.mean([m.memory_percent for m in before_metrics[-5:]]),
                'fps': np.mean([m.processing_fps for m in before_metrics[-5:]]),
                'latency': np.mean([m.latency_ms for m in before_metrics[-5:]])
            }
            
            after_avg = {
                'cpu': np.mean([m.cpu_percent for m in after_metrics[:5]]),
                'memory': np.mean([m.memory_percent for m in after_metrics[:5]]),
                'fps': np.mean([m.processing_fps for m in after_metrics[:5]]),
                'latency': np.mean([m.latency_ms for m in after_metrics[:5]])
            }
            
            # Calculate changes
            impact_data = {
                'timestamp': time.time(),
                'adjustment_time': adjustment_time,
                'quality_change': recent_adjustment['to_level'],
                'cpu_change': after_avg['cpu'] - before_avg['cpu'],
                'memory_change': after_avg['memory'] - before_avg['memory'],
                'fps_change': after_avg['fps'] - before_avg['fps'],
                'latency_change': after_avg['latency'] - before_avg['latency'],
                'before_metrics': before_avg,
                'after_metrics': after_avg
            }
            
            # Update performance gains tracking
            if impact_data['fps_change'] > 0:
                self.performance_gains += impact_data['fps_change']
            
            return impact_data
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error calculating performance impact: {e}")
            return None
    
    def get_adjustment_statistics(self) -> Dict[str, Any]:
        """Get quality adjustment statistics"""
        current_time = time.time()
        
        # Component statistics
        component_stats = {}
        for component_name, info in self.registered_components.items():
            component_stats[component_name] = {
                'adjustment_count': info['adjustment_count'],
                'current_quality': info['current_quality'],
                'priority': info['priority'],
                'last_adjustment': info['last_adjustment']
            }
        
        return {
            'timestamp': current_time,
            'current_quality_level': self.current_quality_level.name,
            'current_quality_value': self.current_quality_level.value,
            'total_adjustments': self.total_adjustments,
            'quality_improvements': self.quality_improvements,
            'quality_reductions': self.quality_reductions,
            'performance_gains': self.performance_gains,
            'registered_components': len(self.registered_components),
            'component_statistics': component_stats,
            'adjustment_history_length': len(self.adjustment_history),
            'resource_history_length': len(self.resource_history),
            'raspberry_pi_mode': self.raspberry_pi_mode
        }
    
    def force_quality_level(self, quality_level: QualityLevel) -> bool:
        """Force a specific quality level (for testing/debugging)"""
        try:
            if not self.resource_history:
                # Create dummy metrics
                dummy_metrics = ResourceMetrics(
                    cpu_percent=50.0,
                    memory_percent=50.0,
                    memory_available_mb=1024.0,
                    temperature_celsius=None,
                    processing_fps=self.target_fps,
                    target_fps=self.target_fps,
                    latency_ms=100.0,
                    queue_depth=0,
                    timestamp=time.time()
                )
            else:
                dummy_metrics = self.resource_history[-1]
            
            success = self._apply_quality_adjustment(quality_level, dummy_metrics)
            
            if success:
                self.current_quality_level = quality_level
                rospy.loginfo(f"[{self.node_name}] Forced quality level to: {quality_level.name}")
                return True
            else:
                rospy.logerr(f"[{self.node_name}] Failed to force quality level to: {quality_level.name}")
                return False
                
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error forcing quality level: {e}")
            return False
    
    def shutdown(self):
        """Shutdown the dynamic quality adjuster"""
        rospy.loginfo(f"[{self.node_name}] Dynamic Quality Adjuster shutting down")
        
        self.adjustment_active = False
        
        # Shutdown timers
        if hasattr(self, 'adjustment_timer'):
            self.adjustment_timer.shutdown()
        if hasattr(self, 'monitoring_timer'):
            self.monitoring_timer.shutdown()
        
        # Log final statistics
        stats = self.get_adjustment_statistics()
        rospy.loginfo(f"[{self.node_name}] Final adjustment statistics:")
        rospy.loginfo(f"[{self.node_name}] Current quality: {stats['current_quality_level']}")
        rospy.loginfo(f"[{self.node_name}] Total adjustments: {stats['total_adjustments']}")
        rospy.loginfo(f"[{self.node_name}] Quality improvements: {stats['quality_improvements']}")
        rospy.loginfo(f"[{self.node_name}] Quality reductions: {stats['quality_reductions']}")
        rospy.loginfo(f"[{self.node_name}] Performance gains: {stats['performance_gains']:.1f} FPS")
        rospy.loginfo(f"[{self.node_name}] Registered components: {stats['registered_components']}")
        
        rospy.loginfo(f"[{self.node_name}] Dynamic Quality Adjuster shutdown complete")


def main():
    """Main function to run the dynamic quality adjuster"""
    try:
        adjuster = DynamicQualityAdjuster()
        rospy.loginfo("Dynamic Quality Adjuster running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Dynamic Quality Adjuster interrupted")
    except Exception as e:
        rospy.logerr(f"Dynamic Quality Adjuster error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Integration Data Logger for Enhanced Autonomous Navigation

Logs comprehensive data from the integrated YOLO detection and navigation system
for analysis, debugging, and performance optimization.

Author: Duckietown
"""

import rospy
import json
import csv
import os
import time
from datetime import datetime
from threading import Lock
from collections import deque
from typing import Dict, List, Optional, Any

# ROS messages
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import ObjectDetectionArray, LanePose, Twist2DStamped

# Custom message types (fallback to basic types if not available)
try:
    from duckietown_msgs.msg import (
        RiskAssessmentStatus,
        SystemStatus,
        CoordinationLog
    )
except ImportError:
    rospy.logwarn("Custom integration messages not available, using basic types")
    RiskAssessmentStatus = String
    SystemStatus = String
    CoordinationLog = String


class IntegrationDataLogger:
    """
    Comprehensive data logger for the integrated autonomous navigation system.
    
    Logs:
    - YOLO detection results with timestamps
    - Risk assessment decisions and reasoning
    - Coordination decisions and behavior switches
    - System performance metrics
    - Integration workflow execution data
    - Component synchronization and data flow
    """
    
    def __init__(self):
        """Initialize the integration data logger."""
        self.node_name = "integration_data_logger"
        rospy.init_node(self.node_name, anonymous=False)
        
        # Thread safety
        self.lock = Lock()
        
        # Configuration
        self.log_directory = rospy.get_param("~log_directory", "/tmp/integration_logs")
        self.max_log_size_mb = rospy.get_param("~max_log_size_mb", 100)
        self.log_rotation_enabled = rospy.get_param("~log_rotation_enabled", True)
        self.detailed_logging = rospy.get_param("~detailed_logging", True)
        
        # Create log directory
        os.makedirs(self.log_directory, exist_ok=True)
        
        # Log files
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_files = {
            'detections': os.path.join(self.log_directory, f"detections_{timestamp}.csv"),
            'risk_assessments': os.path.join(self.log_directory, f"risk_assessments_{timestamp}.csv"),
            'coordination': os.path.join(self.log_directory, f"coordination_{timestamp}.csv"),
            'system_status': os.path.join(self.log_directory, f"system_status_{timestamp}.csv"),
            'integration_events': os.path.join(self.log_directory, f"integration_events_{timestamp}.json")
        }
        
        # Data buffers
        self.data_buffers = {
            'detections': deque(maxlen=1000),
            'risk_assessments': deque(maxlen=1000),
            'coordination': deque(maxlen=1000),
            'system_status': deque(maxlen=1000),
            'integration_events': deque(maxlen=500)
        }
        
        # Statistics
        self.logging_stats = {
            'detections_logged': 0,
            'risk_assessments_logged': 0,
            'coordination_decisions_logged': 0,
            'system_status_logged': 0,
            'integration_events_logged': 0,
            'start_time': time.time()
        }
        
        # CSV writers
        self.csv_writers = {}
        self._setup_csv_files()
        
        self._setup_subscribers()
        self._setup_timers()
        
        rospy.loginfo(f"[{self.node_name}] Integration Data Logger initialized")
        rospy.loginfo(f"[{self.node_name}] Log directory: {self.log_directory}")
        rospy.loginfo(f"[{self.node_name}] Detailed logging: {self.detailed_logging}")
    
    def _setup_csv_files(self):
        """Setup CSV files and writers for structured data logging."""
        try:
            # Detection log CSV
            det_file = open(self.log_files['detections'], 'w', newline='')
            det_writer = csv.writer(det_file)
            det_writer.writerow([
                'timestamp', 'ros_time', 'detection_count', 'class_name', 
                'confidence', 'bbox_x', 'bbox_y', 'bbox_width', 'bbox_height',
                'distance', 'relative_velocity', 'risk_level', 'processing_time_ms'
            ])
            self.csv_writers['detections'] = (det_file, det_writer)
            
            # Risk assessment log CSV
            risk_file = open(self.log_files['risk_assessments'], 'w', newline='')
            risk_writer = csv.writer(risk_file)
            risk_writer.writerow([
                'timestamp', 'ros_time', 'overall_risk_level', 'critical_objects',
                'min_distance', 'min_ttc', 'avoidance_required', 'emergency_stop_required',
                'risk_factors', 'assessment_time_ms'
            ])
            self.csv_writers['risk_assessments'] = (risk_file, risk_writer)
            
            # Coordination log CSV
            coord_file = open(self.log_files['coordination'], 'w', newline='')
            coord_writer = csv.writer(coord_file)
            coord_writer.writerow([
                'timestamp', 'ros_time', 'current_behavior', 'previous_behavior',
                'behavior_switch', 'decision_factors', 'control_v', 'control_omega',
                'has_detections', 'has_navigation', 'has_lane_pose', 'emergency_active',
                'coordination_time_ms'
            ])
            self.csv_writers['coordination'] = (coord_file, coord_writer)
            
            # System status log CSV
            status_file = open(self.log_files['system_status'], 'w', newline='')
            status_writer = csv.writer(status_file)
            status_writer.writerow([
                'timestamp', 'ros_time', 'system_active', 'overall_health',
                'yolo_active', 'navigation_active', 'integration_active',
                'cpu_percent', 'memory_percent', 'fps', 'avg_latency_ms'
            ])
            self.csv_writers['system_status'] = (status_file, status_writer)
            
            rospy.loginfo(f"[{self.node_name}] CSV log files initialized")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error setting up CSV files: {e}")
    
    def _setup_subscribers(self):
        """Setup subscribers for all integration data sources."""
        rospy.loginfo(f"[{self.node_name}] Setting up data subscribers...")
        
        # YOLO detections
        self.detections_sub = rospy.Subscriber(
            "~detections",
            ObjectDetectionArray,
            self._detections_callback,
            queue_size=10
        )
        
        # Risk assessment status
        self.risk_sub = rospy.Subscriber(
            "~risk_status",
            RiskAssessmentStatus,
            self._risk_status_callback,
            queue_size=10
        )
        
        # Coordination decisions
        self.coordination_sub = rospy.Subscriber(
            "~coordination_decisions",
            CoordinationLog,
            self._coordination_callback,
            queue_size=10
        )
        
        # System status
        self.system_status_sub = rospy.Subscriber(
            "~system_status",
            SystemStatus,
            self._system_status_callback,
            queue_size=10
        )
        
        rospy.loginfo(f"[{self.node_name}] Data subscribers initialized")
    
    def _setup_timers(self):
        """Setup periodic timers for data flushing and maintenance."""
        # Data flush timer (high frequency)
        self.flush_timer = rospy.Timer(
            rospy.Duration(1.0),  # 1 Hz
            self._flush_data_callback
        )
        
        # Statistics reporting timer
        self.stats_timer = rospy.Timer(
            rospy.Duration(30.0),  # Every 30 seconds
            self._stats_callback
        )
        
        # Log rotation check timer
        if self.log_rotation_enabled:
            self.rotation_timer = rospy.Timer(
                rospy.Duration(300.0),  # Every 5 minutes
                self._log_rotation_callback
            )
        
        rospy.loginfo(f"[{self.node_name}] Logging timers initialized")
    
    def _detections_callback(self, msg):
        """Log YOLO detection results."""
        current_time = time.time()
        ros_time = msg.header.stamp.to_sec()
        
        with self.lock:
            # Log each detection
            for i, detection in enumerate(msg.detections):
                detection_data = {
                    'timestamp': current_time,
                    'ros_time': ros_time,
                    'detection_count': len(msg.detections),
                    'detection_index': i,
                    'class_name': getattr(detection, 'class_name', 'unknown'),
                    'confidence': getattr(detection, 'confidence', 0.0),
                    'bbox': {
                        'x': getattr(detection, 'bbox_x', 0),
                        'y': getattr(detection, 'bbox_y', 0),
                        'width': getattr(detection, 'bbox_width', 0),
                        'height': getattr(detection, 'bbox_height', 0)
                    },
                    'distance': getattr(detection, 'distance', 0.0),
                    'relative_velocity': getattr(detection, 'relative_velocity', 0.0),
                    'risk_level': getattr(detection, 'risk_level', 'unknown'),
                    'processing_time_ms': getattr(msg, 'processing_time_ms', 0.0)
                }
                
                self.data_buffers['detections'].append(detection_data)
                self.logging_stats['detections_logged'] += 1
                
                # Write to CSV
                if 'detections' in self.csv_writers:
                    _, writer = self.csv_writers['detections']
                    writer.writerow([
                        detection_data['timestamp'],
                        detection_data['ros_time'],
                        detection_data['detection_count'],
                        detection_data['class_name'],
                        detection_data['confidence'],
                        detection_data['bbox']['x'],
                        detection_data['bbox']['y'],
                        detection_data['bbox']['width'],
                        detection_data['bbox']['height'],
                        detection_data['distance'],
                        detection_data['relative_velocity'],
                        detection_data['risk_level'],
                        detection_data['processing_time_ms']
                    ])
            
            # Log integration event
            if self.detailed_logging:
                integration_event = {
                    'timestamp': current_time,
                    'event_type': 'yolo_detections',
                    'detection_count': len(msg.detections),
                    'classes_detected': [getattr(d, 'class_name', 'unknown') for d in msg.detections],
                    'max_confidence': max([getattr(d, 'confidence', 0.0) for d in msg.detections]) if msg.detections else 0.0,
                    'min_distance': min([getattr(d, 'distance', float('inf')) for d in msg.detections]) if msg.detections else float('inf'),
                    'processing_time_ms': getattr(msg, 'processing_time_ms', 0.0)
                }
                self.data_buffers['integration_events'].append(integration_event)
                self.logging_stats['integration_events_logged'] += 1
        
        rospy.logdebug(f"[{self.node_name}] Logged {len(msg.detections)} detections")
    
    def _risk_status_callback(self, msg):
        """Log risk assessment results."""
        current_time = time.time()
        
        with self.lock:
            # Extract risk assessment data
            if hasattr(msg, 'overall_risk_level'):
                risk_data = {
                    'timestamp': current_time,
                    'ros_time': msg.header.stamp.to_sec() if hasattr(msg, 'header') else current_time,
                    'overall_risk_level': msg.overall_risk_level,
                    'critical_objects': getattr(msg, 'critical_objects', 0),
                    'min_distance': getattr(msg, 'min_distance', float('inf')),
                    'min_ttc': getattr(msg, 'min_ttc', float('inf')),
                    'avoidance_required': getattr(msg, 'avoidance_required', False),
                    'emergency_stop_required': getattr(msg, 'emergency_stop_required', False),
                    'risk_factors': getattr(msg, 'risk_factors', []),
                    'assessment_time_ms': getattr(msg, 'assessment_time_ms', 0.0)
                }
            else:
                # Fallback for basic message types
                risk_data = {
                    'timestamp': current_time,
                    'ros_time': current_time,
                    'overall_risk_level': 'unknown',
                    'critical_objects': 0,
                    'min_distance': float('inf'),
                    'min_ttc': float('inf'),
                    'avoidance_required': False,
                    'emergency_stop_required': False,
                    'risk_factors': [],
                    'assessment_time_ms': 0.0
                }
            
            self.data_buffers['risk_assessments'].append(risk_data)
            self.logging_stats['risk_assessments_logged'] += 1
            
            # Write to CSV
            if 'risk_assessments' in self.csv_writers:
                _, writer = self.csv_writers['risk_assessments']
                writer.writerow([
                    risk_data['timestamp'],
                    risk_data['ros_time'],
                    risk_data['overall_risk_level'],
                    risk_data['critical_objects'],
                    risk_data['min_distance'],
                    risk_data['min_ttc'],
                    risk_data['avoidance_required'],
                    risk_data['emergency_stop_required'],
                    json.dumps(risk_data['risk_factors']),
                    risk_data['assessment_time_ms']
                ])
            
            # Log integration event
            if self.detailed_logging and risk_data['overall_risk_level'] in ['HIGH', 'CRITICAL']:
                integration_event = {
                    'timestamp': current_time,
                    'event_type': 'high_risk_detected',
                    'risk_level': risk_data['overall_risk_level'],
                    'critical_objects': risk_data['critical_objects'],
                    'min_distance': risk_data['min_distance'],
                    'avoidance_required': risk_data['avoidance_required'],
                    'emergency_required': risk_data['emergency_stop_required']
                }
                self.data_buffers['integration_events'].append(integration_event)
                self.logging_stats['integration_events_logged'] += 1
        
        rospy.logdebug(f"[{self.node_name}] Logged risk assessment: {risk_data.get('overall_risk_level', 'unknown')}")
    
    def _coordination_callback(self, msg):
        """Log coordination decisions and behavior switches."""
        current_time = time.time()
        
        with self.lock:
            # Extract coordination data
            if hasattr(msg, 'behavior'):
                coord_data = {
                    'timestamp': current_time,
                    'ros_time': msg.header.stamp.to_sec() if hasattr(msg, 'header') else current_time,
                    'current_behavior': msg.behavior,
                    'previous_behavior': getattr(msg, 'previous_behavior', 'unknown'),
                    'behavior_switch': getattr(msg, 'behavior_switch', False),
                    'decision_factors': getattr(msg, 'decision_factors', []),
                    'control_v': getattr(msg, 'control_v', 0.0),
                    'control_omega': getattr(msg, 'control_omega', 0.0),
                    'has_detections': getattr(msg, 'has_detections', False),
                    'has_navigation': getattr(msg, 'has_navigation', False),
                    'has_lane_pose': getattr(msg, 'has_lane_pose', False),
                    'emergency_active': getattr(msg, 'emergency_active', False),
                    'coordination_time_ms': getattr(msg, 'coordination_time_ms', 0.0)
                }
            else:
                # Fallback for basic message types
                coord_data = {
                    'timestamp': current_time,
                    'ros_time': current_time,
                    'current_behavior': 'unknown',
                    'previous_behavior': 'unknown',
                    'behavior_switch': False,
                    'decision_factors': [],
                    'control_v': 0.0,
                    'control_omega': 0.0,
                    'has_detections': False,
                    'has_navigation': False,
                    'has_lane_pose': False,
                    'emergency_active': False,
                    'coordination_time_ms': 0.0
                }
            
            self.data_buffers['coordination'].append(coord_data)
            self.logging_stats['coordination_decisions_logged'] += 1
            
            # Write to CSV
            if 'coordination' in self.csv_writers:
                _, writer = self.csv_writers['coordination']
                writer.writerow([
                    coord_data['timestamp'],
                    coord_data['ros_time'],
                    coord_data['current_behavior'],
                    coord_data['previous_behavior'],
                    coord_data['behavior_switch'],
                    json.dumps(coord_data['decision_factors']),
                    coord_data['control_v'],
                    coord_data['control_omega'],
                    coord_data['has_detections'],
                    coord_data['has_navigation'],
                    coord_data['has_lane_pose'],
                    coord_data['emergency_active'],
                    coord_data['coordination_time_ms']
                ])
            
            # Log integration event for behavior switches
            if self.detailed_logging and coord_data['behavior_switch']:
                integration_event = {
                    'timestamp': current_time,
                    'event_type': 'behavior_switch',
                    'from_behavior': coord_data['previous_behavior'],
                    'to_behavior': coord_data['current_behavior'],
                    'decision_factors': coord_data['decision_factors'],
                    'emergency_triggered': coord_data['emergency_active']
                }
                self.data_buffers['integration_events'].append(integration_event)
                self.logging_stats['integration_events_logged'] += 1
        
        rospy.logdebug(f"[{self.node_name}] Logged coordination decision: {coord_data.get('current_behavior', 'unknown')}")
    
    def _system_status_callback(self, msg):
        """Log system status and health information."""
        current_time = time.time()
        
        with self.lock:
            # Extract system status data
            if hasattr(msg, 'active'):
                status_data = {
                    'timestamp': current_time,
                    'ros_time': msg.header.stamp.to_sec() if hasattr(msg, 'header') else current_time,
                    'system_active': msg.active,
                    'overall_health': getattr(msg, 'health_score', 0.0),
                    'yolo_active': getattr(msg, 'yolo_active', False),
                    'navigation_active': getattr(msg, 'navigation_active', False),
                    'integration_active': getattr(msg, 'integration_active', False),
                    'cpu_percent': getattr(msg, 'cpu_percent', 0.0),
                    'memory_percent': getattr(msg, 'memory_percent', 0.0),
                    'fps': getattr(msg, 'fps', 0.0),
                    'avg_latency_ms': getattr(msg, 'avg_latency_ms', 0.0)
                }
            else:
                # Fallback for basic message types
                status_data = {
                    'timestamp': current_time,
                    'ros_time': current_time,
                    'system_active': True,
                    'overall_health': 0.5,
                    'yolo_active': False,
                    'navigation_active': False,
                    'integration_active': False,
                    'cpu_percent': 0.0,
                    'memory_percent': 0.0,
                    'fps': 0.0,
                    'avg_latency_ms': 0.0
                }
            
            self.data_buffers['system_status'].append(status_data)
            self.logging_stats['system_status_logged'] += 1
            
            # Write to CSV
            if 'system_status' in self.csv_writers:
                _, writer = self.csv_writers['system_status']
                writer.writerow([
                    status_data['timestamp'],
                    status_data['ros_time'],
                    status_data['system_active'],
                    status_data['overall_health'],
                    status_data['yolo_active'],
                    status_data['navigation_active'],
                    status_data['integration_active'],
                    status_data['cpu_percent'],
                    status_data['memory_percent'],
                    status_data['fps'],
                    status_data['avg_latency_ms']
                ])
            
            # Log integration event for health issues
            if self.detailed_logging and status_data['overall_health'] < 0.5:
                integration_event = {
                    'timestamp': current_time,
                    'event_type': 'low_system_health',
                    'health_score': status_data['overall_health'],
                    'cpu_percent': status_data['cpu_percent'],
                    'memory_percent': status_data['memory_percent'],
                    'fps': status_data['fps']
                }
                self.data_buffers['integration_events'].append(integration_event)
                self.logging_stats['integration_events_logged'] += 1
        
        rospy.logdebug(f"[{self.node_name}] Logged system status: health={status_data.get('overall_health', 0.0):.2f}")
    
    def _flush_data_callback(self, event):
        """Flush buffered data to files."""
        with self.lock:
            # Flush CSV files
            for file_type, (file_handle, _) in self.csv_writers.items():
                try:
                    file_handle.flush()
                except Exception as e:
                    rospy.logwarn(f"[{self.node_name}] Error flushing {file_type} CSV: {e}")
            
            # Write integration events to JSON file
            if self.data_buffers['integration_events']:
                try:
                    events_to_write = list(self.data_buffers['integration_events'])
                    with open(self.log_files['integration_events'], 'a') as f:
                        for event in events_to_write:
                            json.dump(event, f)
                            f.write('\n')
                    
                    # Clear the buffer after writing
                    self.data_buffers['integration_events'].clear()
                    
                except Exception as e:
                    rospy.logwarn(f"[{self.node_name}] Error writing integration events: {e}")
    
    def _stats_callback(self, event):
        """Report logging statistics."""
        current_time = time.time()
        runtime = current_time - self.logging_stats['start_time']
        
        rospy.loginfo(f"[{self.node_name}] Logging Statistics (Runtime: {runtime:.1f}s):")
        rospy.loginfo(f"[{self.node_name}]   Detections logged: {self.logging_stats['detections_logged']}")
        rospy.loginfo(f"[{self.node_name}]   Risk assessments logged: {self.logging_stats['risk_assessments_logged']}")
        rospy.loginfo(f"[{self.node_name}]   Coordination decisions logged: {self.logging_stats['coordination_decisions_logged']}")
        rospy.loginfo(f"[{self.node_name}]   System status logged: {self.logging_stats['system_status_logged']}")
        rospy.loginfo(f"[{self.node_name}]   Integration events logged: {self.logging_stats['integration_events_logged']}")
        
        # Check log file sizes
        for log_type, log_path in self.log_files.items():
            if os.path.exists(log_path):
                size_mb = os.path.getsize(log_path) / (1024 * 1024)
                rospy.loginfo(f"[{self.node_name}]   {log_type} log size: {size_mb:.1f} MB")
    
    def _log_rotation_callback(self, event):
        """Check if log rotation is needed."""
        if not self.log_rotation_enabled:
            return
        
        try:
            for log_type, log_path in self.log_files.items():
                if os.path.exists(log_path):
                    size_mb = os.path.getsize(log_path) / (1024 * 1024)
                    if size_mb > self.max_log_size_mb:
                        rospy.loginfo(f"[{self.node_name}] Rotating {log_type} log (size: {size_mb:.1f} MB)")
                        self._rotate_log_file(log_type, log_path)
        
        except Exception as e:
            rospy.logwarn(f"[{self.node_name}] Error during log rotation: {e}")
    
    def _rotate_log_file(self, log_type: str, log_path: str):
        """Rotate a log file when it gets too large."""
        try:
            # Close current CSV writer if it exists
            if log_type in self.csv_writers:
                file_handle, _ = self.csv_writers[log_type]
                file_handle.close()
                del self.csv_writers[log_type]
            
            # Rename current log file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            rotated_path = log_path.replace('.csv', f'_rotated_{timestamp}.csv')
            rotated_path = rotated_path.replace('.json', f'_rotated_{timestamp}.json')
            os.rename(log_path, rotated_path)
            
            # Create new log file
            new_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            if log_type.endswith('.csv'):
                new_path = log_path.replace(log_path.split('_')[-1], f"{new_timestamp}.csv")
            else:
                new_path = log_path.replace(log_path.split('_')[-1], f"{new_timestamp}.json")
            
            self.log_files[log_type] = new_path
            
            # Recreate CSV writer if needed
            if log_type in ['detections', 'risk_assessments', 'coordination', 'system_status']:
                self._recreate_csv_writer(log_type)
            
            rospy.loginfo(f"[{self.node_name}] Log rotation complete: {log_type}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error rotating log file {log_type}: {e}")
    
    def _recreate_csv_writer(self, log_type: str):
        """Recreate a CSV writer after log rotation."""
        try:
            file_handle = open(self.log_files[log_type], 'w', newline='')
            writer = csv.writer(file_handle)
            
            # Write appropriate header
            if log_type == 'detections':
                writer.writerow([
                    'timestamp', 'ros_time', 'detection_count', 'class_name', 
                    'confidence', 'bbox_x', 'bbox_y', 'bbox_width', 'bbox_height',
                    'distance', 'relative_velocity', 'risk_level', 'processing_time_ms'
                ])
            elif log_type == 'risk_assessments':
                writer.writerow([
                    'timestamp', 'ros_time', 'overall_risk_level', 'critical_objects',
                    'min_distance', 'min_ttc', 'avoidance_required', 'emergency_stop_required',
                    'risk_factors', 'assessment_time_ms'
                ])
            elif log_type == 'coordination':
                writer.writerow([
                    'timestamp', 'ros_time', 'current_behavior', 'previous_behavior',
                    'behavior_switch', 'decision_factors', 'control_v', 'control_omega',
                    'has_detections', 'has_navigation', 'has_lane_pose', 'emergency_active',
                    'coordination_time_ms'
                ])
            elif log_type == 'system_status':
                writer.writerow([
                    'timestamp', 'ros_time', 'system_active', 'overall_health',
                    'yolo_active', 'navigation_active', 'integration_active',
                    'cpu_percent', 'memory_percent', 'fps', 'avg_latency_ms'
                ])
            
            self.csv_writers[log_type] = (file_handle, writer)
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error recreating CSV writer for {log_type}: {e}")
    
    def shutdown(self):
        """Clean shutdown of the data logger."""
        rospy.loginfo(f"[{self.node_name}] Shutting down integration data logger...")
        
        with self.lock:
            # Flush all remaining data
            self._flush_data_callback(None)
            
            # Close all CSV files
            for log_type, (file_handle, _) in self.csv_writers.items():
                try:
                    file_handle.close()
                    rospy.loginfo(f"[{self.node_name}] Closed {log_type} log file")
                except Exception as e:
                    rospy.logwarn(f"[{self.node_name}] Error closing {log_type} log: {e}")
            
            # Final statistics
            rospy.loginfo(f"[{self.node_name}] Final logging statistics:")
            for stat_name, stat_value in self.logging_stats.items():
                if stat_name != 'start_time':
                    rospy.loginfo(f"[{self.node_name}]   {stat_name}: {stat_value}")
        
        rospy.loginfo(f"[{self.node_name}] Integration data logger shutdown complete")


def main():
    """Main function to run the integration data logger."""
    try:
        logger = IntegrationDataLogger()
        rospy.loginfo("Integration Data Logger running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Integration Data Logger interrupted")
    except Exception as e:
        rospy.logerr(f"Integration Data Logger error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'logger' in locals():
            logger.shutdown()


if __name__ == '__main__':
    main()
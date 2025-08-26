#!/usr/bin/env python3
"""
Unit tests for FSM safety monitoring functionality.
Tests multi-layer safety monitoring including hardware, sensor, algorithm, and behavioral safety.
"""

import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock

import rospy
import rostest
from duckietown_msgs.msg import SafetyStatus, FSMState
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

# Import the classes we're testing
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from fsm_node import SafetyMonitor, FSMNode


class TestSafetyMonitor(unittest.TestCase):
    """Test cases for SafetyMonitor class."""
    
    def setUp(self):
        """Set up test fixtures."""
        rospy.init_node('test_safety_monitor', anonymous=True)
        self.node_name = "test_safety_monitor"
        
        # Mock rospy parameters
        with patch('rospy.get_param') as mock_param:
            mock_param.side_effect = lambda key, default=None: {
                "~cpu_temp_warning": 70.0,
                "~cpu_temp_critical": 85.0,
                "~memory_warning": 80.0,
                "~memory_critical": 95.0,
                "~sensor_timeout": 2.0,
                "~algorithm_fps_warning": 15.0,
                "~algorithm_fps_critical": 10.0
            }.get(key, default)
            
            self.safety_monitor = SafetyMonitor(self.node_name)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if hasattr(self.safety_monitor, 'monitoring_active'):
            self.safety_monitor.stop_monitoring()
    
    def test_safety_monitor_initialization(self):
        """Test SafetyMonitor initialization."""
        self.assertEqual(self.safety_monitor.node_name, self.node_name)
        self.assertEqual(self.safety_monitor.cpu_temp_warning_threshold, 70.0)
        self.assertEqual(self.safety_monitor.cpu_temp_critical_threshold, 85.0)
        self.assertEqual(self.safety_monitor.memory_warning_threshold, 80.0)
        self.assertEqual(self.safety_monitor.memory_critical_threshold, 95.0)
        self.assertEqual(self.safety_monitor.sensor_timeout_threshold, 2.0)
        self.assertEqual(self.safety_monitor.algorithm_fps_warning, 15.0)
        self.assertEqual(self.safety_monitor.algorithm_fps_critical, 10.0)
        
        # Check initial safety status
        self.assertEqual(self.safety_monitor.safety_status.safety_level, SafetyStatus.SAFE)
        self.assertEqual(self.safety_monitor.safety_status.hardware_health, SafetyStatus.HEALTH_OK)
        self.assertFalse(self.safety_monitor.safety_status.emergency_stop_active)
        self.assertEqual(self.safety_monitor.safety_status.system_health_score, 100.0)
    
    @patch('psutil.virtual_memory')
    @patch.object(SafetyMonitor, '_get_cpu_temperature')
    def test_hardware_health_monitoring(self, mock_cpu_temp, mock_memory):
        """Test hardware health monitoring functionality."""
        # Test normal conditions
        mock_cpu_temp.return_value = 65.0
        mock_memory.return_value = Mock(percent=75.0)
        
        self.safety_monitor._monitor_hardware_health()
        
        self.assertEqual(self.safety_monitor.safety_status.hardware_health, SafetyStatus.HEALTH_OK)
        self.assertEqual(self.safety_monitor.safety_status.cpu_temperature, 65.0)
        self.assertEqual(self.safety_monitor.safety_status.memory_usage, 75.0)
        
        # Test warning conditions
        mock_cpu_temp.return_value = 75.0
        mock_memory.return_value = Mock(percent=85.0)
        
        self.safety_monitor._monitor_hardware_health()
        
        self.assertEqual(self.safety_monitor.safety_status.hardware_health, SafetyStatus.HEALTH_WARNING)
        self.assertTrue(any("CPU temperature warning" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        self.assertTrue(any("Memory usage warning" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        
        # Test critical conditions
        mock_cpu_temp.return_value = 90.0
        mock_memory.return_value = Mock(percent=98.0)
        
        self.safety_monitor.safety_status.active_warnings = []  # Clear previous warnings
        self.safety_monitor._monitor_hardware_health()
        
        self.assertEqual(self.safety_monitor.safety_status.hardware_health, SafetyStatus.HEALTH_CRITICAL)
        self.assertTrue(any("CPU temperature critical" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        self.assertTrue(any("Memory usage critical" in warning for warning in self.safety_monitor.safety_status.active_warnings))
    
    def test_sensor_status_monitoring(self):
        """Test sensor status monitoring functionality."""
        current_time = rospy.Time.now()
        
        # Test normal sensor operation
        self.safety_monitor.last_camera_time = current_time
        self.safety_monitor.last_imu_time = current_time
        self.safety_monitor.last_encoder_time = current_time
        
        self.safety_monitor._monitor_sensor_status()
        
        self.assertEqual(self.safety_monitor.safety_status.camera_status, SafetyStatus.SENSOR_OK)
        self.assertEqual(self.safety_monitor.safety_status.imu_status, SafetyStatus.SENSOR_OK)
        self.assertEqual(self.safety_monitor.safety_status.encoder_status, SafetyStatus.SENSOR_OK)
        
        # Test sensor timeout
        old_time = rospy.Time(current_time.secs - 5, current_time.nsecs)  # 5 seconds ago
        self.safety_monitor.last_camera_time = old_time
        self.safety_monitor.last_imu_time = old_time
        self.safety_monitor.last_encoder_time = old_time
        
        self.safety_monitor.safety_status.active_warnings = []  # Clear previous warnings
        self.safety_monitor._monitor_sensor_status()
        
        self.assertEqual(self.safety_monitor.safety_status.camera_status, SafetyStatus.SENSOR_FAILED)
        self.assertEqual(self.safety_monitor.safety_status.imu_status, SafetyStatus.SENSOR_FAILED)
        self.assertEqual(self.safety_monitor.safety_status.encoder_status, SafetyStatus.SENSOR_FAILED)
        self.assertTrue(any("Camera timeout" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        self.assertTrue(any("IMU timeout" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        self.assertTrue(any("Encoder timeout" in warning for warning in self.safety_monitor.safety_status.active_warnings))
    
    def test_algorithm_performance_monitoring(self):
        """Test algorithm performance monitoring functionality."""
        # Test normal performance
        self.safety_monitor.lane_detection_confidence = 0.8
        self.safety_monitor.object_detection_fps = 20.0
        self.safety_monitor.control_loop_frequency = 15.0
        
        self.safety_monitor._monitor_algorithm_performance()
        
        self.assertEqual(self.safety_monitor.safety_status.lane_detection_confidence, 0.8)
        self.assertEqual(self.safety_monitor.safety_status.object_detection_fps, 20.0)
        self.assertEqual(self.safety_monitor.safety_status.control_loop_frequency, 15.0)
        
        # Test poor performance
        self.safety_monitor.lane_detection_confidence = 0.3
        self.safety_monitor.object_detection_fps = 8.0
        self.safety_monitor.control_loop_frequency = 5.0
        
        self.safety_monitor.safety_status.active_warnings = []  # Clear previous warnings
        self.safety_monitor._monitor_algorithm_performance()
        
        self.assertTrue(any("Low lane detection confidence" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        self.assertTrue(any("Critical object detection FPS" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        self.assertTrue(any("Low control loop frequency" in warning for warning in self.safety_monitor.safety_status.active_warnings))
    
    def test_behavioral_safety_monitoring(self):
        """Test behavioral safety monitoring functionality."""
        # Test normal behavior
        self.safety_monitor._monitor_behavioral_safety()
        self.assertEqual(len(self.safety_monitor.safety_status.active_warnings), 0)
        
        # Test safety violations
        self.safety_monitor.add_safety_violation("Test violation 1")
        self.safety_monitor.add_safety_violation("Test violation 2")
        
        self.safety_monitor.safety_status.active_warnings = []  # Clear previous warnings
        self.safety_monitor._monitor_behavioral_safety()
        
        self.assertTrue(any("Behavioral violation: Test violation 1" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        self.assertTrue(any("Behavioral violation: Test violation 2" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        
        # Test emergency triggers
        self.safety_monitor.emergency_triggers = ["Emergency trigger 1", "Emergency trigger 2"]
        
        self.safety_monitor.safety_status.active_warnings = []  # Clear previous warnings
        self.safety_monitor._monitor_behavioral_safety()
        
        self.assertTrue(any("Emergency trigger: Emergency trigger 1" in warning for warning in self.safety_monitor.safety_status.active_warnings))
        self.assertTrue(any("Emergency trigger: Emergency trigger 2" in warning for warning in self.safety_monitor.safety_status.active_warnings))
    
    def test_overall_safety_level_calculation(self):
        """Test overall safety level calculation."""
        # Test safe conditions
        self.safety_monitor.safety_status.hardware_health = SafetyStatus.HEALTH_OK
        self.safety_monitor.safety_status.camera_status = SafetyStatus.SENSOR_OK
        self.safety_monitor.safety_status.imu_status = SafetyStatus.SENSOR_OK
        self.safety_monitor.safety_status.encoder_status = SafetyStatus.SENSOR_OK
        self.safety_monitor.lane_detection_confidence = 0.8
        self.safety_monitor.object_detection_fps = 20.0
        self.safety_monitor.control_loop_frequency = 15.0
        self.safety_monitor.emergency_triggers = []
        self.safety_monitor.safety_status.emergency_stop_active = False
        
        self.safety_monitor._calculate_overall_safety_level()
        
        self.assertEqual(self.safety_monitor.safety_status.safety_level, SafetyStatus.SAFE)
        self.assertEqual(self.safety_monitor.safety_status.system_health_score, 100.0)
        
        # Test warning conditions
        self.safety_monitor.safety_status.hardware_health = SafetyStatus.HEALTH_WARNING
        self.safety_monitor.lane_detection_confidence = 0.4
        
        self.safety_monitor._calculate_overall_safety_level()
        
        self.assertEqual(self.safety_monitor.safety_status.safety_level, SafetyStatus.WARNING)
        self.assertLess(self.safety_monitor.safety_status.system_health_score, 100.0)
        
        # Test critical conditions
        self.safety_monitor.safety_status.hardware_health = SafetyStatus.HEALTH_CRITICAL
        self.safety_monitor.safety_status.camera_status = SafetyStatus.SENSOR_FAILED
        self.safety_monitor.object_detection_fps = 5.0
        
        self.safety_monitor._calculate_overall_safety_level()
        
        self.assertEqual(self.safety_monitor.safety_status.safety_level, SafetyStatus.CRITICAL)
        self.assertLess(self.safety_monitor.safety_status.system_health_score, 30.0)
        
        # Test emergency conditions
        self.safety_monitor.emergency_triggers = ["Test emergency"]
        
        self.safety_monitor._calculate_overall_safety_level()
        
        self.assertEqual(self.safety_monitor.safety_status.safety_level, SafetyStatus.EMERGENCY)
    
    def test_emergency_stop_functionality(self):
        """Test emergency stop trigger and clear functionality."""
        # Test emergency stop trigger
        test_reason = "Test emergency stop"
        self.safety_monitor.trigger_emergency_stop(test_reason)
        
        self.assertTrue(self.safety_monitor.safety_status.emergency_stop_active)
        self.assertEqual(self.safety_monitor.safety_status.emergency_reason, test_reason)
        self.assertIn(test_reason, self.safety_monitor.emergency_triggers)
        
        # Test emergency stop clear
        self.safety_monitor.clear_emergency_stop()
        
        self.assertFalse(self.safety_monitor.safety_status.emergency_stop_active)
        self.assertEqual(self.safety_monitor.safety_status.emergency_reason, "")
        self.assertEqual(self.safety_monitor.recovery_attempts, 1)
    
    def test_sensor_status_updates(self):
        """Test sensor status update functionality."""
        test_timestamp = rospy.Time.now()
        
        # Test camera status update
        self.safety_monitor.update_sensor_status("camera", test_timestamp)
        self.assertEqual(self.safety_monitor.last_camera_time, test_timestamp)
        self.assertEqual(self.safety_monitor.camera_fps_counter, 1)
        
        # Test IMU status update
        self.safety_monitor.update_sensor_status("imu", test_timestamp)
        self.assertEqual(self.safety_monitor.last_imu_time, test_timestamp)
        
        # Test encoder status update
        self.safety_monitor.update_sensor_status("encoder", test_timestamp)
        self.assertEqual(self.safety_monitor.last_encoder_time, test_timestamp)
    
    def test_algorithm_performance_updates(self):
        """Test algorithm performance update functionality."""
        # Test individual updates
        self.safety_monitor.update_algorithm_performance(lane_confidence=0.75)
        self.assertEqual(self.safety_monitor.lane_detection_confidence, 0.75)
        
        self.safety_monitor.update_algorithm_performance(object_fps=18.5)
        self.assertEqual(self.safety_monitor.object_detection_fps, 18.5)
        
        self.safety_monitor.update_algorithm_performance(control_freq=12.3)
        self.assertEqual(self.safety_monitor.control_loop_frequency, 12.3)
        
        # Test combined update
        self.safety_monitor.update_algorithm_performance(
            lane_confidence=0.9,
            object_fps=25.0,
            control_freq=20.0
        )
        self.assertEqual(self.safety_monitor.lane_detection_confidence, 0.9)
        self.assertEqual(self.safety_monitor.object_detection_fps, 25.0)
        self.assertEqual(self.safety_monitor.control_loop_frequency, 20.0)


class TestFSMSafetyIntegration(unittest.TestCase):
    """Test cases for FSM safety monitoring integration."""
    
    def setUp(self):
        """Set up test fixtures."""
        rospy.init_node('test_fsm_safety', anonymous=True)
        
        # Mock rospy parameters for FSM
        self.mock_params = {
            "~states": {
                "LANE_FOLLOWING": {"active_nodes": ["test_node"], "lights": "GREEN"},
                "EMERGENCY_STOP": {"active_nodes": [], "lights": "RED"},
                "SAFE_MODE": {"active_nodes": ["test_node"], "lights": "YELLOW"}
            },
            "~global_transitions": {},
            "~initial_state": "LANE_FOLLOWING",
            "~nodes": {"test_node": "/test_service"},
            "~events": {},
            "~enable_safety_monitoring": True,
            "~emergency_stop_timeout": 0.2,
            "~safety_check_frequency": 1.0,
            "~enable_apriltag_stops": False
        }
        
        with patch('rospy.get_param') as mock_param:
            mock_param.side_effect = lambda key, default=None: self.mock_params.get(key, default)
            
            with patch('rospy.ServiceProxy'), \
                 patch('rospy.wait_for_service'), \
                 patch('rospy.Publisher'), \
                 patch('rospy.Subscriber'), \
                 patch('rospy.Timer'):
                
                self.fsm_node = FSMNode()
    
    def tearDown(self):
        """Clean up test fixtures."""
        if hasattr(self.fsm_node, 'safety_monitor'):
            self.fsm_node.safety_monitor.stop_monitoring()
    
    def test_fsm_safety_initialization(self):
        """Test FSM safety monitoring initialization."""
        self.assertTrue(self.fsm_node.enable_safety_monitoring)
        self.assertEqual(self.fsm_node.emergency_stop_timeout, 0.2)
        self.assertEqual(self.fsm_node.safety_check_frequency, 1.0)
        self.assertIsNotNone(self.fsm_node.safety_monitor)
        self.assertFalse(self.fsm_node.safety_emergency_active)
        self.assertEqual(self.fsm_node.safety_state_transitions, 0)
        self.assertEqual(self.fsm_node.emergency_stop_count, 0)
    
    @patch('rospy.logwarn')
    @patch('rospy.loginfo')
    def test_safety_emergency_handling(self, mock_loginfo, mock_logwarn):
        """Test safety emergency handling."""
        # Create mock safety status with emergency condition
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.EMERGENCY
        safety_status.emergency_reason = "Test emergency"
        safety_status.system_health_score = 10.0
        
        # Test emergency handling
        self.fsm_node._handle_safety_emergency(safety_status)
        
        self.assertTrue(self.fsm_node.safety_emergency_active)
        self.assertEqual(self.fsm_node.emergency_stop_count, 1)
        self.assertEqual(self.fsm_node.state_msg.state, "EMERGENCY_STOP")
        self.assertEqual(self.fsm_node.safety_state_transitions, 1)
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("SAFETY EMERGENCY DETECTED" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.logwarn')
    def test_safety_critical_handling(self, mock_logwarn):
        """Test safety critical condition handling."""
        # Create mock safety status with critical condition
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.CRITICAL
        safety_status.system_health_score = 25.0
        safety_status.active_warnings = ["Critical warning 1", "Critical warning 2"]
        
        # Set initial state
        self.fsm_node.state_msg.state = "LANE_FOLLOWING"
        
        # Test critical handling
        self.fsm_node._handle_safety_critical(safety_status)
        
        self.assertEqual(self.fsm_node.state_msg.state, "SAFE_MODE")
        self.assertEqual(self.fsm_node.safety_state_transitions, 1)
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("SAFETY CRITICAL" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.logwarn')
    def test_safety_warning_handling(self, mock_logwarn):
        """Test safety warning condition handling."""
        # Create mock safety status with warning condition
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.WARNING
        safety_status.system_health_score = 65.0
        safety_status.active_warnings = ["Warning 1", "Warning 2"]
        
        # Test warning handling
        self.fsm_node._handle_safety_warning(safety_status)
        
        # State should not change for warnings
        self.assertEqual(self.fsm_node.state_msg.state, "LANE_FOLLOWING")
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("SAFETY WARNING" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.loginfo')
    def test_safety_recovery_handling(self, mock_loginfo):
        """Test safety recovery handling."""
        # Set up emergency state
        self.fsm_node.safety_emergency_active = True
        self.fsm_node.state_msg.state = "EMERGENCY_STOP"
        
        # Create mock safety status with recovered condition
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.WARNING
        safety_status.system_health_score = 75.0
        
        # Test recovery handling
        self.fsm_node._handle_safety_recovery(safety_status)
        
        self.assertFalse(self.fsm_node.safety_emergency_active)
        self.assertEqual(self.fsm_node.state_msg.state, "SAFE_MODE")
        self.assertEqual(self.fsm_node.safety_state_transitions, 1)
        
        # Verify logging
        mock_loginfo.assert_called()
        self.assertTrue(any("SAFETY RECOVERY" in str(call) for call in mock_loginfo.call_args_list))
    
    @patch('rospy.logwarn')
    def test_manual_emergency_stop(self, mock_logwarn):
        """Test manual emergency stop trigger."""
        test_reason = "Manual test emergency"
        
        # Test manual emergency stop
        self.fsm_node.trigger_manual_emergency_stop(test_reason)
        
        self.assertTrue(self.fsm_node.safety_emergency_active)
        self.assertEqual(self.fsm_node.emergency_stop_count, 1)
        self.assertEqual(self.fsm_node.state_msg.state, "EMERGENCY_STOP")
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("Manual emergency stop triggered" in str(call) for call in mock_logwarn.call_args_list))


if __name__ == '__main__':
    # Run the tests
    rostest.rosrun('fsm', 'test_safety_monitoring', TestSafetyMonitor)
    rostest.rosrun('fsm', 'test_fsm_safety_integration', TestFSMSafetyIntegration)
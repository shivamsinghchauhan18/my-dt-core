#!/usr/bin/env python3
"""
Unit tests for LED Emergency Response System functionality.
Tests emergency response patterns, motor control integration, and safety monitoring.
"""

import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock

import rospy
import rostest
from duckietown_enhanced_msgs.msg import SafetyStatus
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# Import the classes we're testing
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from led_emitter_node import EmergencyResponseSystem, LEDEmitterNode


class TestEmergencyResponseSystem(unittest.TestCase):
    """Test cases for EmergencyResponseSystem class."""
    
    def setUp(self):
        """Set up test fixtures."""
        rospy.init_node('test_emergency_response', anonymous=True)
        
        # Create mock LED emitter node
        self.mock_led_emitter = Mock()
        self.mock_led_emitter.node_name = "test_led_emitter"
        self.mock_led_emitter.changePattern = Mock()
        
        # Mock rospy parameters
        with patch('rospy.get_param') as mock_param:
            mock_param.side_effect = lambda key, default=None: {
                "~emergency_response_enabled": True,
                "~emergency_pattern_priority": 100,
                "~warning_pattern_duration": 5.0,
                "~critical_pattern_duration": 10.0,
                "~emergency_motor_stop_timeout": 0.2
            }.get(key, default)
            
            with patch('rospy.Publisher'), \
                 patch('rospy.Subscriber'):
                
                self.emergency_response = EmergencyResponseSystem(self.mock_led_emitter)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if hasattr(self.emergency_response, 'emergency_timer') and self.emergency_response.emergency_timer:
            self.emergency_response.emergency_timer.shutdown()
    
    def test_emergency_response_initialization(self):
        """Test EmergencyResponseSystem initialization."""
        self.assertTrue(self.emergency_response.emergency_response_enabled)
        self.assertEqual(self.emergency_response.emergency_pattern_priority, 100)
        self.assertEqual(self.emergency_response.warning_pattern_duration, 5.0)
        self.assertEqual(self.emergency_response.critical_pattern_duration, 10.0)
        self.assertEqual(self.emergency_response.emergency_motor_stop_timeout, 0.2)
        
        # Check initial state
        self.assertEqual(self.emergency_response.current_emergency_level, SafetyStatus.SAFE)
        self.assertFalse(self.emergency_response.emergency_pattern_active)
        self.assertFalse(self.emergency_response.motor_stop_active)
        self.assertEqual(self.emergency_response.emergency_pattern_count, 0)
        self.assertEqual(self.emergency_response.warning_pattern_count, 0)
        self.assertEqual(self.emergency_response.critical_pattern_count, 0)
    
    @patch('rospy.logwarn')
    def test_emergency_response_activation(self, mock_logwarn):
        """Test emergency response activation."""
        # Create emergency safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.EMERGENCY
        safety_status.emergency_reason = "Test emergency"
        safety_status.system_health_score = 10.0
        
        # Test emergency response
        self.emergency_response._handle_emergency_response(safety_status)
        
        self.assertTrue(self.emergency_response.emergency_pattern_active)
        self.assertTrue(self.emergency_response.motor_stop_active)
        self.assertEqual(self.emergency_response.emergency_pattern_count, 1)
        self.assertIsNotNone(self.emergency_response.emergency_start_time)
        
        # Verify LED pattern change
        self.mock_led_emitter.changePattern.assert_called_with("EMERGENCY_STOP")
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("EMERGENCY RESPONSE ACTIVATED" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.logwarn')
    def test_critical_response_activation(self, mock_logwarn):
        """Test critical response activation."""
        # Create critical safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.CRITICAL
        safety_status.system_health_score = 25.0
        safety_status.active_warnings = ["Critical warning"]
        
        # Test critical response
        self.emergency_response._handle_critical_response(safety_status)
        
        self.assertTrue(self.emergency_response.emergency_pattern_active)
        self.assertFalse(self.emergency_response.motor_stop_active)  # No motor stop for critical
        self.assertEqual(self.emergency_response.critical_pattern_count, 1)
        
        # Verify LED pattern change
        self.mock_led_emitter.changePattern.assert_called_with("CRITICAL_WARNING")
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("CRITICAL RESPONSE ACTIVATED" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.logwarn')
    def test_warning_response_activation(self, mock_logwarn):
        """Test warning response activation."""
        # Create warning safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.WARNING
        safety_status.system_health_score = 65.0
        safety_status.active_warnings = ["Warning message"]
        
        # Test warning response
        self.emergency_response._handle_warning_response(safety_status)
        
        self.assertTrue(self.emergency_response.emergency_pattern_active)
        self.assertFalse(self.emergency_response.motor_stop_active)  # No motor stop for warning
        self.assertEqual(self.emergency_response.warning_pattern_count, 1)
        
        # Verify LED pattern change
        self.mock_led_emitter.changePattern.assert_called_with("SAFETY_WARNING")
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("WARNING RESPONSE ACTIVATED" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.loginfo')
    def test_recovery_response(self, mock_loginfo):
        """Test recovery from emergency conditions."""
        # Set up emergency state
        self.emergency_response.emergency_pattern_active = True
        self.emergency_response.motor_stop_active = True
        self.emergency_response.emergency_start_time = time.time() - 5.0  # 5 seconds ago
        
        # Create recovery safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.SAFE
        safety_status.system_health_score = 85.0
        
        # Test recovery response
        self.emergency_response._handle_recovery_response(safety_status)
        
        self.assertFalse(self.emergency_response.emergency_pattern_active)
        self.assertFalse(self.emergency_response.motor_stop_active)
        
        # Verify LED pattern change back to normal
        self.mock_led_emitter.changePattern.assert_called_with("CAR_DRIVING")
        
        # Verify logging
        mock_loginfo.assert_called()
        self.assertTrue(any("EMERGENCY RECOVERY" in str(call) for call in mock_loginfo.call_args_list))
    
    def test_emergency_pattern_activation(self):
        """Test emergency pattern activation functionality."""
        pattern_name = "EMERGENCY_STOP"
        
        # Test pattern activation
        self.emergency_response._activate_emergency_pattern(pattern_name)
        
        self.assertTrue(self.emergency_response.emergency_pattern_active)
        self.mock_led_emitter.changePattern.assert_called_with(pattern_name)
    
    def test_emergency_pattern_deactivation(self):
        """Test emergency pattern deactivation functionality."""
        # Set up active pattern
        self.emergency_response.emergency_pattern_active = True
        self.emergency_response.emergency_start_time = time.time()
        
        # Test pattern deactivation
        self.emergency_response._deactivate_emergency_pattern()
        
        self.assertFalse(self.emergency_response.emergency_pattern_active)
        self.assertIsNone(self.emergency_response.emergency_start_time)
        self.mock_led_emitter.changePattern.assert_called_with("CAR_DRIVING")
    
    @patch('rospy.Publisher')
    def test_emergency_motor_stop(self, mock_publisher):
        """Test emergency motor stop functionality."""
        # Mock publishers
        mock_stop_pub = Mock()
        mock_cmd_pub = Mock()
        self.emergency_response.motor_stop_publisher = mock_stop_pub
        self.emergency_response.motor_cmd_publisher = mock_cmd_pub
        
        test_reason = "Test emergency stop"
        
        # Test motor stop execution
        self.emergency_response._execute_emergency_motor_stop(test_reason)
        
        self.assertTrue(self.emergency_response.motor_stop_active)
        
        # Verify emergency stop signal published
        mock_stop_pub.publish.assert_called()
        stop_msg = mock_stop_pub.publish.call_args[0][0]
        self.assertTrue(stop_msg.data)
        
        # Verify zero velocity command published
        mock_cmd_pub.publish.assert_called()
        cmd_msg = mock_cmd_pub.publish.call_args[0][0]
        self.assertEqual(cmd_msg.linear.x, 0.0)
        self.assertEqual(cmd_msg.angular.z, 0.0)
    
    @patch('rospy.Publisher')
    def test_emergency_motor_stop_clear(self, mock_publisher):
        """Test emergency motor stop clear functionality."""
        # Mock publishers
        mock_stop_pub = Mock()
        self.emergency_response.motor_stop_publisher = mock_stop_pub
        self.emergency_response.motor_stop_active = True
        
        # Test motor stop clear
        self.emergency_response._clear_emergency_motor_stop()
        
        self.assertFalse(self.emergency_response.motor_stop_active)
        
        # Verify emergency stop clear signal published
        mock_stop_pub.publish.assert_called()
        stop_msg = mock_stop_pub.publish.call_args[0][0]
        self.assertFalse(stop_msg.data)
    
    @patch('rospy.Timer')
    def test_pattern_timeout_scheduling(self, mock_timer):
        """Test pattern timeout scheduling."""
        duration = 5.0
        
        # Test timeout scheduling
        self.emergency_response._schedule_pattern_timeout(duration)
        
        # Verify timer creation
        mock_timer.assert_called()
        timer_args = mock_timer.call_args
        self.assertEqual(timer_args[0][0].to_sec(), duration)
        self.assertTrue(timer_args[1]['oneshot'])
    
    def test_pattern_timeout_callback(self):
        """Test pattern timeout callback."""
        # Set up active pattern in non-emergency state
        self.emergency_response.emergency_pattern_active = True
        self.emergency_response.current_emergency_level = SafetyStatus.WARNING
        
        # Test timeout callback
        self.emergency_response._pattern_timeout_callback(None)
        
        # Pattern should be deactivated for non-emergency states
        self.assertFalse(self.emergency_response.emergency_pattern_active)
        
        # Test timeout callback in emergency state
        self.emergency_response.emergency_pattern_active = True
        self.emergency_response.current_emergency_level = SafetyStatus.EMERGENCY
        
        self.emergency_response._pattern_timeout_callback(None)
        
        # Pattern should remain active for emergency states
        self.assertTrue(self.emergency_response.emergency_pattern_active)
    
    def test_safety_status_callback(self):
        """Test safety status callback functionality."""
        # Create safety status message
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.WARNING
        safety_status.system_health_score = 70.0
        safety_status.active_warnings = ["Test warning"]
        
        # Test callback
        self.emergency_response.cb_safety_status(safety_status)
        
        self.assertEqual(self.emergency_response.current_emergency_level, SafetyStatus.WARNING)
        self.assertEqual(self.emergency_response.last_safety_status, safety_status)
    
    def test_emergency_status_reporting(self):
        """Test emergency status reporting."""
        # Set up some state
        self.emergency_response.current_emergency_level = SafetyStatus.CRITICAL
        self.emergency_response.emergency_pattern_active = True
        self.emergency_response.motor_stop_active = False
        self.emergency_response.emergency_pattern_count = 2
        self.emergency_response.warning_pattern_count = 5
        self.emergency_response.critical_pattern_count = 3
        
        # Get status
        status = self.emergency_response.get_emergency_status()
        
        self.assertEqual(status["emergency_level"], SafetyStatus.CRITICAL)
        self.assertTrue(status["pattern_active"])
        self.assertFalse(status["motor_stop_active"])
        self.assertEqual(status["emergency_count"], 2)
        self.assertEqual(status["warning_count"], 5)
        self.assertEqual(status["critical_count"], 3)
    
    @patch('rospy.loginfo')
    def test_emergency_response_shutdown(self, mock_loginfo):
        """Test emergency response system shutdown."""
        # Set up active emergency state
        self.emergency_response.emergency_pattern_active = True
        self.emergency_response.motor_stop_active = True
        self.emergency_response.emergency_pattern_count = 3
        self.emergency_response.critical_pattern_count = 2
        self.emergency_response.warning_pattern_count = 5
        
        # Test shutdown
        self.emergency_response.shutdown()
        
        self.assertFalse(self.emergency_response.emergency_pattern_active)
        self.assertFalse(self.emergency_response.motor_stop_active)
        
        # Verify logging
        mock_loginfo.assert_called()
        self.assertTrue(any("Shutting down Emergency Response System" in str(call) for call in mock_loginfo.call_args_list))
        self.assertTrue(any("Emergency patterns: 3" in str(call) for call in mock_loginfo.call_args_list))


class TestLEDEmitterIntegration(unittest.TestCase):
    """Test cases for LED Emitter emergency response integration."""
    
    def setUp(self):
        """Set up test fixtures."""
        rospy.init_node('test_led_emitter_integration', anonymous=True)
        
        # Mock DTROS parent class
        with patch('led_emitter_node.DTROS.__init__'):
            with patch('rospy.get_param') as mock_param:
                mock_param.side_effect = lambda key, default=None: {
                    "~robot_type": "duckiebot",
                    "~LED_protocol": {
                        "colors": {"red": [1, 0, 0], "green": [0, 1, 0], "switchedoff": [0, 0, 0]},
                        "signals": {
                            "LIGHT_OFF": {"color_list": "switchedoff", "frequency": 0, "frequency_mask": []},
                            "EMERGENCY_STOP": {"color_list": "red", "frequency": 10, "frequency_mask": [1,1,1,1,1]},
                            "CAR_DRIVING": {"color_list": ["white","red","white","red","white"], "frequency": 0, "frequency_mask": []}
                        }
                    },
                    "~LED_scale": 0.8,
                    "~channel_order": {"duckiebot": "RGB"}
                }.get(key, default)
                
                with patch('rospy.Publisher'), \
                     patch('rospy.Service'), \
                     patch('rospy.Timer'), \
                     patch('rospy.Subscriber'):
                    
                    self.led_emitter = LEDEmitterNode("test_led_emitter")
    
    def tearDown(self):
        """Clean up test fixtures."""
        if hasattr(self.led_emitter, 'emergency_response'):
            self.led_emitter.emergency_response.shutdown()
    
    def test_led_emitter_emergency_integration(self):
        """Test LED emitter emergency response integration."""
        self.assertIsNotNone(self.led_emitter.emergency_response)
        self.assertIsInstance(self.led_emitter.emergency_response, EmergencyResponseSystem)
        self.assertEqual(self.led_emitter.emergency_response.led_emitter, self.led_emitter)
    
    def test_led_emitter_shutdown_integration(self):
        """Test LED emitter shutdown with emergency response cleanup."""
        # Mock the emergency response shutdown
        self.led_emitter.emergency_response.shutdown = Mock()
        
        # Test shutdown
        self.led_emitter.on_shutdown()
        
        # Verify emergency response shutdown was called
        self.led_emitter.emergency_response.shutdown.assert_called_once()


if __name__ == '__main__':
    # Run the tests
    rostest.rosrun('led_emitter', 'test_emergency_response', TestEmergencyResponseSystem)
    rostest.rosrun('led_emitter', 'test_led_emitter_integration', TestLEDEmitterIntegration)
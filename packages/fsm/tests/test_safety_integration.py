#!/usr/bin/env python3
"""
Integration tests for complete safety monitoring system.
Tests FSM safety monitoring, LED emergency response, and coordination integration.
"""

import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock

import rospy
import rostest
from duckietown_msgs.msg import SafetyStatus, FSMState, LEDPattern
from std_msgs.msg import String, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import MarkerArray

# Import the classes we're testing
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from fsm_node import FSMNode, SafetyMonitor
from safety_status_publisher import SafetyStatusPublisher


class TestSafetySystemIntegration(unittest.TestCase):
    """Test cases for complete safety system integration."""
    
    def setUp(self):
        """Set up test fixtures."""
        rospy.init_node('test_safety_integration', anonymous=True)
        
        # Mock rospy parameters for complete system
        self.mock_params = {
            "~states": {
                "LANE_FOLLOWING": {"active_nodes": ["test_node"], "lights": "GREEN"},
                "EMERGENCY_STOP": {"active_nodes": [], "lights": "RED"},
                "SAFE_MODE": {"active_nodes": ["test_node"], "lights": "YELLOW"},
                "CRITICAL_WARNING": {"active_nodes": ["test_node"], "lights": "ORANGE"}
            },
            "~global_transitions": {},
            "~initial_state": "LANE_FOLLOWING",
            "~nodes": {"test_node": "/test_service"},
            "~events": {},
            "~enable_safety_monitoring": True,
            "~enable_apriltag_stops": False,
            "~emergency_stop_timeout": 0.2,
            "~safety_check_frequency": 2.0,
            "~safety_publish_rate": 1.0,
            "~enable_diagnostics": True,
            "~enable_visualization": True,
            "~enable_system_health": True
        }
        
        # Create mock publishers and subscribers
        self.mock_publishers = {}
        self.mock_subscribers = {}
        
        def mock_publisher_factory(topic, msg_type, **kwargs):
            mock_pub = Mock()
            self.mock_publishers[topic] = mock_pub
            return mock_pub
        
        def mock_subscriber_factory(topic, msg_type, callback, **kwargs):
            mock_sub = Mock()
            mock_sub.callback = callback
            self.mock_subscribers[topic] = mock_sub
            return mock_sub
        
        with patch('rospy.get_param') as mock_param:
            mock_param.side_effect = lambda key, default=None: self.mock_params.get(key, default)
            
            with patch('rospy.ServiceProxy'), \
                 patch('rospy.wait_for_service'), \
                 patch('rospy.Publisher', side_effect=mock_publisher_factory), \
                 patch('rospy.Subscriber', side_effect=mock_subscriber_factory), \
                 patch('rospy.Timer'), \
                 patch('rospy.Service'):
                
                self.fsm_node = FSMNode()
    
    def tearDown(self):
        """Clean up test fixtures."""
        if hasattr(self.fsm_node, 'safety_monitor'):
            self.fsm_node.safety_monitor.stop_monitoring()
        if hasattr(self.fsm_node, 'safety_status_publisher'):
            self.fsm_node.safety_status_publisher.shutdown()
    
    def test_complete_system_initialization(self):
        """Test complete safety system initialization."""
        # Verify FSM components
        self.assertTrue(self.fsm_node.enable_safety_monitoring)
        self.assertIsNotNone(self.fsm_node.safety_monitor)
        self.assertIsNotNone(self.fsm_node.safety_status_publisher)
        
        # Verify safety monitor initialization
        self.assertEqual(self.fsm_node.safety_monitor.node_name, self.fsm_node.node_name)
        self.assertTrue(self.fsm_node.safety_monitor.monitoring_active)
        
        # Verify safety status publisher initialization
        self.assertEqual(self.fsm_node.safety_status_publisher.node_name, self.fsm_node.node_name)
        self.assertTrue(self.fsm_node.safety_status_publisher.enable_diagnostics)
        self.assertTrue(self.fsm_node.safety_status_publisher.enable_visualization)
        
        # Verify publishers were created
        expected_topics = [
            "~safety_status",
            "/diagnostics", 
            "~system_health_score",
            "~emergency_active",
            "~safety_level",
            "~safety_visualization",
            "~health_indicator"
        ]
        
        for topic in expected_topics:
            self.assertIn(topic, self.mock_publishers)
    
    @patch('rospy.logwarn')
    @patch('rospy.loginfo')
    def test_emergency_workflow_integration(self, mock_loginfo, mock_logwarn):
        """Test complete emergency workflow integration."""
        # Create emergency safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.EMERGENCY
        safety_status.emergency_reason = "Integration test emergency"
        safety_status.system_health_score = 15.0
        safety_status.emergency_stop_active = True
        safety_status.active_warnings = ["Critical system failure"]
        
        # Simulate emergency detection
        self.fsm_node.safety_monitor.trigger_emergency_stop("Integration test emergency")
        
        # Handle emergency through FSM
        self.fsm_node._handle_safety_emergency(safety_status)
        
        # Verify FSM state transition
        self.assertTrue(self.fsm_node.safety_emergency_active)
        self.assertEqual(self.fsm_node.state_msg.state, "EMERGENCY_STOP")
        self.assertEqual(self.fsm_node.emergency_stop_count, 1)
        
        # Publish safety status through integrated system
        self.fsm_node.safety_status_publisher.publish_safety_status(safety_status)
        
        # Verify safety status publishing
        safety_pub = self.mock_publishers.get("~safety_status")
        self.assertIsNotNone(safety_pub)
        safety_pub.publish.assert_called()
        
        # Verify diagnostic publishing
        diag_pub = self.mock_publishers.get("/diagnostics")
        self.assertIsNotNone(diag_pub)
        diag_pub.publish.assert_called()
        
        # Verify system health publishing
        health_pub = self.mock_publishers.get("~system_health_score")
        emergency_pub = self.mock_publishers.get("~emergency_active")
        level_pub = self.mock_publishers.get("~safety_level")
        
        self.assertIsNotNone(health_pub)
        self.assertIsNotNone(emergency_pub)
        self.assertIsNotNone(level_pub)
        
        health_pub.publish.assert_called()
        emergency_pub.publish.assert_called()
        level_pub.publish.assert_called()
        
        # Verify visualization publishing
        viz_pub = self.mock_publishers.get("~safety_visualization")
        health_indicator_pub = self.mock_publishers.get("~health_indicator")
        
        self.assertIsNotNone(viz_pub)
        self.assertIsNotNone(health_indicator_pub)
        
        viz_pub.publish.assert_called()
        health_indicator_pub.publish.assert_called()
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("SAFETY EMERGENCY DETECTED" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.logwarn')
    def test_critical_workflow_integration(self, mock_logwarn):
        """Test critical safety workflow integration."""
        # Create critical safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.CRITICAL
        safety_status.system_health_score = 25.0
        safety_status.hardware_health = SafetyStatus.HEALTH_CRITICAL
        safety_status.active_warnings = ["High CPU temperature", "Low memory"]
        
        # Handle critical condition through FSM
        self.fsm_node._handle_safety_critical(safety_status)
        
        # Verify FSM state transition
        self.assertEqual(self.fsm_node.state_msg.state, "SAFE_MODE")
        self.assertEqual(self.fsm_node.safety_state_transitions, 1)
        
        # Publish safety status
        self.fsm_node.safety_status_publisher.publish_safety_status(safety_status)
        
        # Verify publishing occurred
        safety_pub = self.mock_publishers.get("~safety_status")
        self.assertIsNotNone(safety_pub)
        safety_pub.publish.assert_called()
        
        # Verify diagnostic level is ERROR for critical
        diag_pub = self.mock_publishers.get("/diagnostics")
        self.assertIsNotNone(diag_pub)
        diag_pub.publish.assert_called()
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("SAFETY CRITICAL" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.logwarn')
    def test_warning_workflow_integration(self, mock_logwarn):
        """Test warning safety workflow integration."""
        # Create warning safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.WARNING
        safety_status.system_health_score = 65.0
        safety_status.active_warnings = ["Sensor timeout warning"]
        
        # Handle warning condition through FSM
        self.fsm_node._handle_safety_warning(safety_status)
        
        # Verify FSM state remains unchanged for warnings
        self.assertEqual(self.fsm_node.state_msg.state, "LANE_FOLLOWING")
        
        # Publish safety status
        self.fsm_node.safety_status_publisher.publish_safety_status(safety_status)
        
        # Verify publishing occurred
        safety_pub = self.mock_publishers.get("~safety_status")
        self.assertIsNotNone(safety_pub)
        safety_pub.publish.assert_called()
        
        # Verify system health score published
        health_pub = self.mock_publishers.get("~system_health_score")
        self.assertIsNotNone(health_pub)
        health_pub.publish.assert_called()
        
        # Verify logging
        mock_logwarn.assert_called()
        self.assertTrue(any("SAFETY WARNING" in str(call) for call in mock_logwarn.call_args_list))
    
    @patch('rospy.loginfo')
    def test_recovery_workflow_integration(self, mock_loginfo):
        """Test recovery workflow integration."""
        # Set up emergency state
        self.fsm_node.safety_emergency_active = True
        self.fsm_node.state_msg.state = "EMERGENCY_STOP"
        
        # Create recovery safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.SAFE
        safety_status.system_health_score = 85.0
        safety_status.emergency_stop_active = False
        
        # Handle recovery through FSM
        self.fsm_node._handle_safety_recovery(safety_status)
        
        # Verify FSM recovery
        self.assertFalse(self.fsm_node.safety_emergency_active)
        self.assertEqual(self.fsm_node.state_msg.state, "SAFE_MODE")
        
        # Publish recovery status
        self.fsm_node.safety_status_publisher.publish_safety_status(safety_status)
        
        # Verify publishing occurred
        safety_pub = self.mock_publishers.get("~safety_status")
        self.assertIsNotNone(safety_pub)
        safety_pub.publish.assert_called()
        
        # Verify emergency status cleared
        emergency_pub = self.mock_publishers.get("~emergency_active")
        self.assertIsNotNone(emergency_pub)
        emergency_pub.publish.assert_called()
        
        # Verify logging
        mock_loginfo.assert_called()
        self.assertTrue(any("SAFETY RECOVERY" in str(call) for call in mock_loginfo.call_args_list))
    
    def test_sensor_health_monitoring_integration(self):
        """Test sensor health monitoring integration."""
        # Simulate sensor updates
        current_time = rospy.Time.now()
        
        # Update sensor status through safety monitor
        self.fsm_node.safety_monitor.update_sensor_status("camera", current_time)
        self.fsm_node.safety_monitor.update_sensor_status("imu", current_time)
        self.fsm_node.safety_monitor.update_sensor_status("encoder", current_time)
        
        # Verify sensor timestamps updated
        self.assertEqual(self.fsm_node.safety_monitor.last_camera_time, current_time)
        self.assertEqual(self.fsm_node.safety_monitor.last_imu_time, current_time)
        self.assertEqual(self.fsm_node.safety_monitor.last_encoder_time, current_time)
        
        # Get safety status
        safety_status = self.fsm_node.safety_monitor.get_safety_status()
        
        # Verify sensor status is OK
        self.assertEqual(safety_status.camera_status, SafetyStatus.SENSOR_OK)
        self.assertEqual(safety_status.imu_status, SafetyStatus.SENSOR_OK)
        self.assertEqual(safety_status.encoder_status, SafetyStatus.SENSOR_OK)
    
    def test_algorithm_performance_monitoring_integration(self):
        """Test algorithm performance monitoring integration."""
        # Update algorithm performance through safety monitor
        self.fsm_node.safety_monitor.update_algorithm_performance(
            lane_confidence=0.85,
            object_fps=22.5,
            control_freq=18.0
        )
        
        # Get safety status
        safety_status = self.fsm_node.safety_monitor.get_safety_status()
        
        # Verify algorithm performance metrics
        self.assertEqual(safety_status.lane_detection_confidence, 0.85)
        self.assertEqual(safety_status.object_detection_fps, 22.5)
        self.assertEqual(safety_status.control_loop_frequency, 18.0)
    
    def test_safety_statistics_integration(self):
        """Test safety statistics integration."""
        # Create multiple safety status updates
        for i in range(5):
            safety_status = SafetyStatus()
            safety_status.safety_level = SafetyStatus.SAFE if i < 3 else SafetyStatus.WARNING
            safety_status.system_health_score = 90.0 - (i * 5.0)
            
            self.fsm_node.safety_status_publisher.publish_safety_status(safety_status)
            time.sleep(0.1)  # Small delay between updates
        
        # Get statistics
        stats = self.fsm_node.safety_status_publisher.get_safety_statistics()
        
        # Verify statistics
        self.assertEqual(stats['total_events'], 5)
        self.assertGreater(stats['avg_health_score'], 0.0)
        self.assertGreaterEqual(stats['recent_warning_count'], 0)
    
    @patch('rospy.loginfo')
    def test_system_shutdown_integration(self, mock_loginfo):
        """Test complete system shutdown integration."""
        # Set up some system state
        self.fsm_node.safety_state_transitions = 3
        self.fsm_node.emergency_stop_count = 1
        
        # Shutdown system
        self.fsm_node.on_shutdown()
        
        # Verify logging
        mock_loginfo.assert_called()
        self.assertTrue(any("Final safety statistics" in str(call) for call in mock_loginfo.call_args_list))
        self.assertTrue(any("Safety state transitions: 3" in str(call) for call in mock_loginfo.call_args_list))
        self.assertTrue(any("Emergency stop count: 1" in str(call) for call in mock_loginfo.call_args_list))


class TestSafetyStatusPublisher(unittest.TestCase):
    """Test cases for SafetyStatusPublisher class."""
    
    def setUp(self):
        """Set up test fixtures."""
        rospy.init_node('test_safety_status_publisher', anonymous=True)
        
        # Create mock publishers
        self.mock_publishers = {}
        
        def mock_publisher_factory(topic, msg_type, **kwargs):
            mock_pub = Mock()
            self.mock_publishers[topic] = mock_pub
            return mock_pub
        
        with patch('rospy.get_param') as mock_param:
            mock_param.side_effect = lambda key, default=None: {
                "~safety_publish_rate": 2.0,
                "~enable_diagnostics": True,
                "~enable_visualization": True,
                "~enable_system_health": True,
                "~max_history_length": 50
            }.get(key, default)
            
            with patch('rospy.Publisher', side_effect=mock_publisher_factory), \
                 patch('rospy.Timer'):
                
                self.publisher = SafetyStatusPublisher("test_publisher")
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.publisher.shutdown()
    
    def test_safety_status_publisher_initialization(self):
        """Test SafetyStatusPublisher initialization."""
        self.assertEqual(self.publisher.publish_rate, 2.0)
        self.assertTrue(self.publisher.enable_diagnostics)
        self.assertTrue(self.publisher.enable_visualization)
        self.assertTrue(self.publisher.enable_system_health)
        self.assertEqual(self.publisher.max_history_length, 50)
        
        # Verify publishers created
        expected_topics = [
            "~safety_status",
            "/diagnostics",
            "~system_health_score",
            "~emergency_active",
            "~safety_level",
            "~safety_visualization",
            "~health_indicator"
        ]
        
        for topic in expected_topics:
            self.assertIn(topic, self.mock_publishers)
    
    def test_comprehensive_safety_status_publishing(self):
        """Test comprehensive safety status publishing."""
        # Create comprehensive safety status
        safety_status = SafetyStatus()
        safety_status.safety_level = SafetyStatus.WARNING
        safety_status.system_health_score = 75.0
        safety_status.hardware_health = SafetyStatus.HEALTH_WARNING
        safety_status.cpu_temperature = 72.0
        safety_status.memory_usage = 85.0
        safety_status.camera_status = SafetyStatus.SENSOR_OK
        safety_status.imu_status = SafetyStatus.SENSOR_OK
        safety_status.encoder_status = SafetyStatus.SENSOR_OK
        safety_status.lane_detection_confidence = 0.8
        safety_status.object_detection_fps = 18.0
        safety_status.control_loop_frequency = 16.0
        safety_status.emergency_stop_active = False
        safety_status.active_warnings = ["High CPU temperature", "High memory usage"]
        
        # Publish safety status
        self.publisher.publish_safety_status(safety_status)
        
        # Verify all publishers called
        for topic, pub in self.mock_publishers.items():
            pub.publish.assert_called()
        
        # Verify history updated
        self.assertEqual(len(self.publisher.safety_status_history), 1)
        self.assertEqual(self.publisher.safety_event_count, 1)
        self.assertEqual(self.publisher.last_safety_status, safety_status)


if __name__ == '__main__':
    # Run the tests
    rostest.rosrun('fsm', 'test_safety_system_integration', TestSafetySystemIntegration)
    rostest.rosrun('fsm', 'test_safety_status_publisher', TestSafetyStatusPublisher)
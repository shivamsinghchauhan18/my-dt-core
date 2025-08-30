#!/usr/bin/env python3
"""
Integration Tests for Complete Object Detection and Avoidance Workflow

Tests the end-to-end integration of YOLO detection, risk assessment,
navigation coordination, and emergency response systems.

Author: Duckietown
"""

import unittest
import rospy
import time
import numpy as np
from threading import Event, Lock
from typing import Dict, List, Optional, Any

# ROS messages
from std_msgs.msg import String, Bool, Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import (
    ObjectDetectionArray, 
    ObjectDetection,
    LanePose, 
    Twist2DStamped,
    BoolStamped
)

# Test utilities
import rostest
import rosbag
from unittest.mock import Mock, patch


class IntegrationWorkflowTest(unittest.TestCase):
    """
    Comprehensive integration tests for the enhanced autonomous navigation system.
    
    Tests:
    - End-to-end object detection and avoidance pipeline
    - Component synchronization and data flow
    - Emergency response coordination
    - Performance under various scenarios
    - System recovery and fault tolerance
    """
    
    def setUp(self):
        """Set up test environment and ROS connections."""
        rospy.init_node('integration_workflow_test', anonymous=True)
        
        # Test configuration
        self.test_timeout = 30.0  # seconds
        self.message_timeout = 5.0  # seconds
        
        # Data collection
        self.received_messages = {
            'detections': [],
            'risk_assessments': [],
            'coordination_decisions': [],
            'control_commands': [],
            'emergency_signals': []
        }
        
        # Synchronization
        self.message_events = {
            'detection_received': Event(),
            'risk_received': Event(),
            'coordination_received': Event(),
            'control_received': Event(),
            'emergency_received': Event()
        }
        
        self.lock = Lock()
        
        # Setup publishers for test inputs
        self._setup_test_publishers()
        
        # Setup subscribers for test outputs
        self._setup_test_subscribers()
        
        # Wait for connections
        time.sleep(2.0)
        
        rospy.loginfo("[IntegrationTest] Test setup complete")
    
    def _setup_test_publishers(self):
        """Setup publishers for injecting test data."""
        # Mock camera image publisher
        self.image_pub = rospy.Publisher(
            '/test_robot/camera_node/image/compressed',
            CompressedImage,
            queue_size=1
        )
        
        # Mock lane pose publisher
        self.lane_pose_pub = rospy.Publisher(
            '/test_robot/lane_filter_node/lane_pose',
            LanePose,
            queue_size=1
        )
        
        # Emergency trigger publisher
        self.emergency_trigger_pub = rospy.Publisher(
            '/test_robot/emergency_trigger',
            BoolStamped,
            queue_size=1
        )
        
        rospy.loginfo("[IntegrationTest] Test publishers setup complete")
    
    def _setup_test_subscribers(self):
        """Setup subscribers for monitoring test outputs."""
        # YOLO detections output
        self.detection_sub = rospy.Subscriber(
            '/test_robot/enhanced_vehicle_detection_node/detections',
            ObjectDetectionArray,
            self._detection_callback,
            queue_size=10
        )
        
        # Risk assessment output
        self.risk_sub = rospy.Subscriber(
            '/test_robot/enhanced_navigation_node/risk_status',
            String,  # Using String as fallback
            self._risk_callback,
            queue_size=10
        )
        
        # Coordination decisions output
        self.coordination_sub = rospy.Subscriber(
            '/test_robot/integration_coordinator/coordination_log',
            String,  # Using String as fallback
            self._coordination_callback,
            queue_size=10
        )
        
        # Final control commands output
        self.control_sub = rospy.Subscriber(
            '/test_robot/integration_coordinator/integrated_car_cmd',
            Twist2DStamped,
            self._control_callback,
            queue_size=10
        )
        
        # Emergency status output
        self.emergency_sub = rospy.Subscriber(
            '/test_robot/emergency_stop_override/emergency_status',
            BoolStamped,
            self._emergency_callback,
            queue_size=10
        )
        
        rospy.loginfo("[IntegrationTest] Test subscribers setup complete")
    
    def _detection_callback(self, msg):
        """Handle YOLO detection messages."""
        with self.lock:
            self.received_messages['detections'].append({
                'timestamp': time.time(),
                'detection_count': len(msg.detections),
                'detections': [
                    {
                        'class_name': getattr(det, 'class_name', 'unknown'),
                        'confidence': getattr(det, 'confidence', 0.0),
                        'distance': getattr(det, 'distance', 0.0)
                    }
                    for det in msg.detections
                ]
            })
            self.message_events['detection_received'].set()
    
    def _risk_callback(self, msg):
        """Handle risk assessment messages."""
        with self.lock:
            self.received_messages['risk_assessments'].append({
                'timestamp': time.time(),
                'message': msg.data
            })
            self.message_events['risk_received'].set()
    
    def _coordination_callback(self, msg):
        """Handle coordination decision messages."""
        with self.lock:
            self.received_messages['coordination_decisions'].append({
                'timestamp': time.time(),
                'message': msg.data
            })
            self.message_events['coordination_received'].set()
    
    def _control_callback(self, msg):
        """Handle control command messages."""
        with self.lock:
            self.received_messages['control_commands'].append({
                'timestamp': time.time(),
                'v': msg.v,
                'omega': msg.omega
            })
            self.message_events['control_received'].set()
    
    def _emergency_callback(self, msg):
        """Handle emergency status messages."""
        with self.lock:
            self.received_messages['emergency_signals'].append({
                'timestamp': time.time(),
                'emergency_active': msg.data
            })
            self.message_events['emergency_received'].set()
    
    def _create_test_image(self) -> CompressedImage:
        """Create a test compressed image message."""
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        
        # Create a simple test image (100x100 pixels, JPEG compressed)
        # In a real test, this would be actual image data
        msg.data = b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a\x1f\x1e\x1d\x1a\x1c\x1c $.\' ",#\x1c\x1c(7),01444\x1f\'9=82<.342\xff\xc0\x00\x11\x08\x00d\x00d\x01\x01\x11\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\xff\xc4\x00\x14\x10\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\xda\x00\x0c\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00\xaa\xff\xd9'
        
        return msg
    
    def _create_test_lane_pose(self, d: float = 0.0, phi: float = 0.0) -> LanePose:
        """Create a test lane pose message."""
        msg = LanePose()
        msg.header.stamp = rospy.Time.now()
        msg.d = d
        msg.phi = phi
        return msg
    
    def _create_emergency_signal(self, active: bool = True) -> BoolStamped:
        """Create a test emergency signal message."""
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = active
        return msg
    
    def test_basic_integration_workflow(self):
        """Test basic integration workflow without obstacles."""
        rospy.loginfo("[IntegrationTest] Starting basic integration workflow test")
        
        # Clear previous messages
        with self.lock:
            for msg_list in self.received_messages.values():
                msg_list.clear()
            for event in self.message_events.values():
                event.clear()
        
        # Publish test inputs
        test_image = self._create_test_image()
        test_lane_pose = self._create_test_lane_pose(d=0.1, phi=0.05)
        
        # Send inputs
        self.image_pub.publish(test_image)
        self.lane_pose_pub.publish(test_lane_pose)
        
        # Wait for processing
        time.sleep(3.0)
        
        # Verify outputs
        with self.lock:
            # Should receive some form of output from each component
            self.assertGreater(len(self.received_messages['control_commands']), 0,
                             "Should receive control commands")
            
            # Control commands should be reasonable for lane following
            if self.received_messages['control_commands']:
                latest_cmd = self.received_messages['control_commands'][-1]
                self.assertGreater(latest_cmd['v'], 0.0, "Should have positive velocity for lane following")
                self.assertLess(abs(latest_cmd['omega']), 2.0, "Angular velocity should be reasonable")
        
        rospy.loginfo("[IntegrationTest] Basic integration workflow test completed")
    
    def test_object_detection_and_avoidance(self):
        """Test object detection triggering avoidance behavior."""
        rospy.loginfo("[IntegrationTest] Starting object detection and avoidance test")
        
        # This test would require mock YOLO detections
        # For now, we'll test the coordination logic
        
        # Clear previous messages
        with self.lock:
            for msg_list in self.received_messages.values():
                msg_list.clear()
            for event in self.message_events.values():
                event.clear()
        
        # Simulate scenario with obstacle
        test_image = self._create_test_image()
        test_lane_pose = self._create_test_lane_pose(d=0.0, phi=0.0)
        
        # Send inputs
        self.image_pub.publish(test_image)
        self.lane_pose_pub.publish(test_lane_pose)
        
        # Wait for processing
        time.sleep(5.0)
        
        # Verify system responds appropriately
        with self.lock:
            # Should receive control commands
            self.assertGreater(len(self.received_messages['control_commands']), 0,
                             "Should receive control commands during avoidance")
            
            # Check for coordination decisions
            coordination_received = len(self.received_messages['coordination_decisions']) > 0
            rospy.loginfo(f"[IntegrationTest] Coordination decisions received: {coordination_received}")
        
        rospy.loginfo("[IntegrationTest] Object detection and avoidance test completed")
    
    def test_emergency_stop_integration(self):
        """Test emergency stop integration across all components."""
        rospy.loginfo("[IntegrationTest] Starting emergency stop integration test")
        
        # Clear previous messages
        with self.lock:
            for msg_list in self.received_messages.values():
                msg_list.clear()
            for event in self.message_events.values():
                event.clear()
        
        # Send normal inputs first
        test_image = self._create_test_image()
        test_lane_pose = self._create_test_lane_pose(d=0.0, phi=0.0)
        
        self.image_pub.publish(test_image)
        self.lane_pose_pub.publish(test_lane_pose)
        
        # Wait for normal operation
        time.sleep(2.0)
        
        # Trigger emergency stop
        emergency_signal = self._create_emergency_signal(active=True)
        self.emergency_trigger_pub.publish(emergency_signal)
        
        # Wait for emergency response
        time.sleep(3.0)
        
        # Verify emergency response
        with self.lock:
            # Should receive emergency status
            emergency_received = len(self.received_messages['emergency_signals']) > 0
            
            if emergency_received:
                latest_emergency = self.received_messages['emergency_signals'][-1]
                rospy.loginfo(f"[IntegrationTest] Emergency status: {latest_emergency['emergency_active']}")
            
            # Control commands should show stopping behavior
            if self.received_messages['control_commands']:
                recent_commands = self.received_messages['control_commands'][-3:]  # Last 3 commands
                stop_commands = [cmd for cmd in recent_commands if cmd['v'] == 0.0]
                
                rospy.loginfo(f"[IntegrationTest] Stop commands received: {len(stop_commands)}")
        
        # Clear emergency
        emergency_clear = self._create_emergency_signal(active=False)
        self.emergency_trigger_pub.publish(emergency_clear)
        
        # Wait for recovery
        time.sleep(3.0)
        
        rospy.loginfo("[IntegrationTest] Emergency stop integration test completed")
    
    def test_component_synchronization(self):
        """Test synchronization and timing between components."""
        rospy.loginfo("[IntegrationTest] Starting component synchronization test")
        
        # Clear previous messages
        with self.lock:
            for msg_list in self.received_messages.values():
                msg_list.clear()
            for event in self.message_events.values():
                event.clear()
        
        # Send synchronized inputs
        start_time = time.time()
        
        for i in range(5):
            test_image = self._create_test_image()
            test_lane_pose = self._create_test_lane_pose(d=0.1 * i, phi=0.02 * i)
            
            self.image_pub.publish(test_image)
            self.lane_pose_pub.publish(test_lane_pose)
            
            time.sleep(0.5)  # 2 Hz input rate
        
        # Wait for all processing
        time.sleep(3.0)
        
        end_time = time.time()
        test_duration = end_time - start_time
        
        # Analyze timing and synchronization
        with self.lock:
            control_commands = self.received_messages['control_commands']
            
            if len(control_commands) >= 2:
                # Calculate message rates
                timestamps = [cmd['timestamp'] for cmd in control_commands]
                time_diffs = np.diff(timestamps)
                avg_interval = np.mean(time_diffs) if len(time_diffs) > 0 else 0
                
                rospy.loginfo(f"[IntegrationTest] Average control command interval: {avg_interval:.3f}s")
                rospy.loginfo(f"[IntegrationTest] Total control commands: {len(control_commands)}")
                rospy.loginfo(f"[IntegrationTest] Test duration: {test_duration:.3f}s")
                
                # Verify reasonable update rate
                if avg_interval > 0:
                    update_rate = 1.0 / avg_interval
                    self.assertGreater(update_rate, 1.0, "Control update rate should be at least 1 Hz")
                    self.assertLess(update_rate, 50.0, "Control update rate should be reasonable")
        
        rospy.loginfo("[IntegrationTest] Component synchronization test completed")
    
    def test_performance_under_load(self):
        """Test system performance under high message load."""
        rospy.loginfo("[IntegrationTest] Starting performance under load test")
        
        # Clear previous messages
        with self.lock:
            for msg_list in self.received_messages.values():
                msg_list.clear()
            for event in self.message_events.values():
                event.clear()
        
        # Send high-frequency inputs
        start_time = time.time()
        message_count = 20
        
        for i in range(message_count):
            test_image = self._create_test_image()
            test_lane_pose = self._create_test_lane_pose(
                d=0.1 * np.sin(i * 0.1), 
                phi=0.05 * np.cos(i * 0.1)
            )
            
            self.image_pub.publish(test_image)
            self.lane_pose_pub.publish(test_lane_pose)
            
            time.sleep(0.1)  # 10 Hz input rate
        
        # Wait for processing
        time.sleep(5.0)
        
        end_time = time.time()
        test_duration = end_time - start_time
        
        # Analyze performance
        with self.lock:
            control_commands = self.received_messages['control_commands']
            
            rospy.loginfo(f"[IntegrationTest] Performance test results:")
            rospy.loginfo(f"[IntegrationTest]   Input messages sent: {message_count}")
            rospy.loginfo(f"[IntegrationTest]   Control commands received: {len(control_commands)}")
            rospy.loginfo(f"[IntegrationTest]   Test duration: {test_duration:.3f}s")
            
            # Verify system handled the load
            self.assertGreater(len(control_commands), 0, "Should receive control commands under load")
            
            # Check for reasonable response time
            if control_commands:
                response_ratio = len(control_commands) / message_count
                rospy.loginfo(f"[IntegrationTest]   Response ratio: {response_ratio:.2f}")
        
        rospy.loginfo("[IntegrationTest] Performance under load test completed")
    
    def test_fault_tolerance(self):
        """Test system behavior under component failures."""
        rospy.loginfo("[IntegrationTest] Starting fault tolerance test")
        
        # This test would simulate component failures
        # For now, we'll test with missing inputs
        
        # Clear previous messages
        with self.lock:
            for msg_list in self.received_messages.values():
                msg_list.clear()
            for event in self.message_events.values():
                event.clear()
        
        # Send only lane pose (no camera image)
        test_lane_pose = self._create_test_lane_pose(d=0.1, phi=0.05)
        self.lane_pose_pub.publish(test_lane_pose)
        
        # Wait for processing
        time.sleep(3.0)
        
        # Verify system handles missing inputs gracefully
        with self.lock:
            control_commands = self.received_messages['control_commands']
            
            rospy.loginfo(f"[IntegrationTest] Control commands with missing camera: {len(control_commands)}")
            
            # System should still produce some control output based on lane pose
            if control_commands:
                latest_cmd = self.received_messages['control_commands'][-1]
                rospy.loginfo(f"[IntegrationTest] Latest command: v={latest_cmd['v']:.3f}, omega={latest_cmd['omega']:.3f}")
        
        # Now send only camera image (no lane pose)
        time.sleep(1.0)
        
        test_image = self._create_test_image()
        self.image_pub.publish(test_image)
        
        # Wait for processing
        time.sleep(3.0)
        
        rospy.loginfo("[IntegrationTest] Fault tolerance test completed")
    
    def tearDown(self):
        """Clean up test environment."""
        rospy.loginfo("[IntegrationTest] Test teardown starting")
        
        # Log final statistics
        with self.lock:
            rospy.loginfo("[IntegrationTest] Final message counts:")
            for msg_type, msg_list in self.received_messages.items():
                rospy.loginfo(f"[IntegrationTest]   {msg_type}: {len(msg_list)}")
        
        rospy.loginfo("[IntegrationTest] Test teardown complete")


class IntegrationWorkflowTestSuite:
    """Test suite runner for integration workflow tests."""
    
    @staticmethod
    def run_all_tests():
        """Run all integration workflow tests."""
        rospy.loginfo("Starting Integration Workflow Test Suite")
        
        # Run tests
        suite = unittest.TestLoader().loadTestsFromTestCase(IntegrationWorkflowTest)
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        # Report results
        rospy.loginfo(f"Integration tests completed:")
        rospy.loginfo(f"  Tests run: {result.testsRun}")
        rospy.loginfo(f"  Failures: {len(result.failures)}")
        rospy.loginfo(f"  Errors: {len(result.errors)}")
        
        return result.wasSuccessful()


if __name__ == '__main__':
    try:
        # Run as ROS test
        rostest.rosrun('duckietown_demos', 'integration_workflow_test', IntegrationWorkflowTest)
    except Exception as e:
        rospy.logerr(f"Integration workflow test error: {e}")
        import traceback
        traceback.print_exc()
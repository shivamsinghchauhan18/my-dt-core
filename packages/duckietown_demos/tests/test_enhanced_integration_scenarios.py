#!/usr/bin/env python3
"""
Enhanced Integration Testing with Advanced Scenarios

This module extends the existing ROS-based integration tests with cross-component
communication testing, end-to-end pipeline validation, and hardware-in-the-loop
testing capabilities for the Advanced Autonomous Duckietown System.
"""

import unittest
import time
import threading
import queue
import json
import tempfile
import shutil
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
from unittest.mock import Mock, patch, MagicMock
import numpy as np
import sys
import os

# Mock ROS modules for testing
sys.modules['rospy'] = Mock()
sys.modules['std_msgs.msg'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['geometry_msgs.msg'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['cv_bridge'] = Mock()

# Add project paths
PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Configure comprehensive logging
import logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [INTEGRATION] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


class MockROSNode:
    """Mock ROS node for integration testing."""
    
    def __init__(self, node_name: str):
        self.node_name = node_name
        self.publishers = {}
        self.subscribers = {}
        self.services = {}
        self.message_queue = queue.Queue()
        self.is_running = False
        self.processing_thread = None
        
        logger.info(f"MockROSNode '{node_name}' initialized")
    
    def create_publisher(self, topic: str, msg_type: str):
        """Create a mock publisher."""
        publisher = Mock()
        publisher.publish = lambda msg: self.message_queue.put((topic, msg_type, msg))
        self.publishers[topic] = publisher
        logger.info(f"Publisher created for topic '{topic}' with type '{msg_type}'")
        return publisher
    
    def create_subscriber(self, topic: str, msg_type: str, callback):
        """Create a mock subscriber."""
        subscriber = Mock()
        subscriber.callback = callback
        self.subscribers[topic] = subscriber
        logger.info(f"Subscriber created for topic '{topic}' with type '{msg_type}'")
        return subscriber
    
    def create_service(self, service_name: str, service_type: str, handler):
        """Create a mock service."""
        service = Mock()
        service.handler = handler
        self.services[service_name] = service
        logger.info(f"Service created: '{service_name}' with type '{service_type}'")
        return service
    
    def start(self):
        """Start the mock node."""
        self.is_running = True
        self.processing_thread = threading.Thread(target=self._process_messages)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        logger.info(f"MockROSNode '{self.node_name}' started")
    
    def stop(self):
        """Stop the mock node."""
        self.is_running = False
        if self.processing_thread:
            self.processing_thread.join(timeout=1.0)
        logger.info(f"MockROSNode '{self.node_name}' stopped")
    
    def _process_messages(self):
        """Process messages in background thread."""
        while self.is_running:
            try:
                topic, msg_type, msg = self.message_queue.get(timeout=0.1)
                logger.debug(f"Processing message on topic '{topic}': {msg_type}")
                
                # Simulate message delivery to subscribers
                for sub_topic, subscriber in self.subscribers.items():
                    if sub_topic == topic:
                        subscriber.callback(msg)
                        
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error processing message: {e}")


class IntegrationTestScenario:
    """Base class for integration test scenarios."""
    
    def __init__(self, scenario_name: str, description: str):
        self.scenario_name = scenario_name
        self.description = description
        self.nodes = {}
        self.start_time = None
        self.end_time = None
        self.results = {}
        self.errors = []
        
        logger.info(f"Integration scenario '{scenario_name}' created: {description}")
    
    def add_node(self, node_name: str, node: MockROSNode):
        """Add a node to the scenario."""
        self.nodes[node_name] = node
        logger.info(f"Node '{node_name}' added to scenario '{self.scenario_name}'")
    
    def setup(self):
        """Set up the scenario."""
        logger.info(f"Setting up scenario '{self.scenario_name}'")
        self.start_time = time.time()
        
        # Start all nodes
        for node_name, node in self.nodes.items():
            node.start()
    
    def teardown(self):
        """Tear down the scenario."""
        logger.info(f"Tearing down scenario '{self.scenario_name}'")
        
        # Stop all nodes
        for node_name, node in self.nodes.items():
            node.stop()
        
        self.end_time = time.time()
        execution_time = self.end_time - self.start_time if self.start_time else 0
        logger.info(f"Scenario '{self.scenario_name}' completed in {execution_time:.3f}s")
    
    def execute(self) -> Dict[str, Any]:
        """Execute the scenario (to be implemented by subclasses)."""
        raise NotImplementedError("Subclasses must implement execute method")
    
    def validate_results(self) -> bool:
        """Validate scenario results (to be implemented by subclasses)."""
        raise NotImplementedError("Subclasses must implement validate_results method")


class LaneFollowingIntegrationScenario(IntegrationTestScenario):
    """Integration scenario for lane following pipeline."""
    
    def __init__(self):
        super().__init__(
            "lane_following_integration",
            "End-to-end lane following with camera, detection, control, and actuation"
        )
        
        # Create mock nodes
        self.camera_node = MockROSNode("camera_node")
        self.lane_detector_node = MockROSNode("lane_detector_node")
        self.lane_filter_node = MockROSNode("lane_filter_node")
        self.lane_controller_node = MockROSNode("lane_controller_node")
        self.motor_node = MockROSNode("motor_node")
        
        # Add nodes to scenario
        self.add_node("camera", self.camera_node)
        self.add_node("lane_detector", self.lane_detector_node)
        self.add_node("lane_filter", self.lane_filter_node)
        self.add_node("lane_controller", self.lane_controller_node)
        self.add_node("motor", self.motor_node)
        
        # Set up communication
        self._setup_communication()
    
    def _setup_communication(self):
        """Set up inter-node communication."""
        logger.info("Setting up lane following communication pipeline")
        
        # Camera -> Lane Detector
        self.camera_pub = self.camera_node.create_publisher("/camera/image_raw", "sensor_msgs/Image")
        self.lane_detector_sub = self.lane_detector_node.create_subscriber(
            "/camera/image_raw", "sensor_msgs/Image", self._lane_detector_callback
        )
        
        # Lane Detector -> Lane Filter
        self.detector_pub = self.lane_detector_node.create_publisher("/lane_detector/segments", "duckietown_msgs/SegmentList")
        self.filter_sub = self.lane_filter_node.create_subscriber(
            "/lane_detector/segments", "duckietown_msgs/SegmentList", self._lane_filter_callback
        )
        
        # Lane Filter -> Lane Controller
        self.filter_pub = self.lane_filter_node.create_publisher("/lane_filter/lane_pose", "duckietown_msgs/LanePose")
        self.controller_sub = self.lane_controller_node.create_subscriber(
            "/lane_filter/lane_pose", "duckietown_msgs/LanePose", self._lane_controller_callback
        )
        
        # Lane Controller -> Motor
        self.controller_pub = self.lane_controller_node.create_publisher("/lane_controller/car_cmd", "duckietown_msgs/Twist2DStamped")
        self.motor_sub = self.motor_node.create_subscriber(
            "/lane_controller/car_cmd", "duckietown_msgs/Twist2DStamped", self._motor_callback
        )
        
        # Initialize result tracking
        self.results = {
            'images_processed': 0,
            'segments_detected': 0,
            'poses_estimated': 0,
            'commands_generated': 0,
            'motor_commands_received': 0,
            'processing_times': [],
            'communication_delays': [],
            'errors': []
        }
    
    def _lane_detector_callback(self, image_msg):
        """Mock lane detector callback."""
        logger.debug("Lane detector processing image")
        
        start_time = time.time()
        
        try:
            # Simulate lane detection processing
            time.sleep(0.01)  # 10ms processing time
            
            # Generate mock segments
            segments_msg = Mock()
            segments_msg.segments = [Mock() for _ in range(np.random.randint(2, 8))]
            
            # Publish segments
            self.detector_pub.publish(segments_msg)
            
            processing_time = time.time() - start_time
            self.results['images_processed'] += 1
            self.results['processing_times'].append(('lane_detector', processing_time))
            
            logger.debug(f"Lane detector processed image in {processing_time:.3f}s")
            
        except Exception as e:
            self.results['errors'].append(f"Lane detector error: {e}")
            logger.error(f"Lane detector error: {e}")
    
    def _lane_filter_callback(self, segments_msg):
        """Mock lane filter callback."""
        logger.debug("Lane filter processing segments")
        
        start_time = time.time()
        
        try:
            # Simulate lane pose estimation
            time.sleep(0.005)  # 5ms processing time
            
            # Generate mock lane pose
            pose_msg = Mock()
            pose_msg.d = np.random.normal(0, 0.1)  # Lateral offset
            pose_msg.phi = np.random.normal(0, 0.1)  # Heading error
            
            # Publish pose
            self.filter_pub.publish(pose_msg)
            
            processing_time = time.time() - start_time
            self.results['segments_detected'] += len(segments_msg.segments)
            self.results['poses_estimated'] += 1
            self.results['processing_times'].append(('lane_filter', processing_time))
            
            logger.debug(f"Lane filter processed segments in {processing_time:.3f}s")
            
        except Exception as e:
            self.results['errors'].append(f"Lane filter error: {e}")
            logger.error(f"Lane filter error: {e}")
    
    def _lane_controller_callback(self, pose_msg):
        """Mock lane controller callback."""
        logger.debug("Lane controller processing pose")
        
        start_time = time.time()
        
        try:
            # Simulate control computation
            time.sleep(0.002)  # 2ms processing time
            
            # Generate mock control command
            cmd_msg = Mock()
            cmd_msg.v = 0.3 + np.random.normal(0, 0.05)  # Linear velocity
            cmd_msg.omega = -pose_msg.d * 2.0 + np.random.normal(0, 0.1)  # Angular velocity
            
            # Publish command
            self.controller_pub.publish(cmd_msg)
            
            processing_time = time.time() - start_time
            self.results['commands_generated'] += 1
            self.results['processing_times'].append(('lane_controller', processing_time))
            
            logger.debug(f"Lane controller processed pose in {processing_time:.3f}s")
            
        except Exception as e:
            self.results['errors'].append(f"Lane controller error: {e}")
            logger.error(f"Lane controller error: {e}")
    
    def _motor_callback(self, cmd_msg):
        """Mock motor callback."""
        logger.debug("Motor executing command")
        
        try:
            # Simulate motor execution
            time.sleep(0.001)  # 1ms execution time
            
            self.results['motor_commands_received'] += 1
            
            logger.debug(f"Motor executed command: v={cmd_msg.v:.3f}, omega={cmd_msg.omega:.3f}")
            
        except Exception as e:
            self.results['errors'].append(f"Motor error: {e}")
            logger.error(f"Motor error: {e}")
    
    def execute(self) -> Dict[str, Any]:
        """Execute the lane following integration scenario."""
        logger.info("Executing lane following integration scenario")
        
        # Generate test images
        num_images = 10
        for i in range(num_images):
            # Create mock image message
            image_msg = Mock()
            image_msg.header = Mock()
            image_msg.header.stamp = time.time()
            image_msg.width = 640
            image_msg.height = 480
            image_msg.data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8).tobytes()
            
            # Publish image
            self.camera_pub.publish(image_msg)
            
            # Wait for processing
            time.sleep(0.1)  # 100ms between images (10 FPS)
        
        # Wait for final processing
        time.sleep(0.5)
        
        logger.info(f"Lane following scenario executed with {num_images} images")
        return self.results
    
    def validate_results(self) -> bool:
        """Validate lane following scenario results."""
        logger.info("Validating lane following scenario results")
        
        validation_passed = True
        
        # Check that images were processed
        if self.results['images_processed'] == 0:
            logger.error("No images were processed")
            validation_passed = False
        
        # Check pipeline throughput
        expected_min_throughput = 0.5  # At least 50% of images should flow through
        actual_throughput = self.results['motor_commands_received'] / max(self.results['images_processed'], 1)
        
        if actual_throughput < expected_min_throughput:
            logger.error(f"Pipeline throughput too low: {actual_throughput:.2f} < {expected_min_throughput}")
            validation_passed = False
        
        # Check processing times
        if self.results['processing_times']:
            avg_times = {}
            for component, proc_time in self.results['processing_times']:
                if component not in avg_times:
                    avg_times[component] = []
                avg_times[component].append(proc_time)
            
            for component, times in avg_times.items():
                avg_time = np.mean(times)
                max_time = np.max(times)
                
                logger.info(f"{component}: avg={avg_time:.3f}s, max={max_time:.3f}s")
                
                # Check timing requirements
                if component == 'lane_detector' and avg_time > 0.05:  # 50ms limit
                    logger.warning(f"Lane detector average time too high: {avg_time:.3f}s")
                elif component == 'lane_filter' and avg_time > 0.02:  # 20ms limit
                    logger.warning(f"Lane filter average time too high: {avg_time:.3f}s")
                elif component == 'lane_controller' and avg_time > 0.01:  # 10ms limit
                    logger.warning(f"Lane controller average time too high: {avg_time:.3f}s")
        
        # Check for errors
        if self.results['errors']:
            logger.warning(f"Scenario had {len(self.results['errors'])} errors:")
            for error in self.results['errors']:
                logger.warning(f"  - {error}")
        
        logger.info(f"Lane following validation {'PASSED' if validation_passed else 'FAILED'}")
        return validation_passed


class ObjectDetectionAvoidanceScenario(IntegrationTestScenario):
    """Integration scenario for object detection and avoidance."""
    
    def __init__(self):
        super().__init__(
            "object_detection_avoidance",
            "End-to-end object detection and avoidance with YOLO, risk assessment, and path planning"
        )
        
        # Create mock nodes
        self.camera_node = MockROSNode("camera_node")
        self.yolo_node = MockROSNode("yolo_detector_node")
        self.risk_assessment_node = MockROSNode("risk_assessment_node")
        self.path_planner_node = MockROSNode("path_planner_node")
        self.coordinator_node = MockROSNode("coordinator_node")
        
        # Add nodes to scenario
        self.add_node("camera", self.camera_node)
        self.add_node("yolo_detector", self.yolo_node)
        self.add_node("risk_assessment", self.risk_assessment_node)
        self.add_node("path_planner", self.path_planner_node)
        self.add_node("coordinator", self.coordinator_node)
        
        # Set up communication
        self._setup_communication()
    
    def _setup_communication(self):
        """Set up object detection and avoidance communication."""
        logger.info("Setting up object detection and avoidance communication pipeline")
        
        # Camera -> YOLO Detector
        self.camera_pub = self.camera_node.create_publisher("/camera/image_raw", "sensor_msgs/Image")
        self.yolo_sub = self.yolo_node.create_subscriber(
            "/camera/image_raw", "sensor_msgs/Image", self._yolo_callback
        )
        
        # YOLO Detector -> Risk Assessment
        self.yolo_pub = self.yolo_node.create_publisher("/yolo_detector/detections", "duckietown_msgs/ObjectDetectionArray")
        self.risk_sub = self.risk_assessment_node.create_subscriber(
            "/yolo_detector/detections", "duckietown_msgs/ObjectDetectionArray", self._risk_assessment_callback
        )
        
        # Risk Assessment -> Path Planner
        self.risk_pub = self.risk_assessment_node.create_publisher("/risk_assessment/risk_status", "duckietown_msgs/RiskStatus")
        self.planner_sub = self.path_planner_node.create_subscriber(
            "/risk_assessment/risk_status", "duckietown_msgs/RiskStatus", self._path_planner_callback
        )
        
        # Path Planner -> Coordinator
        self.planner_pub = self.path_planner_node.create_publisher("/path_planner/avoidance_cmd", "geometry_msgs/Twist")
        self.coordinator_sub = self.coordinator_node.create_subscriber(
            "/path_planner/avoidance_cmd", "geometry_msgs/Twist", self._coordinator_callback
        )
        
        # Initialize result tracking
        self.results = {
            'images_processed': 0,
            'objects_detected': 0,
            'risk_assessments': 0,
            'avoidance_commands': 0,
            'coordinator_decisions': 0,
            'detection_accuracy': [],
            'risk_levels': [],
            'avoidance_strategies': [],
            'processing_times': [],
            'errors': []
        }
    
    def _yolo_callback(self, image_msg):
        """Mock YOLO detector callback."""
        logger.debug("YOLO detector processing image")
        
        start_time = time.time()
        
        try:
            # Simulate YOLO processing
            time.sleep(0.08)  # 80ms processing time (realistic for YOLO)
            
            # Generate mock detections
            num_objects = np.random.randint(0, 4)  # 0-3 objects
            detections = []
            
            for i in range(num_objects):
                detection = Mock()
                detection.class_name = np.random.choice(['duckiebot', 'duckie', 'cone', 'sign'])
                detection.confidence = np.random.uniform(0.6, 0.95)
                detection.bbox = Mock()
                detection.bbox.x = np.random.randint(0, 500)
                detection.bbox.y = np.random.randint(0, 300)
                detection.bbox.width = np.random.randint(50, 150)
                detection.bbox.height = np.random.randint(50, 150)
                detection.distance = np.random.uniform(0.5, 3.0)
                detections.append(detection)
            
            # Create detection array message
            detection_msg = Mock()
            detection_msg.detections = detections
            detection_msg.header = Mock()
            detection_msg.header.stamp = time.time()
            
            # Publish detections
            self.yolo_pub.publish(detection_msg)
            
            processing_time = time.time() - start_time
            self.results['images_processed'] += 1
            self.results['objects_detected'] += len(detections)
            self.results['processing_times'].append(('yolo_detector', processing_time))
            
            # Track detection accuracy (simulated)
            if detections:
                avg_confidence = np.mean([d.confidence for d in detections])
                self.results['detection_accuracy'].append(avg_confidence)
            
            logger.debug(f"YOLO detected {len(detections)} objects in {processing_time:.3f}s")
            
        except Exception as e:
            self.results['errors'].append(f"YOLO detector error: {e}")
            logger.error(f"YOLO detector error: {e}")
    
    def _risk_assessment_callback(self, detection_msg):
        """Mock risk assessment callback."""
        logger.debug("Risk assessment processing detections")
        
        start_time = time.time()
        
        try:
            # Simulate risk assessment
            time.sleep(0.01)  # 10ms processing time
            
            # Calculate risk level based on detections
            max_risk = 0
            for detection in detection_msg.detections:
                # Risk based on distance and object type
                distance_risk = max(0, (2.0 - detection.distance) / 2.0)  # Higher risk for closer objects
                
                type_risk = {
                    'duckiebot': 0.8,
                    'duckie': 0.3,
                    'cone': 0.6,
                    'sign': 0.2
                }.get(detection.class_name, 0.5)
                
                object_risk = distance_risk * type_risk * detection.confidence
                max_risk = max(max_risk, object_risk)
            
            # Create risk status message
            risk_msg = Mock()
            risk_msg.risk_level = max_risk
            risk_msg.high_risk_objects = [d for d in detection_msg.detections if d.distance < 1.0]
            risk_msg.recommended_action = 'stop' if max_risk > 0.7 else 'slow' if max_risk > 0.4 else 'continue'
            
            # Publish risk status
            self.risk_pub.publish(risk_msg)
            
            processing_time = time.time() - start_time
            self.results['risk_assessments'] += 1
            self.results['risk_levels'].append(max_risk)
            self.results['processing_times'].append(('risk_assessment', processing_time))
            
            logger.debug(f"Risk assessment completed in {processing_time:.3f}s, risk={max_risk:.2f}")
            
        except Exception as e:
            self.results['errors'].append(f"Risk assessment error: {e}")
            logger.error(f"Risk assessment error: {e}")
    
    def _path_planner_callback(self, risk_msg):
        """Mock path planner callback."""
        logger.debug("Path planner processing risk status")
        
        start_time = time.time()
        
        try:
            # Simulate path planning
            time.sleep(0.015)  # 15ms processing time
            
            # Generate avoidance command based on risk
            avoidance_cmd = Mock()
            
            if risk_msg.recommended_action == 'stop':
                avoidance_cmd.linear = Mock()
                avoidance_cmd.linear.x = 0.0
                avoidance_cmd.angular = Mock()
                avoidance_cmd.angular.z = 0.0
                strategy = 'emergency_stop'
            elif risk_msg.recommended_action == 'slow':
                avoidance_cmd.linear = Mock()
                avoidance_cmd.linear.x = 0.1  # Slow down
                avoidance_cmd.angular = Mock()
                avoidance_cmd.angular.z = np.random.uniform(-0.5, 0.5)  # Slight steering
                strategy = 'slow_and_steer'
            else:
                avoidance_cmd.linear = Mock()
                avoidance_cmd.linear.x = 0.3  # Normal speed
                avoidance_cmd.angular = Mock()
                avoidance_cmd.angular.z = 0.0
                strategy = 'continue'
            
            # Publish avoidance command
            self.planner_pub.publish(avoidance_cmd)
            
            processing_time = time.time() - start_time
            self.results['avoidance_commands'] += 1
            self.results['avoidance_strategies'].append(strategy)
            self.results['processing_times'].append(('path_planner', processing_time))
            
            logger.debug(f"Path planner generated {strategy} command in {processing_time:.3f}s")
            
        except Exception as e:
            self.results['errors'].append(f"Path planner error: {e}")
            logger.error(f"Path planner error: {e}")
    
    def _coordinator_callback(self, avoidance_cmd):
        """Mock coordinator callback."""
        logger.debug("Coordinator processing avoidance command")
        
        try:
            # Simulate coordination decision
            time.sleep(0.002)  # 2ms processing time
            
            self.results['coordinator_decisions'] += 1
            
            logger.debug(f"Coordinator executed avoidance: v={avoidance_cmd.linear.x:.3f}, omega={avoidance_cmd.angular.z:.3f}")
            
        except Exception as e:
            self.results['errors'].append(f"Coordinator error: {e}")
            logger.error(f"Coordinator error: {e}")
    
    def execute(self) -> Dict[str, Any]:
        """Execute the object detection and avoidance scenario."""
        logger.info("Executing object detection and avoidance scenario")
        
        # Generate test scenarios with varying object densities
        scenarios = [
            {'num_images': 5, 'description': 'clear_path'},
            {'num_images': 5, 'description': 'moderate_obstacles'},
            {'num_images': 5, 'description': 'high_obstacle_density'}
        ]
        
        for scenario in scenarios:
            logger.info(f"Running {scenario['description']} scenario")
            
            for i in range(scenario['num_images']):
                # Create mock image message
                image_msg = Mock()
                image_msg.header = Mock()
                image_msg.header.stamp = time.time()
                image_msg.width = 640
                image_msg.height = 480
                image_msg.data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8).tobytes()
                
                # Publish image
                self.camera_pub.publish(image_msg)
                
                # Wait for processing
                time.sleep(0.2)  # 200ms between images (5 FPS)
        
        # Wait for final processing
        time.sleep(1.0)
        
        logger.info("Object detection and avoidance scenario completed")
        return self.results
    
    def validate_results(self) -> bool:
        """Validate object detection and avoidance scenario results."""
        logger.info("Validating object detection and avoidance scenario results")
        
        validation_passed = True
        
        # Check that images were processed
        if self.results['images_processed'] == 0:
            logger.error("No images were processed")
            validation_passed = False
        
        # Check detection performance
        if self.results['detection_accuracy']:
            avg_accuracy = np.mean(self.results['detection_accuracy'])
            logger.info(f"Average detection confidence: {avg_accuracy:.3f}")
            
            if avg_accuracy < 0.7:
                logger.warning(f"Detection confidence below threshold: {avg_accuracy:.3f} < 0.7")
        
        # Check risk assessment coverage
        risk_coverage = self.results['risk_assessments'] / max(self.results['images_processed'], 1)
        logger.info(f"Risk assessment coverage: {risk_coverage:.2f}")
        
        if risk_coverage < 0.8:
            logger.warning(f"Risk assessment coverage low: {risk_coverage:.2f} < 0.8")
        
        # Check avoidance strategy distribution
        if self.results['avoidance_strategies']:
            strategy_counts = {}
            for strategy in self.results['avoidance_strategies']:
                strategy_counts[strategy] = strategy_counts.get(strategy, 0) + 1
            
            logger.info("Avoidance strategy distribution:")
            for strategy, count in strategy_counts.items():
                percentage = (count / len(self.results['avoidance_strategies'])) * 100
                logger.info(f"  {strategy}: {count} ({percentage:.1f}%)")
        
        # Check processing times
        if self.results['processing_times']:
            component_times = {}
            for component, proc_time in self.results['processing_times']:
                if component not in component_times:
                    component_times[component] = []
                component_times[component].append(proc_time)
            
            for component, times in component_times.items():
                avg_time = np.mean(times)
                max_time = np.max(times)
                
                logger.info(f"{component}: avg={avg_time:.3f}s, max={max_time:.3f}s")
                
                # Check timing requirements
                if component == 'yolo_detector' and avg_time > 0.1:  # 100ms limit
                    logger.warning(f"YOLO detector average time too high: {avg_time:.3f}s")
                elif component == 'risk_assessment' and avg_time > 0.02:  # 20ms limit
                    logger.warning(f"Risk assessment average time too high: {avg_time:.3f}s")
                elif component == 'path_planner' and avg_time > 0.03:  # 30ms limit
                    logger.warning(f"Path planner average time too high: {avg_time:.3f}s")
        
        # Check for errors
        if self.results['errors']:
            logger.warning(f"Scenario had {len(self.results['errors'])} errors:")
            for error in self.results['errors']:
                logger.warning(f"  - {error}")
        
        logger.info(f"Object detection and avoidance validation {'PASSED' if validation_passed else 'FAILED'}")
        return validation_passed


class HardwareInTheLoopTestRunner:
    """Hardware-in-the-loop test runner for enhanced testing capabilities."""
    
    def __init__(self, output_dir: str = "hitl_test_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        self.test_scenarios = []
        self.test_results = []
        self.hardware_available = False
        
        logger.info(f"HardwareInTheLoopTestRunner initialized with output dir: {self.output_dir}")
    
    def add_scenario(self, scenario: IntegrationTestScenario):
        """Add a test scenario."""
        self.test_scenarios.append(scenario)
        logger.info(f"Added scenario: {scenario.scenario_name}")
    
    def check_hardware_availability(self) -> bool:
        """Check if hardware is available for testing."""
        # In a real implementation, this would check for actual hardware
        # For testing purposes, we'll simulate hardware availability
        self.hardware_available = True  # Simulate hardware available
        
        logger.info(f"Hardware availability check: {'Available' if self.hardware_available else 'Not Available'}")
        return self.hardware_available
    
    def run_all_scenarios(self) -> List[Dict[str, Any]]:
        """Run all test scenarios."""
        logger.info(f"Running {len(self.test_scenarios)} integration test scenarios")
        
        self.test_results = []
        
        for scenario in self.test_scenarios:
            logger.info(f"Starting scenario: {scenario.scenario_name}")
            
            try:
                # Set up scenario
                scenario.setup()
                
                # Execute scenario
                results = scenario.execute()
                
                # Validate results
                validation_passed = scenario.validate_results()
                
                # Compile scenario results
                scenario_result = {
                    'scenario_name': scenario.scenario_name,
                    'description': scenario.description,
                    'execution_time': scenario.end_time - scenario.start_time if scenario.end_time and scenario.start_time else 0,
                    'validation_passed': validation_passed,
                    'results': results,
                    'timestamp': datetime.now().isoformat()
                }
                
                self.test_results.append(scenario_result)
                
                logger.info(f"Scenario {scenario.scenario_name} {'PASSED' if validation_passed else 'FAILED'}")
                
            except Exception as e:
                logger.error(f"Scenario {scenario.scenario_name} failed with error: {e}")
                
                error_result = {
                    'scenario_name': scenario.scenario_name,
                    'description': scenario.description,
                    'execution_time': 0,
                    'validation_passed': False,
                    'results': {'error': str(e)},
                    'timestamp': datetime.now().isoformat()
                }
                
                self.test_results.append(error_result)
                
            finally:
                # Clean up scenario
                scenario.teardown()
        
        # Generate comprehensive report
        self._generate_test_report()
        
        logger.info(f"All scenarios completed. Results saved to {self.output_dir}")
        return self.test_results
    
    def _generate_test_report(self):
        """Generate comprehensive test report."""
        logger.info("Generating comprehensive integration test report")
        
        # Calculate summary statistics
        total_scenarios = len(self.test_results)
        passed_scenarios = sum(1 for r in self.test_results if r['validation_passed'])
        failed_scenarios = total_scenarios - passed_scenarios
        
        total_execution_time = sum(r['execution_time'] for r in self.test_results)
        
        # Create comprehensive report
        report = {
            'test_summary': {
                'total_scenarios': total_scenarios,
                'passed_scenarios': passed_scenarios,
                'failed_scenarios': failed_scenarios,
                'success_rate': (passed_scenarios / total_scenarios) * 100 if total_scenarios > 0 else 0,
                'total_execution_time': total_execution_time,
                'hardware_available': self.hardware_available
            },
            'scenario_results': self.test_results,
            'performance_analysis': self._analyze_performance(),
            'communication_analysis': self._analyze_communication(),
            'generated_at': datetime.now().isoformat()
        }
        
        # Save report to file
        report_file = self.output_dir / f"integration_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        # Log summary
        logger.info("Integration Test Report Summary:")
        logger.info(f"  Total scenarios: {total_scenarios}")
        logger.info(f"  Passed: {passed_scenarios}")
        logger.info(f"  Failed: {failed_scenarios}")
        logger.info(f"  Success rate: {report['test_summary']['success_rate']:.1f}%")
        logger.info(f"  Total execution time: {total_execution_time:.3f}s")
        logger.info(f"  Report saved to: {report_file}")
    
    def _analyze_performance(self) -> Dict[str, Any]:
        """Analyze performance across all scenarios."""
        performance_data = {
            'processing_times': {},
            'throughput_analysis': {},
            'latency_analysis': {}
        }
        
        # Collect processing times from all scenarios
        all_processing_times = {}
        
        for result in self.test_results:
            if 'processing_times' in result['results']:
                for component, proc_time in result['results']['processing_times']:
                    if component not in all_processing_times:
                        all_processing_times[component] = []
                    all_processing_times[component].append(proc_time)
        
        # Calculate statistics for each component
        for component, times in all_processing_times.items():
            performance_data['processing_times'][component] = {
                'count': len(times),
                'average': np.mean(times),
                'median': np.median(times),
                'std_dev': np.std(times),
                'min': np.min(times),
                'max': np.max(times),
                'p95': np.percentile(times, 95),
                'p99': np.percentile(times, 99)
            }
        
        return performance_data
    
    def _analyze_communication(self) -> Dict[str, Any]:
        """Analyze inter-component communication patterns."""
        communication_data = {
            'message_flow_analysis': {},
            'pipeline_efficiency': {},
            'error_analysis': {}
        }
        
        # Analyze message flow for each scenario
        for result in self.test_results:
            scenario_name = result['scenario_name']
            scenario_results = result['results']
            
            if 'images_processed' in scenario_results:
                # Calculate pipeline efficiency
                images_in = scenario_results.get('images_processed', 0)
                commands_out = scenario_results.get('motor_commands_received', 0) or scenario_results.get('coordinator_decisions', 0)
                
                efficiency = (commands_out / images_in) * 100 if images_in > 0 else 0
                
                communication_data['pipeline_efficiency'][scenario_name] = {
                    'input_messages': images_in,
                    'output_messages': commands_out,
                    'efficiency_percent': efficiency
                }
            
            # Analyze errors
            if 'errors' in scenario_results:
                communication_data['error_analysis'][scenario_name] = {
                    'error_count': len(scenario_results['errors']),
                    'errors': scenario_results['errors']
                }
        
        return communication_data


class TestEnhancedIntegrationScenarios(unittest.TestCase):
    """Test suite for enhanced integration scenarios."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_runner = HardwareInTheLoopTestRunner(output_dir=self.temp_dir)
    
    def tearDown(self):
        """Clean up test fixtures."""
        shutil.rmtree(self.temp_dir)
    
    def test_lane_following_integration_scenario(self):
        """Test lane following integration scenario."""
        print(f"[{self._get_timestamp()}] [TEST] Testing lane following integration scenario")
        
        scenario = LaneFollowingIntegrationScenario()
        self.test_runner.add_scenario(scenario)
        
        # Run scenario
        results = self.test_runner.run_all_scenarios()
        
        # Validate results
        self.assertEqual(len(results), 1)
        result = results[0]
        
        self.assertEqual(result['scenario_name'], 'lane_following_integration')
        self.assertIn('results', result)
        self.assertIn('validation_passed', result)
        
        # Check that some processing occurred
        scenario_results = result['results']
        self.assertGreater(scenario_results.get('images_processed', 0), 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Lane following integration scenario test passed")
    
    def test_object_detection_avoidance_scenario(self):
        """Test object detection and avoidance scenario."""
        print(f"[{self._get_timestamp()}] [TEST] Testing object detection and avoidance scenario")
        
        scenario = ObjectDetectionAvoidanceScenario()
        self.test_runner.add_scenario(scenario)
        
        # Run scenario
        results = self.test_runner.run_all_scenarios()
        
        # Validate results
        self.assertEqual(len(results), 1)
        result = results[0]
        
        self.assertEqual(result['scenario_name'], 'object_detection_avoidance')
        self.assertIn('results', result)
        self.assertIn('validation_passed', result)
        
        # Check that processing occurred
        scenario_results = result['results']
        self.assertGreater(scenario_results.get('images_processed', 0), 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Object detection and avoidance scenario test passed")
    
    def test_multiple_scenarios_execution(self):
        """Test execution of multiple scenarios."""
        print(f"[{self._get_timestamp()}] [TEST] Testing multiple scenarios execution")
        
        # Add multiple scenarios
        lane_scenario = LaneFollowingIntegrationScenario()
        object_scenario = ObjectDetectionAvoidanceScenario()
        
        self.test_runner.add_scenario(lane_scenario)
        self.test_runner.add_scenario(object_scenario)
        
        # Run all scenarios
        results = self.test_runner.run_all_scenarios()
        
        # Validate results
        self.assertEqual(len(results), 2)
        
        scenario_names = [r['scenario_name'] for r in results]
        self.assertIn('lane_following_integration', scenario_names)
        self.assertIn('object_detection_avoidance', scenario_names)
        
        # Check that test report was generated
        report_files = list(Path(self.temp_dir).glob("integration_test_report_*.json"))
        self.assertEqual(len(report_files), 1)
        
        # Validate report content
        with open(report_files[0], 'r') as f:
            report = json.load(f)
        
        self.assertIn('test_summary', report)
        self.assertIn('scenario_results', report)
        self.assertIn('performance_analysis', report)
        self.assertIn('communication_analysis', report)
        
        print(f"[{self._get_timestamp()}] [TEST] Multiple scenarios execution test passed")
    
    def test_hardware_availability_check(self):
        """Test hardware availability checking."""
        print(f"[{self._get_timestamp()}] [TEST] Testing hardware availability check")
        
        # Check hardware availability
        hardware_available = self.test_runner.check_hardware_availability()
        
        # Should return a boolean
        self.assertIsInstance(hardware_available, bool)
        
        # Should set the hardware_available flag
        self.assertEqual(self.test_runner.hardware_available, hardware_available)
        
        print(f"[{self._get_timestamp()}] [TEST] Hardware availability check test passed")
    
    def test_performance_analysis(self):
        """Test performance analysis functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing performance analysis")
        
        # Add and run a scenario
        scenario = LaneFollowingIntegrationScenario()
        self.test_runner.add_scenario(scenario)
        results = self.test_runner.run_all_scenarios()
        
        # Check that performance analysis was generated
        report_files = list(Path(self.temp_dir).glob("integration_test_report_*.json"))
        self.assertEqual(len(report_files), 1)
        
        with open(report_files[0], 'r') as f:
            report = json.load(f)
        
        # Validate performance analysis structure
        self.assertIn('performance_analysis', report)
        perf_analysis = report['performance_analysis']
        
        self.assertIn('processing_times', perf_analysis)
        self.assertIn('throughput_analysis', perf_analysis)
        self.assertIn('latency_analysis', perf_analysis)
        
        print(f"[{self._get_timestamp()}] [TEST] Performance analysis test passed")
    
    def test_communication_analysis(self):
        """Test communication analysis functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing communication analysis")
        
        # Add and run a scenario
        scenario = ObjectDetectionAvoidanceScenario()
        self.test_runner.add_scenario(scenario)
        results = self.test_runner.run_all_scenarios()
        
        # Check that communication analysis was generated
        report_files = list(Path(self.temp_dir).glob("integration_test_report_*.json"))
        self.assertEqual(len(report_files), 1)
        
        with open(report_files[0], 'r') as f:
            report = json.load(f)
        
        # Validate communication analysis structure
        self.assertIn('communication_analysis', report)
        comm_analysis = report['communication_analysis']
        
        self.assertIn('message_flow_analysis', comm_analysis)
        self.assertIn('pipeline_efficiency', comm_analysis)
        self.assertIn('error_analysis', comm_analysis)
        
        print(f"[{self._get_timestamp()}] [TEST] Communication analysis test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def run_enhanced_integration_tests():
    """Run all enhanced integration tests."""
    print(f"[{TestEnhancedIntegrationScenarios()._get_timestamp()}] [TEST] Starting enhanced integration test suite...")
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test class
    tests = unittest.TestLoader().loadTestsFromTestCase(TestEnhancedIntegrationScenarios)
    test_suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n[{TestEnhancedIntegrationScenarios()._get_timestamp()}] [TEST] Enhanced Integration Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\n[{TestEnhancedIntegrationScenarios()._get_timestamp()}] [TEST] Failures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\n[{TestEnhancedIntegrationScenarios()._get_timestamp()}] [TEST] Errors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_enhanced_integration_tests()
    sys.exit(0 if success else 1)
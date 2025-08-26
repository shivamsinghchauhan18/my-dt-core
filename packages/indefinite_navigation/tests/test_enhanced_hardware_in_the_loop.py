#!/usr/bin/env python3
"""
Enhanced Hardware-in-the-Loop Testing for Indefinite Navigation

This module provides comprehensive hardware-in-the-loop testing capabilities
for the indefinite navigation system with real-time monitoring, performance
validation, and automated test reporting.
"""

import unittest
import time
import threading
import queue
import json
import tempfile
import shutil
import subprocess
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
from unittest.mock import Mock, patch, MagicMock
import numpy as np
import sys
import os

# Mock hardware interfaces for testing
sys.modules['rospy'] = Mock()
sys.modules['std_msgs.msg'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['geometry_msgs.msg'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['cv_bridge'] = Mock()
sys.modules['tf'] = Mock()

# Configure comprehensive logging
import logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [HITL] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


class HardwareInterface:
    """Mock hardware interface for testing."""
    
    def __init__(self, interface_name: str):
        self.interface_name = interface_name
        self.is_connected = False
        self.is_operational = False
        self.last_heartbeat = None
        self.error_count = 0
        self.data_queue = queue.Queue()
        
        logger.info(f"HardwareInterface '{interface_name}' initialized")
    
    def connect(self) -> bool:
        """Connect to hardware interface."""
        try:
            # Simulate hardware connection
            time.sleep(0.1)  # Connection delay
            self.is_connected = True
            self.last_heartbeat = time.time()
            
            logger.info(f"Hardware interface '{self.interface_name}' connected")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to '{self.interface_name}': {e}")
            return False
    
    def disconnect(self):
        """Disconnect from hardware interface."""
        self.is_connected = False
        self.is_operational = False
        logger.info(f"Hardware interface '{self.interface_name}' disconnected")
    
    def start_operation(self) -> bool:
        """Start hardware operation."""
        if not self.is_connected:
            logger.error(f"Cannot start operation - '{self.interface_name}' not connected")
            return False
        
        self.is_operational = True
        logger.info(f"Hardware interface '{self.interface_name}' operational")
        return True
    
    def stop_operation(self):
        """Stop hardware operation."""
        self.is_operational = False
        logger.info(f"Hardware interface '{self.interface_name}' stopped")
    
    def send_command(self, command: Dict[str, Any]) -> bool:
        """Send command to hardware."""
        if not self.is_operational:
            logger.warning(f"Cannot send command - '{self.interface_name}' not operational")
            return False
        
        try:
            # Simulate command execution
            time.sleep(0.001)  # Command processing delay
            self.last_heartbeat = time.time()
            
            logger.debug(f"Command sent to '{self.interface_name}': {command}")
            return True
            
        except Exception as e:
            self.error_count += 1
            logger.error(f"Command failed on '{self.interface_name}': {e}")
            return False
    
    def read_sensor_data(self) -> Optional[Dict[str, Any]]:
        """Read sensor data from hardware."""
        if not self.is_operational:
            return None
        
        try:
            # Simulate sensor data
            sensor_data = {
                'timestamp': time.time(),
                'interface': self.interface_name,
                'data': self._generate_mock_sensor_data()
            }
            
            self.last_heartbeat = time.time()
            return sensor_data
            
        except Exception as e:
            self.error_count += 1
            logger.error(f"Sensor read failed on '{self.interface_name}': {e}")
            return None
    
    def _generate_mock_sensor_data(self) -> Dict[str, Any]:
        """Generate mock sensor data based on interface type."""
        if 'camera' in self.interface_name.lower():
            return {
                'image_width': 640,
                'image_height': 480,
                'fps': 30.0,
                'exposure': np.random.uniform(0.01, 0.1),
                'brightness': np.random.uniform(0.3, 0.8)
            }
        elif 'motor' in self.interface_name.lower():
            return {
                'left_wheel_speed': np.random.uniform(-1.0, 1.0),
                'right_wheel_speed': np.random.uniform(-1.0, 1.0),
                'battery_voltage': np.random.uniform(11.0, 12.6),
                'motor_temperature': np.random.uniform(25.0, 45.0)
            }
        elif 'imu' in self.interface_name.lower():
            return {
                'acceleration_x': np.random.normal(0, 0.1),
                'acceleration_y': np.random.normal(0, 0.1),
                'acceleration_z': np.random.normal(9.81, 0.1),
                'angular_velocity_x': np.random.normal(0, 0.05),
                'angular_velocity_y': np.random.normal(0, 0.05),
                'angular_velocity_z': np.random.normal(0, 0.05)
            }
        else:
            return {
                'value': np.random.random(),
                'status': 'ok'
            }
    
    def get_health_status(self) -> Dict[str, Any]:
        """Get hardware health status."""
        current_time = time.time()
        time_since_heartbeat = current_time - self.last_heartbeat if self.last_heartbeat else float('inf')
        
        health_status = {
            'interface_name': self.interface_name,
            'is_connected': self.is_connected,
            'is_operational': self.is_operational,
            'error_count': self.error_count,
            'time_since_heartbeat': time_since_heartbeat,
            'health_score': self._calculate_health_score(time_since_heartbeat)
        }
        
        return health_status
    
    def _calculate_health_score(self, time_since_heartbeat: float) -> float:
        """Calculate health score (0.0 to 1.0)."""
        if not self.is_connected:
            return 0.0
        
        if not self.is_operational:
            return 0.3
        
        # Penalize for errors and stale heartbeat
        error_penalty = min(self.error_count * 0.1, 0.5)
        heartbeat_penalty = min(time_since_heartbeat * 0.1, 0.3)
        
        health_score = 1.0 - error_penalty - heartbeat_penalty
        return max(0.0, health_score)


class HardwareInTheLoopTestScenario:
    """Base class for hardware-in-the-loop test scenarios."""
    
    def __init__(self, scenario_name: str, description: str):
        self.scenario_name = scenario_name
        self.description = description
        self.hardware_interfaces = {}
        self.monitoring_thread = None
        self.monitoring_active = False
        self.test_data = []
        self.performance_metrics = {}
        self.start_time = None
        self.end_time = None
        
        logger.info(f"HITL scenario '{scenario_name}' created: {description}")
    
    def add_hardware_interface(self, name: str, interface: HardwareInterface):
        """Add hardware interface to scenario."""
        self.hardware_interfaces[name] = interface
        logger.info(f"Hardware interface '{name}' added to scenario '{self.scenario_name}'")
    
    def setup_hardware(self) -> bool:
        """Set up all hardware interfaces."""
        logger.info(f"Setting up hardware for scenario '{self.scenario_name}'")
        
        setup_success = True
        for name, interface in self.hardware_interfaces.items():
            if not interface.connect():
                logger.error(f"Failed to connect to hardware interface '{name}'")
                setup_success = False
            elif not interface.start_operation():
                logger.error(f"Failed to start operation on hardware interface '{name}'")
                setup_success = False
        
        if setup_success:
            logger.info("All hardware interfaces set up successfully")
        else:
            logger.error("Hardware setup failed")
        
        return setup_success
    
    def teardown_hardware(self):
        """Tear down all hardware interfaces."""
        logger.info(f"Tearing down hardware for scenario '{self.scenario_name}'")
        
        for name, interface in self.hardware_interfaces.items():
            interface.stop_operation()
            interface.disconnect()
        
        logger.info("Hardware teardown completed")
    
    def start_monitoring(self):
        """Start real-time hardware monitoring."""
        if self.monitoring_active:
            logger.warning("Monitoring already active")
            return
        
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=self._monitor_hardware)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        logger.info("Hardware monitoring started")
    
    def stop_monitoring(self):
        """Stop hardware monitoring."""
        if not self.monitoring_active:
            return
        
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=2.0)
        
        logger.info("Hardware monitoring stopped")
    
    def _monitor_hardware(self):
        """Monitor hardware interfaces in background thread."""
        while self.monitoring_active:
            try:
                monitoring_data = {
                    'timestamp': time.time(),
                    'interfaces': {}
                }
                
                for name, interface in self.hardware_interfaces.items():
                    # Get health status
                    health_status = interface.get_health_status()
                    
                    # Get sensor data
                    sensor_data = interface.read_sensor_data()
                    
                    monitoring_data['interfaces'][name] = {
                        'health': health_status,
                        'sensor_data': sensor_data
                    }
                
                self.test_data.append(monitoring_data)
                
                # Log critical health issues
                for name, data in monitoring_data['interfaces'].items():
                    health_score = data['health']['health_score']
                    if health_score < 0.5:
                        logger.warning(f"Hardware interface '{name}' health degraded: {health_score:.2f}")
                
                time.sleep(0.1)  # 10 Hz monitoring
                
            except Exception as e:
                logger.error(f"Error in hardware monitoring: {e}")
                time.sleep(0.5)  # Longer delay on error
    
    def execute_test_sequence(self, test_commands: List[Dict[str, Any]]) -> bool:
        """Execute a sequence of test commands."""
        logger.info(f"Executing test sequence with {len(test_commands)} commands")
        
        execution_success = True
        
        for i, command in enumerate(test_commands):
            logger.debug(f"Executing command {i+1}/{len(test_commands)}: {command}")
            
            try:
                # Extract command parameters
                interface_name = command.get('interface')
                action = command.get('action')
                parameters = command.get('parameters', {})
                expected_duration = command.get('duration', 1.0)
                
                # Get target interface
                if interface_name not in self.hardware_interfaces:
                    logger.error(f"Unknown hardware interface: {interface_name}")
                    execution_success = False
                    continue
                
                interface = self.hardware_interfaces[interface_name]
                
                # Execute command
                command_start = time.time()
                
                if action == 'send_command':
                    success = interface.send_command(parameters)
                elif action == 'read_sensors':
                    sensor_data = interface.read_sensor_data()
                    success = sensor_data is not None
                elif action == 'wait':
                    time.sleep(expected_duration)
                    success = True
                else:
                    logger.error(f"Unknown action: {action}")
                    success = False
                
                command_duration = time.time() - command_start
                
                # Record command execution
                command_result = {
                    'command_index': i,
                    'interface': interface_name,
                    'action': action,
                    'success': success,
                    'duration': command_duration,
                    'timestamp': command_start
                }
                
                if not success:
                    execution_success = False
                    logger.error(f"Command {i+1} failed: {command}")
                
                # Wait for expected duration if needed
                remaining_time = expected_duration - command_duration
                if remaining_time > 0:
                    time.sleep(remaining_time)
                
            except Exception as e:
                logger.error(f"Error executing command {i+1}: {e}")
                execution_success = False
        
        logger.info(f"Test sequence execution {'completed successfully' if execution_success else 'failed'}")
        return execution_success
    
    def analyze_performance(self) -> Dict[str, Any]:
        """Analyze performance from collected test data."""
        logger.info("Analyzing hardware performance data")
        
        if not self.test_data:
            logger.warning("No test data available for analysis")
            return {}
        
        analysis = {
            'data_points': len(self.test_data),
            'test_duration': self.end_time - self.start_time if self.end_time and self.start_time else 0,
            'interface_analysis': {},
            'overall_health': {},
            'performance_summary': {}
        }
        
        # Analyze each interface
        for interface_name in self.hardware_interfaces.keys():
            interface_data = []
            health_scores = []
            error_counts = []
            
            for data_point in self.test_data:
                if interface_name in data_point['interfaces']:
                    interface_info = data_point['interfaces'][interface_name]
                    interface_data.append(interface_info)
                    
                    if interface_info['health']:
                        health_scores.append(interface_info['health']['health_score'])
                        error_counts.append(interface_info['health']['error_count'])
            
            if health_scores:
                analysis['interface_analysis'][interface_name] = {
                    'data_points': len(interface_data),
                    'avg_health_score': np.mean(health_scores),
                    'min_health_score': np.min(health_scores),
                    'max_health_score': np.max(health_scores),
                    'final_error_count': error_counts[-1] if error_counts else 0,
                    'health_degradation_events': sum(1 for score in health_scores if score < 0.5)
                }
        
        # Overall health analysis
        all_health_scores = []
        for interface_analysis in analysis['interface_analysis'].values():
            all_health_scores.append(interface_analysis['avg_health_score'])
        
        if all_health_scores:
            analysis['overall_health'] = {
                'avg_system_health': np.mean(all_health_scores),
                'min_interface_health': np.min(all_health_scores),
                'healthy_interfaces': sum(1 for score in all_health_scores if score > 0.8),
                'total_interfaces': len(all_health_scores)
            }
        
        # Performance summary
        analysis['performance_summary'] = {
            'test_success_rate': analysis['overall_health'].get('avg_system_health', 0.0),
            'data_collection_rate': len(self.test_data) / analysis['test_duration'] if analysis['test_duration'] > 0 else 0,
            'critical_issues': sum(
                interface_data.get('health_degradation_events', 0) 
                for interface_data in analysis['interface_analysis'].values()
            )
        }
        
        logger.info(f"Performance analysis completed:")
        logger.info(f"  System health: {analysis['overall_health'].get('avg_system_health', 0):.2f}")
        logger.info(f"  Data points: {analysis['data_points']}")
        logger.info(f"  Critical issues: {analysis['performance_summary']['critical_issues']}")
        
        return analysis
    
    def execute(self) -> Dict[str, Any]:
        """Execute the HITL test scenario (to be implemented by subclasses)."""
        raise NotImplementedError("Subclasses must implement execute method")
    
    def validate_results(self) -> bool:
        """Validate scenario results (to be implemented by subclasses)."""
        raise NotImplementedError("Subclasses must implement validate_results method")


class NavigationHITLScenario(HardwareInTheLoopTestScenario):
    """Hardware-in-the-loop scenario for navigation testing."""
    
    def __init__(self):
        super().__init__(
            "navigation_hitl",
            "Hardware-in-the-loop navigation testing with camera, motors, and sensors"
        )
        
        # Create hardware interfaces
        self.camera_interface = HardwareInterface("camera")
        self.motor_interface = HardwareInterface("motors")
        self.imu_interface = HardwareInterface("imu")
        self.encoder_interface = HardwareInterface("encoders")
        
        # Add interfaces to scenario
        self.add_hardware_interface("camera", self.camera_interface)
        self.add_hardware_interface("motors", self.motor_interface)
        self.add_hardware_interface("imu", self.imu_interface)
        self.add_hardware_interface("encoders", self.encoder_interface)
    
    def execute(self) -> Dict[str, Any]:
        """Execute navigation HITL scenario."""
        logger.info("Executing navigation HITL scenario")
        
        self.start_time = time.time()
        
        # Define test sequence
        test_commands = [
            # Initialize sensors
            {'interface': 'camera', 'action': 'send_command', 'parameters': {'cmd': 'start_capture'}, 'duration': 0.5},
            {'interface': 'imu', 'action': 'send_command', 'parameters': {'cmd': 'calibrate'}, 'duration': 1.0},
            {'interface': 'encoders', 'action': 'send_command', 'parameters': {'cmd': 'reset'}, 'duration': 0.1},
            
            # Navigation sequence
            {'interface': 'motors', 'action': 'send_command', 'parameters': {'left_speed': 0.3, 'right_speed': 0.3}, 'duration': 2.0},
            {'interface': 'camera', 'action': 'read_sensors', 'duration': 0.1},
            {'interface': 'imu', 'action': 'read_sensors', 'duration': 0.1},
            
            # Turn maneuver
            {'interface': 'motors', 'action': 'send_command', 'parameters': {'left_speed': 0.1, 'right_speed': 0.5}, 'duration': 1.0},
            {'interface': 'encoders', 'action': 'read_sensors', 'duration': 0.1},
            
            # Stop and assess
            {'interface': 'motors', 'action': 'send_command', 'parameters': {'left_speed': 0.0, 'right_speed': 0.0}, 'duration': 0.5},
            {'interface': 'camera', 'action': 'read_sensors', 'duration': 0.1},
            
            # Reverse maneuver
            {'interface': 'motors', 'action': 'send_command', 'parameters': {'left_speed': -0.2, 'right_speed': -0.2}, 'duration': 1.0},
            
            # Final stop
            {'interface': 'motors', 'action': 'send_command', 'parameters': {'left_speed': 0.0, 'right_speed': 0.0}, 'duration': 0.1}
        ]
        
        # Execute test sequence
        execution_success = self.execute_test_sequence(test_commands)
        
        self.end_time = time.time()
        
        # Analyze performance
        performance_analysis = self.analyze_performance()
        
        # Compile results
        results = {
            'execution_success': execution_success,
            'test_duration': self.end_time - self.start_time,
            'commands_executed': len(test_commands),
            'data_points_collected': len(self.test_data),
            'performance_analysis': performance_analysis,
            'hardware_status': {name: interface.get_health_status() 
                              for name, interface in self.hardware_interfaces.items()}
        }
        
        logger.info(f"Navigation HITL scenario completed in {results['test_duration']:.2f}s")
        return results
    
    def validate_results(self) -> bool:
        """Validate navigation HITL results."""
        logger.info("Validating navigation HITL results")
        
        validation_passed = True
        
        # Check that all hardware interfaces are healthy
        for name, interface in self.hardware_interfaces.items():
            health_status = interface.get_health_status()
            health_score = health_status['health_score']
            
            if health_score < 0.7:
                logger.warning(f"Hardware interface '{name}' health below threshold: {health_score:.2f}")
                validation_passed = False
        
        # Check data collection
        if len(self.test_data) < 10:
            logger.error(f"Insufficient data collected: {len(self.test_data)} points")
            validation_passed = False
        
        # Check performance analysis
        performance_analysis = self.analyze_performance()
        if performance_analysis:
            system_health = performance_analysis.get('overall_health', {}).get('avg_system_health', 0)
            if system_health < 0.8:
                logger.warning(f"System health below threshold: {system_health:.2f}")
        
        logger.info(f"Navigation HITL validation {'PASSED' if validation_passed else 'FAILED'}")
        return validation_passed


class TestEnhancedHardwareInTheLoop(unittest.TestCase):
    """Test suite for enhanced hardware-in-the-loop testing."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
    
    def tearDown(self):
        """Clean up test fixtures."""
        shutil.rmtree(self.temp_dir)
    
    def test_hardware_interface_creation(self):
        """Test hardware interface creation and basic operations."""
        print(f"[{self._get_timestamp()}] [TEST] Testing hardware interface creation")
        
        interface = HardwareInterface("test_camera")
        
        self.assertEqual(interface.interface_name, "test_camera")
        self.assertFalse(interface.is_connected)
        self.assertFalse(interface.is_operational)
        self.assertEqual(interface.error_count, 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Hardware interface creation test passed")
    
    def test_hardware_interface_connection(self):
        """Test hardware interface connection and operation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing hardware interface connection")
        
        interface = HardwareInterface("test_motors")
        
        # Test connection
        success = interface.connect()
        self.assertTrue(success)
        self.assertTrue(interface.is_connected)
        self.assertIsNotNone(interface.last_heartbeat)
        
        # Test operation start
        success = interface.start_operation()
        self.assertTrue(success)
        self.assertTrue(interface.is_operational)
        
        # Test command sending
        command = {'left_speed': 0.5, 'right_speed': 0.5}
        success = interface.send_command(command)
        self.assertTrue(success)
        
        # Test sensor reading
        sensor_data = interface.read_sensor_data()
        self.assertIsNotNone(sensor_data)
        self.assertIn('timestamp', sensor_data)
        self.assertIn('interface', sensor_data)
        self.assertIn('data', sensor_data)
        
        # Test disconnection
        interface.disconnect()
        self.assertFalse(interface.is_connected)
        self.assertFalse(interface.is_operational)
        
        print(f"[{self._get_timestamp()}] [TEST] Hardware interface connection test passed")
    
    def test_hardware_interface_health_monitoring(self):
        """Test hardware interface health monitoring."""
        print(f"[{self._get_timestamp()}] [TEST] Testing hardware interface health monitoring")
        
        interface = HardwareInterface("test_imu")
        
        # Test initial health status
        health = interface.get_health_status()
        self.assertEqual(health['interface_name'], "test_imu")
        self.assertFalse(health['is_connected'])
        self.assertEqual(health['health_score'], 0.0)
        
        # Connect and test health
        interface.connect()
        interface.start_operation()
        
        health = interface.get_health_status()
        self.assertTrue(health['is_connected'])
        self.assertTrue(health['is_operational'])
        self.assertGreater(health['health_score'], 0.5)
        
        # Simulate errors and test health degradation
        interface.error_count = 5
        health = interface.get_health_status()
        self.assertLess(health['health_score'], 0.8)  # Should be penalized for errors
        
        print(f"[{self._get_timestamp()}] [TEST] Hardware interface health monitoring test passed")
    
    def test_navigation_hitl_scenario_creation(self):
        """Test navigation HITL scenario creation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing navigation HITL scenario creation")
        
        scenario = NavigationHITLScenario()
        
        self.assertEqual(scenario.scenario_name, "navigation_hitl")
        self.assertIn("navigation testing", scenario.description)
        self.assertEqual(len(scenario.hardware_interfaces), 4)
        
        # Check that all expected interfaces are present
        expected_interfaces = ["camera", "motors", "imu", "encoders"]
        for interface_name in expected_interfaces:
            self.assertIn(interface_name, scenario.hardware_interfaces)
        
        print(f"[{self._get_timestamp()}] [TEST] Navigation HITL scenario creation test passed")
    
    def test_navigation_hitl_scenario_execution(self):
        """Test navigation HITL scenario execution."""
        print(f"[{self._get_timestamp()}] [TEST] Testing navigation HITL scenario execution")
        
        scenario = NavigationHITLScenario()
        
        # Set up hardware
        setup_success = scenario.setup_hardware()
        self.assertTrue(setup_success)
        
        # Start monitoring
        scenario.start_monitoring()
        
        # Execute scenario
        results = scenario.execute()
        
        # Stop monitoring
        scenario.stop_monitoring()
        
        # Validate results structure
        self.assertIn('execution_success', results)
        self.assertIn('test_duration', results)
        self.assertIn('commands_executed', results)
        self.assertIn('data_points_collected', results)
        self.assertIn('performance_analysis', results)
        self.assertIn('hardware_status', results)
        
        # Check that some data was collected
        self.assertGreater(results['data_points_collected'], 0)
        self.assertGreater(results['test_duration'], 0)
        
        # Validate scenario results
        validation_passed = scenario.validate_results()
        self.assertIsInstance(validation_passed, bool)
        
        # Clean up
        scenario.teardown_hardware()
        
        print(f"[{self._get_timestamp()}] [TEST] Navigation HITL scenario execution test passed")
    
    def test_performance_analysis(self):
        """Test performance analysis functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing performance analysis")
        
        scenario = NavigationHITLScenario()
        scenario.setup_hardware()
        
        # Generate some test data
        for i in range(20):
            test_data_point = {
                'timestamp': time.time() + i * 0.1,
                'interfaces': {}
            }
            
            for name, interface in scenario.hardware_interfaces.items():
                health_status = interface.get_health_status()
                sensor_data = interface.read_sensor_data()
                
                test_data_point['interfaces'][name] = {
                    'health': health_status,
                    'sensor_data': sensor_data
                }
            
            scenario.test_data.append(test_data_point)
        
        scenario.start_time = time.time() - 2.0
        scenario.end_time = time.time()
        
        # Analyze performance
        analysis = scenario.analyze_performance()
        
        # Validate analysis structure
        self.assertIn('data_points', analysis)
        self.assertIn('test_duration', analysis)
        self.assertIn('interface_analysis', analysis)
        self.assertIn('overall_health', analysis)
        self.assertIn('performance_summary', analysis)
        
        # Check that analysis contains data for all interfaces
        for interface_name in scenario.hardware_interfaces.keys():
            self.assertIn(interface_name, analysis['interface_analysis'])
        
        # Check overall health metrics
        self.assertIn('avg_system_health', analysis['overall_health'])
        self.assertIn('healthy_interfaces', analysis['overall_health'])
        
        scenario.teardown_hardware()
        
        print(f"[{self._get_timestamp()}] [TEST] Performance analysis test passed")
    
    def test_monitoring_thread_functionality(self):
        """Test monitoring thread functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing monitoring thread functionality")
        
        scenario = NavigationHITLScenario()
        scenario.setup_hardware()
        
        # Start monitoring
        scenario.start_monitoring()
        self.assertTrue(scenario.monitoring_active)
        self.assertIsNotNone(scenario.monitoring_thread)
        
        # Let monitoring run for a short time
        time.sleep(0.5)
        
        # Check that data was collected
        initial_data_count = len(scenario.test_data)
        self.assertGreater(initial_data_count, 0)
        
        # Let it run a bit more
        time.sleep(0.3)
        
        # Should have more data
        final_data_count = len(scenario.test_data)
        self.assertGreater(final_data_count, initial_data_count)
        
        # Stop monitoring
        scenario.stop_monitoring()
        self.assertFalse(scenario.monitoring_active)
        
        scenario.teardown_hardware()
        
        print(f"[{self._get_timestamp()}] [TEST] Monitoring thread functionality test passed")
    
    def test_error_handling_in_hitl_scenario(self):
        """Test error handling in HITL scenario."""
        print(f"[{self._get_timestamp()}] [TEST] Testing error handling in HITL scenario")
        
        scenario = NavigationHITLScenario()
        
        # Test with hardware setup failure (simulate by not connecting)
        # This should be handled gracefully
        
        # Test command execution with invalid interface
        invalid_commands = [
            {'interface': 'nonexistent', 'action': 'send_command', 'parameters': {}, 'duration': 0.1}
        ]
        
        success = scenario.execute_test_sequence(invalid_commands)
        self.assertFalse(success)  # Should fail gracefully
        
        # Test with valid hardware
        scenario.setup_hardware()
        
        # Test with invalid action
        invalid_action_commands = [
            {'interface': 'camera', 'action': 'invalid_action', 'parameters': {}, 'duration': 0.1}
        ]
        
        success = scenario.execute_test_sequence(invalid_action_commands)
        self.assertFalse(success)  # Should fail gracefully
        
        scenario.teardown_hardware()
        
        print(f"[{self._get_timestamp()}] [TEST] Error handling in HITL scenario test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def run_enhanced_hitl_tests():
    """Run all enhanced hardware-in-the-loop tests."""
    print(f"[{TestEnhancedHardwareInTheLoop()._get_timestamp()}] [TEST] Starting enhanced HITL test suite...")
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test class
    tests = unittest.TestLoader().loadTestsFromTestCase(TestEnhancedHardwareInTheLoop)
    test_suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n[{TestEnhancedHardwareInTheLoop()._get_timestamp()}] [TEST] Enhanced HITL Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\n[{TestEnhancedHardwareInTheLoop()._get_timestamp()}] [TEST] Failures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\n[{TestEnhancedHardwareInTheLoop()._get_timestamp()}] [TEST] Errors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_enhanced_hitl_tests()
    sys.exit(0 if success else 1)
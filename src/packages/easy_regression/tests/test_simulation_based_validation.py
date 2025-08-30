#!/usr/bin/env python3
"""
Unit tests for Simulation-Based Validation module.

This test suite validates the simulation-based testing capabilities including
Gazebo integration, scenario generation, automated execution, and performance
regression testing.
"""

import unittest
import tempfile
import shutil
import json
import time
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock
import numpy as np
import sys
import os
from datetime import datetime

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from simulation_based_validation import (
    SimulationEnvironment, TestScenario, SimulationResult,
    GazeboSimulationManager, ScenarioGenerator, PerformanceRegressionTester
)


class TestSimulationEnvironment(unittest.TestCase):
    """Test SimulationEnvironment data structure."""
    
    def test_simulation_environment_creation(self):
        """Test SimulationEnvironment creation and serialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing SimulationEnvironment creation")
        
        env = SimulationEnvironment(
            name="test_env",
            world_file="test.world",
            robot_model="test_robot.sdf",
            physics_engine="ode",
            real_time_factor=1.5,
            step_size=0.002
        )
        
        self.assertEqual(env.name, "test_env")
        self.assertEqual(env.world_file, "test.world")
        self.assertEqual(env.robot_model, "test_robot.sdf")
        self.assertEqual(env.physics_engine, "ode")
        self.assertEqual(env.real_time_factor, 1.5)
        self.assertEqual(env.step_size, 0.002)
        
        # Test serialization
        env_dict = env.to_dict()
        self.assertIsInstance(env_dict, dict)
        self.assertEqual(env_dict['name'], "test_env")
        self.assertEqual(env_dict['world_file'], "test.world")
        
        print(f"[{self._get_timestamp()}] [TEST] SimulationEnvironment creation test passed")
    
    def test_simulation_environment_defaults(self):
        """Test SimulationEnvironment with default values."""
        print(f"[{self._get_timestamp()}] [TEST] Testing SimulationEnvironment defaults")
        
        env = SimulationEnvironment(
            name="minimal_env",
            world_file="minimal.world",
            robot_model="minimal.sdf"
        )
        
        self.assertEqual(env.physics_engine, "ode")
        self.assertEqual(env.real_time_factor, 1.0)
        self.assertEqual(env.step_size, 0.001)
        self.assertEqual(env.max_step_size, 0.001)
        self.assertEqual(env.gravity, (0.0, 0.0, -9.81))
        
        print(f"[{self._get_timestamp()}] [TEST] SimulationEnvironment defaults test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestTestScenario(unittest.TestCase):
    """Test TestScenario data structure."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.test_env = SimulationEnvironment(
            name="test_env",
            world_file="test.world",
            robot_model="test.sdf"
        )
    
    def test_test_scenario_creation(self):
        """Test TestScenario creation and serialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing TestScenario creation")
        
        scenario = TestScenario(
            scenario_id="test_001",
            name="Test Scenario",
            description="A test scenario for validation",
            environment=self.test_env,
            initial_conditions={'robot_x': 0.0, 'robot_y': 0.0},
            test_objectives=["Objective 1", "Objective 2"],
            success_criteria={'completion_time': {'max': 30.0}},
            timeout_seconds=45.0
        )
        
        self.assertEqual(scenario.scenario_id, "test_001")
        self.assertEqual(scenario.name, "Test Scenario")
        self.assertEqual(scenario.environment, self.test_env)
        self.assertEqual(len(scenario.test_objectives), 2)
        self.assertEqual(scenario.timeout_seconds, 45.0)
        
        # Test serialization
        scenario_dict = scenario.to_dict()
        self.assertIsInstance(scenario_dict, dict)
        self.assertEqual(scenario_dict['scenario_id'], "test_001")
        self.assertIn('environment', scenario_dict)
        self.assertIn('initial_conditions', scenario_dict)
        
        print(f"[{self._get_timestamp()}] [TEST] TestScenario creation test passed")
    
    def test_test_scenario_defaults(self):
        """Test TestScenario with default timeout."""
        print(f"[{self._get_timestamp()}] [TEST] Testing TestScenario defaults")
        
        scenario = TestScenario(
            scenario_id="test_002",
            name="Default Timeout Scenario",
            description="Test default timeout",
            environment=self.test_env,
            initial_conditions={},
            test_objectives=[],
            success_criteria={}
        )
        
        self.assertEqual(scenario.timeout_seconds, 60.0)
        
        print(f"[{self._get_timestamp()}] [TEST] TestScenario defaults test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestSimulationResult(unittest.TestCase):
    """Test SimulationResult data structure."""
    
    def test_simulation_result_creation(self):
        """Test SimulationResult creation and serialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing SimulationResult creation")
        
        result = SimulationResult(
            scenario_id="test_001",
            success=True,
            execution_time=15.5,
            metrics={'distance': 2.5, 'accuracy': 0.85},
            errors=[],
            warnings=["Minor warning"],
            timestamp="2024-01-01T12:00:00"
        )
        
        self.assertEqual(result.scenario_id, "test_001")
        self.assertTrue(result.success)
        self.assertEqual(result.execution_time, 15.5)
        self.assertEqual(result.metrics['distance'], 2.5)
        self.assertEqual(len(result.warnings), 1)
        
        # Test serialization
        result_dict = result.to_dict()
        self.assertIsInstance(result_dict, dict)
        self.assertEqual(result_dict['scenario_id'], "test_001")
        self.assertTrue(result_dict['success'])
        
        print(f"[{self._get_timestamp()}] [TEST] SimulationResult creation test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestGazeboSimulationManager(unittest.TestCase):
    """Test GazeboSimulationManager functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.sim_manager = GazeboSimulationManager(
            gazebo_path="mock_gazebo",
            output_dir=self.temp_dir
        )
    
    def tearDown(self):
        """Clean up test fixtures."""
        shutil.rmtree(self.temp_dir)
    
    def test_simulation_manager_initialization(self):
        """Test GazeboSimulationManager initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing GazeboSimulationManager initialization")
        
        self.assertEqual(self.sim_manager.gazebo_path, "mock_gazebo")
        self.assertEqual(str(self.sim_manager.output_dir), self.temp_dir)
        self.assertFalse(self.sim_manager.is_running)
        self.assertIsNone(self.sim_manager.simulation_process)
        
        print(f"[{self._get_timestamp()}] [TEST] GazeboSimulationManager initialization test passed")
    
    @patch('subprocess.run')
    def test_gazebo_availability_check_success(self, mock_subprocess):
        """Test successful Gazebo availability check."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Gazebo availability check (success)")
        
        # Mock successful Gazebo version check
        mock_result = Mock()
        mock_result.returncode = 0
        mock_result.stdout = "Gazebo multi-robot simulator, version 11.0.0"
        mock_subprocess.return_value = mock_result
        
        available = self.sim_manager.check_gazebo_availability()
        
        self.assertTrue(available)
        mock_subprocess.assert_called_once()
        
        print(f"[{self._get_timestamp()}] [TEST] Gazebo availability check (success) test passed")
    
    @patch('subprocess.run')
    def test_gazebo_availability_check_failure(self, mock_subprocess):
        """Test failed Gazebo availability check."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Gazebo availability check (failure)")
        
        # Mock failed Gazebo version check
        mock_result = Mock()
        mock_result.returncode = 1
        mock_result.stderr = "Command not found"
        mock_subprocess.return_value = mock_result
        
        available = self.sim_manager.check_gazebo_availability()
        
        self.assertFalse(available)
        
        print(f"[{self._get_timestamp()}] [TEST] Gazebo availability check (failure) test passed")
    
    @patch('subprocess.run')
    def test_gazebo_availability_check_timeout(self, mock_subprocess):
        """Test Gazebo availability check timeout."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Gazebo availability check (timeout)")
        
        # Mock timeout
        mock_subprocess.side_effect = subprocess.TimeoutExpired("gazebo", 10)
        
        available = self.sim_manager.check_gazebo_availability()
        
        self.assertFalse(available)
        
        print(f"[{self._get_timestamp()}] [TEST] Gazebo availability check (timeout) test passed")
    
    def test_create_duckietown_world(self):
        """Test Duckietown world file creation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Duckietown world creation")
        
        world_file = self.sim_manager.create_duckietown_world("test_world")
        
        # Check that file was created
        self.assertTrue(Path(world_file).exists())
        
        # Check file content
        with open(world_file, 'r') as f:
            content = f.read()
        
        self.assertIn('<?xml version="1.0" ?>', content)
        self.assertIn('<sdf version="1.6">', content)
        self.assertIn('<world name="test_world">', content)
        self.assertIn('straight_road_1', content)
        self.assertIn('lane_marking', content)
        self.assertIn('duckiebot_spawn', content)
        
        print(f"[{self._get_timestamp()}] [TEST] Duckietown world creation test passed")
    
    def test_create_duckiebot_model(self):
        """Test Duckiebot model file creation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Duckiebot model creation")
        
        model_file = self.sim_manager.create_duckiebot_model("test_duckiebot")
        
        # Check that file was created
        self.assertTrue(Path(model_file).exists())
        
        # Check file content
        with open(model_file, 'r') as f:
            content = f.read()
        
        self.assertIn('<?xml version="1.0"?>', content)
        self.assertIn('<sdf version="1.6">', content)
        self.assertIn('<model name="test_duckiebot">', content)
        self.assertIn('base_link', content)
        self.assertIn('left_wheel', content)
        self.assertIn('right_wheel', content)
        self.assertIn('camera_link', content)
        self.assertIn('differential_drive_controller', content)
        
        print(f"[{self._get_timestamp()}] [TEST] Duckiebot model creation test passed")
    
    @patch('subprocess.Popen')
    def test_start_simulation_success(self, mock_popen):
        """Test successful simulation start."""
        print(f"[{self._get_timestamp()}] [TEST] Testing simulation start (success)")
        
        # Create a test world file
        world_file = self.sim_manager.create_duckietown_world("test_world")
        
        # Mock successful Gazebo process
        mock_process = Mock()
        mock_process.poll.return_value = None  # Process is running
        mock_popen.return_value = mock_process
        
        environment = SimulationEnvironment(
            name="test_env",
            world_file=world_file,
            robot_model="test.sdf"
        )
        
        with patch('time.sleep'):  # Skip the sleep
            success = self.sim_manager.start_simulation(environment)
        
        self.assertTrue(success)
        self.assertTrue(self.sim_manager.is_running)
        self.assertEqual(self.sim_manager.simulation_process, mock_process)
        
        print(f"[{self._get_timestamp()}] [TEST] Simulation start (success) test passed")
    
    @patch('subprocess.Popen')
    def test_start_simulation_failure(self, mock_popen):
        """Test failed simulation start."""
        print(f"[{self._get_timestamp()}] [TEST] Testing simulation start (failure)")
        
        # Create a test world file
        world_file = self.sim_manager.create_duckietown_world("test_world")
        
        # Mock failed Gazebo process
        mock_process = Mock()
        mock_process.poll.return_value = 1  # Process exited with error
        mock_process.communicate.return_value = ("", "Gazebo failed to start")
        mock_popen.return_value = mock_process
        
        environment = SimulationEnvironment(
            name="test_env",
            world_file=world_file,
            robot_model="test.sdf"
        )
        
        with patch('time.sleep'):  # Skip the sleep
            success = self.sim_manager.start_simulation(environment)
        
        self.assertFalse(success)
        self.assertFalse(self.sim_manager.is_running)
        
        print(f"[{self._get_timestamp()}] [TEST] Simulation start (failure) test passed")
    
    def test_stop_simulation(self):
        """Test simulation stop."""
        print(f"[{self._get_timestamp()}] [TEST] Testing simulation stop")
        
        # Mock running simulation
        mock_process = Mock()
        mock_process.terminate = Mock()
        mock_process.wait = Mock()
        
        self.sim_manager.simulation_process = mock_process
        self.sim_manager.is_running = True
        
        self.sim_manager.stop_simulation()
        
        self.assertFalse(self.sim_manager.is_running)
        self.assertIsNone(self.sim_manager.simulation_process)
        mock_process.terminate.assert_called_once()
        mock_process.wait.assert_called_once()
        
        print(f"[{self._get_timestamp()}] [TEST] Simulation stop test passed")
    
    def test_run_scenario(self):
        """Test complete scenario execution."""
        print(f"[{self._get_timestamp()}] [TEST] Testing scenario execution")
        
        # Create test scenario
        environment = SimulationEnvironment(
            name="test_env",
            world_file="test.world",
            robot_model="test.sdf"
        )
        
        scenario = TestScenario(
            scenario_id="test_scenario",
            name="Test Scenario",
            description="Test scenario execution",
            environment=environment,
            initial_conditions={},
            test_objectives=["Test objective"],
            success_criteria={'completion_time': {'max': 10.0}},
            timeout_seconds=5.0
        )
        
        # Mock simulation methods
        with patch.object(self.sim_manager, 'start_simulation', return_value=True), \
             patch.object(self.sim_manager, 'stop_simulation'), \
             patch.object(self.sim_manager, '_execute_scenario_logic', return_value=(True, {'completion_time': 8.0})), \
             patch.object(self.sim_manager, '_evaluate_success_criteria', return_value=True):
            
            result = self.sim_manager.run_scenario(scenario)
        
        self.assertIsInstance(result, SimulationResult)
        self.assertEqual(result.scenario_id, "test_scenario")
        self.assertTrue(result.success)
        self.assertGreater(result.execution_time, 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Scenario execution test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestScenarioGenerator(unittest.TestCase):
    """Test ScenarioGenerator functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.generator = ScenarioGenerator()
    
    def test_scenario_generator_initialization(self):
        """Test ScenarioGenerator initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ScenarioGenerator initialization")
        
        self.assertEqual(len(self.generator.scenario_templates), 0)
        self.assertEqual(len(self.generator.generated_scenarios), 0)
        
        print(f"[{self._get_timestamp()}] [TEST] ScenarioGenerator initialization test passed")
    
    def test_add_scenario_template(self):
        """Test adding scenario templates."""
        print(f"[{self._get_timestamp()}] [TEST] Testing scenario template addition")
        
        template = {
            'name': 'Test Template',
            'description': 'A test template',
            'parameters': {'param1': 'value1'}
        }
        
        self.generator.add_scenario_template("test_template", template)
        
        self.assertEqual(len(self.generator.scenario_templates), 1)
        self.assertIn("test_template", self.generator.scenario_templates)
        self.assertEqual(self.generator.scenario_templates["test_template"], template)
        
        print(f"[{self._get_timestamp()}] [TEST] Scenario template addition test passed")
    
    def test_generate_lane_following_scenarios(self):
        """Test lane following scenario generation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing lane following scenario generation")
        
        scenarios = self.generator.generate_lane_following_scenarios(count=5)
        
        self.assertEqual(len(scenarios), 5)
        self.assertEqual(len(self.generator.generated_scenarios), 5)
        
        # Check scenario properties
        for i, scenario in enumerate(scenarios):
            self.assertEqual(scenario.scenario_id, f"lane_following_{i:03d}")
            self.assertIn("Lane Following Test", scenario.name)
            self.assertIn("lane following", scenario.description.lower())
            self.assertIn("Maintain lane position", scenario.test_objectives)
            self.assertIn("lane_following_accuracy", scenario.success_criteria)
            self.assertEqual(scenario.timeout_seconds, 20.0)
        
        print(f"[{self._get_timestamp()}] [TEST] Lane following scenario generation test passed")
    
    def test_generate_obstacle_avoidance_scenarios(self):
        """Test obstacle avoidance scenario generation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing obstacle avoidance scenario generation")
        
        scenarios = self.generator.generate_obstacle_avoidance_scenarios(count=3)
        
        self.assertEqual(len(scenarios), 3)
        
        # Check scenario properties
        for i, scenario in enumerate(scenarios):
            self.assertEqual(scenario.scenario_id, f"obstacle_avoidance_{i:03d}")
            self.assertIn("Obstacle Avoidance Test", scenario.name)
            self.assertIn("obstacle avoidance", scenario.description.lower())
            self.assertIn("Detect obstacles", scenario.test_objectives)
            self.assertIn("obstacle_avoidance_success", scenario.success_criteria)
            self.assertEqual(scenario.timeout_seconds, 30.0)
            
            # Check that obstacles are defined in initial conditions
            self.assertIn("obstacles", scenario.initial_conditions)
            obstacles = scenario.initial_conditions["obstacles"]
            self.assertGreater(len(obstacles), 0)
            self.assertLessEqual(len(obstacles), 3)
        
        print(f"[{self._get_timestamp()}] [TEST] Obstacle avoidance scenario generation test passed")
    
    def test_generate_intersection_scenarios(self):
        """Test intersection scenario generation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing intersection scenario generation")
        
        scenarios = self.generator.generate_intersection_scenarios(count=2)
        
        self.assertEqual(len(scenarios), 2)
        
        # Check scenario properties
        for i, scenario in enumerate(scenarios):
            self.assertEqual(scenario.scenario_id, f"intersection_{i:03d}")
            self.assertIn("Intersection Navigation Test", scenario.name)
            self.assertIn("intersection", scenario.description.lower())
            self.assertIn("Approach intersection safely", scenario.test_objectives)
            self.assertIn("intersection_navigation_success", scenario.success_criteria)
            self.assertEqual(scenario.timeout_seconds, 40.0)
            
            # Check intersection-specific initial conditions
            self.assertIn("intersection_type", scenario.initial_conditions)
            self.assertIn("traffic_density", scenario.initial_conditions)
            self.assertIn("target_direction", scenario.initial_conditions)
        
        print(f"[{self._get_timestamp()}] [TEST] Intersection scenario generation test passed")
    
    def test_save_and_load_scenarios(self):
        """Test scenario saving and loading."""
        print(f"[{self._get_timestamp()}] [TEST] Testing scenario save and load")
        
        # Generate some scenarios
        scenarios = self.generator.generate_lane_following_scenarios(count=2)
        
        # Save scenarios
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            temp_file = f.name
        
        try:
            self.generator.save_scenarios(temp_file)
            
            # Check that file was created and contains data
            self.assertTrue(Path(temp_file).exists())
            
            with open(temp_file, 'r') as f:
                saved_data = json.load(f)
            
            self.assertEqual(len(saved_data), 2)
            
            # Load scenarios
            loaded_scenarios = self.generator.load_scenarios(temp_file)
            
            self.assertEqual(len(loaded_scenarios), 2)
            
            # Check that loaded scenarios match original
            for original, loaded in zip(scenarios, loaded_scenarios):
                self.assertEqual(original.scenario_id, loaded.scenario_id)
                self.assertEqual(original.name, loaded.name)
                self.assertEqual(original.description, loaded.description)
                self.assertEqual(original.timeout_seconds, loaded.timeout_seconds)
        
        finally:
            # Clean up
            if Path(temp_file).exists():
                Path(temp_file).unlink()
        
        print(f"[{self._get_timestamp()}] [TEST] Scenario save and load test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestPerformanceRegressionTester(unittest.TestCase):
    """Test PerformanceRegressionTester functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.baseline_file = str(Path(self.temp_dir) / "test_baseline.json")
        self.tester = PerformanceRegressionTester(baseline_file=self.baseline_file)
    
    def tearDown(self):
        """Clean up test fixtures."""
        shutil.rmtree(self.temp_dir)
    
    def test_regression_tester_initialization(self):
        """Test PerformanceRegressionTester initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing PerformanceRegressionTester initialization")
        
        self.assertEqual(self.tester.baseline_file, self.baseline_file)
        self.assertEqual(len(self.tester.baseline_data), 0)
        self.assertEqual(len(self.tester.current_results), 0)
        
        print(f"[{self._get_timestamp()}] [TEST] PerformanceRegressionTester initialization test passed")
    
    def test_add_result(self):
        """Test adding simulation results."""
        print(f"[{self._get_timestamp()}] [TEST] Testing result addition")
        
        result = SimulationResult(
            scenario_id="test_001",
            success=True,
            execution_time=10.0,
            metrics={'accuracy': 0.9},
            errors=[],
            warnings=[],
            timestamp=datetime.now().isoformat()
        )
        
        self.tester.add_result(result)
        
        self.assertEqual(len(self.tester.current_results), 1)
        self.assertEqual(self.tester.current_results[0], result)
        
        print(f"[{self._get_timestamp()}] [TEST] Result addition test passed")
    
    def test_create_new_baseline(self):
        """Test creating new baseline from current results."""
        print(f"[{self._get_timestamp()}] [TEST] Testing new baseline creation")
        
        # Add some results
        for i in range(3):
            result = SimulationResult(
                scenario_id=f"test_{i:03d}",
                success=True,
                execution_time=10.0 + i,
                metrics={'accuracy': 0.9 - i * 0.1},
                errors=[],
                warnings=[],
                timestamp=datetime.now().isoformat()
            )
            self.tester.add_result(result)
        
        # Analyze regression (should create new baseline)
        analysis = self.tester.analyze_regression()
        
        self.assertIn('baseline_created', analysis)
        self.assertTrue(analysis['baseline_created'])
        self.assertEqual(analysis['scenarios_count'], 3)
        
        # Check that baseline file was created
        self.assertTrue(Path(self.baseline_file).exists())
        
        # Check baseline data
        self.assertEqual(len(self.tester.baseline_data), 3)
        
        print(f"[{self._get_timestamp()}] [TEST] New baseline creation test passed")
    
    def test_regression_analysis_with_baseline(self):
        """Test regression analysis with existing baseline."""
        print(f"[{self._get_timestamp()}] [TEST] Testing regression analysis with baseline")
        
        # Create baseline data
        baseline_data = {
            "test_001": {
                "scenario_id": "test_001",
                "success": True,
                "execution_time": 10.0,
                "metrics": {"accuracy": 0.9, "distance": 2.0}
            },
            "test_002": {
                "scenario_id": "test_002",
                "success": True,
                "execution_time": 12.0,
                "metrics": {"accuracy": 0.85, "distance": 2.2}
            }
        }
        
        # Save baseline
        with open(self.baseline_file, 'w') as f:
            json.dump(baseline_data, f)
        
        # Reload tester to pick up baseline
        self.tester = PerformanceRegressionTester(baseline_file=self.baseline_file)
        
        # Add current results (some with regressions)
        results = [
            SimulationResult(
                scenario_id="test_001",
                success=True,
                execution_time=10.5,  # Slight increase, within tolerance
                metrics={"accuracy": 0.88, "distance": 2.0},  # Slight accuracy decrease
                errors=[], warnings=[], timestamp=datetime.now().isoformat()
            ),
            SimulationResult(
                scenario_id="test_002",
                success=False,  # Success regression
                execution_time=15.0,  # Significant time increase
                metrics={"accuracy": 0.7, "distance": 1.8},  # Significant accuracy decrease
                errors=[], warnings=[], timestamp=datetime.now().isoformat()
            )
        ]
        
        for result in results:
            self.tester.add_result(result)
        
        # Analyze regression
        analysis = self.tester.analyze_regression()
        
        self.assertIn('regression_detected', analysis)
        self.assertTrue(analysis['regression_detected'])  # Should detect regression
        self.assertEqual(analysis['total_scenarios'], 2)
        self.assertIn('detailed_analysis', analysis)
        self.assertEqual(len(analysis['detailed_analysis']), 2)
        
        # Check that test_002 shows regression
        test_002_analysis = next(a for a in analysis['detailed_analysis'] if a['scenario_id'] == 'test_002')
        self.assertTrue(test_002_analysis['regression_detected'])
        self.assertIn('success_regression', test_002_analysis['changes'])
        
        print(f"[{self._get_timestamp()}] [TEST] Regression analysis with baseline test passed")
    
    def test_update_baseline(self):
        """Test baseline update functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing baseline update")
        
        # Add some results
        result = SimulationResult(
            scenario_id="test_001",
            success=True,
            execution_time=10.0,
            metrics={'accuracy': 0.9},
            errors=[],
            warnings=[],
            timestamp=datetime.now().isoformat()
        )
        
        self.tester.add_result(result)
        
        # Update baseline
        self.tester.update_baseline()
        
        # Check that baseline was updated
        self.assertEqual(len(self.tester.baseline_data), 1)
        self.assertIn("test_001", self.tester.baseline_data)
        
        # Check that baseline file was created
        self.assertTrue(Path(self.baseline_file).exists())
        
        with open(self.baseline_file, 'r') as f:
            saved_baseline = json.load(f)
        
        self.assertEqual(len(saved_baseline), 1)
        self.assertIn("test_001", saved_baseline)
        
        print(f"[{self._get_timestamp()}] [TEST] Baseline update test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestIntegration(unittest.TestCase):
    """Integration tests for simulation-based validation."""
    
    def setUp(self):
        """Set up integration test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
    
    def tearDown(self):
        """Clean up integration test fixtures."""
        shutil.rmtree(self.temp_dir)
    
    def test_end_to_end_simulation_workflow(self):
        """Test complete end-to-end simulation workflow."""
        print(f"[{self._get_timestamp()}] [TEST] Testing end-to-end simulation workflow")
        
        # 1. Create simulation manager
        sim_manager = GazeboSimulationManager(output_dir=self.temp_dir)
        
        # 2. Generate scenarios
        scenario_gen = ScenarioGenerator()
        scenarios = scenario_gen.generate_lane_following_scenarios(count=2)
        
        # 3. Create regression tester
        baseline_file = str(Path(self.temp_dir) / "baseline.json")
        regression_tester = PerformanceRegressionTester(baseline_file=baseline_file)
        
        # 4. Mock scenario execution (since we don't have actual Gazebo)
        for scenario in scenarios:
            # Create mock result
            mock_result = SimulationResult(
                scenario_id=scenario.scenario_id,
                success=True,
                execution_time=np.random.uniform(8.0, 12.0),
                metrics={
                    'lane_following_accuracy': np.random.uniform(0.8, 0.95),
                    'robot_distance_traveled': np.random.uniform(2.0, 3.0),
                    'completion_time': np.random.uniform(10.0, 15.0)
                },
                errors=[],
                warnings=[],
                timestamp=datetime.now().isoformat()
            )
            
            regression_tester.add_result(mock_result)
        
        # 5. Analyze regression
        analysis = regression_tester.analyze_regression()
        
        # 6. Verify workflow results
        self.assertIn('baseline_created', analysis)
        self.assertTrue(analysis['baseline_created'])
        self.assertEqual(len(regression_tester.current_results), 2)
        
        # 7. Save scenarios
        scenario_file = str(Path(self.temp_dir) / "scenarios.json")
        scenario_gen.save_scenarios(scenario_file)
        self.assertTrue(Path(scenario_file).exists())
        
        # 8. Verify world and model files were created
        world_files = list(Path(self.temp_dir).glob("*.world"))
        model_files = list(Path(self.temp_dir).glob("*.sdf"))
        
        self.assertGreater(len(world_files), 0)
        
        print(f"[{self._get_timestamp()}] [TEST] End-to-end simulation workflow test passed")
    
    def test_scenario_generation_and_execution_consistency(self):
        """Test consistency between scenario generation and execution."""
        print(f"[{self._get_timestamp()}] [TEST] Testing scenario generation and execution consistency")
        
        # Generate different types of scenarios
        scenario_gen = ScenarioGenerator()
        
        lane_scenarios = scenario_gen.generate_lane_following_scenarios(count=2)
        obstacle_scenarios = scenario_gen.generate_obstacle_avoidance_scenarios(count=2)
        intersection_scenarios = scenario_gen.generate_intersection_scenarios(count=1)
        
        all_scenarios = lane_scenarios + obstacle_scenarios + intersection_scenarios
        
        # Verify scenario properties are consistent
        scenario_ids = [s.scenario_id for s in all_scenarios]
        self.assertEqual(len(scenario_ids), len(set(scenario_ids)))  # All IDs unique
        
        # Verify each scenario has required properties
        for scenario in all_scenarios:
            self.assertIsNotNone(scenario.scenario_id)
            self.assertIsNotNone(scenario.name)
            self.assertIsNotNone(scenario.description)
            self.assertIsNotNone(scenario.environment)
            self.assertIsInstance(scenario.initial_conditions, dict)
            self.assertIsInstance(scenario.test_objectives, list)
            self.assertIsInstance(scenario.success_criteria, dict)
            self.assertGreater(scenario.timeout_seconds, 0)
            
            # Verify environment properties
            self.assertIsNotNone(scenario.environment.name)
            self.assertIsNotNone(scenario.environment.world_file)
            self.assertIsNotNone(scenario.environment.robot_model)
        
        # Test serialization consistency
        temp_file = str(Path(self.temp_dir) / "test_scenarios.json")
        scenario_gen.save_scenarios(temp_file)
        
        loaded_scenarios = scenario_gen.load_scenarios(temp_file)
        
        self.assertEqual(len(loaded_scenarios), len(all_scenarios))
        
        for original, loaded in zip(all_scenarios, loaded_scenarios):
            self.assertEqual(original.scenario_id, loaded.scenario_id)
            self.assertEqual(original.name, loaded.name)
            self.assertEqual(original.environment.name, loaded.environment.name)
        
        print(f"[{self._get_timestamp()}] [TEST] Scenario generation and execution consistency test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def run_simulation_validation_tests():
    """Run all simulation-based validation tests."""
    print(f"[{TestSimulationEnvironment()._get_timestamp()}] [TEST] Starting simulation-based validation test suite...")
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestSimulationEnvironment,
        TestTestScenario,
        TestSimulationResult,
        TestGazeboSimulationManager,
        TestScenarioGenerator,
        TestPerformanceRegressionTester,
        TestIntegration
    ]
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n[{TestSimulationEnvironment()._get_timestamp()}] [TEST] Simulation Validation Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\n[{TestSimulationEnvironment()._get_timestamp()}] [TEST] Failures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\n[{TestSimulationEnvironment()._get_timestamp()}] [TEST] Errors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_simulation_validation_tests()
    sys.exit(0 if success else 1)
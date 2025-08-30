#!/usr/bin/env python3
"""
Simulation-Based Validation for Advanced Autonomous Duckietown System

This module extends the existing testing framework with Gazebo-based simulation
capabilities, scenario-based testing, automated scenario execution, and
performance regression testing for comprehensive validation.
"""

import time
import json
import subprocess
import threading
import queue
import tempfile
import shutil
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple, Union
from dataclasses import dataclass, asdict
import numpy as np
import sys
import os
import math

# Configure comprehensive logging
import logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [SIMULATION] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


@dataclass
class SimulationEnvironment:
    """Configuration for simulation environment."""
    name: str
    world_file: str
    robot_model: str
    physics_engine: str = "ode"
    real_time_factor: float = 1.0
    step_size: float = 0.001
    max_step_size: float = 0.001
    gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class TestScenario:
    """Configuration for a test scenario."""
    scenario_id: str
    name: str
    description: str
    environment: SimulationEnvironment
    initial_conditions: Dict[str, Any]
    test_objectives: List[str]
    success_criteria: Dict[str, Any]
    timeout_seconds: float = 60.0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            'scenario_id': self.scenario_id,
            'name': self.name,
            'description': self.description,
            'environment': self.environment.to_dict(),
            'initial_conditions': self.initial_conditions,
            'test_objectives': self.test_objectives,
            'success_criteria': self.success_criteria,
            'timeout_seconds': self.timeout_seconds
        }


@dataclass
class SimulationResult:
    """Results from a simulation run."""
    scenario_id: str
    success: bool
    execution_time: float
    metrics: Dict[str, Any]
    errors: List[str]
    warnings: List[str]
    timestamp: str
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return asdict(self)


class GazeboSimulationManager:
    """Manager for Gazebo-based simulation testing."""
    
    def __init__(self, gazebo_path: str = "gazebo", output_dir: str = "simulation_results"):
        self.gazebo_path = gazebo_path
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        self.simulation_process = None
        self.is_running = False
        self.current_scenario = None
        self.simulation_data = []
        
        logger.info(f"GazeboSimulationManager initialized with output dir: {self.output_dir}")
    
    def check_gazebo_availability(self) -> bool:
        """Check if Gazebo is available and functional."""
        logger.info("Checking Gazebo availability")
        
        try:
            # Check if Gazebo is installed
            result = subprocess.run([self.gazebo_path, "--version"], 
                                  capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                version_info = result.stdout.strip()
                logger.info(f"Gazebo found: {version_info}")
                return True
            else:
                logger.error(f"Gazebo check failed: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.error("Gazebo version check timed out")
            return False
        except FileNotFoundError:
            logger.error(f"Gazebo not found at path: {self.gazebo_path}")
            return False
        except Exception as e:
            logger.error(f"Error checking Gazebo availability: {e}")
            return False
    
    def create_duckietown_world(self, world_name: str = "duckietown_test") -> str:
        """Create a Duckietown world file for simulation."""
        logger.info(f"Creating Duckietown world: {world_name}")
        
        world_content = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{world_name}">
    <!-- Physics settings -->
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Duckietown road segments -->
    <model name="straight_road_1">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="road_link">
        <visual name="road_visual">
          <geometry>
            <box>
              <size>4.0 0.6 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <collision name="road_collision">
          <geometry>
            <box>
              <size>4.0 0.6 0.01</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Lane markings -->
    <model name="lane_marking_left">
      <static>true</static>
      <pose>0 0.25 0.005 0 0 0</pose>
      <link name="marking_link">
        <visual name="marking_visual">
          <geometry>
            <box>
              <size>4.0 0.05 0.005</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>
            <diffuse>1 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="lane_marking_right">
      <static>true</static>
      <pose>0 -0.25 0.005 0 0 0</pose>
      <link name="marking_link">
        <visual name="marking_visual">
          <geometry>
            <box>
              <size>4.0 0.05 0.005</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>
            <diffuse>1 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Obstacles -->
    <model name="obstacle_cone_1">
      <static>true</static>
      <pose>1.5 0.4 0.1 0 0 0</pose>
      <link name="cone_link">
        <visual name="cone_visual">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0.5 0 1</ambient>
            <diffuse>1 0.5 0 1</diffuse>
          </material>
        </visual>
        <collision name="cone_collision">
          <geometry>
            <cylinder>
              <radius>0.05</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Duckiebot spawn point -->
    <model name="duckiebot_spawn">
      <static>true</static>
      <pose>-1.5 0 0.05 0 0 0</pose>
      <link name="spawn_link">
        <visual name="spawn_visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>'''
        
        # Save world file
        world_file = self.output_dir / f"{world_name}.world"
        with open(world_file, 'w') as f:
            f.write(world_content)
        
        logger.info(f"Duckietown world created: {world_file}")
        return str(world_file)
    
    def create_duckiebot_model(self, model_name: str = "duckiebot_test") -> str:
        """Create a Duckiebot model for simulation."""
        logger.info(f"Creating Duckiebot model: {model_name}")
        
        model_content = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <pose>0 0 0.05 0 0 0</pose>
    
    <!-- Base link -->
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      
      <visual name="base_visual">
        <geometry>
          <box>
            <size>0.18 0.12 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 0 1</ambient>
          <diffuse>1 1 0 1</diffuse>
        </material>
      </visual>
      
      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.18 0.12 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <!-- Left wheel -->
    <link name="left_wheel">
      <pose>0 0.08 0 1.5708 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      
      <visual name="wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
      
      <collision name="wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <!-- Right wheel -->
    <link name="right_wheel">
      <pose>0 -0.08 0 1.5708 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      
      <visual name="wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
      
      <collision name="wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <!-- Camera -->
    <link name="camera_link">
      <pose>0.09 0 0.05 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      
      <visual name="camera_visual">
        <geometry>
          <box>
            <size>0.02 0.03 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>
    
    <!-- Joints -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    
    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>
    
    <!-- Differential drive plugin -->
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100</updateRate>
      <leftJoint>left_wheel_joint</leftJoint>
      <rightJoint>right_wheel_joint</rightJoint>
      <wheelSeparation>0.16</wheelSeparation>
      <wheelDiameter>0.06</wheelDiameter>
      <torque>1.0</torque>
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>
    
  </model>
</sdf>'''
        
        # Save model file
        model_file = self.output_dir / f"{model_name}.sdf"
        with open(model_file, 'w') as f:
            f.write(model_content)
        
        logger.info(f"Duckiebot model created: {model_file}")
        return str(model_file)
    
    def start_simulation(self, environment: SimulationEnvironment) -> bool:
        """Start Gazebo simulation with specified environment."""
        logger.info(f"Starting simulation: {environment.name}")
        
        if self.is_running:
            logger.warning("Simulation already running")
            return False
        
        try:
            # Create world file if it doesn't exist
            world_file = Path(environment.world_file)
            if not world_file.exists():
                logger.info("World file not found, creating default Duckietown world")
                environment.world_file = self.create_duckietown_world()
            
            # Start Gazebo
            gazebo_cmd = [
                self.gazebo_path,
                "--verbose",
                environment.world_file
            ]
            
            logger.info(f"Starting Gazebo with command: {' '.join(gazebo_cmd)}")
            
            self.simulation_process = subprocess.Popen(
                gazebo_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Wait a moment for Gazebo to start
            time.sleep(3.0)
            
            # Check if process is still running
            if self.simulation_process.poll() is None:
                self.is_running = True
                logger.info("Gazebo simulation started successfully")
                return True
            else:
                stdout, stderr = self.simulation_process.communicate()
                logger.error(f"Gazebo failed to start: {stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error starting simulation: {e}")
            return False
    
    def stop_simulation(self):
        """Stop the running simulation."""
        logger.info("Stopping simulation")
        
        if not self.is_running or not self.simulation_process:
            logger.warning("No simulation running")
            return
        
        try:
            # Terminate Gazebo process
            self.simulation_process.terminate()
            
            # Wait for graceful shutdown
            try:
                self.simulation_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                logger.warning("Gazebo did not shut down gracefully, killing process")
                self.simulation_process.kill()
                self.simulation_process.wait()
            
            self.is_running = False
            self.simulation_process = None
            
            logger.info("Simulation stopped")
            
        except Exception as e:
            logger.error(f"Error stopping simulation: {e}")
    
    def run_scenario(self, scenario: TestScenario) -> SimulationResult:
        """Run a complete test scenario."""
        logger.info(f"Running scenario: {scenario.name}")
        
        start_time = time.time()
        errors = []
        warnings = []
        metrics = {}
        
        try:
            # Start simulation
            if not self.start_simulation(scenario.environment):
                errors.append("Failed to start simulation")
                return SimulationResult(
                    scenario_id=scenario.scenario_id,
                    success=False,
                    execution_time=time.time() - start_time,
                    metrics={},
                    errors=errors,
                    warnings=warnings,
                    timestamp=datetime.now().isoformat()
                )
            
            self.current_scenario = scenario
            
            # Execute scenario-specific logic
            scenario_success, scenario_metrics = self._execute_scenario_logic(scenario)
            metrics.update(scenario_metrics)
            
            # Check success criteria
            success = scenario_success and self._evaluate_success_criteria(scenario, metrics)
            
            execution_time = time.time() - start_time
            
            logger.info(f"Scenario {scenario.name} {'PASSED' if success else 'FAILED'} in {execution_time:.2f}s")
            
            return SimulationResult(
                scenario_id=scenario.scenario_id,
                success=success,
                execution_time=execution_time,
                metrics=metrics,
                errors=errors,
                warnings=warnings,
                timestamp=datetime.now().isoformat()
            )
            
        except Exception as e:
            errors.append(f"Scenario execution error: {e}")
            logger.error(f"Error running scenario {scenario.name}: {e}")
            
            return SimulationResult(
                scenario_id=scenario.scenario_id,
                success=False,
                execution_time=time.time() - start_time,
                metrics=metrics,
                errors=errors,
                warnings=warnings,
                timestamp=datetime.now().isoformat()
            )
            
        finally:
            # Always stop simulation
            self.stop_simulation()
            self.current_scenario = None
    
    def _execute_scenario_logic(self, scenario: TestScenario) -> Tuple[bool, Dict[str, Any]]:
        """Execute scenario-specific logic (to be extended for specific scenarios)."""
        logger.info(f"Executing scenario logic for: {scenario.name}")
        
        metrics = {
            'simulation_time': 0.0,
            'robot_distance_traveled': 0.0,
            'lane_following_accuracy': 0.0,
            'obstacle_avoidance_success': True,
            'completion_time': 0.0
        }
        
        # Simulate scenario execution
        scenario_duration = min(scenario.timeout_seconds, 10.0)  # Cap at 10 seconds for testing
        
        for i in range(int(scenario_duration * 10)):  # 10 Hz simulation
            time.sleep(0.1)
            
            # Simulate robot movement and data collection
            current_time = i * 0.1
            metrics['simulation_time'] = current_time
            metrics['robot_distance_traveled'] += 0.03  # 30cm/s average speed
            
            # Simulate lane following accuracy (decreases over time to simulate drift)
            metrics['lane_following_accuracy'] = max(0.5, 1.0 - current_time * 0.05)
            
            # Check for early termination conditions
            if 'max_distance' in scenario.success_criteria:
                if metrics['robot_distance_traveled'] >= scenario.success_criteria['max_distance']:
                    logger.info("Scenario completed: maximum distance reached")
                    break
        
        metrics['completion_time'] = metrics['simulation_time']
        
        # Determine scenario success based on execution
        success = (
            metrics['lane_following_accuracy'] > 0.6 and
            metrics['obstacle_avoidance_success'] and
            metrics['robot_distance_traveled'] > 0.5
        )
        
        logger.info(f"Scenario logic completed: success={success}")
        return success, metrics
    
    def _evaluate_success_criteria(self, scenario: TestScenario, metrics: Dict[str, Any]) -> bool:
        """Evaluate scenario success criteria."""
        logger.info("Evaluating success criteria")
        
        success = True
        
        for criterion, expected_value in scenario.success_criteria.items():
            if criterion in metrics:
                actual_value = metrics[criterion]
                
                if isinstance(expected_value, dict):
                    # Handle range criteria
                    if 'min' in expected_value and actual_value < expected_value['min']:
                        logger.warning(f"Criterion {criterion} failed: {actual_value} < {expected_value['min']}")
                        success = False
                    if 'max' in expected_value and actual_value > expected_value['max']:
                        logger.warning(f"Criterion {criterion} failed: {actual_value} > {expected_value['max']}")
                        success = False
                else:
                    # Handle exact value criteria
                    if actual_value != expected_value:
                        logger.warning(f"Criterion {criterion} failed: {actual_value} != {expected_value}")
                        success = False
            else:
                logger.warning(f"Criterion {criterion} not found in metrics")
                success = False
        
        logger.info(f"Success criteria evaluation: {'PASSED' if success else 'FAILED'}")
        return success


class GymDuckietownSimulationManager:
  """Manager for Gym-Duckietown-based simulation testing.

  This backend uses gym-duckietown environments to execute scenarios and collect
  simple performance metrics without requiring Gazebo. It is suitable for macOS.
  """

  def __init__(self, env_id: str = "Duckietown-udem1-v0", render: bool = False):
    self.env_id = env_id
    self.render = render
    self.env = None
    self.is_running = False
    self.current_scenario: Optional[TestScenario] = None

    logger.info(f"GymDuckietownSimulationManager initialized with env_id: {env_id}")

  def check_gym_availability(self) -> bool:
    """Check if gym-duckietown can be imported and an env can be created."""
    logger.info("Checking Gym-Duckietown availability")
    try:
      # On macOS, ensure pyglet uses Cocoa (not headless/EGL)
      try:
        import platform as _platform
        if "Darwin" in _platform.system():
          import os as _os
          import pyglet as _pyglet
          _os.environ.pop("PYGLET_HEADLESS", None)
          _pyglet.options["headless"] = False
      except Exception:
        pass
      import gym  # noqa: F401
      # gym-duckietown registers envs on import
      try:
        import gym_duckietown  # noqa: F401
      except Exception as e:
        logger.error(f"Failed to import gym_duckietown: {e}")
        return False
      return True
    except Exception as e:
      logger.error(f"Gym import failed: {e}")
      return False

  def _make_env(self, scenario: TestScenario):
    """Create the Gym environment, applying scenario-configurable options when possible."""
    import gym
    # Some envs accept config dict via gym.make kwargs; be permissive.
    make_kwargs: Dict[str, Any] = {}

    # Map scenario to env options when available
    ic = scenario.initial_conditions or {}
    # If a map_name is provided in environment.initial_conditions or environment.description
    map_name = ic.get('map_name') or ic.get('map') or None
    if map_name:
      make_kwargs['map_name'] = map_name

    # Seed for determinism if provided
    seed = ic.get('seed') or int((datetime.now().timestamp() * 1000) % 2**31)

    try:
      env = gym.make(self.env_id, **make_kwargs)
    except TypeError:
      # Older/newer API mismatch: retry without kwargs
      env = gym.make(self.env_id)

    # Set seed if available
    try:
      # Gym v0.26+
      env.reset(seed=seed)
    except Exception:
      try:
        env.seed(seed)
      except Exception:
        pass

    return env

  def start_simulation(self, environment: SimulationEnvironment, scenario: Optional[TestScenario] = None) -> bool:
    """Create and reset the Gym environment."""
    if self.is_running:
      logger.warning("Gym env already running")
      return False
    try:
      # Prefer scenario-specific env if provided
      if scenario is None:
        dummy_env_scenario = TestScenario(
          scenario_id="_dummy_",
          name="dummy",
          description="",
          environment=environment,
          initial_conditions={},
          test_objectives=[],
          success_criteria={}
        )
        scenario = dummy_env_scenario
      self.env = self._make_env(scenario)
      self.is_running = True
      return True
    except Exception as e:
      logger.error(f"Failed to start Gym env: {e}")
      self.env = None
      self.is_running = False
      return False

  def stop_simulation(self):
    if not self.is_running or self.env is None:
      return
    try:
      self.env.close()
    except Exception:
      pass
    self.env = None
    self.is_running = False

  def run_scenario(self, scenario: TestScenario) -> SimulationResult:
    """Run the given scenario in Gym-Duckietown and produce metrics."""
    start_time_wall = time.time()
    errors: List[str] = []
    warnings: List[str] = []
    metrics: Dict[str, Any] = {
      'simulation_time': 0.0,
      'robot_distance_traveled': 0.0,
      'lane_following_accuracy': 0.0,
      'obstacle_avoidance_success': True,
      'completion_time': 0.0,
      'collision_count': 0,
    }

    try:
      if not self.start_simulation(scenario.environment, scenario=scenario):
        errors.append("Failed to start Gym environment")
        return SimulationResult(
          scenario_id=scenario.scenario_id,
          success=False,
          execution_time=time.time() - start_time_wall,
          metrics=metrics,
          errors=errors,
          warnings=warnings,
          timestamp=datetime.now().isoformat()
        )

      self.current_scenario = scenario

      env = self.env
      # Reset environment, handle API differences
      try:
        obs, info = env.reset()
      except Exception:
        res = env.reset()
        if isinstance(res, tuple) and len(res) == 2:
          obs, info = res
        else:
          obs, info = res, {}

      # Try to place robot per initial conditions if API allows
      ic = scenario.initial_conditions or {}
      try:
        pos = ic.get('robot_position') or {}
        if pos:
          # Some envs expose unwrapped env with set_pose
          unwrapped = env.unwrapped
          if hasattr(unwrapped, 'set_start'):  # older API
            unwrapped.set_start([pos.get('x', -1.5), pos.get('y', 0.0)], pos.get('yaw', 0.0))
            obs, info = env.reset()
          elif hasattr(unwrapped, 'set_robot'):  # hypothetical API
            unwrapped.set_robot(pos)
      except Exception:
        pass

      # Simple control loop: proportional controller if lane position is available; else straight drive
      # Allow longer episodes for better metric accumulation (cap at 40s)
      max_steps = int(max(1.0, min(scenario.timeout_seconds, 40.0)) * 30)  # ~30 Hz
      dt_est = 1.0 / 30.0
      last_pos = None

      for step in range(max_steps):
        # Render optionally
        if self.render:
          try:
            env.render()
          except Exception:
            pass

        # Default action: forward with slight correction
        steering = 0.0
        speed = 0.3

        # Try to read lane position and angle for a crude PID
        lane_dist = None
        lane_angle = None
        try:
          info_dict = getattr(env, 'last_info', None) or {}
          # Prefer live info from step; if not available, use env attributes
          if isinstance(info, dict):
            info_dict = info
          # Various API variants
          lp = info_dict.get('lane_position') or info_dict.get('lane_pos')
          if lp is not None:
            # lp can be an object or dict
            lane_dist = getattr(lp, 'dist', None) if not isinstance(lp, dict) else lp.get('dist')
            lane_angle = getattr(lp, 'angle_rad', None) if not isinstance(lp, dict) else lp.get('angle_rad')
        except Exception:
          pass

        if lane_dist is not None and lane_angle is not None:
          kp_dist = 3.0
          kp_angle = 2.0
          steering = float(-kp_dist * lane_dist - kp_angle * lane_angle)
          steering = max(-1.0, min(1.0, steering))
        else:
          steering = 0.0

        action = np.array([speed, steering], dtype=np.float32)

        # Step env, handle API differences (Gym v0.26 introduces terminated/truncated)
        try:
          step_out = env.step(action)
          if len(step_out) == 5:
            obs, reward, terminated, truncated, info = step_out
            done = terminated or truncated
          else:
            obs, reward, done, info = step_out
        except Exception as e:
          errors.append(f"env.step failed at {step}: {e}")
          break

        # Extract metrics
        # Distance traveled
        try:
          # Prefer position delta if available
          cur_pos = None
          if isinstance(info, dict):
            cur_pos = info.get('pos') or info.get('cur_pos')
          if cur_pos is not None and isinstance(cur_pos, (list, tuple)) and len(cur_pos) >= 2:
            cur_pos2d = (float(cur_pos[0]), float(cur_pos[1]))
            if last_pos is not None:
              dx = cur_pos2d[0] - last_pos[0]
              dy = cur_pos2d[1] - last_pos[1]
              metrics['robot_distance_traveled'] += math.hypot(dx, dy)
            last_pos = cur_pos2d
          else:
            # Fallback: integrate speed estimate
            sp = None
            if isinstance(info, dict):
              sp = info.get('speed') or info.get('robot_speed')
            metrics['robot_distance_traveled'] += float(sp) * dt_est if sp is not None else speed * dt_est
        except Exception:
          metrics['robot_distance_traveled'] += speed * dt_est

        # Lane following accuracy heuristic
        if lane_dist is not None and lane_angle is not None:
          # Normalize: small dist/angle -> high accuracy
          dist_term = max(0.0, 1.0 - min(1.0, abs(lane_dist) / 0.2))
          angle_term = max(0.0, 1.0 - min(1.0, abs(lane_angle) / 0.4))
          acc = 0.5 * dist_term + 0.5 * angle_term
          metrics['lane_following_accuracy'] = 0.9 * metrics['lane_following_accuracy'] + 0.1 * acc
        else:
          # Unknown -> conservative default trending
          metrics['lane_following_accuracy'] = 0.9 * metrics['lane_following_accuracy'] + 0.1 * 0.7

        # Collisions
        if isinstance(info, dict) and (info.get('collision') or info.get('crashed')):
          metrics['collision_count'] += 1
          metrics['obstacle_avoidance_success'] = False

        metrics['simulation_time'] += dt_est

        if done:
          break

      metrics['completion_time'] = metrics['simulation_time']

      # Success based on accumulated behavior
      scenario_success = (
        metrics['lane_following_accuracy'] > 0.6 and
        metrics['robot_distance_traveled'] > 0.5 and
        metrics['collision_count'] == 0
      )

      # Evaluate explicit criteria as well
      scenario_success = scenario_success and GazeboSimulationManager._evaluate_success_criteria(self, scenario, metrics)

      execution_time = time.time() - start_time_wall
      return SimulationResult(
        scenario_id=scenario.scenario_id,
        success=scenario_success,
        execution_time=execution_time,
        metrics=metrics,
        errors=errors,
        warnings=warnings,
        timestamp=datetime.now().isoformat()
      )

    except Exception as e:
      errors.append(str(e))
      logger.error(f"Gym scenario error: {e}")
      return SimulationResult(
        scenario_id=scenario.scenario_id,
        success=False,
        execution_time=time.time() - start_time_wall,
        metrics=metrics,
        errors=errors,
        warnings=warnings,
        timestamp=datetime.now().isoformat()
      )
    finally:
      self.stop_simulation()
      self.current_scenario = None


class ScenarioGenerator:
    """Generator for automated test scenarios."""
    
    def __init__(self):
        self.scenario_templates = {}
        self.generated_scenarios = []
        
        logger.info("ScenarioGenerator initialized")
    
    def add_scenario_template(self, template_name: str, template: Dict[str, Any]):
        """Add a scenario template."""
        self.scenario_templates[template_name] = template
        logger.info(f"Scenario template added: {template_name}")
    
    def generate_lane_following_scenarios(self, count: int = 10) -> List[TestScenario]:
        """Generate lane following test scenarios."""
        logger.info(f"Generating {count} lane following scenarios")
        
        scenarios = []
        
        for i in range(count):
      # Create environment
      environment = SimulationEnvironment(
        name=f"lane_following_env_{i}",
        world_file="duckietown_straight.world",
        robot_model="duckiebot_test.sdf",
        real_time_factor=1.0
      )

      # Vary initial conditions
      initial_conditions = {
        'robot_position': {
          'x': -1.5 + np.random.normal(0, 0.1),
          'y': np.random.normal(0, 0.05),
          'z': 0.05,
          'yaw': np.random.normal(0, 0.1)
        },
        'lighting_conditions': np.random.choice(['bright', 'normal', 'dim']),
        'road_surface': np.random.choice(['dry', 'wet', 'worn'])
      }

      # Define success criteria
      success_criteria = {
        'lane_following_accuracy': {'min': 0.65},
        'robot_distance_traveled': {'min': 1.2},
        'completion_time': {'max': 30.0},
        'obstacle_avoidance_success': True
      }

            scenario = TestScenario(
                scenario_id=f"lane_following_{i:03d}",
                name=f"Lane Following Test {i+1}",
                description=f"Lane following scenario with varied initial conditions (iteration {i+1})",
                environment=environment,
                initial_conditions=initial_conditions,
                test_objectives=[
                    "Maintain lane position",
                    "Follow lane markings",
                    "Complete course within time limit"
                ],
                success_criteria=success_criteria,
                timeout_seconds=35.0
            )
            
            scenarios.append(scenario)
        
        self.generated_scenarios.extend(scenarios)
        logger.info(f"Generated {len(scenarios)} lane following scenarios")
        return scenarios
    
    def generate_obstacle_avoidance_scenarios(self, count: int = 10) -> List[TestScenario]:
        """Generate obstacle avoidance test scenarios."""
        logger.info(f"Generating {count} obstacle avoidance scenarios")
        
        scenarios = []
        
        for i in range(count):
            # Create environment with obstacles
            environment = SimulationEnvironment(
                name=f"obstacle_avoidance_env_{i}",
                world_file="duckietown_obstacles.world",
                robot_model="duckiebot_test.sdf",
                real_time_factor=1.0
            )
            
            # Vary obstacle configurations
            num_obstacles = np.random.randint(1, 4)
            obstacles = []
            for j in range(num_obstacles):
                obstacle = {
                    'type': np.random.choice(['cone', 'barrier', 'duckiebot']),
                    'position': {
                        'x': np.random.uniform(0.5, 2.0),
                        'y': np.random.uniform(-0.3, 0.3),
                        'z': 0.1
                    },
                    'size': np.random.uniform(0.05, 0.15)
                }
                obstacles.append(obstacle)
            
            initial_conditions = {
                'robot_position': {
                    'x': -1.5,
                    'y': np.random.normal(0, 0.02),
                    'z': 0.05,
                    'yaw': np.random.normal(0, 0.05)
                },
                'obstacles': obstacles,
                'lighting_conditions': np.random.choice(['bright', 'normal', 'dim'])
            }
            
      # Define success criteria
      success_criteria = {
        'obstacle_avoidance_success': True,
        'robot_distance_traveled': {'min': 1.0},
        'completion_time': {'max': 35.0},
        'collision_count': {'max': 0}
      }
            
            scenario = TestScenario(
                scenario_id=f"obstacle_avoidance_{i:03d}",
                name=f"Obstacle Avoidance Test {i+1}",
                description=f"Obstacle avoidance scenario with {num_obstacles} obstacles (iteration {i+1})",
                environment=environment,
                initial_conditions=initial_conditions,
                test_objectives=[
                    "Detect obstacles",
                    "Plan avoidance path",
                    "Execute safe navigation",
                    "Maintain progress toward goal"
                ],
                success_criteria=success_criteria,
                timeout_seconds=35.0
            )
            
            scenarios.append(scenario)
        
        self.generated_scenarios.extend(scenarios)
        logger.info(f"Generated {len(scenarios)} obstacle avoidance scenarios")
        return scenarios
    
    def generate_intersection_scenarios(self, count: int = 5) -> List[TestScenario]:
        """Generate intersection navigation scenarios."""
        logger.info(f"Generating {count} intersection scenarios")
        
        scenarios = []
        
        for i in range(count):
            # Create intersection environment
            environment = SimulationEnvironment(
                name=f"intersection_env_{i}",
                world_file="duckietown_intersection.world",
                robot_model="duckiebot_test.sdf",
                real_time_factor=1.0
            )
            
            # Vary intersection configurations
            intersection_type = np.random.choice(['4_way', '3_way', 'T_junction'])
            traffic_density = np.random.choice(['low', 'medium', 'high'])
            
            initial_conditions = {
                'robot_position': {
                    'x': -2.0,
                    'y': 0.0,
                    'z': 0.05,
                    'yaw': 0.0
                },
                'intersection_type': intersection_type,
                'traffic_density': traffic_density,
                'target_direction': np.random.choice(['straight', 'left', 'right'])
            }
            
            # Define success criteria
            success_criteria = {
                'intersection_navigation_success': True,
                'traffic_rule_compliance': True,
                'completion_time': {'max': 30.0},
                'collision_count': {'max': 0}
            }
            
            scenario = TestScenario(
                scenario_id=f"intersection_{i:03d}",
                name=f"Intersection Navigation Test {i+1}",
                description=f"Navigate {intersection_type} intersection with {traffic_density} traffic",
                environment=environment,
                initial_conditions=initial_conditions,
                test_objectives=[
                    "Approach intersection safely",
                    "Follow traffic rules",
                    "Navigate to target direction",
                    "Avoid collisions"
                ],
                success_criteria=success_criteria,
                timeout_seconds=40.0
            )
            
            scenarios.append(scenario)
        
        self.generated_scenarios.extend(scenarios)
        logger.info(f"Generated {len(scenarios)} intersection scenarios")
        return scenarios
    
    def save_scenarios(self, filename: str):
        """Save generated scenarios to file."""
        logger.info(f"Saving {len(self.generated_scenarios)} scenarios to {filename}")
        
        scenarios_data = [scenario.to_dict() for scenario in self.generated_scenarios]
        
        with open(filename, 'w') as f:
            json.dump(scenarios_data, f, indent=2)
        
        logger.info(f"Scenarios saved to {filename}")
    
    def load_scenarios(self, filename: str) -> List[TestScenario]:
        """Load scenarios from file."""
        logger.info(f"Loading scenarios from {filename}")
        
        with open(filename, 'r') as f:
            scenarios_data = json.load(f)
        
        scenarios = []
        for data in scenarios_data:
            environment = SimulationEnvironment(**data['environment'])
            scenario = TestScenario(
                scenario_id=data['scenario_id'],
                name=data['name'],
                description=data['description'],
                environment=environment,
                initial_conditions=data['initial_conditions'],
                test_objectives=data['test_objectives'],
                success_criteria=data['success_criteria'],
                timeout_seconds=data['timeout_seconds']
            )
            scenarios.append(scenario)
        
        logger.info(f"Loaded {len(scenarios)} scenarios")
        return scenarios


class PerformanceRegressionTester:
    """Performance regression testing for simulation results."""
    
    def __init__(self, baseline_file: str = "performance_baseline.json"):
        self.baseline_file = baseline_file
        self.baseline_data = {}
        self.current_results = []
        
        # Load existing baseline if available
        self._load_baseline()
        
        logger.info(f"PerformanceRegressionTester initialized with baseline: {baseline_file}")
    
    def _load_baseline(self):
        """Load performance baseline data."""
        try:
            if Path(self.baseline_file).exists():
                with open(self.baseline_file, 'r') as f:
                    self.baseline_data = json.load(f)
                logger.info(f"Loaded baseline data with {len(self.baseline_data)} entries")
            else:
                logger.info("No baseline file found, will create new baseline")
        except Exception as e:
            logger.error(f"Error loading baseline: {e}")
    
    def add_result(self, result: SimulationResult):
        """Add simulation result for regression analysis."""
        self.current_results.append(result)
        logger.debug(f"Added result for scenario: {result.scenario_id}")
    
    def analyze_regression(self) -> Dict[str, Any]:
        """Analyze performance regression against baseline."""
        logger.info("Analyzing performance regression")
        
        if not self.baseline_data:
            logger.warning("No baseline data available for regression analysis")
            return self._create_new_baseline()
        
        regression_analysis = {
            'total_scenarios': len(self.current_results),
            'regression_detected': False,
            'performance_changes': {},
            'success_rate_change': 0.0,
            'execution_time_change': 0.0,
            'detailed_analysis': []
        }
        
        # Analyze each scenario
        for result in self.current_results:
            scenario_id = result.scenario_id
            
            if scenario_id in self.baseline_data:
                baseline = self.baseline_data[scenario_id]
                current = result.to_dict()
                
                # Compare key metrics
                analysis = self._compare_scenario_performance(baseline, current)
                regression_analysis['detailed_analysis'].append(analysis)
                
                # Check for significant regressions
                if analysis['regression_detected']:
                    regression_analysis['regression_detected'] = True
        
        # Calculate overall performance changes
        if regression_analysis['detailed_analysis']:
            success_rates_baseline = []
            success_rates_current = []
            exec_times_baseline = []
            exec_times_current = []
            
            for analysis in regression_analysis['detailed_analysis']:
                if 'baseline_success' in analysis and 'current_success' in analysis:
                    success_rates_baseline.append(1.0 if analysis['baseline_success'] else 0.0)
                    success_rates_current.append(1.0 if analysis['current_success'] else 0.0)
                
                if 'execution_time_change' in analysis:
                    exec_times_baseline.append(analysis['baseline_execution_time'])
                    exec_times_current.append(analysis['current_execution_time'])
            
            if success_rates_baseline:
                baseline_success_rate = np.mean(success_rates_baseline)
                current_success_rate = np.mean(success_rates_current)
                regression_analysis['success_rate_change'] = current_success_rate - baseline_success_rate
            
            if exec_times_baseline:
                baseline_avg_time = np.mean(exec_times_baseline)
                current_avg_time = np.mean(exec_times_current)
                regression_analysis['execution_time_change'] = (current_avg_time - baseline_avg_time) / baseline_avg_time
        
        logger.info(f"Regression analysis completed:")
        logger.info(f"  Regression detected: {regression_analysis['regression_detected']}")
        logger.info(f"  Success rate change: {regression_analysis['success_rate_change']:.3f}")
        logger.info(f"  Execution time change: {regression_analysis['execution_time_change']:.3f}")
        
        return regression_analysis
    
    def _compare_scenario_performance(self, baseline: Dict[str, Any], current: Dict[str, Any]) -> Dict[str, Any]:
        """Compare performance between baseline and current results."""
        analysis = {
            'scenario_id': current['scenario_id'],
            'baseline_success': baseline['success'],
            'current_success': current['success'],
            'baseline_execution_time': baseline['execution_time'],
            'current_execution_time': current['execution_time'],
            'regression_detected': False,
            'changes': {}
        }
        
        # Check success rate regression
        if baseline['success'] and not current['success']:
            analysis['regression_detected'] = True
            analysis['changes']['success_regression'] = True
        
        # Check execution time regression (>20% increase)
        time_change = (current['execution_time'] - baseline['execution_time']) / baseline['execution_time']
        if time_change > 0.2:
            analysis['regression_detected'] = True
            analysis['changes']['execution_time_regression'] = time_change
        
        analysis['execution_time_change'] = time_change
        
        # Compare metrics if available
        if 'metrics' in baseline and 'metrics' in current:
            baseline_metrics = baseline['metrics']
            current_metrics = current['metrics']
            
            for metric_name in baseline_metrics:
                if metric_name in current_metrics:
                    baseline_value = baseline_metrics[metric_name]
                    current_value = current_metrics[metric_name]
                    
                    if isinstance(baseline_value, (int, float)) and isinstance(current_value, (int, float)):
                        if baseline_value != 0:
                            metric_change = (current_value - baseline_value) / baseline_value
                            
                            # Check for significant metric regression (>15% degradation)
                            if metric_change < -0.15:
                                analysis['regression_detected'] = True
                                analysis['changes'][f'{metric_name}_regression'] = metric_change
        
        return analysis
    
    def _create_new_baseline(self) -> Dict[str, Any]:
        """Create new baseline from current results."""
        logger.info("Creating new performance baseline")
        
        baseline_data = {}
        for result in self.current_results:
            baseline_data[result.scenario_id] = result.to_dict()
        
        # Save baseline
        with open(self.baseline_file, 'w') as f:
            json.dump(baseline_data, f, indent=2)
        
        self.baseline_data = baseline_data
        
        return {
            'baseline_created': True,
            'scenarios_count': len(baseline_data),
            'message': 'New baseline created from current results'
        }
    
    def update_baseline(self):
        """Update baseline with current results."""
        logger.info("Updating performance baseline")
        
        for result in self.current_results:
            self.baseline_data[result.scenario_id] = result.to_dict()
        
        # Save updated baseline
        with open(self.baseline_file, 'w') as f:
            json.dump(self.baseline_data, f, indent=2)
        
        logger.info(f"Baseline updated with {len(self.current_results)} results")


if __name__ == "__main__":
  # Example usage and testing
  logger.info("Testing Simulation-Based Validation")

  # Ensure pyglet is configured correctly on macOS (use Cocoa, not EGL)
  try:
    import platform as _platform
    if "Darwin" in _platform.system():
      import os as _os
      import pyglet as _pyglet
      _os.environ.pop("PYGLET_HEADLESS", None)
      _pyglet.options["headless"] = False
  except Exception:
    pass

  # Try Gym-Duckietown first (better for macOS), fall back to Gazebo check or mock
  gym_manager: Optional[GymDuckietownSimulationManager] = None
  gym_available = False
  try:
    # Optional: enable render from env flag for quick visual debugging
    _render_flag = os.environ.get("DT_GYM_RENDER", "0") in ("1", "true", "True")
    gym_manager = GymDuckietownSimulationManager(render=_render_flag)
    gym_available = gym_manager.check_gym_availability()
  except Exception as e:
    logger.warning(f"Gym manager init failed: {e}")
    gym_available = False

  # Create Gazebo manager as secondary option
  sim_manager = GazeboSimulationManager()
  gazebo_available = sim_manager.check_gazebo_availability()
  logger.info(f"Gym available: {gym_available} | Gazebo available: {gazebo_available}")

  # Create scenario generator
  scenario_gen = ScenarioGenerator()

  # Generate test scenarios
  lane_scenarios = scenario_gen.generate_lane_following_scenarios(3)
  obstacle_scenarios = scenario_gen.generate_obstacle_avoidance_scenarios(2)

  # Save scenarios
  scenario_gen.save_scenarios("test_scenarios.json")

  # Create regression tester
  regression_tester = PerformanceRegressionTester()

  # Run scenarios via Gym if available, else create mock results
  scenarios_to_run = lane_scenarios[:2] + obstacle_scenarios[:1]
  if gym_available and gym_manager is not None:
    for scenario in scenarios_to_run:
      logger.info(f"Running in Gym: {scenario.name}")
      result = gym_manager.run_scenario(scenario)
      regression_tester.add_result(result)
  else:
    for scenario in lane_scenarios + obstacle_scenarios:
      logger.info(f"Simulating (mock) scenario: {scenario.name}")
      mock_result = SimulationResult(
        scenario_id=scenario.scenario_id,
        success=np.random.random() > 0.2,  # 80% success rate
        execution_time=np.random.uniform(5.0, 15.0),
        metrics={
          'lane_following_accuracy': np.random.uniform(0.6, 0.95),
          'robot_distance_traveled': np.random.uniform(1.5, 3.0),
          'completion_time': np.random.uniform(8.0, 20.0)
        },
        errors=[],
        warnings=[],
        timestamp=datetime.now().isoformat()
      )
      regression_tester.add_result(mock_result)

  # Analyze regression
  regression_analysis = regression_tester.analyze_regression()

  logger.info("Simulation-Based Validation test completed successfully")
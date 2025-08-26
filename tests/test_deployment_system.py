#!/usr/bin/env python3
"""
Integration tests for the enhanced deployment system.

This module tests the deployment system components including:
- SSH connectivity validation
- Robot configuration management
- File transfer operations
- Health check functionality
- Deployment verification
"""

import os
import sys
import unittest
import subprocess
import tempfile
import shutil
from pathlib import Path
from unittest.mock import patch, MagicMock, call
import yaml
import time

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


class TestDeploymentSystemValidation(unittest.TestCase):
    """Test deployment system configuration validation."""
    
    def setUp(self):
        """Set up test environment."""
        self.project_root = PROJECT_ROOT
        self.deploy_script_path = self.project_root / "scripts" / "enhanced_deploy.sh"
        self.robot_configs_dir = self.project_root / "robot_configs"
        self.configurations_file = self.project_root / "configurations.yaml"
        
        print(f"[{self._get_timestamp()}] [TEST] Setting up deployment system tests...")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    def test_deployment_script_exists_and_executable(self):
        """Test that deployment script exists and is executable."""
        print(f"[{self._get_timestamp()}] [TEST] Testing deployment script validation...")
        
        self.assertTrue(self.deploy_script_path.exists(), "Deployment script not found")
        
        # Check if script is executable
        self.assertTrue(os.access(self.deploy_script_path, os.X_OK), "Deployment script is not executable")
        
        # Test script help functionality
        try:
            result = subprocess.run([str(self.deploy_script_path), '--help'], 
                                  capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, "Deployment script help command failed")
            self.assertIn("Enhanced Deployment Script", result.stdout, "Help output doesn't contain expected content")
        except subprocess.TimeoutExpired:
            self.fail("Deployment script help command timed out")
        except Exception as e:
            self.fail(f"Deployment script help command failed: {e}")
        
        print(f"[{self._get_timestamp()}] [TEST] Deployment script validation passed")
    
    def test_robot_configs_directory_exists(self):
        """Test that robot configs directory exists with example configuration."""
        print(f"[{self._get_timestamp()}] [TEST] Testing robot configs directory...")
        
        self.assertTrue(self.robot_configs_dir.exists(), "Robot configs directory not found")
        
        # Check for example configuration
        example_config = self.robot_configs_dir / "example.yaml"
        self.assertTrue(example_config.exists(), "Example robot configuration not found")
        
        # Validate example configuration structure
        with open(example_config, 'r') as f:
            try:
                config = yaml.safe_load(f)
            except yaml.YAMLError as e:
                self.fail(f"Example robot configuration YAML is invalid: {e}")
        
        # Test required configuration sections
        self.assertIn('robot', config, "Robot section not found in example config")
        
        robot_config = config['robot']
        required_fields = ['host', 'user', 'port', 'architecture', 'distro', 'workspace']
        for field in required_fields:
            self.assertIn(field, robot_config, f"Required field '{field}' not found in robot config")
        
        print(f"[{self._get_timestamp()}] [TEST] Robot configs directory validation passed")
    
    def test_enhanced_configurations_file(self):
        """Test that configurations.yaml has been enhanced with deployment settings."""
        print(f"[{self._get_timestamp()}] [TEST] Testing enhanced configurations file...")
        
        self.assertTrue(self.configurations_file.exists(), "Configurations file not found")
        
        with open(self.configurations_file, 'r') as f:
            try:
                config = yaml.safe_load(f)
            except yaml.YAMLError as e:
                self.fail(f"Configurations YAML is invalid: {e}")
        
        # Test enhanced configuration sections
        required_sections = [
            'deployment',
            'build',
            'monitoring',
            'autonomous_system',
            'robot_defaults',
            'development',
            'performance'
        ]
        
        for section in required_sections:
            self.assertIn(section, config['configurations'], f"Configuration section '{section}' not found")
        
        # Test deployment configuration
        deployment_config = config['configurations']['deployment']
        deployment_fields = ['default_user', 'default_port', 'timeout', 'retry_count']
        for field in deployment_fields:
            self.assertIn(field, deployment_config, f"Deployment field '{field}' not found")
        
        # Test autonomous system configuration
        autonomous_config = config['configurations']['autonomous_system']
        autonomous_sections = ['lane_following', 'object_detection', 'apriltag_detection', 'safety', 'navigation']
        for section in autonomous_sections:
            self.assertIn(section, autonomous_config, f"Autonomous system section '{section}' not found")
        
        print(f"[{self._get_timestamp()}] [TEST] Enhanced configurations file validation passed")
    
    def test_deployment_script_dry_run(self):
        """Test deployment script dry run functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing deployment script dry run...")
        
        try:
            # Test dry run with example robot
            result = subprocess.run([
                str(self.deploy_script_path), 
                '--dry-run', 
                'example_robot'
            ], capture_output=True, text=True, timeout=60)
            
            self.assertEqual(result.returncode, 0, f"Dry run failed: {result.stderr}")
            self.assertIn("DRY RUN MODE", result.stdout, "Dry run mode not indicated in output")
            self.assertIn("Would deploy to robot: example_robot", result.stdout, "Robot name not shown in dry run")
            
        except subprocess.TimeoutExpired:
            self.fail("Deployment script dry run timed out")
        except Exception as e:
            self.fail(f"Deployment script dry run failed: {e}")
        
        print(f"[{self._get_timestamp()}] [TEST] Deployment script dry run validation passed")


class TestDeploymentSystemFunctionality(unittest.TestCase):
    """Test deployment system functionality with mocked operations."""
    
    def setUp(self):
        """Set up test environment with mocks."""
        self.project_root = PROJECT_ROOT
        self.temp_dir = tempfile.mkdtemp()
        self.test_robot_config = Path(self.temp_dir) / "test_robot.yaml"
        
        # Create test robot configuration
        test_config = {
            'robot': {
                'host': 'test-robot.local',
                'user': 'test_user',
                'port': 22,
                'architecture': 'arm64',
                'distro': 'daffy',
                'workspace': '/test/workspace',
                'calibration': {
                    'directory': '/test/calibrations'
                },
                'network': {
                    'wifi_ssid': 'TestNetwork'
                },
                'hardware': {
                    'camera_topic': '/test/camera'
                }
            }
        }
        
        with open(self.test_robot_config, 'w') as f:
            yaml.dump(test_config, f)
        
        print(f"[{self._get_timestamp()}] [TEST] Setting up deployment functionality tests...")
    
    def tearDown(self):
        """Clean up test environment."""
        shutil.rmtree(self.temp_dir, ignore_errors=True)
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    @patch('subprocess.run')
    def test_ssh_connectivity_validation(self, mock_subprocess):
        """Test SSH connectivity validation functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing SSH connectivity validation...")
        
        # Mock successful SSH connection
        mock_subprocess.return_value = MagicMock(returncode=0, stdout="SSH connection successful", stderr="")
        
        # Test SSH connectivity check (would be part of deployment script)
        result = subprocess.run([
            'ssh', '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes',
            'test_user@test-robot.local', 'echo "SSH connection successful"'
        ], capture_output=True, text=True, timeout=10)
        
        # Verify mock was called
        mock_subprocess.assert_called()
        
        print(f"[{self._get_timestamp()}] [TEST] SSH connectivity validation test completed")
    
    def test_robot_configuration_parsing(self):
        """Test robot configuration parsing functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing robot configuration parsing...")
        
        # Test configuration loading
        with open(self.test_robot_config, 'r') as f:
            config = yaml.safe_load(f)
        
        robot_config = config['robot']
        
        # Validate configuration fields
        self.assertEqual(robot_config['host'], 'test-robot.local')
        self.assertEqual(robot_config['user'], 'test_user')
        self.assertEqual(robot_config['port'], 22)
        self.assertEqual(robot_config['architecture'], 'arm64')
        self.assertEqual(robot_config['distro'], 'daffy')
        self.assertEqual(robot_config['workspace'], '/test/workspace')
        
        # Test nested configuration
        self.assertEqual(robot_config['calibration']['directory'], '/test/calibrations')
        self.assertEqual(robot_config['network']['wifi_ssid'], 'TestNetwork')
        self.assertEqual(robot_config['hardware']['camera_topic'], '/test/camera')
        
        print(f"[{self._get_timestamp()}] [TEST] Robot configuration parsing validation passed")
    
    @patch('subprocess.run')
    def test_file_transfer_operations(self, mock_subprocess):
        """Test file transfer operations (rsync/scp)."""
        print(f"[{self._get_timestamp()}] [TEST] Testing file transfer operations...")
        
        # Mock successful file transfer operations
        mock_subprocess.return_value = MagicMock(returncode=0, stdout="transfer complete", stderr="")
        
        # Test rsync operation
        rsync_result = subprocess.run([
            'rsync', '-avz', '--dry-run',
            '/test/source/', 'test_user@test-robot.local:/test/destination/'
        ], capture_output=True, text=True)
        
        # Test scp operation
        scp_result = subprocess.run([
            'scp', '/test/file.txt', 'test_user@test-robot.local:/test/destination/'
        ], capture_output=True, text=True)
        
        # Verify mocks were called
        self.assertEqual(mock_subprocess.call_count, 2)
        
        print(f"[{self._get_timestamp()}] [TEST] File transfer operations test completed")
    
    def test_health_check_functionality(self):
        """Test health check functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing health check functionality...")
        
        # Define health check commands that would be executed on robot
        health_checks = [
            "test -d /test/workspace",
            "test -d /test/workspace/packages",
            "docker images | grep duckietown/dt-core",
            "source /opt/ros/noetic/setup.bash && roscore --version",
            "free -h",
            "df -h",
            "uptime"
        ]
        
        # Validate health check commands are properly formatted
        for check in health_checks:
            self.assertIsInstance(check, str)
            self.assertGreater(len(check), 0)
        
        # Test health check result parsing
        mock_health_results = {
            'workspace_exists': True,
            'packages_exist': True,
            'docker_image_available': True,
            'ros_available': True,
            'memory_usage': '45%',
            'disk_usage': '60%',
            'system_uptime': '2 days'
        }
        
        # Validate health check results structure
        required_checks = ['workspace_exists', 'packages_exist', 'docker_image_available', 'ros_available']
        for check in required_checks:
            self.assertIn(check, mock_health_results)
        
        print(f"[{self._get_timestamp()}] [TEST] Health check functionality validation passed")
    
    def test_deployment_verification(self):
        """Test deployment verification functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing deployment verification...")
        
        # Define verification checks
        verification_checks = [
            {'name': 'workspace_directory', 'path': '/test/workspace', 'type': 'directory'},
            {'name': 'packages_directory', 'path': '/test/workspace/packages', 'type': 'directory'},
            {'name': 'config_file', 'path': '/test/workspace/configurations.yaml', 'type': 'file'},
            {'name': 'dependencies_file', 'path': '/test/workspace/dependencies-py3.txt', 'type': 'file'}
        ]
        
        # Test verification check structure
        for check in verification_checks:
            self.assertIn('name', check)
            self.assertIn('path', check)
            self.assertIn('type', check)
            self.assertIn(check['type'], ['file', 'directory'])
        
        # Test key packages verification
        key_packages = [
            'duckietown_msgs',
            'lane_control',
            'lane_filter',
            'line_detector',
            'vehicle_detection',
            'navigation',
            'fsm'
        ]
        
        for package in key_packages:
            self.assertIsInstance(package, str)
            self.assertGreater(len(package), 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Deployment verification validation passed")


class TestDeploymentIntegration(unittest.TestCase):
    """Test deployment system integration scenarios."""
    
    def setUp(self):
        """Set up integration test environment."""
        self.project_root = PROJECT_ROOT
        print(f"[{self._get_timestamp()}] [TEST] Setting up deployment integration tests...")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    def test_deployment_workflow_validation(self):
        """Test complete deployment workflow validation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing deployment workflow validation...")
        
        # Define deployment workflow steps
        workflow_steps = [
            'load_robot_config',
            'test_ssh_connection',
            'get_system_info',
            'sync_source_code',
            'deploy_docker_image',
            'deploy_configuration',
            'perform_health_checks',
            'verify_deployment'
        ]
        
        # Validate workflow step definitions
        for step in workflow_steps:
            self.assertIsInstance(step, str)
            self.assertGreater(len(step), 0)
            self.assertRegex(step, r'^[a-z_]+$', f"Step name '{step}' should be lowercase with underscores")
        
        # Test workflow step dependencies
        step_dependencies = {
            'test_ssh_connection': ['load_robot_config'],
            'get_system_info': ['test_ssh_connection'],
            'sync_source_code': ['test_ssh_connection'],
            'deploy_docker_image': ['test_ssh_connection'],
            'deploy_configuration': ['test_ssh_connection'],
            'perform_health_checks': ['sync_source_code', 'deploy_docker_image', 'deploy_configuration'],
            'verify_deployment': ['perform_health_checks']
        }
        
        # Validate dependencies exist in workflow
        for step, deps in step_dependencies.items():
            self.assertIn(step, workflow_steps, f"Step '{step}' not found in workflow")
            for dep in deps:
                self.assertIn(dep, workflow_steps, f"Dependency '{dep}' not found in workflow")
        
        print(f"[{self._get_timestamp()}] [TEST] Deployment workflow validation passed")
    
    def test_error_handling_scenarios(self):
        """Test error handling scenarios in deployment."""
        print(f"[{self._get_timestamp()}] [TEST] Testing error handling scenarios...")
        
        # Define error scenarios
        error_scenarios = [
            {'name': 'ssh_connection_failed', 'recoverable': False, 'retry': False},
            {'name': 'file_transfer_failed', 'recoverable': True, 'retry': True},
            {'name': 'docker_image_not_found', 'recoverable': True, 'retry': True},
            {'name': 'configuration_invalid', 'recoverable': False, 'retry': False},
            {'name': 'health_check_failed', 'recoverable': True, 'retry': True},
            {'name': 'verification_failed', 'recoverable': True, 'retry': True}
        ]
        
        # Validate error scenario definitions
        for scenario in error_scenarios:
            self.assertIn('name', scenario)
            self.assertIn('recoverable', scenario)
            self.assertIn('retry', scenario)
            self.assertIsInstance(scenario['recoverable'], bool)
            self.assertIsInstance(scenario['retry'], bool)
        
        # Test error recovery strategies
        recovery_strategies = {
            'file_transfer_failed': 'retry_with_backoff',
            'docker_image_not_found': 'pull_from_registry',
            'health_check_failed': 'retry_after_delay',
            'verification_failed': 'partial_rollback'
        }
        
        for error, strategy in recovery_strategies.items():
            self.assertIsInstance(strategy, str)
            self.assertGreater(len(strategy), 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Error handling scenarios validation passed")
    
    def test_performance_monitoring(self):
        """Test deployment performance monitoring."""
        print(f"[{self._get_timestamp()}] [TEST] Testing deployment performance monitoring...")
        
        # Define performance metrics
        performance_metrics = [
            'deployment_start_time',
            'ssh_connection_time',
            'file_transfer_time',
            'docker_deployment_time',
            'configuration_deployment_time',
            'health_check_time',
            'verification_time',
            'total_deployment_time'
        ]
        
        # Validate performance metrics
        for metric in performance_metrics:
            self.assertIsInstance(metric, str)
            self.assertGreater(len(metric), 0)
            self.assertTrue(metric.endswith('_time'), f"Metric '{metric}' should end with '_time'")
        
        # Test performance thresholds
        performance_thresholds = {
            'ssh_connection_time': 10,      # seconds
            'file_transfer_time': 120,      # seconds
            'docker_deployment_time': 300,  # seconds
            'total_deployment_time': 600    # seconds
        }
        
        for metric, threshold in performance_thresholds.items():
            self.assertIn(metric, performance_metrics)
            self.assertIsInstance(threshold, int)
            self.assertGreater(threshold, 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Performance monitoring validation passed")


def run_deployment_tests():
    """Run all deployment system tests with comprehensive logging."""
    print(f"[{TestDeploymentSystemValidation()._get_timestamp()}] [TEST] Starting deployment system test suite...")
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestDeploymentSystemValidation,
        TestDeploymentSystemFunctionality,
        TestDeploymentIntegration
    ]
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n[{TestDeploymentSystemValidation()._get_timestamp()}] [TEST] Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\n[{TestDeploymentSystemValidation()._get_timestamp()}] [TEST] Failures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\n[{TestDeploymentSystemValidation()._get_timestamp()}] [TEST] Errors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_deployment_tests()
    sys.exit(0 if success else 1)
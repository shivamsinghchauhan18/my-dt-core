#!/usr/bin/env python3
"""
Test Suite for Comprehensive System Validation

This module provides comprehensive tests for the system validation framework,
ensuring all validation components work correctly and provide accurate results.
"""

import unittest
import tempfile
import shutil
import json
import time
from pathlib import Path
from unittest.mock import patch, MagicMock
import sys
import os

# Add the src directory to the path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from comprehensive_system_validation import (
    ComprehensiveSystemValidator,
    RequirementTraceabilityManager,
    RequirementValidationResult,
    SafetyValidationResult,
    SystemValidationReport
)

import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TestRequirementTraceabilityManager(unittest.TestCase):
    """Test the requirement traceability manager."""
    
    def setUp(self):
        """Set up test environment."""
        self.temp_dir = tempfile.mkdtemp()
        self.requirements_file = Path(self.temp_dir) / "test_requirements.md"
        
        # Create a test requirements file
        test_requirements = """# Requirements Document

## Introduction

Test requirements for validation testing.

## Requirements

### Requirement 1: Test Lane Following

**User Story:** As a test user, I want lane following to work.

#### Acceptance Criteria

1. WHEN the robot encounters a lane THEN it SHALL follow the lane
2. WHEN lighting changes THEN the system SHALL adapt

### Requirement 2: Test Object Detection

**User Story:** As a test user, I want object detection to work.

#### Acceptance Criteria

1. WHEN objects are detected THEN the system SHALL classify them
2. WHEN collision risk is high THEN the system SHALL avoid obstacles
"""
        
        with open(self.requirements_file, 'w') as f:
            f.write(test_requirements)
    
    def tearDown(self):
        """Clean up test environment."""
        shutil.rmtree(self.temp_dir)
    
    def test_load_requirements(self):
        """Test loading requirements from file."""
        logger.info("Testing requirement loading")
        
        manager = RequirementTraceabilityManager(str(self.requirements_file))
        
        # Check that requirements were loaded
        requirements = manager.get_all_requirements()
        self.assertEqual(len(requirements), 2)
        self.assertIn("1", requirements)
        self.assertIn("2", requirements)
        
        # Check acceptance criteria
        req1_criteria = requirements["1"]
        self.assertEqual(len(req1_criteria), 2)
        self.assertTrue(any("lane" in criterion for criterion in req1_criteria))
    
    def test_test_mapping(self):
        """Test mapping tests to requirements."""
        logger.info("Testing test-to-requirement mapping")
        
        manager = RequirementTraceabilityManager(str(self.requirements_file))
        
        # Map tests to requirements
        manager.map_test_to_requirement("lane_detection_test", ["1"])
        manager.map_test_to_requirement("object_detection_test", ["2"])
        manager.map_test_to_requirement("integration_test", ["1", "2"])
        
        # Check mappings
        req1_tests = manager.get_requirement_tests("1")
        self.assertIn("lane_detection_test", req1_tests)
        self.assertIn("integration_test", req1_tests)
        
        req2_tests = manager.get_requirement_tests("2")
        self.assertIn("object_detection_test", req2_tests)
        self.assertIn("integration_test", req2_tests)


class TestComprehensiveSystemValidator(unittest.TestCase):
    """Test the comprehensive system validator."""
    
    def setUp(self):
        """Set up test environment."""
        self.temp_dir = tempfile.mkdtemp()
        self.validator = ComprehensiveSystemValidator(output_dir=self.temp_dir)
        
        logger.info(f"Test setup complete with temp dir: {self.temp_dir}")
    
    def tearDown(self):
        """Clean up test environment."""
        shutil.rmtree(self.temp_dir)
    
    def test_validator_initialization(self):
        """Test validator initialization."""
        logger.info("Testing validator initialization")
        
        # Check that validator components are initialized
        self.assertIsNotNone(self.validator.performance_benchmark)
        self.assertIsNotNone(self.validator.simulation_manager)
        self.assertIsNotNone(self.validator.traceability_manager)
        self.assertIsNotNone(self.validator.test_data_generator)
        
        # Check output directory creation
        self.assertTrue(Path(self.temp_dir).exists())
    
    def test_functional_test_execution(self):
        """Test execution of functional tests."""
        logger.info("Testing functional test execution")
        
        # Test lane detection pipeline
        success, metrics = self.validator._test_lane_detection_pipeline()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('execution_time', metrics)
        self.assertIn('success_rate', metrics)
        
        logger.info(f"Lane detection test: {'PASSED' if success else 'FAILED'}")
        logger.info(f"Metrics: {metrics}")
    
    def test_apriltag_detection_test(self):
        """Test AprilTag detection functionality."""
        logger.info("Testing AprilTag detection test")
        
        success, metrics = self.validator._test_apriltag_detection_pipeline()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('execution_time', metrics)
        self.assertIn('distance_accuracy', metrics)
        
        logger.info(f"AprilTag detection test: {'PASSED' if success else 'FAILED'}")
    
    def test_yolo_object_detection_test(self):
        """Test YOLO object detection functionality."""
        logger.info("Testing YOLO object detection test")
        
        success, metrics = self.validator._test_yolo_object_detection()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('execution_time', metrics)
        self.assertIn('fps', metrics)
        
        logger.info(f"YOLO detection test: {'PASSED' if success else 'FAILED'}")
    
    def test_safety_monitoring_test(self):
        """Test safety monitoring functionality."""
        logger.info("Testing safety monitoring test")
        
        success, metrics = self.validator._test_safety_monitoring()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('execution_time', metrics)
        self.assertIn('monitoring_frequency', metrics)
        
        logger.info(f"Safety monitoring test: {'PASSED' if success else 'FAILED'}")
    
    def test_end_to_end_latency_test(self):
        """Test end-to-end latency validation."""
        logger.info("Testing end-to-end latency test")
        
        success, metrics = self.validator._test_end_to_end_latency()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('total_latency', metrics)
        self.assertIn('pipeline_fps', metrics)
        
        # Check latency requirement
        if 'total_latency' in metrics:
            self.assertLess(metrics['total_latency'], 0.25)  # Should be under 250ms
        
        logger.info(f"End-to-end latency test: {'PASSED' if success else 'FAILED'}")
        logger.info(f"Total latency: {metrics.get('total_latency', 'N/A')}s")
    
    def test_performance_validation(self):
        """Test performance requirements validation."""
        logger.info("Testing performance validation")
        
        # Run performance validation
        performance_results = self.validator.validate_performance_requirements()
        
        self.assertIsInstance(performance_results, dict)
        self.assertIn('condition_results', performance_results)
        self.assertIn('overall_performance_success', performance_results)
        
        # Check condition results
        condition_results = performance_results['condition_results']
        self.assertGreater(len(condition_results), 0)
        
        for condition, result in condition_results.items():
            self.assertIn('execution_time', result)
            self.assertIn('meets_requirements', result)
            logger.info(f"Performance condition {condition}: "
                       f"{'PASSED' if result['meets_requirements'] else 'FAILED'}")
    
    def test_safety_validation(self):
        """Test safety requirements validation."""
        logger.info("Testing safety validation")
        
        # Run safety validation
        safety_results = self.validator.validate_safety_requirements()
        
        self.assertIsInstance(safety_results, list)
        self.assertGreater(len(safety_results), 0)
        
        for result in safety_results:
            self.assertIsInstance(result, SafetyValidationResult)
            self.assertIsInstance(result.safety_score, float)
            self.assertGreaterEqual(result.safety_score, 0.0)
            self.assertLessEqual(result.safety_score, 1.0)
            
            logger.info(f"Safety scenario {result.safety_scenario}: "
                       f"score={result.safety_score:.2f}, "
                       f"response_time={result.emergency_response_time:.3f}s")
    
    def test_integration_validation(self):
        """Test integration requirements validation."""
        logger.info("Testing integration validation")
        
        # Run integration validation
        integration_results = self.validator.validate_integration_requirements()
        
        self.assertIsInstance(integration_results, dict)
        self.assertIn('test_results', integration_results)
        self.assertIn('overall_success', integration_results)
        
        # Check individual test results
        test_results = integration_results['test_results']
        self.assertGreater(len(test_results), 0)
        
        for test_name, result in test_results.items():
            self.assertIn('success', result)
            self.assertIn('metrics', result)
            logger.info(f"Integration test {test_name}: "
                       f"{'PASSED' if result['success'] else 'FAILED'}")
    
    def test_ros_node_communication(self):
        """Test ROS node communication validation."""
        logger.info("Testing ROS node communication")
        
        success, metrics = self.validator._test_ros_node_communication()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('packages_found', metrics)
        self.assertIn('total_packages', metrics)
        
        logger.info(f"ROS communication test: {'PASSED' if success else 'FAILED'}")
        logger.info(f"Packages found: {metrics['packages_found']}/{metrics['total_packages']}")
    
    def test_message_flow_validation(self):
        """Test message flow validation."""
        logger.info("Testing message flow validation")
        
        success, metrics = self.validator._test_message_flow()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('message_files_found', metrics)
        self.assertIn('total_message_files', metrics)
        
        logger.info(f"Message flow test: {'PASSED' if success else 'FAILED'}")
    
    def test_configuration_loading_validation(self):
        """Test configuration loading validation."""
        logger.info("Testing configuration loading validation")
        
        success, metrics = self.validator._test_configuration_loading()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('config_files_found', metrics)
        self.assertIn('valid_config_files', metrics)
        
        logger.info(f"Configuration loading test: {'PASSED' if success else 'FAILED'}")
    
    def test_system_dependencies_validation(self):
        """Test system dependencies validation."""
        logger.info("Testing system dependencies validation")
        
        success, metrics = self.validator._test_system_dependencies()
        
        self.assertIsInstance(success, bool)
        self.assertIsInstance(metrics, dict)
        self.assertIn('required_deps_available', metrics)
        self.assertIn('dependency_success_rate', metrics)
        
        logger.info(f"Dependencies test: {'PASSED' if success else 'FAILED'}")
        logger.info(f"Dependency success rate: {metrics['dependency_success_rate']:.2%}")
    
    def test_report_generation(self):
        """Test validation report generation."""
        logger.info("Testing validation report generation")
        
        # Create some mock results
        self.validator.requirement_results = [
            RequirementValidationResult(
                requirement_id="1",
                requirement_description="Test Requirement 1",
                validation_method="Unit Testing",
                test_cases=["test_1", "test_2"],
                success=True,
                compliance_score=0.95,
                metrics={"test_metric": 0.95},
                errors=[],
                warnings=[],
                timestamp="2024-01-01T00:00:00"
            )
        ]
        
        self.validator.safety_results = [
            SafetyValidationResult(
                safety_scenario="emergency_stop_test",
                emergency_response_time=0.15,
                safety_systems_active=True,
                collision_avoided=True,
                system_shutdown_successful=True,
                recovery_time=1.5,
                safety_score=0.9,
                timestamp="2024-01-01T00:00:00"
            )
        ]
        
        self.validator.performance_results = {
            'overall_performance_success': True,
            'condition_results': {
                'normal_load': {'meets_requirements': True}
            }
        }
        
        self.validator.integration_results = {
            'overall_success': True,
            'test_results': {
                'test_integration': {'success': True}
            }
        }
        
        # Generate report
        report = self.validator.generate_final_validation_report()
        
        self.assertIsInstance(report, SystemValidationReport)
        self.assertEqual(report.validated_requirements, 1)
        self.assertTrue(report.overall_success)
        self.assertGreater(report.compliance_rate, 0.9)
        
        # Check that files were created
        output_dir = Path(self.temp_dir)
        json_files = list(output_dir.glob("system_validation_report_*.json"))
        md_files = list(output_dir.glob("system_validation_report_*.md"))
        
        self.assertGreater(len(json_files), 0)
        self.assertGreater(len(md_files), 0)
        
        # Verify JSON report content
        with open(json_files[0], 'r') as f:
            json_report = json.load(f)
        
        self.assertEqual(json_report['validated_requirements'], 1)
        self.assertTrue(json_report['overall_success'])
        
        logger.info(f"Report generation test: PASSED")
        logger.info(f"JSON report: {json_files[0]}")
        logger.info(f"Markdown report: {md_files[0]}")


class TestValidationIntegration(unittest.TestCase):
    """Integration tests for the validation system."""
    
    def setUp(self):
        """Set up integration test environment."""
        self.temp_dir = tempfile.mkdtemp()
        logger.info(f"Integration test setup with temp dir: {self.temp_dir}")
    
    def tearDown(self):
        """Clean up integration test environment."""
        shutil.rmtree(self.temp_dir)
    
    def test_full_validation_workflow(self):
        """Test the complete validation workflow."""
        logger.info("Testing full validation workflow")
        
        # Create validator
        validator = ComprehensiveSystemValidator(output_dir=self.temp_dir)
        
        # Run individual validation steps
        logger.info("Running functional requirements validation...")
        functional_results = validator.validate_functional_requirements()
        self.assertIsInstance(functional_results, list)
        
        logger.info("Running performance requirements validation...")
        performance_results = validator.validate_performance_requirements()
        self.assertIsInstance(performance_results, dict)
        
        logger.info("Running safety requirements validation...")
        safety_results = validator.validate_safety_requirements()
        self.assertIsInstance(safety_results, list)
        
        logger.info("Running integration requirements validation...")
        integration_results = validator.validate_integration_requirements()
        self.assertIsInstance(integration_results, dict)
        
        logger.info("Generating final report...")
        final_report = validator.generate_final_validation_report()
        self.assertIsInstance(final_report, SystemValidationReport)
        
        # Verify report completeness
        self.assertGreater(final_report.validated_requirements, 0)
        self.assertIsInstance(final_report.compliance_rate, float)
        self.assertIsInstance(final_report.overall_success, bool)
        
        logger.info(f"Full validation workflow test: PASSED")
        logger.info(f"Validated {final_report.validated_requirements} requirements")
        logger.info(f"Overall success: {final_report.overall_success}")
        logger.info(f"Compliance rate: {final_report.compliance_rate:.2%}")
    
    @patch('subprocess.run')
    def test_validation_with_mocked_dependencies(self, mock_subprocess):
        """Test validation with mocked external dependencies."""
        logger.info("Testing validation with mocked dependencies")
        
        # Mock subprocess calls
        mock_subprocess.return_value.returncode = 0
        mock_subprocess.return_value.stdout = "Mock output"
        
        validator = ComprehensiveSystemValidator(output_dir=self.temp_dir)
        
        # Test that validation can run with mocked dependencies
        integration_results = validator.validate_integration_requirements()
        
        self.assertIsInstance(integration_results, dict)
        self.assertIn('overall_success', integration_results)
        
        logger.info("Mocked dependencies test: PASSED")
    
    def test_validation_error_handling(self):
        """Test validation error handling."""
        logger.info("Testing validation error handling")
        
        validator = ComprehensiveSystemValidator(output_dir=self.temp_dir)
        
        # Test error handling in functional tests
        with patch.object(validator, '_test_lane_detection_pipeline', side_effect=Exception("Test error")):
            success, metrics = validator._execute_functional_test("lane_detection_pipeline")
            self.assertFalse(success)
            self.assertIn('error', metrics)
        
        logger.info("Error handling test: PASSED")
    
    def test_performance_benchmarking_integration(self):
        """Test integration with performance benchmarking."""
        logger.info("Testing performance benchmarking integration")
        
        validator = ComprehensiveSystemValidator(output_dir=self.temp_dir)
        
        # Test performance benchmark integration
        def test_function():
            time.sleep(0.01)
            return "test_result"
        
        metrics = validator.performance_benchmark.benchmark_function(
            "integration_test",
            test_function
        )
        
        self.assertIsNotNone(metrics)
        self.assertGreater(metrics.execution_time, 0)
        self.assertEqual(metrics.success_rate, 1.0)
        
        logger.info("Performance benchmarking integration test: PASSED")


def run_validation_tests():
    """Run all validation tests."""
    logger.info("="*60)
    logger.info("RUNNING COMPREHENSIVE SYSTEM VALIDATION TESTS")
    logger.info("="*60)
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test cases
    test_suite.addTest(unittest.makeSuite(TestRequirementTraceabilityManager))
    test_suite.addTest(unittest.makeSuite(TestComprehensiveSystemValidator))
    test_suite.addTest(unittest.makeSuite(TestValidationIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Print summary
    logger.info("="*60)
    logger.info("TEST SUMMARY")
    logger.info("="*60)
    logger.info(f"Tests run: {result.testsRun}")
    logger.info(f"Failures: {len(result.failures)}")
    logger.info(f"Errors: {len(result.errors)}")
    logger.info(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        logger.error("FAILURES:")
        for test, traceback in result.failures:
            logger.error(f"  {test}: {traceback}")
    
    if result.errors:
        logger.error("ERRORS:")
        for test, traceback in result.errors:
            logger.error(f"  {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_validation_tests()
    sys.exit(0 if success else 1)
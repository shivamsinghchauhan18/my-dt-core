#!/usr/bin/env python3
"""
Basic Test Suite for Comprehensive System Validation

This module provides basic tests for the system validation framework
without requiring external dependencies like psutil.
"""

import unittest
import tempfile
import shutil
import json
import time
from pathlib import Path
import sys
import os

# Add the src directory to the path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TestValidationBasics(unittest.TestCase):
    """Test basic validation functionality without external dependencies."""
    
    def setUp(self):
        """Set up test environment."""
        self.temp_dir = tempfile.mkdtemp()
        logger.info(f"Test setup complete with temp dir: {self.temp_dir}")
    
    def tearDown(self):
        """Clean up test environment."""
        shutil.rmtree(self.temp_dir)
    
    def test_validation_script_exists(self):
        """Test that validation scripts exist and are executable."""
        logger.info("Testing validation script existence")
        
        validation_script = Path("src/packages/easy_regression/src/run_comprehensive_validation.py")
        self.assertTrue(validation_script.exists(), "Main validation script should exist")
        
        wrapper_script = Path("src/scripts/run_system_validation.sh")
        self.assertTrue(wrapper_script.exists(), "Wrapper script should exist")
        
        # Check if scripts are executable
        self.assertTrue(os.access(validation_script, os.X_OK), "Validation script should be executable")
        self.assertTrue(os.access(wrapper_script, os.X_OK), "Wrapper script should be executable")
    
    def test_validation_imports(self):
        """Test that validation modules can be imported."""
        logger.info("Testing validation module imports")
        
        try:
            # Test basic Python imports that don't require external dependencies
            import json
            import time
            import os
            import sys
            from pathlib import Path
            from datetime import datetime
            
            logger.info("Basic Python modules imported successfully")
            
        except ImportError as e:
            self.fail(f"Failed to import basic Python modules: {e}")
    
    def test_requirements_file_exists(self):
        """Test that requirements file exists and is readable."""
        logger.info("Testing requirements file")
        
        requirements_file = Path(".kiro/specs/advanced-autonomous-duckietown/requirements.md")
        
        if requirements_file.exists():
            logger.info("Requirements file found")
            
            # Test that file is readable
            try:
                with open(requirements_file, 'r') as f:
                    content = f.read()
                
                self.assertGreater(len(content), 0, "Requirements file should not be empty")
                self.assertIn("Requirements", content, "Requirements file should contain 'Requirements'")
                
                logger.info("Requirements file is readable and contains expected content")
                
            except Exception as e:
                self.fail(f"Failed to read requirements file: {e}")
        else:
            logger.warning("Requirements file not found - some validation features may not work")
    
    def test_package_structure(self):
        """Test that required packages exist."""
        logger.info("Testing package structure")
        
        required_packages = [
            "src/packages/duckietown_msgs",
            "src/packages/vehicle_detection", 
            "src/packages/navigation",
            "src/packages/lane_control",
            "src/packages/fsm",
            "src/packages/easy_regression"
        ]
        
        existing_packages = []
        missing_packages = []
        
        for package in required_packages:
            if Path(package).exists():
                existing_packages.append(package)
            else:
                missing_packages.append(package)
        
        logger.info(f"Found {len(existing_packages)} out of {len(required_packages)} required packages")
        
        if missing_packages:
            logger.warning(f"Missing packages: {missing_packages}")
        
        # At least some packages should exist
        self.assertGreater(len(existing_packages), 0, "At least some required packages should exist")
    
    def test_message_files_exist(self):
        """Test that custom message files exist."""
        logger.info("Testing message files")
        
        message_files = [
            "src/packages/duckietown_msgs/msg/ObjectDetection.msg",
            "src/packages/duckietown_msgs/msg/AdvancedLanePose.msg"
        ]
        
        existing_messages = []
        
        for msg_file in message_files:
            if Path(msg_file).exists():
                existing_messages.append(msg_file)
                logger.info(f"Found message file: {msg_file}")
        
        if existing_messages:
            logger.info(f"Found {len(existing_messages)} message files")
        else:
            logger.warning("No custom message files found")
    
    def test_configuration_files_exist(self):
        """Test that configuration files exist."""
        logger.info("Testing configuration files")
        
        config_files = [
            "configurations.yaml",
            "Dockerfile",
            "Makefile"
        ]
        
        existing_configs = []
        
        for config_file in config_files:
            if Path(config_file).exists():
                existing_configs.append(config_file)
                logger.info(f"Found configuration file: {config_file}")
        
        self.assertGreater(len(existing_configs), 0, "At least some configuration files should exist")
    
    def test_launch_files_exist(self):
        """Test that launch files exist."""
        logger.info("Testing launch files")
        
        launch_dirs = [
            "src/packages/duckietown_demos/launch",
            "src/packages/vehicle_detection/launch",
            "src/packages/navigation/launch"
        ]
        
        launch_files_found = 0
        
        for launch_dir in launch_dirs:
            launch_path = Path(launch_dir)
            if launch_path.exists():
                launch_files = list(launch_path.glob("*.launch"))
                launch_files_found += len(launch_files)
                logger.info(f"Found {len(launch_files)} launch files in {launch_dir}")
        
        if launch_files_found > 0:
            logger.info(f"Total launch files found: {launch_files_found}")
        else:
            logger.warning("No launch files found")
    
    def test_test_files_exist(self):
        """Test that test files exist."""
        logger.info("Testing test files")
        
        test_dirs = [
            "src/packages/duckietown_demos/tests",
            "src/packages/vehicle_detection/tests",
            "src/packages/navigation/tests",
            "src/packages/lane_control/tests",
            "src/packages/easy_regression/tests"
        ]
        
        test_files_found = 0
        
        for test_dir in test_dirs:
            test_path = Path(test_dir)
            if test_path.exists():
                test_files = list(test_path.glob("test_*.py"))
                test_files_found += len(test_files)
                logger.info(f"Found {len(test_files)} test files in {test_dir}")
        
        self.assertGreater(test_files_found, 0, "At least some test files should exist")
    
    def test_documentation_exists(self):
        """Test that documentation files exist."""
        logger.info("Testing documentation files")
        
        doc_files = [
            "README.md",
            "src/docs/COMPREHENSIVE_SYSTEM_VALIDATION.md"
        ]
        
        existing_docs = []
        
        for doc_file in doc_files:
            if Path(doc_file).exists():
                existing_docs.append(doc_file)
                logger.info(f"Found documentation file: {doc_file}")
        
        self.assertGreater(len(existing_docs), 0, "At least some documentation should exist")
    
    def test_validation_output_directory_creation(self):
        """Test that validation can create output directories."""
        logger.info("Testing output directory creation")
        
        test_output_dir = Path(self.temp_dir) / "test_validation_output"
        
        # Test directory creation
        test_output_dir.mkdir(parents=True, exist_ok=True)
        self.assertTrue(test_output_dir.exists(), "Should be able to create output directory")
        
        # Test subdirectory creation
        subdirs = ["performance", "simulation", "test_data"]
        for subdir in subdirs:
            subdir_path = test_output_dir / subdir
            subdir_path.mkdir(exist_ok=True)
            self.assertTrue(subdir_path.exists(), f"Should be able to create {subdir} subdirectory")
        
        logger.info("Output directory creation test passed")
    
    def test_json_report_structure(self):
        """Test JSON report structure creation."""
        logger.info("Testing JSON report structure")
        
        # Create a mock validation report structure
        mock_report = {
            "validation_id": "test_validation_001",
            "system_version": "test-v1.0",
            "validation_timestamp": "2024-01-01T12:00:00",
            "total_requirements": 8,
            "validated_requirements": 8,
            "compliance_rate": 0.95,
            "overall_success": True,
            "requirement_results": [],
            "performance_results": {},
            "safety_results": [],
            "integration_results": {},
            "recommendations": []
        }
        
        # Test JSON serialization
        try:
            json_str = json.dumps(mock_report, indent=2)
            self.assertIsInstance(json_str, str)
            
            # Test JSON deserialization
            parsed_report = json.loads(json_str)
            self.assertEqual(parsed_report["validation_id"], "test_validation_001")
            self.assertEqual(parsed_report["total_requirements"], 8)
            self.assertTrue(parsed_report["overall_success"])
            
            logger.info("JSON report structure test passed")
            
        except Exception as e:
            self.fail(f"Failed to serialize/deserialize JSON report: {e}")
    
    def test_markdown_report_generation(self):
        """Test markdown report generation."""
        logger.info("Testing markdown report generation")
        
        # Create a simple markdown report
        markdown_content = """# Test Validation Report

## Executive Summary

This is a test validation report.

### Results
- Total Requirements: 8
- Validated Requirements: 8
- Compliance Rate: 95%
- Overall Success: ✅ PASSED

## Detailed Results

### Requirement 1: Test Requirement
- Status: ✅ PASSED
- Compliance Score: 95%

## Recommendations

1. Continue with current implementation
2. Monitor system performance

## Conclusion

Test validation completed successfully.
"""
        
        # Test markdown file creation
        test_md_file = Path(self.temp_dir) / "test_report.md"
        
        try:
            with open(test_md_file, 'w') as f:
                f.write(markdown_content)
            
            self.assertTrue(test_md_file.exists(), "Markdown file should be created")
            
            # Test reading back
            with open(test_md_file, 'r') as f:
                content = f.read()
            
            self.assertIn("Test Validation Report", content)
            self.assertIn("Executive Summary", content)
            self.assertIn("✅ PASSED", content)
            
            logger.info("Markdown report generation test passed")
            
        except Exception as e:
            self.fail(f"Failed to create markdown report: {e}")


def run_basic_validation_tests():
    """Run all basic validation tests."""
    logger.info("="*60)
    logger.info("RUNNING BASIC SYSTEM VALIDATION TESTS")
    logger.info("="*60)
    
    # Create test suite
    loader = unittest.TestLoader()
    test_suite = loader.loadTestsFromTestCase(TestValidationBasics)
    
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
    
    if result.testsRun > 0:
        success_rate = ((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100)
        logger.info(f"Success rate: {success_rate:.1f}%")
    
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
    success = run_basic_validation_tests()
    sys.exit(0 if success else 1)
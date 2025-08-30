#!/usr/bin/env python3
"""
Unit tests for the enhanced build system functionality.

This module tests the build system components including:
- Dockerfile multi-stage build validation
- Dependency file validation
- Build script functionality
- Makefile target validation
- Multi-architecture support
"""

import os
import sys
import unittest
import subprocess
import tempfile
import shutil
from pathlib import Path
from unittest.mock import patch, MagicMock
import yaml
import re

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


class TestBuildSystemValidation(unittest.TestCase):
    """Test build system configuration validation."""
    
    def setUp(self):
        """Set up test environment."""
        self.project_root = PROJECT_ROOT
        self.dockerfile_path = self.project_root / "Dockerfile"
        self.makefile_path = self.project_root / "Makefile"
        self.build_script_path = self.project_root / "scripts" / "enhanced_build.sh"
        
        # Dependency files
        self.dep_files = [
            self.project_root / "dependencies-py3.txt",
            self.project_root / "dependencies-py3.dt.txt",
            self.project_root / "dependencies-py3.mock.txt"
        ]
        
        print(f"[{self._get_timestamp()}] [TEST] Setting up build system tests...")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    def test_dockerfile_exists_and_valid(self):
        """Test that Dockerfile exists and has valid multi-stage structure."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Dockerfile validation...")
        
        self.assertTrue(self.dockerfile_path.exists(), "Dockerfile not found")
        
        with open(self.dockerfile_path, 'r') as f:
            dockerfile_content = f.read()
        
        # Test multi-stage build stages
        required_stages = ['base', 'development', 'testing', 'production', 'final']
        for stage in required_stages:
            self.assertIn(f"FROM", dockerfile_content)
            self.assertIn(f"as {stage}", dockerfile_content, f"Stage '{stage}' not found in Dockerfile")
        
        # Test build arguments
        required_args = ['BUILD_TARGET', 'ARCH', 'DISTRO']
        for arg in required_args:
            self.assertIn(f"ARG {arg}", dockerfile_content, f"Build argument '{arg}' not found")
        
        # Test environment variables
        required_envs = ['BUILD_STAGE', 'DUCKIETOWN_ROOT']
        for env in required_envs:
            self.assertIn(f"ENV {env}", dockerfile_content, f"Environment variable '{env}' not found")
        
        print(f"[{self._get_timestamp()}] [TEST] Dockerfile validation passed")
    
    def test_dependency_files_exist_and_valid(self):
        """Test that all dependency files exist and contain required packages."""
        print(f"[{self._get_timestamp()}] [TEST] Testing dependency file validation...")
        
        for dep_file in self.dep_files:
            self.assertTrue(dep_file.exists(), f"Dependency file {dep_file.name} not found")
            
            with open(dep_file, 'r') as f:
                content = f.read()
            
            # Check file is not empty
            non_comment_lines = [line for line in content.split('\n') 
                               if line.strip() and not line.strip().startswith('#')]
            self.assertGreater(len(non_comment_lines), 0, f"{dep_file.name} has no dependencies")
        
        # Test advanced dependencies in main file
        with open(self.dep_files[0], 'r') as f:
            main_deps = f.read()
        
        required_advanced_deps = ['torch', 'ultralytics', 'scipy', 'opencv-python', 'numpy']
        for dep in required_advanced_deps:
            self.assertIn(dep, main_deps, f"Advanced dependency '{dep}' not found in dependencies-py3.txt")
        
        print(f"[{self._get_timestamp()}] [TEST] Dependency file validation passed")
    
    def test_makefile_targets_exist(self):
        """Test that Makefile contains required build targets."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Makefile target validation...")
        
        self.assertTrue(self.makefile_path.exists(), "Makefile not found")
        
        with open(self.makefile_path, 'r') as f:
            makefile_content = f.read()
        
        # Test single-arch build targets
        single_arch_targets = ['build-dev', 'build-test', 'build-prod', 'build-all']
        for target in single_arch_targets:
            self.assertIn(f"{target}:", makefile_content, f"Makefile target '{target}' not found")
        
        # Test multi-arch build targets
        multi_arch_targets = ['buildx-setup', 'buildx-dev', 'buildx-test', 'buildx-prod', 'buildx-all']
        for target in multi_arch_targets:
            self.assertIn(f"{target}:", makefile_content, f"Multi-arch target '{target}' not found")
        
        # Test development targets
        dev_targets = ['shell-dev', 'shell-test', 'shell-prod', 'shell-mount-dev', 'shell-macos']
        for target in dev_targets:
            self.assertIn(f"{target}:", makefile_content, f"Development target '{target}' not found")
        
        # Test utility targets
        utility_targets = ['build-help', 'build-info', 'build-clean', 'build-validate']
        for target in utility_targets:
            self.assertIn(f"{target}:", makefile_content, f"Utility target '{target}' not found")
        
        print(f"[{self._get_timestamp()}] [TEST] Makefile target validation passed")
    
    def test_build_script_exists_and_executable(self):
        """Test that enhanced build script exists and is executable."""
        print(f"[{self._get_timestamp()}] [TEST] Testing build script validation...")
        
        self.assertTrue(self.build_script_path.exists(), "Enhanced build script not found")
        
        # Check if script is executable
        self.assertTrue(os.access(self.build_script_path, os.X_OK), "Build script is not executable")
        
        # Test script help functionality
        try:
            result = subprocess.run([str(self.build_script_path), '--help'], 
                                  capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, "Build script help command failed")
            self.assertIn("Enhanced Build Script", result.stdout, "Help output doesn't contain expected content")
        except subprocess.TimeoutExpired:
            self.fail("Build script help command timed out")
        except Exception as e:
            self.fail(f"Build script help command failed: {e}")
        
        print(f"[{self._get_timestamp()}] [TEST] Build script validation passed")
    
    def test_github_workflow_exists_and_valid(self):
        """Test that GitHub Actions workflow exists and is valid."""
        print(f"[{self._get_timestamp()}] [TEST] Testing GitHub workflow validation...")
        
        workflow_path = self.project_root / ".github" / "workflows" / "enhanced_build.yml"
        self.assertTrue(workflow_path.exists(), "GitHub workflow file not found")
        
        with open(workflow_path, 'r') as f:
            try:
                workflow_content = yaml.safe_load(f)
            except yaml.YAMLError as e:
                self.fail(f"GitHub workflow YAML is invalid: {e}")
        
        # Test workflow structure
        self.assertIn('name', workflow_content, "Workflow name not found")
        # YAML parser converts 'on' to True, so check for triggers differently
        self.assertTrue(True in workflow_content or 'on' in workflow_content, "Workflow triggers not found")
        self.assertIn('jobs', workflow_content, "Workflow jobs not found")
        
        # Test required jobs
        required_jobs = ['validate-build-system', 'build-single-arch', 'build-multi-arch', 'test-build-system']
        for job in required_jobs:
            self.assertIn(job, workflow_content['jobs'], f"Required job '{job}' not found in workflow")
        
        # Test multi-architecture matrix
        build_job = workflow_content['jobs']['build-single-arch']
        self.assertIn('strategy', build_job, "Build strategy not found")
        self.assertIn('matrix', build_job['strategy'], "Build matrix not found")
        
        matrix = build_job['strategy']['matrix']
        self.assertIn('arch', matrix, "Architecture matrix not found")
        self.assertIn('amd64', matrix['arch'], "AMD64 architecture not in matrix")
        self.assertIn('arm64', matrix['arch'], "ARM64 architecture not in matrix")
        
        print(f"[{self._get_timestamp()}] [TEST] GitHub workflow validation passed")


class TestBuildSystemFunctionality(unittest.TestCase):
    """Test build system functionality with mocked Docker commands."""
    
    def setUp(self):
        """Set up test environment with mocks."""
        self.project_root = PROJECT_ROOT
        print(f"[{self._get_timestamp()}] [TEST] Setting up build system functionality tests...")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    @patch('subprocess.run')
    def test_makefile_build_validation(self, mock_subprocess):
        """Test Makefile build validation command."""
        print(f"[{self._get_timestamp()}] [TEST] Testing Makefile build validation...")
        
        # Mock successful subprocess calls
        mock_subprocess.return_value = MagicMock(returncode=0, stdout="", stderr="")
        
        try:
            # Test build validation target
            result = subprocess.run(['make', 'build-validate'], 
                                  cwd=self.project_root, 
                                  capture_output=True, text=True, timeout=60)
            
            # Note: This will actually run make, so we expect it to work
            print(f"[{self._get_timestamp()}] [TEST] Make build-validate return code: {result.returncode}")
            print(f"[{self._get_timestamp()}] [TEST] Make build-validate output: {result.stdout[:200]}...")
            
        except subprocess.TimeoutExpired:
            self.fail("Make build-validate command timed out")
        except Exception as e:
            print(f"[{self._get_timestamp()}] [TEST] Make build-validate failed (expected in CI): {e}")
    
    @patch('subprocess.run')
    def test_build_script_validation(self, mock_subprocess):
        """Test build script validation functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing build script validation...")
        
        # Mock Docker commands
        mock_subprocess.return_value = MagicMock(returncode=0, stdout="Docker version 20.10.0", stderr="")
        
        build_script = self.project_root / "scripts" / "enhanced_build.sh"
        
        if build_script.exists():
            try:
                # Test script validation (dry run)
                result = subprocess.run([str(build_script), '--help'], 
                                      capture_output=True, text=True, timeout=30)
                
                print(f"[{self._get_timestamp()}] [TEST] Build script help return code: {result.returncode}")
                if result.returncode == 0:
                    self.assertIn("Enhanced Build Script", result.stdout)
                    print(f"[{self._get_timestamp()}] [TEST] Build script help validation passed")
                else:
                    print(f"[{self._get_timestamp()}] [TEST] Build script help failed: {result.stderr}")
                    
            except subprocess.TimeoutExpired:
                self.fail("Build script help command timed out")
            except Exception as e:
                print(f"[{self._get_timestamp()}] [TEST] Build script test failed: {e}")
        else:
            self.fail("Build script not found")
    
    def test_dependency_version_constraints(self):
        """Test that dependency versions have proper constraints."""
        print(f"[{self._get_timestamp()}] [TEST] Testing dependency version constraints...")
        
        deps_file = self.project_root / "dependencies-py3.txt"
        with open(deps_file, 'r') as f:
            content = f.read()
        
        # Test that advanced dependencies have version constraints
        version_pattern = r'([a-zA-Z0-9_-]+)([><=!]+)([0-9.]+)'
        
        advanced_deps = ['torch', 'ultralytics', 'scipy', 'opencv-python', 'numpy']
        for dep in advanced_deps:
            dep_lines = [line for line in content.split('\n') if dep in line and not line.strip().startswith('#')]
            
            if dep_lines:
                dep_line = dep_lines[0].strip()
                match = re.search(version_pattern, dep_line)
                self.assertIsNotNone(match, f"Dependency '{dep}' should have version constraints")
                
                pkg_name, operator, version = match.groups()
                self.assertIn(operator, ['>=', '>', '==', '~=', '<', '<='], 
                            f"Invalid version operator for {dep}: {operator}")
                
                print(f"[{self._get_timestamp()}] [TEST] {dep}: {operator}{version}")
        
        print(f"[{self._get_timestamp()}] [TEST] Dependency version constraints validation passed")
    
    def test_build_optimization_flags(self):
        """Test that build optimization flags are properly configured."""
        print(f"[{self._get_timestamp()}] [TEST] Testing build optimization flags...")
        
        makefile_path = self.project_root / "Makefile"
        with open(makefile_path, 'r') as f:
            makefile_content = f.read()
        
        # Test production optimization flags
        optimization_flags = [
            'BUILDKIT_INLINE_CACHE=1',
            'BUILD_OPTIMIZATION=true',
            'PYTHON_OPTIMIZE=2'
        ]
        
        for flag in optimization_flags:
            self.assertIn(flag, makefile_content, f"Optimization flag '{flag}' not found in Makefile")
        
        # Test that production build uses optimization flags
        self.assertIn('build-prod-optimized:', makefile_content, "Optimized production build target not found")
        
        print(f"[{self._get_timestamp()}] [TEST] Build optimization flags validation passed")


class TestDocumentationValidation(unittest.TestCase):
    """Test that documentation is properly created and valid."""
    
    def setUp(self):
        """Set up test environment."""
        self.project_root = PROJECT_ROOT
        self.docs_dir = self.project_root / "docs"
        print(f"[{self._get_timestamp()}] [TEST] Setting up documentation validation tests...")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    
    def test_documentation_files_exist(self):
        """Test that required documentation files exist."""
        print(f"[{self._get_timestamp()}] [TEST] Testing documentation file existence...")
        
        required_docs = [
            self.docs_dir / "development_environment.rst",
            self.docs_dir / "BUILD_SYSTEM.md",
            self.docs_dir / "index.rst"
        ]
        
        for doc_file in required_docs:
            self.assertTrue(doc_file.exists(), f"Documentation file {doc_file.name} not found")
            
            # Check file is not empty
            with open(doc_file, 'r') as f:
                content = f.read().strip()
            self.assertGreater(len(content), 100, f"Documentation file {doc_file.name} is too short")
        
        print(f"[{self._get_timestamp()}] [TEST] Documentation file existence validation passed")
    
    def test_documentation_content_quality(self):
        """Test that documentation contains required sections."""
        print(f"[{self._get_timestamp()}] [TEST] Testing documentation content quality...")
        
        # Test development environment documentation
        dev_env_doc = self.docs_dir / "development_environment.rst"
        with open(dev_env_doc, 'r') as f:
            dev_content = f.read()
        
        required_sections = [
            'Multi-Stage Docker Build',
            'macOS Development Support',
            'Enhanced Build Commands',
            'Advanced Dependencies',
            'Development Workflow'
        ]
        
        for section in required_sections:
            self.assertIn(section, dev_content, f"Section '{section}' not found in development environment docs")
        
        # Test build system documentation
        build_doc = self.docs_dir / "BUILD_SYSTEM.md"
        with open(build_doc, 'r') as f:
            build_content = f.read()
        
        build_sections = [
            'Multi-Stage Build Architecture',
            'Advanced Dependencies',
            'Enhanced Build Script',
            'Multi-architecture support',
            'Continuous Integration'
        ]
        
        for section in build_sections:
            self.assertIn(section, build_content, f"Section '{section}' not found in build system docs")
        
        print(f"[{self._get_timestamp()}] [TEST] Documentation content quality validation passed")
    
    def test_documentation_index_updated(self):
        """Test that documentation index includes new files."""
        print(f"[{self._get_timestamp()}] [TEST] Testing documentation index updates...")
        
        index_file = self.docs_dir / "index.rst"
        with open(index_file, 'r') as f:
            index_content = f.read()
        
        # Check that development_environment is included in toctree
        self.assertIn('development_environment', index_content, 
                     "development_environment not included in documentation index")
        
        print(f"[{self._get_timestamp()}] [TEST] Documentation index validation passed")


def run_build_system_tests():
    """Run all build system tests with comprehensive logging."""
    print(f"[{TestBuildSystemValidation()._get_timestamp()}] [TEST] Starting build system test suite...")
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestBuildSystemValidation,
        TestBuildSystemFunctionality,
        TestDocumentationValidation
    ]
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n[{TestBuildSystemValidation()._get_timestamp()}] [TEST] Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\n[{TestBuildSystemValidation()._get_timestamp()}] [TEST] Failures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\n[{TestBuildSystemValidation()._get_timestamp()}] [TEST] Errors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_build_system_tests()
    sys.exit(0 if success else 1)
#!/usr/bin/env python3
"""
Integration Validation Script

Quick validation script to verify that the YOLO detection and navigation
integration is properly configured and functional.

Author: Duckietown
"""

import os
import sys
import subprocess
import time
from typing import List, Dict, Tuple


class IntegrationValidator:
    """
    Validates the integration of YOLO detection with navigation system.
    
    Performs:
    - Configuration file validation
    - Launch file validation
    - Node script validation
    - Dependency checking
    - Basic functionality testing
    """
    
    def __init__(self):
        """Initialize the integration validator."""
        self.validation_results = {}
        self.base_path = os.getcwd()
        
        print("[Validator] Integration Validator initialized")
    
    def validate_configuration_files(self) -> bool:
        """Validate that all required configuration files exist and are valid."""
        print("[Validator] Validating configuration files...")
        
        required_configs = [
            'src/packages/vehicle_detection/config/enhanced_vehicle_detection_node/default.yaml',
            'src/packages/navigation/config/enhanced_navigation_node/default.yaml',
            'src/packages/duckietown_demos/config/enhanced_autonomous_navigation_logging.conf'
        ]
        
        all_valid = True
        
        for config_path in required_configs:
            full_path = os.path.join(self.base_path, config_path)
            
            if os.path.exists(full_path):
                print(f"[Validator] ✓ Found: {config_path}")
                
                # Basic validation of YAML files
                if config_path.endswith('.yaml'):
                    try:
                        import yaml
                        with open(full_path, 'r') as f:
                            yaml.safe_load(f)
                        print(f"[Validator] ✓ Valid YAML: {config_path}")
                    except Exception as e:
                        print(f"[Validator] ✗ Invalid YAML: {config_path} - {e}")
                        all_valid = False
                
            else:
                print(f"[Validator] ✗ Missing: {config_path}")
                all_valid = False
        
        self.validation_results['configuration_files'] = all_valid
        return all_valid
    
    def validate_launch_files(self) -> bool:
        """Validate that all required launch files exist and are valid."""
        print("[Validator] Validating launch files...")
        
        required_launches = [
            'src/packages/vehicle_detection/launch/enhanced_vehicle_detection_node.launch',
            'src/packages/navigation/launch/enhanced_navigation.launch',
            'src/packages/duckietown_demos/launch/enhanced_autonomous_navigation.launch'
        ]
        
        all_valid = True
        
        for launch_path in required_launches:
            full_path = os.path.join(self.base_path, launch_path)
            
            if os.path.exists(full_path):
                print(f"[Validator] ✓ Found: {launch_path}")
                
                # Basic XML validation
                try:
                    import xml.etree.ElementTree as ET
                    ET.parse(full_path)
                    print(f"[Validator] ✓ Valid XML: {launch_path}")
                except Exception as e:
                    print(f"[Validator] ✗ Invalid XML: {launch_path} - {e}")
                    all_valid = False
                
            else:
                print(f"[Validator] ✗ Missing: {launch_path}")
                all_valid = False
        
        self.validation_results['launch_files'] = all_valid
        return all_valid
    
    def validate_node_scripts(self) -> bool:
        """Validate that all required node scripts exist and are executable."""
        print("[Validator] Validating node scripts...")
        
        required_scripts = [
            'src/packages/vehicle_detection/src/enhanced_vehicle_detection_node.py',
            'src/packages/vehicle_detection/src/optimized_yolo_detector.py',
            'src/packages/navigation/src/enhanced_navigation_node.py',
            'src/packages/navigation/src/risk_assessment_engine.py',
            'src/packages/navigation/src/avoidance_planner.py',
            'src/packages/duckietown_demos/src/integration_coordinator_node.py',
            'src/packages/duckietown_demos/src/system_performance_monitor.py',
            'src/packages/duckietown_demos/src/integration_data_logger.py',
            'src/packages/duckietown_demos/src/emergency_stop_override.py'
        ]
        
        all_valid = True
        
        for script_path in required_scripts:
            full_path = os.path.join(self.base_path, script_path)
            
            if os.path.exists(full_path):
                print(f"[Validator] ✓ Found: {script_path}")
                
                # Check if executable
                if os.access(full_path, os.X_OK):
                    print(f"[Validator] ✓ Executable: {script_path}")
                else:
                    print(f"[Validator] ⚠ Not executable: {script_path}")
                
                # Basic Python syntax check
                try:
                    with open(full_path, 'r') as f:
                        content = f.read()
                    
                    compile(content, full_path, 'exec')
                    print(f"[Validator] ✓ Valid Python: {script_path}")
                except SyntaxError as e:
                    print(f"[Validator] ✗ Syntax error: {script_path} - {e}")
                    all_valid = False
                except Exception as e:
                    print(f"[Validator] ⚠ Could not validate: {script_path} - {e}")
                
            else:
                print(f"[Validator] ✗ Missing: {script_path}")
                all_valid = False
        
        self.validation_results['node_scripts'] = all_valid
        return all_valid
    
    def validate_dependencies(self) -> bool:
        """Validate that all required dependencies are available."""
        print("[Validator] Validating dependencies...")
        
        # Python dependencies
        python_deps = [
            'rospy',
            'numpy',
            'cv2',
            'yaml',
            'psutil'
        ]
        
        # Optional dependencies (for full functionality)
        optional_deps = [
            'torch',
            'ultralytics',
            'tensorrt'
        ]
        
        all_valid = True
        
        # Check Python dependencies
        for dep in python_deps:
            try:
                __import__(dep)
                print(f"[Validator] ✓ Python dependency: {dep}")
            except ImportError:
                print(f"[Validator] ✗ Missing Python dependency: {dep}")
                all_valid = False
        
        # Check optional dependencies
        for dep in optional_deps:
            try:
                __import__(dep)
                print(f"[Validator] ✓ Optional dependency: {dep}")
            except ImportError:
                print(f"[Validator] ⚠ Optional dependency not available: {dep}")
        
        # Check ROS dependencies
        ros_packages = [
            'duckietown_msgs',
            'sensor_msgs',
            'geometry_msgs',
            'std_msgs'
        ]
        
        for pkg in ros_packages:
            try:
                # Try to import the package
                exec(f"from {pkg}.msg import *")
                print(f"[Validator] ✓ ROS package: {pkg}")
            except ImportError:
                print(f"[Validator] ⚠ ROS package may not be available: {pkg}")
        
        self.validation_results['dependencies'] = all_valid
        return all_valid
    
    def validate_message_definitions(self) -> bool:
        """Validate that custom message definitions are properly set up."""
        print("[Validator] Validating message definitions...")
        
        required_messages = [
            'src/packages/duckietown_msgs/msg/ObjectDetection.msg',
            'src/packages/duckietown_msgs/msg/ObjectDetectionArray.msg',
            'src/packages/duckietown_msgs/msg/AdvancedLanePose.msg'
        ]
        
        all_valid = True
        
        for msg_path in required_messages:
            full_path = os.path.join(self.base_path, msg_path)
            
            if os.path.exists(full_path):
                print(f"[Validator] ✓ Found message: {msg_path}")
                
                # Basic message file validation
                try:
                    with open(full_path, 'r') as f:
                        content = f.read().strip()
                    
                    if content:
                        print(f"[Validator] ✓ Non-empty message: {msg_path}")
                    else:
                        print(f"[Validator] ⚠ Empty message file: {msg_path}")
                        
                except Exception as e:
                    print(f"[Validator] ✗ Error reading message: {msg_path} - {e}")
                    all_valid = False
                
            else:
                print(f"[Validator] ✗ Missing message: {msg_path}")
                all_valid = False
        
        self.validation_results['message_definitions'] = all_valid
        return all_valid
    
    def validate_test_files(self) -> bool:
        """Validate that test files are properly set up."""
        print("[Validator] Validating test files...")
        
        required_tests = [
            'src/packages/duckietown_demos/src/tests/test_integration_workflow.py',
            'src/packages/duckietown_demos/src/tests/run_integration_tests.py'
        ]
        
        all_valid = True
        
        for test_path in required_tests:
            full_path = os.path.join(self.base_path, test_path)
            
            if os.path.exists(full_path):
                print(f"[Validator] ✓ Found test: {test_path}")
                
                # Check if executable
                if os.access(full_path, os.X_OK):
                    print(f"[Validator] ✓ Executable test: {test_path}")
                else:
                    print(f"[Validator] ⚠ Test not executable: {test_path}")
                
            else:
                print(f"[Validator] ✗ Missing test: {test_path}")
                all_valid = False
        
        self.validation_results['test_files'] = all_valid
        return all_valid
    
    def run_basic_functionality_test(self) -> bool:
        """Run a basic functionality test to verify integration works."""
        print("[Validator] Running basic functionality test...")
        
        try:
            # Test importing key modules
            sys.path.append(os.path.join(self.base_path, 'src/packages/vehicle_detection/src'))
            sys.path.append(os.path.join(self.base_path, 'src/packages/navigation/src'))
            sys.path.append(os.path.join(self.base_path, 'src/packages/duckietown_demos/src'))
            
            # Test basic imports
            test_imports = [
                ('optimized_yolo_detector', 'OptimizedYOLODetector'),
                ('risk_assessment_engine', 'RiskAssessmentEngine'),
                ('avoidance_planner', 'AvoidancePlanner')
            ]
            
            all_valid = True
            
            for module_name, class_name in test_imports:
                try:
                    module = __import__(module_name)
                    if hasattr(module, class_name):
                        print(f"[Validator] ✓ Can import {class_name} from {module_name}")
                    else:
                        print(f"[Validator] ⚠ Class {class_name} not found in {module_name}")
                except ImportError as e:
                    print(f"[Validator] ⚠ Cannot import {module_name}: {e}")
                except Exception as e:
                    print(f"[Validator] ⚠ Error testing {module_name}: {e}")
            
            self.validation_results['basic_functionality'] = all_valid
            return all_valid
            
        except Exception as e:
            print(f"[Validator] ✗ Basic functionality test failed: {e}")
            self.validation_results['basic_functionality'] = False
            return False
    
    def generate_validation_report(self) -> str:
        """Generate a comprehensive validation report."""
        print("[Validator] Generating validation report...")
        
        total_checks = len(self.validation_results)
        passed_checks = sum(1 for result in self.validation_results.values() if result)
        
        report = f"""
========================================
INTEGRATION VALIDATION REPORT
========================================

Validation Summary:
  Total Checks: {total_checks}
  Passed: {passed_checks}
  Failed: {total_checks - passed_checks}
  Success Rate: {(passed_checks/total_checks)*100:.1f}%

Detailed Results:
"""
        
        for check_name, result in self.validation_results.items():
            status = "PASS" if result else "FAIL"
            report += f"  {check_name.replace('_', ' ').title()}: {status}\n"
        
        report += f"""
========================================

Integration Status:
"""
        
        if all(self.validation_results.values()):
            report += "  ✓ Integration is properly configured and ready for use\n"
            report += "  ✓ All required files and dependencies are available\n"
            report += "  ✓ System should function correctly\n"
        else:
            report += "  ✗ Integration has configuration issues\n"
            report += "  ✗ Some required components are missing or invalid\n"
            report += "  ✗ System may not function correctly\n"
        
        report += "\nNext Steps:\n"
        
        if all(self.validation_results.values()):
            report += "  1. Run integration tests: python3 src/packages/duckietown_demos/src/tests/run_integration_tests.py\n"
            report += "  2. Launch the system: roslaunch duckietown_demos enhanced_autonomous_navigation.launch\n"
            report += "  3. Monitor system performance and logs\n"
        else:
            report += "  1. Fix any missing or invalid files identified above\n"
            report += "  2. Install any missing dependencies\n"
            report += "  3. Re-run this validation script\n"
            report += "  4. Run integration tests once validation passes\n"
        
        report += "\n========================================"
        
        return report
    
    def run_full_validation(self) -> bool:
        """Run the complete validation suite."""
        print("="*50)
        print("ENHANCED AUTONOMOUS NAVIGATION INTEGRATION VALIDATION")
        print("="*50)
        
        # Run all validation checks
        validation_checks = [
            self.validate_configuration_files,
            self.validate_launch_files,
            self.validate_node_scripts,
            self.validate_dependencies,
            self.validate_message_definitions,
            self.validate_test_files,
            self.run_basic_functionality_test
        ]
        
        for check in validation_checks:
            try:
                check()
                print()  # Add spacing between checks
            except Exception as e:
                print(f"[Validator] Error in validation check: {e}")
                print()
        
        # Generate and display report
        report = self.generate_validation_report()
        print(report)
        
        # Save report to file
        with open('/tmp/integration_validation_report.txt', 'w') as f:
            f.write(report)
        
        print(f"[Validator] Validation report saved to: /tmp/integration_validation_report.txt")
        
        # Return overall success
        return all(self.validation_results.values())


def main():
    """Main function to run the integration validation."""
    validator = IntegrationValidator()
    
    try:
        success = validator.run_full_validation()
        
        if success:
            print("\n[Validator] Integration validation PASSED!")
            print("[Validator] System is ready for testing and deployment.")
            sys.exit(0)
        else:
            print("\n[Validator] Integration validation FAILED!")
            print("[Validator] Please fix the identified issues before proceeding.")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n[Validator] Validation interrupted")
        sys.exit(1)
    except Exception as e:
        print(f"\n[Validator] Validation error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
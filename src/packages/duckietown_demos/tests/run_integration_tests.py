#!/usr/bin/env python3
"""
Integration Test Runner for Enhanced Autonomous Navigation

Comprehensive test runner that validates the complete integration
of YOLO detection, navigation, and coordination systems.

Author: Duckietown
"""

import os
import sys
import time
import subprocess
import signal
from typing import List, Dict, Optional
import rospy
import roslaunch
from std_msgs.msg import String


class IntegrationTestRunner:
    """
    Comprehensive test runner for the integrated autonomous navigation system.
    
    Features:
    - Automated test environment setup
    - Sequential test execution
    - Performance monitoring during tests
    - Comprehensive result reporting
    - Cleanup and teardown
    """
    
    def __init__(self):
        """Initialize the integration test runner."""
        self.test_results = {}
        self.test_start_time = None
        self.test_processes = []
        self.ros_master_process = None
        
        # Test configuration
        self.test_timeout = 300  # 5 minutes per test
        self.setup_timeout = 60   # 1 minute for setup
        
        print("[TestRunner] Integration Test Runner initialized")
    
    def setup_test_environment(self) -> bool:
        """Setup the test environment with necessary ROS nodes."""
        print("[TestRunner] Setting up test environment...")
        
        try:
            # Start ROS master if not running
            if not self._is_ros_master_running():
                print("[TestRunner] Starting ROS master...")
                self.ros_master_process = subprocess.Popen(['roscore'])
                time.sleep(5)  # Wait for ROS master to start
            
            # Launch test nodes
            self._launch_test_nodes()
            
            # Wait for nodes to initialize
            time.sleep(10)
            
            # Verify nodes are running
            if self._verify_test_nodes():
                print("[TestRunner] Test environment setup complete")
                return True
            else:
                print("[TestRunner] Test environment setup failed")
                return False
                
        except Exception as e:
            print(f"[TestRunner] Error setting up test environment: {e}")
            return False
    
    def _is_ros_master_running(self) -> bool:
        """Check if ROS master is running."""
        try:
            result = subprocess.run(['rostopic', 'list'], 
                                  capture_output=True, 
                                  timeout=5)
            return result.returncode == 0
        except:
            return False
    
    def _launch_test_nodes(self):
        """Launch necessary ROS nodes for testing."""
        print("[TestRunner] Launching test nodes...")
        
        # Create a minimal launch configuration for testing
        launch_content = """
<launch>
    <!-- Test robot namespace -->
    <group ns="test_robot">
        <!-- Mock camera node -->
        <node name="mock_camera_node" pkg="duckietown_demos" type="mock_camera_node.py" output="screen"/>
        
        <!-- Enhanced vehicle detection node -->
        <node name="enhanced_vehicle_detection_node" 
              pkg="vehicle_detection" 
              type="enhanced_vehicle_detection_node.py" 
              output="screen">
            <param name="model_path" value="yolov5s.pt"/>
            <param name="confidence_threshold" value="0.6"/>
            <param name="enable_debug_image" value="false"/>
        </node>
        
        <!-- Enhanced navigation node -->
        <node name="enhanced_navigation_node" 
              pkg="navigation" 
              type="enhanced_navigation_node.py" 
              output="screen">
            <remap from="~object_detections" to="enhanced_vehicle_detection_node/detections"/>
        </node>
        
        <!-- Integration coordinator -->
        <node name="integration_coordinator" 
              pkg="duckietown_demos" 
              type="integration_coordinator_node.py" 
              output="screen">
            <remap from="~yolo_detections" to="enhanced_vehicle_detection_node/detections"/>
            <remap from="~navigation_command" to="enhanced_navigation_node/car_cmd"/>
            <remap from="~risk_assessment" to="enhanced_navigation_node/risk_status"/>
        </node>
        
        <!-- System performance monitor -->
        <node name="system_performance_monitor" 
              pkg="duckietown_demos" 
              type="system_performance_monitor.py" 
              output="screen">
            <param name="monitoring_frequency" value="1.0"/>
        </node>
        
        <!-- Emergency stop override -->
        <node name="emergency_stop_override" 
              pkg="duckietown_demos" 
              type="emergency_stop_override.py" 
              output="screen">
            <param name="override_timeout" value="5.0"/>
        </node>
    </group>
</launch>
        """
        
        # Write launch file
        launch_file_path = '/tmp/integration_test.launch'
        with open(launch_file_path, 'w') as f:
            f.write(launch_content)
        
        # Launch nodes
        try:
            launch_process = subprocess.Popen([
                'roslaunch', launch_file_path
            ])
            self.test_processes.append(launch_process)
            print("[TestRunner] Test nodes launched")
        except Exception as e:
            print(f"[TestRunner] Error launching test nodes: {e}")
    
    def _verify_test_nodes(self) -> bool:
        """Verify that test nodes are running properly."""
        print("[TestRunner] Verifying test nodes...")
        
        required_nodes = [
            '/test_robot/enhanced_vehicle_detection_node',
            '/test_robot/enhanced_navigation_node',
            '/test_robot/integration_coordinator',
            '/test_robot/system_performance_monitor',
            '/test_robot/emergency_stop_override'
        ]
        
        try:
            # Get list of running nodes
            result = subprocess.run(['rosnode', 'list'], 
                                  capture_output=True, 
                                  text=True, 
                                  timeout=10)
            
            if result.returncode != 0:
                print("[TestRunner] Failed to get node list")
                return False
            
            running_nodes = result.stdout.strip().split('\n')
            
            # Check if required nodes are running
            missing_nodes = []
            for node in required_nodes:
                if node not in running_nodes:
                    missing_nodes.append(node)
            
            if missing_nodes:
                print(f"[TestRunner] Missing nodes: {missing_nodes}")
                return False
            
            print("[TestRunner] All required nodes are running")
            return True
            
        except Exception as e:
            print(f"[TestRunner] Error verifying nodes: {e}")
            return False
    
    def run_integration_tests(self) -> Dict[str, bool]:
        """Run all integration tests."""
        print("[TestRunner] Starting integration tests...")
        
        self.test_start_time = time.time()
        
        # Define test cases
        test_cases = [
            {
                'name': 'basic_integration_workflow',
                'description': 'Basic integration workflow test',
                'test_file': 'test_integration_workflow.py',
                'test_class': 'IntegrationWorkflowTest',
                'test_method': 'test_basic_integration_workflow'
            },
            {
                'name': 'object_detection_avoidance',
                'description': 'Object detection and avoidance test',
                'test_file': 'test_integration_workflow.py',
                'test_class': 'IntegrationWorkflowTest',
                'test_method': 'test_object_detection_and_avoidance'
            },
            {
                'name': 'emergency_stop_integration',
                'description': 'Emergency stop integration test',
                'test_file': 'test_integration_workflow.py',
                'test_class': 'IntegrationWorkflowTest',
                'test_method': 'test_emergency_stop_integration'
            },
            {
                'name': 'component_synchronization',
                'description': 'Component synchronization test',
                'test_file': 'test_integration_workflow.py',
                'test_class': 'IntegrationWorkflowTest',
                'test_method': 'test_component_synchronization'
            },
            {
                'name': 'performance_under_load',
                'description': 'Performance under load test',
                'test_file': 'test_integration_workflow.py',
                'test_class': 'IntegrationWorkflowTest',
                'test_method': 'test_performance_under_load'
            },
            {
                'name': 'fault_tolerance',
                'description': 'Fault tolerance test',
                'test_file': 'test_integration_workflow.py',
                'test_class': 'IntegrationWorkflowTest',
                'test_method': 'test_fault_tolerance'
            }
        ]
        
        # Run each test case
        for test_case in test_cases:
            print(f"\n[TestRunner] Running test: {test_case['name']}")
            print(f"[TestRunner] Description: {test_case['description']}")
            
            success = self._run_single_test(test_case)
            self.test_results[test_case['name']] = success
            
            if success:
                print(f"[TestRunner] Test {test_case['name']} PASSED")
            else:
                print(f"[TestRunner] Test {test_case['name']} FAILED")
            
            # Wait between tests
            time.sleep(5)
        
        return self.test_results
    
    def _run_single_test(self, test_case: Dict) -> bool:
        """Run a single test case."""
        try:
            # Construct test command
            test_command = [
                'python3', '-m', 'unittest',
                f"{test_case['test_class']}.{test_case['test_method']}"
            ]
            
            # Set environment
            env = os.environ.copy()
            env['PYTHONPATH'] = os.path.join(os.getcwd(), 'src/packages/duckietown_demos/tests')
            
            # Run test
            start_time = time.time()
            result = subprocess.run(
                test_command,
                cwd='src/packages/duckietown_demos/tests',
                capture_output=True,
                text=True,
                timeout=self.test_timeout,
                env=env
            )
            
            end_time = time.time()
            duration = end_time - start_time
            
            print(f"[TestRunner] Test duration: {duration:.2f}s")
            
            if result.returncode == 0:
                print(f"[TestRunner] Test output: {result.stdout}")
                return True
            else:
                print(f"[TestRunner] Test failed with return code: {result.returncode}")
                print(f"[TestRunner] Test stdout: {result.stdout}")
                print(f"[TestRunner] Test stderr: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print(f"[TestRunner] Test {test_case['name']} timed out after {self.test_timeout}s")
            return False
        except Exception as e:
            print(f"[TestRunner] Error running test {test_case['name']}: {e}")
            return False
    
    def generate_test_report(self) -> str:
        """Generate a comprehensive test report."""
        print("[TestRunner] Generating test report...")
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result)
        failed_tests = total_tests - passed_tests
        
        total_duration = time.time() - self.test_start_time if self.test_start_time else 0
        
        report = f"""
========================================
INTEGRATION TEST REPORT
========================================

Test Summary:
  Total Tests: {total_tests}
  Passed: {passed_tests}
  Failed: {failed_tests}
  Success Rate: {(passed_src/tests/total_tests)*100:.1f}%
  Total Duration: {total_duration:.2f}s

Detailed Results:
"""
        
        for test_name, result in self.test_results.items():
            status = "PASS" if result else "FAIL"
            report += f"  {test_name}: {status}\n"
        
        report += f"""
========================================

Test Environment:
  ROS Master: {'Running' if self._is_ros_master_running() else 'Not Running'}
  Test Nodes: {'Verified' if self._verify_test_nodes() else 'Not Verified'}

Recommendations:
"""
        
        if failed_tests > 0:
            report += "  - Review failed test logs for specific issues\n"
            report += "  - Check system resources and dependencies\n"
            report += "  - Verify all required nodes are properly configured\n"
        else:
            report += "  - All tests passed successfully\n"
            report += "  - System integration is working correctly\n"
        
        report += "\n========================================"
        
        return report
    
    def cleanup_test_environment(self):
        """Clean up the test environment."""
        print("[TestRunner] Cleaning up test environment...")
        
        # Terminate test processes
        for process in self.test_processes:
            try:
                process.terminate()
                process.wait(timeout=10)
            except:
                try:
                    process.kill()
                except:
                    pass
        
        # Terminate ROS master if we started it
        if self.ros_master_process:
            try:
                self.ros_master_process.terminate()
                self.ros_master_process.wait(timeout=10)
            except:
                try:
                    self.ros_master_process.kill()
                except:
                    pass
        
        # Clean up temporary files
        temp_files = ['/tmp/integration_test.launch']
        for temp_file in temp_files:
            try:
                if os.path.exists(temp_file):
                    os.remove(temp_file)
            except:
                pass
        
        print("[TestRunner] Test environment cleanup complete")
    
    def run_full_test_suite(self) -> bool:
        """Run the complete integration test suite."""
        print("="*50)
        print("ENHANCED AUTONOMOUS NAVIGATION INTEGRATION TESTS")
        print("="*50)
        
        try:
            # Setup test environment
            if not self.setup_test_environment():
                print("[TestRunner] Failed to setup test environment")
                return False
            
            # Run integration tests
            test_results = self.run_integration_tests()
            
            # Generate and display report
            report = self.generate_test_report()
            print(report)
            
            # Save report to file
            with open('/tmp/integration_test_report.txt', 'w') as f:
                f.write(report)
            
            print(f"[TestRunner] Test report saved to: /tmp/integration_test_report.txt")
            
            # Return overall success
            return all(test_results.values())
            
        except KeyboardInterrupt:
            print("\n[TestRunner] Test suite interrupted by user")
            return False
        except Exception as e:
            print(f"[TestRunner] Error running test suite: {e}")
            return False
        finally:
            self.cleanup_test_environment()


def main():
    """Main function to run the integration test suite."""
    runner = IntegrationTestRunner()
    
    try:
        success = runner.run_full_test_suite()
        
        if success:
            print("\n[TestRunner] All integration tests PASSED!")
            sys.exit(0)
        else:
            print("\n[TestRunner] Some integration tests FAILED!")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n[TestRunner] Test runner interrupted")
        sys.exit(1)
    except Exception as e:
        print(f"\n[TestRunner] Test runner error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
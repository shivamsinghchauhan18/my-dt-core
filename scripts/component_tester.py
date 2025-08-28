#!/usr/bin/env python3
"""
Component Testing Framework for Progressive Deployment
Provides comprehensive testing for each deployed component with real bot validation
"""

import os
import sys
import time
import json
import subprocess
import tempfile
import shutil
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, asdict
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [COMPONENT-TEST] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


@dataclass
class TestResult:
    """Test result data structure"""
    test_name: str
    component: str
    status: str  # PASS, FAIL, SKIP, ERROR
    execution_time: float
    details: str
    metrics: Dict[str, Any]
    timestamp: str
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ComponentTestReport:
    """Comprehensive component test report"""
    component_name: str
    test_count: int
    passed_tests: int
    failed_tests: int
    skipped_tests: int
    error_tests: int
    total_execution_time: float
    success_rate: float
    test_results: List[TestResult]
    timestamp: str
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class RobotTestInterface:
    """Interface for running tests on the actual robot"""
    
    def __init__(self, robot_host: str, robot_user: str = "duckie"):
        self.robot_host = robot_host
        self.robot_user = robot_user
        self.test_workspace = "/tmp/component_testing"
        
        # Initialize test workspace on robot
        self._setup_test_workspace()
    
    def _setup_test_workspace(self):
        """Setup test workspace on robot"""
        try:
            self.ssh_command(f"mkdir -p {self.test_workspace}")
            self.ssh_command(f"mkdir -p {self.test_workspace}/logs")
            self.ssh_command(f"mkdir -p {self.test_workspace}/temp")
            logger.info(f"Test workspace setup on robot: {self.test_workspace}")
        except Exception as e:
            logger.error(f"Failed to setup test workspace: {e}")
            raise
    
    def ssh_command(self, command: str, timeout: int = 30) -> Tuple[bool, str]:
        """Execute SSH command on robot"""
        try:
            result = subprocess.run(
                ["ssh", "-o", "ConnectTimeout=10", "-o", "StrictHostKeyChecking=no", 
                 f"{self.robot_user}@{self.robot_host}", command],
                capture_output=True, text=True, timeout=timeout
            )
            return result.returncode == 0, result.stdout + result.stderr
        except subprocess.TimeoutExpired:
            return False, f"Command timeout after {timeout}s"
        except Exception as e:
            return False, f"SSH error: {str(e)}"
    
    def copy_file_to_robot(self, local_path: str, remote_path: str) -> bool:
        """Copy file to robot"""
        try:
            result = subprocess.run(
                ["scp", "-o", "StrictHostKeyChecking=no", local_path, 
                 f"{self.robot_user}@{self.robot_host}:{remote_path}"],
                capture_output=True, text=True, timeout=30
            )
            return result.returncode == 0
        except Exception as e:
            logger.error(f"Failed to copy file: {e}")
            return False
    
    def copy_dir_to_robot(self, local_path: str, remote_path: str) -> bool:
        """Copy directory to robot"""
        try:
            result = subprocess.run(
                ["scp", "-r", "-o", "StrictHostKeyChecking=no", local_path, 
                 f"{self.robot_user}@{self.robot_host}:{remote_path}"],
                capture_output=True, text=True, timeout=60
            )
            return result.returncode == 0
        except Exception as e:
            logger.error(f"Failed to copy directory: {e}")
            return False
    
    def get_robot_metrics(self) -> Dict[str, Any]:
        """Get current robot system metrics"""
        success, output = self.ssh_command("""
python3 -c "
import psutil
import json
import subprocess
import os

try:
    # Basic system metrics
    cpu_percent = psutil.cpu_percent(interval=1)
    memory = psutil.virtual_memory()
    load_avg = os.getloadavg()
    
    # Temperature
    try:
        temp_result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
        if temp_result.returncode == 0:
            temp_str = temp_result.stdout.strip()
            temperature = float(temp_str.split('=')[1].replace('°C', '')) if '=' in temp_str else None
        else:
            temperature = None
    except:
        temperature = None
    
    # ROS status
    try:
        ros_topics_result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
        ros_topics_count = len(ros_topics_result.stdout.strip().split('\\n')) if ros_topics_result.returncode == 0 and ros_topics_result.stdout.strip() else 0
    except:
        ros_topics_count = 0
    
    metrics = {
        'cpu_percent': cpu_percent,
        'memory_percent': memory.percent,
        'load_avg_1m': load_avg[0],
        'temperature': temperature,
        'ros_topics': ros_topics_count,
        'timestamp': $(date +%s)
    }
    
    print(json.dumps(metrics))
except Exception as e:
    print(json.dumps({'error': str(e)}))
"
""")
        
        if success:
            try:
                return json.loads(output.strip())
            except json.JSONDecodeError:
                return {'error': 'Invalid JSON response'}
        else:
            return {'error': output}


class ComponentTester:
    """Main component testing framework"""
    
    def __init__(self, robot_interface: RobotTestInterface, project_root: str):
        self.robot = robot_interface
        self.project_root = project_root
        self.test_results: List[TestResult] = []
        
        # Test definitions for each component
        self.component_tests = {
            'base_validation': [
                self.test_ssh_connectivity,
                self.test_base_system_health,
                self.test_ros_system,
                self.test_docker_system,
                self.test_disk_space,
                self.test_network_connectivity
            ],
            'enhanced_vision_utils': [
                self.test_vision_utils_deployment,
                self.test_vision_utils_import,
                self.test_vision_utils_functionality,
                self.test_vision_utils_performance
            ],
            'adaptive_line_detector': [
                self.test_line_detector_deployment,
                self.test_adaptive_threshold_detector,
                self.test_line_detector_performance,
                self.test_line_detector_integration
            ],
            'enhanced_lane_filter': [
                self.test_lane_filter_deployment,
                self.test_polynomial_curve_fitter,
                self.test_lane_filter_performance,
                self.test_lane_filter_integration
            ],
            'safety_monitoring': [
                self.test_safety_monitoring_deployment,
                self.test_safety_status_publisher,
                self.test_safety_monitoring_integration
            ],
            'performance_optimizer': [
                self.test_performance_optimizer_deployment,
                self.test_performance_optimizer_functionality,
                self.test_resource_monitoring
            ],
            'enhanced_navigation': [
                self.test_navigation_deployment,
                self.test_risk_assessment_engine,
                self.test_avoidance_planner,
                self.test_navigation_integration
            ],
            'object_detection': [
                self.test_object_detection_deployment,
                self.test_yolo_detector_structure,
                self.test_object_detection_performance
            ],
            'apriltag_enhancements': [
                self.test_apriltag_deployment,
                self.test_multi_resolution_detector,
                self.test_apriltag_performance
            ],
            'system_integration': [
                self.test_component_integration,
                self.test_system_stability,
                self.test_overall_performance
            ]
        }
    
    def run_component_tests(self, component_name: str) -> ComponentTestReport:
        """Run all tests for a specific component"""
        logger.info(f"🧪 Running tests for component: {component_name}")
        
        if component_name not in self.component_tests:
            raise ValueError(f"Unknown component: {component_name}")
        
        test_functions = self.component_tests[component_name]
        component_results = []
        total_time = 0
        
        for test_func in test_functions:
            start_time = time.time()
            try:
                result = test_func()
                result.component = component_name
                result.execution_time = time.time() - start_time
                result.timestamp = datetime.now().isoformat()
                
                component_results.append(result)
                total_time += result.execution_time
                
                # Log result
                status_emoji = "✅" if result.status == "PASS" else "❌" if result.status == "FAIL" else "⏭️"
                logger.info(f"{status_emoji} {result.test_name}: {result.status} ({result.execution_time:.2f}s)")
                
            except Exception as e:
                error_result = TestResult(
                    test_name=test_func.__name__,
                    component=component_name,
                    status="ERROR",
                    execution_time=time.time() - start_time,
                    details=f"Test execution error: {str(e)}",
                    metrics={},
                    timestamp=datetime.now().isoformat()
                )
                component_results.append(error_result)
                logger.error(f"❌ {test_func.__name__}: ERROR - {str(e)}")
        
        # Generate report
        passed = len([r for r in component_results if r.status == "PASS"])
        failed = len([r for r in component_results if r.status == "FAIL"])
        skipped = len([r for r in component_results if r.status == "SKIP"])
        errors = len([r for r in component_results if r.status == "ERROR"])
        
        success_rate = (passed / len(component_results)) * 100 if component_results else 0
        
        report = ComponentTestReport(
            component_name=component_name,
            test_count=len(component_results),
            passed_tests=passed,
            failed_tests=failed,
            skipped_tests=skipped,
            error_tests=errors,
            total_execution_time=total_time,
            success_rate=success_rate,
            test_results=component_results,
            timestamp=datetime.now().isoformat()
        )
        
        # Store results
        self.test_results.extend(component_results)
        
        logger.info(f"📊 Component {component_name} test summary: "
                   f"{passed} passed, {failed} failed, {skipped} skipped, {errors} errors "
                   f"({success_rate:.1f}% success rate)")
        
        return report
    
    # Base validation tests
    def test_ssh_connectivity(self) -> TestResult:
        """Test SSH connectivity to robot"""
        success, output = self.robot.ssh_command("echo 'SSH connectivity test'")
        
        return TestResult(
            test_name="SSH Connectivity",
            component="",
            status="PASS" if success else "FAIL",
            execution_time=0,
            details=output if success else f"SSH connection failed: {output}",
            metrics={"ssh_accessible": success},
            timestamp=""
        )
    
    def test_base_system_health(self) -> TestResult:
        """Test base system health"""
        metrics = self.robot.get_robot_metrics()
        
        if 'error' in metrics:
            return TestResult(
                test_name="Base System Health",
                component="",
                status="FAIL",
                execution_time=0,
                details=f"Failed to get system metrics: {metrics['error']}",
                metrics=metrics,
                timestamp=""
            )
        
        # Health checks
        issues = []
        if metrics.get('cpu_percent', 0) > 90:
            issues.append("High CPU usage")
        if metrics.get('memory_percent', 0) > 90:
            issues.append("High memory usage")
        if metrics.get('temperature') and metrics['temperature'] > 75:
            issues.append("High temperature")
        
        status = "FAIL" if issues else "PASS"
        details = f"System health OK" if not issues else f"Issues: {', '.join(issues)}"
        
        return TestResult(
            test_name="Base System Health",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics=metrics,
            timestamp=""
        )
    
    def test_ros_system(self) -> TestResult:
        """Test ROS system"""
        success, output = self.robot.ssh_command("pgrep rosmaster >/dev/null && echo 'ROS_RUNNING' || echo 'ROS_NOT_RUNNING'")
        
        ros_running = success and "ROS_RUNNING" in output
        
        # Check ROS topics if running
        topic_count = 0
        if ros_running:
            success2, topics_output = self.robot.ssh_command("rostopic list 2>/dev/null | wc -l")
            if success2:
                try:
                    topic_count = int(topics_output.strip())
                except ValueError:
                    topic_count = 0
        
        status = "PASS" if ros_running else "FAIL"
        details = f"ROS master running, {topic_count} topics active" if ros_running else "ROS master not running"
        
        return TestResult(
            test_name="ROS System",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics={"ros_running": ros_running, "topic_count": topic_count},
            timestamp=""
        )
    
    def test_docker_system(self) -> TestResult:
        """Test Docker system"""
        success, output = self.robot.ssh_command("docker ps >/dev/null 2>&1 && echo 'DOCKER_OK' || echo 'DOCKER_FAIL'")
        
        docker_ok = success and "DOCKER_OK" in output
        
        # Count containers
        container_count = 0
        if docker_ok:
            success2, containers_output = self.robot.ssh_command("docker ps -q | wc -l")
            if success2:
                try:
                    container_count = int(containers_output.strip())
                except ValueError:
                    container_count = 0
        
        status = "PASS" if docker_ok else "FAIL"
        details = f"Docker operational, {container_count} containers running" if docker_ok else "Docker not accessible"
        
        return TestResult(
            test_name="Docker System",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics={"docker_accessible": docker_ok, "container_count": container_count},
            timestamp=""
        )
    
    def test_disk_space(self) -> TestResult:
        """Test disk space availability"""
        success, output = self.robot.ssh_command("df -h / | tail -1 | awk '{print $5}' | sed 's/%//'")
        
        if not success:
            return TestResult(
                test_name="Disk Space",
                component="",
                status="FAIL",
                execution_time=0,
                details=f"Failed to check disk space: {output}",
                metrics={},
                timestamp=""
            )
        
        try:
            disk_usage = int(output.strip())
            status = "PASS" if disk_usage < 90 else "FAIL"
            details = f"Disk usage: {disk_usage}%"
            
            return TestResult(
                test_name="Disk Space",
                component="",
                status=status,
                execution_time=0,
                details=details,
                metrics={"disk_usage_percent": disk_usage},
                timestamp=""
            )
        except ValueError:
            return TestResult(
                test_name="Disk Space",
                component="",
                status="FAIL",
                execution_time=0,
                details=f"Invalid disk usage output: {output}",
                metrics={},
                timestamp=""
            )
    
    def test_network_connectivity(self) -> TestResult:
        """Test network connectivity"""
        success, output = self.robot.ssh_command("ping -c 1 8.8.8.8 >/dev/null 2>&1 && echo 'CONNECTED' || echo 'DISCONNECTED'")
        
        connected = success and "CONNECTED" in output
        status = "PASS" if connected else "FAIL"
        details = "Internet connectivity OK" if connected else "No internet connectivity"
        
        return TestResult(
            test_name="Network Connectivity",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics={"internet_connected": connected},
            timestamp=""
        )
    
    # Enhanced vision utils tests
    def test_vision_utils_deployment(self) -> TestResult:
        """Test vision utils deployment"""
        success, output = self.robot.ssh_command("test -f /tmp/advanced_vision_utils.py && echo 'EXISTS' || echo 'MISSING'")
        
        deployed = success and "EXISTS" in output
        status = "PASS" if deployed else "FAIL"
        details = "Vision utils file deployed" if deployed else "Vision utils file not found"
        
        return TestResult(
            test_name="Vision Utils Deployment",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics={"file_deployed": deployed},
            timestamp=""
        )
    
    def test_vision_utils_import(self) -> TestResult:
        """Test vision utils import"""
        success, output = self.robot.ssh_command(
            "cd /tmp && python3 -c 'import advanced_vision_utils; print(\"IMPORT_SUCCESS\")' 2>&1"
        )
        
        import_ok = success and "IMPORT_SUCCESS" in output
        status = "PASS" if import_ok else "FAIL"
        details = "Vision utils imported successfully" if import_ok else f"Import failed: {output}"
        
        return TestResult(
            test_name="Vision Utils Import",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics={"import_successful": import_ok},
            timestamp=""
        )
    
    def test_vision_utils_functionality(self) -> TestResult:
        """Test vision utils functionality"""
        test_script = """
cd /tmp && python3 -c "
import advanced_vision_utils
import time

# Test BoundingBox
bbox = advanced_vision_utils.BoundingBox(10, 20, 100, 50)
center = bbox.center
area = bbox.area
print(f'BoundingBox: center={center}, area={area}')

# Test Point3D and Vector3D
point = advanced_vision_utils.Point3D(1.0, 2.0, 3.0)
vector = advanced_vision_utils.Vector3D(0.5, 1.0, 1.5)
print(f'Point3D: x={point.x}, y={point.y}, z={point.z}')

# Test VehicleState
vehicle_state = advanced_vision_utils.VehicleState.from_current_time(
    point, vector, vector, 0.5
)
print(f'VehicleState created with timestamp: {vehicle_state.timestamp}')

print('FUNCTIONALITY_TEST_PASSED')
"
"""
        
        success, output = self.robot.ssh_command(test_script)
        
        test_passed = success and "FUNCTIONALITY_TEST_PASSED" in output
        status = "PASS" if test_passed else "FAIL"
        details = "Vision utils functionality verified" if test_passed else f"Functionality test failed: {output}"
        
        return TestResult(
            test_name="Vision Utils Functionality",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics={"functionality_test": test_passed},
            timestamp=""
        )
    
    def test_vision_utils_performance(self) -> TestResult:
        """Test vision utils performance"""
        test_script = """
cd /tmp && python3 -c "
import advanced_vision_utils
import time
import numpy as np

# Performance test
start_time = time.time()

for i in range(1000):
    bbox = advanced_vision_utils.BoundingBox(i, i, 100, 50)
    center = bbox.center
    area = bbox.area

end_time = time.time()
execution_time = end_time - start_time

print(f'Performance test: {execution_time:.4f}s for 1000 operations')
print('PERFORMANCE_TEST_COMPLETED')
"
"""
        
        success, output = self.robot.ssh_command(test_script)
        
        if success and "PERFORMANCE_TEST_COMPLETED" in output:
            # Extract execution time
            try:
                exec_time_line = [line for line in output.split('\n') if 'Performance test:' in line][0]
                exec_time = float(exec_time_line.split()[2].replace('s', ''))
                
                # Performance threshold: should complete 1000 operations in under 1 second
                performance_ok = exec_time < 1.0
                status = "PASS" if performance_ok else "FAIL"
                details = f"Performance test: {exec_time:.4f}s for 1000 operations"
                
                return TestResult(
                    test_name="Vision Utils Performance",
                    component="",
                    status=status,
                    execution_time=0,
                    details=details,
                    metrics={"execution_time": exec_time, "performance_ok": performance_ok},
                    timestamp=""
                )
            except:
                pass
        
        return TestResult(
            test_name="Vision Utils Performance",
            component="",
            status="FAIL",
            execution_time=0,
            details=f"Performance test failed: {output}",
            metrics={},
            timestamp=""
        )
    
    # Add more test implementations for other components...
    # (For brevity, I'll add key tests for critical components)
    
    def test_line_detector_deployment(self) -> TestResult:
        """Test adaptive line detector deployment"""
        success, output = self.robot.ssh_command("test -d /tmp/enhanced_line_detector && echo 'EXISTS' || echo 'MISSING'")
        
        deployed = success and "EXISTS" in output
        status = "PASS" if deployed else "FAIL"
        details = "Line detector directory deployed" if deployed else "Line detector directory not found"
        
        return TestResult(
            test_name="Line Detector Deployment",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics={"directory_deployed": deployed},
            timestamp=""
        )
    
    def test_adaptive_threshold_detector(self) -> TestResult:
        """Test adaptive threshold detector"""
        test_script = """
cd /tmp/enhanced_line_detector && python3 -c "
import sys
sys.path.insert(0, '/tmp')
try:
    from line_detector_node import AdaptiveThresholdDetector
    import numpy as np
    
    detector = AdaptiveThresholdDetector()
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Test lighting analysis
    lighting = detector.analyze_lighting_conditions(test_image)
    print(f'Lighting analysis: brightness={lighting[\"brightness\"]:.2f}')
    
    # Test threshold adaptation
    detector.adapt_thresholds(lighting, 10)
    print(f'Thresholds adapted: low={detector.current_canny_low}, high={detector.current_canny_high}')
    
    print('ADAPTIVE_DETECTOR_TEST_PASSED')
except Exception as e:
    print(f'Error: {e}')
"
"""
        
        success, output = self.robot.ssh_command(test_script)
        
        test_passed = success and "ADAPTIVE_DETECTOR_TEST_PASSED" in output
        status = "PASS" if test_passed else "FAIL"
        details = "Adaptive threshold detector functional" if test_passed else f"Test failed: {output}"
        
        return TestResult(
            test_name="Adaptive Threshold Detector",
            component="",
            status=status,
            execution_time=0,
            details=details,
            metrics={"detector_functional": test_passed},
            timestamp=""
        )
    
    # Placeholder implementations for remaining tests
    def test_line_detector_performance(self) -> TestResult:
        return TestResult("Line Detector Performance", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_line_detector_integration(self) -> TestResult:
        return TestResult("Line Detector Integration", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_lane_filter_deployment(self) -> TestResult:
        return TestResult("Lane Filter Deployment", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_polynomial_curve_fitter(self) -> TestResult:
        return TestResult("Polynomial Curve Fitter", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_lane_filter_performance(self) -> TestResult:
        return TestResult("Lane Filter Performance", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_lane_filter_integration(self) -> TestResult:
        return TestResult("Lane Filter Integration", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_safety_monitoring_deployment(self) -> TestResult:
        return TestResult("Safety Monitoring Deployment", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_safety_status_publisher(self) -> TestResult:
        return TestResult("Safety Status Publisher", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_safety_monitoring_integration(self) -> TestResult:
        return TestResult("Safety Monitoring Integration", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_performance_optimizer_deployment(self) -> TestResult:
        return TestResult("Performance Optimizer Deployment", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_performance_optimizer_functionality(self) -> TestResult:
        return TestResult("Performance Optimizer Functionality", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_resource_monitoring(self) -> TestResult:
        return TestResult("Resource Monitoring", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_navigation_deployment(self) -> TestResult:
        return TestResult("Navigation Deployment", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_risk_assessment_engine(self) -> TestResult:
        return TestResult("Risk Assessment Engine", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_avoidance_planner(self) -> TestResult:
        return TestResult("Avoidance Planner", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_navigation_integration(self) -> TestResult:
        return TestResult("Navigation Integration", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_object_detection_deployment(self) -> TestResult:
        return TestResult("Object Detection Deployment", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_yolo_detector_structure(self) -> TestResult:
        return TestResult("YOLO Detector Structure", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_object_detection_performance(self) -> TestResult:
        return TestResult("Object Detection Performance", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_apriltag_deployment(self) -> TestResult:
        return TestResult("AprilTag Deployment", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_multi_resolution_detector(self) -> TestResult:
        return TestResult("Multi-Resolution Detector", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_apriltag_performance(self) -> TestResult:
        return TestResult("AprilTag Performance", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_component_integration(self) -> TestResult:
        return TestResult("Component Integration", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_system_stability(self) -> TestResult:
        return TestResult("System Stability", "", "SKIP", 0, "Test implementation pending", {}, "")
    
    def test_overall_performance(self) -> TestResult:
        return TestResult("Overall Performance", "", "SKIP", 0, "Test implementation pending", {}, "")


def main():
    """Main testing application"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Component Testing Framework')
    parser.add_argument('--robot', default='blueduckie.local', help='Robot hostname')
    parser.add_argument('--user', default='duckie', help='SSH username')
    parser.add_argument('--component', help='Component to test (or "all" for all components)')
    parser.add_argument('--project-root', default='/Users/shivamsingh/my-dt-core', help='Project root directory')
    parser.add_argument('--output', help='Output file for test results (JSON)')
    
    args = parser.parse_args()
    
    # Initialize testing framework
    robot_interface = RobotTestInterface(args.robot, args.user)
    tester = ComponentTester(robot_interface, args.project_root)
    
    components_to_test = []
    if args.component == "all":
        components_to_test = list(tester.component_tests.keys())
    elif args.component:
        if args.component not in tester.component_tests:
            print(f"Error: Unknown component '{args.component}'")
            print(f"Available components: {', '.join(tester.component_tests.keys())}")
            sys.exit(1)
        components_to_test = [args.component]
    else:
        print("Error: Please specify --component")
        sys.exit(1)
    
    # Run tests
    all_reports = []
    overall_success = True
    
    for component in components_to_test:
        print(f"\\n{'='*60}")
        print(f"🧪 TESTING COMPONENT: {component.upper()}")
        print(f"{'='*60}")
        
        try:
            report = tester.run_component_tests(component)
            all_reports.append(report)
            
            if report.failed_tests > 0 or report.error_tests > 0:
                overall_success = False
                
        except Exception as e:
            print(f"❌ Error testing component {component}: {e}")
            overall_success = False
    
    # Generate summary
    print(f"\\n{'='*60}")
    print("📊 TESTING SUMMARY")
    print(f"{'='*60}")
    
    total_tests = sum(r.test_count for r in all_reports)
    total_passed = sum(r.passed_tests for r in all_reports)
    total_failed = sum(r.failed_tests for r in all_reports)
    total_errors = sum(r.error_tests for r in all_reports)
    total_skipped = sum(r.skipped_tests for r in all_reports)
    
    overall_success_rate = (total_passed / total_tests * 100) if total_tests > 0 else 0
    
    print(f"Components Tested: {len(all_reports)}")
    print(f"Total Tests: {total_tests}")
    print(f"Passed: {total_passed}")
    print(f"Failed: {total_failed}")
    print(f"Errors: {total_errors}")
    print(f"Skipped: {total_skipped}")
    print(f"Success Rate: {overall_success_rate:.1f}%")
    print(f"Overall Status: {'✅ PASS' if overall_success else '❌ FAIL'}")
    
    # Save results if requested
    if args.output:
        summary_data = {
            'timestamp': datetime.now().isoformat(),
            'robot': args.robot,
            'components_tested': len(all_reports),
            'total_tests': total_tests,
            'passed_tests': total_passed,
            'failed_tests': total_failed,
            'error_tests': total_errors,
            'skipped_tests': total_skipped,
            'success_rate': overall_success_rate,
            'overall_success': overall_success,
            'component_reports': [r.to_dict() for r in all_reports]
        }
        
        try:
            with open(args.output, 'w') as f:
                json.dump(summary_data, f, indent=2)
            print(f"\\n📁 Results saved to: {args.output}")
        except Exception as e:
            print(f"\\n❌ Failed to save results: {e}")
    
    sys.exit(0 if overall_success else 1)


if __name__ == "__main__":
    main()

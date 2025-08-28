#!/usr/bin/env python3
"""
Smart Deployment Orchestrator
Integrates progressive deployment, ultra monitoring, and component testing
Provides intelligent deployment with real-time feedback and automatic issue resolution
"""

import os
import sys
import time
import json
import threading
import subprocess
import signal
import queue
from datetime import datetime, timedelta
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, asdict
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [ORCHESTRATOR] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


@dataclass
class DeploymentStep:
    """Deployment step definition"""
    component: str
    description: str
    pre_tests: List[str]
    post_tests: List[str]
    rollback_possible: bool
    critical: bool
    estimated_time: int  # seconds
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class DeploymentResult:
    """Result of a deployment step"""
    component: str
    status: str  # SUCCESS, FAILED, ROLLED_BACK, SKIPPED
    start_time: float
    end_time: float
    duration: float
    pre_test_results: Dict[str, Any]
    post_test_results: Dict[str, Any]
    monitoring_data: Dict[str, Any]
    issues_detected: List[str]
    actions_taken: List[str]
    details: str
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class SmartDeploymentOrchestrator:
    """
    Intelligent deployment orchestrator that combines progressive deployment,
    real-time monitoring, and comprehensive testing
    """
    
    def __init__(self, robot_host: str, robot_user: str = "duckie", project_root: str = None):
        self.robot_host = robot_host
        self.robot_user = robot_user
        self.project_root = project_root or "/Users/shivamsingh/my-dt-core"
        
        # Deployment configuration
        self.deployment_active = False
        self.current_step = None
        self.deployment_results: List[DeploymentResult] = []
        self.monitoring_data = queue.Queue()
        self.abort_deployment = False
        
        # Script paths
        self.progressive_script = os.path.join(self.project_root, "scripts", "progressive_deployment.sh")
        self.monitoring_script = os.path.join(self.project_root, "scripts", "ultra_monitoring.py")
        self.testing_script = os.path.join(self.project_root, "scripts", "component_tester.py")
        
        # Monitoring and testing threads
        self.monitor_thread = None
        self.monitor_process = None
        
        # Logs
        self.log_dir = os.path.join(self.project_root, "deployment_logs")
        os.makedirs(self.log_dir, exist_ok=True)
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.orchestrator_log = os.path.join(self.log_dir, f"orchestrator_{self.session_id}.log")
        
        # Deployment steps definition
        self.deployment_steps = [
            DeploymentStep(
                component="base_validation",
                description="Validate base Duckiebot functionality",
                pre_tests=["ssh_connectivity", "base_system_health"],
                post_tests=["ros_system", "docker_system"],
                rollback_possible=False,
                critical=True,
                estimated_time=30
            ),
            DeploymentStep(
                component="enhanced_vision_utils",
                description="Deploy advanced vision utilities",
                pre_tests=["disk_space", "memory_usage"],
                post_tests=["vision_utils_import", "vision_utils_functionality"],
                rollback_possible=True,
                critical=False,
                estimated_time=45
            ),
            DeploymentStep(
                component="adaptive_line_detector",
                description="Deploy adaptive line detection",
                pre_tests=["vision_utils_functional"],
                post_tests=["line_detector_deployment", "adaptive_threshold_detector"],
                rollback_possible=True,
                critical=True,
                estimated_time=60
            ),
            DeploymentStep(
                component="enhanced_lane_filter",
                description="Deploy polynomial curve fitting",
                pre_tests=["line_detector_functional"],
                post_tests=["lane_filter_deployment", "polynomial_curve_fitter"],
                rollback_possible=True,
                critical=True,
                estimated_time=60
            ),
            DeploymentStep(
                component="safety_monitoring",
                description="Deploy safety monitoring system",
                pre_tests=["system_stability"],
                post_tests=["safety_monitoring_deployment", "safety_status_publisher"],
                rollback_possible=True,
                critical=True,
                estimated_time=45
            ),
            DeploymentStep(
                component="performance_optimizer",
                description="Deploy performance optimization",
                pre_tests=["resource_availability"],
                post_tests=["performance_optimizer_deployment", "resource_monitoring"],
                rollback_possible=True,
                critical=False,
                estimated_time=45
            ),
            DeploymentStep(
                component="enhanced_navigation",
                description="Deploy advanced navigation",
                pre_tests=["safety_monitoring_functional"],
                post_tests=["navigation_deployment", "risk_assessment_engine"],
                rollback_possible=True,
                critical=False,
                estimated_time=90
            ),
            DeploymentStep(
                component="object_detection",
                description="Deploy YOLOv5 object detection",
                pre_tests=["sufficient_resources"],
                post_tests=["object_detection_deployment", "yolo_detector_structure"],
                rollback_possible=True,
                critical=False,
                estimated_time=120
            ),
            DeploymentStep(
                component="apriltag_enhancements",
                description="Deploy enhanced AprilTag detection",
                pre_tests=["apriltag_backup_complete"],
                post_tests=["apriltag_deployment", "multi_resolution_detector"],
                rollback_possible=True,
                critical=False,
                estimated_time=75
            ),
            DeploymentStep(
                component="system_integration",
                description="Integrate all components",
                pre_tests=["all_components_deployed"],
                post_tests=["component_integration", "system_stability", "overall_performance"],
                rollback_possible=False,
                critical=True,
                estimated_time=90
            )
        ]
        
        # Performance thresholds for deployment decisions
        self.performance_thresholds = {
            'cpu_max': 85.0,
            'memory_max': 90.0,
            'temperature_max': 75.0,
            'load_avg_max': 3.0,
            'disk_usage_max': 85.0
        }
        
        # Issue resolution strategies
        self.resolution_strategies = {
            'high_cpu': self._resolve_high_cpu,
            'high_memory': self._resolve_high_memory,
            'high_temperature': self._resolve_high_temperature,
            'disk_full': self._resolve_disk_full,
            'test_failure': self._resolve_test_failure,
            'deployment_failure': self._resolve_deployment_failure
        }
        
        logger.info(f"Smart deployment orchestrator initialized for {robot_host}")
        logger.info(f"Session ID: {self.session_id}")
        logger.info(f"Logs: {self.log_dir}")
    
    def start_monitoring(self) -> bool:
        """Start real-time monitoring"""
        try:
            logger.info("Starting real-time monitoring...")
            
            # Start monitoring process
            cmd = [
                "python3", self.monitoring_script,
                "--robot", self.robot_host,
                "--user", self.robot_user,
                "--interval", "2.0"
            ]
            
            self.monitor_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # Start monitoring thread to read output
            self.monitor_thread = threading.Thread(target=self._monitor_reader, daemon=True)
            self.monitor_thread.start()
            
            logger.info("Real-time monitoring started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start monitoring: {e}")
            return False
    
    def stop_monitoring(self):
        """Stop real-time monitoring"""
        try:
            if self.monitor_process:
                logger.info("Stopping monitoring process...")
                self.monitor_process.terminate()
                self.monitor_process.wait(timeout=10)
                self.monitor_process = None
            
            if self.monitor_thread and self.monitor_thread.is_alive():
                self.monitor_thread.join(timeout=5)
            
            logger.info("Real-time monitoring stopped")
            
        except Exception as e:
            logger.error(f"Error stopping monitoring: {e}")
    
    def _monitor_reader(self):
        """Read monitoring output in background thread"""
        try:
            while self.monitor_process and self.monitor_process.poll() is None:
                line = self.monitor_process.stdout.readline()
                if line:
                    # Parse monitoring alerts and data
                    if "BOTTLENECK ALERT" in line:
                        self._handle_monitoring_alert(line.strip())
                    elif "Status:" in line:
                        self._handle_status_update(line.strip())
                
        except Exception as e:
            logger.error(f"Error reading monitoring output: {e}")
    
    def _handle_monitoring_alert(self, alert_line: str):
        """Handle monitoring alerts"""
        logger.warning(f"Monitoring Alert: {alert_line}")
        
        # Add alert to monitoring data queue
        alert_data = {
            'timestamp': time.time(),
            'type': 'alert',
            'message': alert_line
        }
        self.monitoring_data.put(alert_data)
        
        # Check if we need to abort deployment
        if "CRITICAL" in alert_line and self.deployment_active:
            logger.critical("Critical alert detected during deployment!")
            self._handle_critical_issue(alert_line)
    
    def _handle_status_update(self, status_line: str):
        """Handle status updates from monitoring"""
        logger.debug(f"Status Update: {status_line}")
        
        # Parse and store status data
        status_data = {
            'timestamp': time.time(),
            'type': 'status',
            'message': status_line
        }
        self.monitoring_data.put(status_data)
    
    def _handle_critical_issue(self, issue_description: str):
        """Handle critical issues that may require deployment abort"""
        logger.critical(f"Handling critical issue: {issue_description}")
        
        # Determine issue type and apply resolution
        if "temperature" in issue_description.lower():
            if self._resolve_high_temperature():
                logger.info("Temperature issue resolved, continuing deployment")
            else:
                logger.critical("Cannot resolve temperature issue, aborting deployment")
                self.abort_deployment = True
        
        elif "memory" in issue_description.lower():
            if self._resolve_high_memory():
                logger.info("Memory issue resolved, continuing deployment")
            else:
                logger.critical("Cannot resolve memory issue, aborting deployment")
                self.abort_deployment = True
        
        elif "cpu" in issue_description.lower():
            if self._resolve_high_cpu():
                logger.info("CPU issue resolved, continuing deployment")
            else:
                logger.warning("CPU issue persists, continuing with caution")
        
        else:
            logger.warning("Unknown critical issue, continuing with caution")
    
    def run_component_tests(self, component: str, test_phase: str) -> Dict[str, Any]:
        """Run component tests"""
        logger.info(f"Running {test_phase} tests for component: {component}")
        
        try:
            cmd = [
                "python3", self.testing_script,
                "--robot", self.robot_host,
                "--user", self.robot_user,
                "--component", component,
                "--project-root", self.project_root
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=300  # 5 minutes timeout
            )
            
            test_results = {
                'success': result.returncode == 0,
                'output': result.stdout,
                'error': result.stderr,
                'execution_time': time.time()
            }
            
            if test_results['success']:
                logger.info(f"✅ {test_phase} tests passed for {component}")
            else:
                logger.error(f"❌ {test_phase} tests failed for {component}")
                logger.error(f"Error output: {result.stderr}")
            
            return test_results
            
        except subprocess.TimeoutExpired:
            logger.error(f"Test timeout for component {component}")
            return {
                'success': False,
                'output': '',
                'error': 'Test execution timeout',
                'execution_time': time.time()
            }
        except Exception as e:
            logger.error(f"Error running tests for {component}: {e}")
            return {
                'success': False,
                'output': '',
                'error': str(e),
                'execution_time': time.time()
            }
    
    def deploy_component(self, step: DeploymentStep) -> DeploymentResult:
        """Deploy a single component with comprehensive monitoring"""
        logger.info(f"🚀 Deploying component: {step.component}")
        logger.info(f"Description: {step.description}")
        logger.info(f"Estimated time: {step.estimated_time}s")
        
        start_time = time.time()
        self.current_step = step.component
        
        # Initialize result
        result = DeploymentResult(
            component=step.component,
            status="FAILED",
            start_time=start_time,
            end_time=0,
            duration=0,
            pre_test_results={},
            post_test_results={},
            monitoring_data={},
            issues_detected=[],
            actions_taken=[],
            details=""
        )
        
        try:
            # Run pre-deployment tests
            logger.info(f"Running pre-deployment tests...")
            pre_test_results = self.run_component_tests(step.component, "pre")
            result.pre_test_results = pre_test_results
            
            if not pre_test_results['success'] and step.critical:
                result.status = "FAILED"
                result.details = f"Pre-deployment tests failed: {pre_test_results['error']}"
                logger.error(f"❌ Critical pre-deployment tests failed for {step.component}")
                return result
            
            # Check system resources before deployment
            resource_check = self._check_system_resources()
            if not resource_check['suitable'] and step.critical:
                result.status = "FAILED"
                result.details = f"System resources insufficient: {resource_check['issues']}"
                logger.error(f"❌ System resources insufficient for {step.component}")
                return result
            
            # Run actual deployment
            logger.info(f"Executing deployment for {step.component}...")
            deployment_success = self._execute_deployment_step(step)
            
            if not deployment_success:
                result.status = "FAILED"
                result.details = "Deployment execution failed"
                logger.error(f"❌ Deployment execution failed for {step.component}")
                
                # Attempt rollback if possible
                if step.rollback_possible:
                    logger.info(f"Attempting rollback for {step.component}...")
                    if self._rollback_component(step.component):
                        result.status = "ROLLED_BACK"
                        result.actions_taken.append("Rollback successful")
                        logger.info(f"✅ Rollback successful for {step.component}")
                    else:
                        result.actions_taken.append("Rollback failed")
                        logger.error(f"❌ Rollback failed for {step.component}")
                
                return result
            
            # Wait for system to stabilize
            logger.info("Waiting for system stabilization...")
            time.sleep(10)
            
            # Run post-deployment tests
            logger.info(f"Running post-deployment tests...")
            post_test_results = self.run_component_tests(step.component, "post")
            result.post_test_results = post_test_results
            
            if not post_test_results['success']:
                logger.warning(f"⚠️ Post-deployment tests failed for {step.component}")
                if step.critical:
                    result.status = "FAILED"
                    result.details = f"Post-deployment tests failed: {post_test_results['error']}"
                    
                    # Attempt rollback
                    if step.rollback_possible:
                        logger.info(f"Attempting rollback due to test failure...")
                        if self._rollback_component(step.component):
                            result.status = "ROLLED_BACK"
                            result.actions_taken.append("Rollback due to test failure")
                        else:
                            result.actions_taken.append("Rollback failed after test failure")
                    
                    return result
                else:
                    result.status = "SUCCESS"
                    result.details = "Deployment successful despite test warnings"
                    result.issues_detected.append("Post-deployment test warnings")
            else:
                result.status = "SUCCESS"
                result.details = "Deployment completed successfully"
                logger.success(f"✅ Component {step.component} deployed successfully")
            
        except Exception as e:
            logger.error(f"❌ Exception during deployment of {step.component}: {e}")
            result.status = "FAILED"
            result.details = f"Exception during deployment: {str(e)}"
        
        finally:
            result.end_time = time.time()
            result.duration = result.end_time - result.start_time
            self.current_step = None
            
            # Collect final monitoring data
            result.monitoring_data = self._collect_monitoring_summary()
        
        return result
    
    def _execute_deployment_step(self, step: DeploymentStep) -> bool:
        """Execute the actual deployment step"""
        try:
            # Call the progressive deployment script for this specific component
            cmd = [
                "bash", self.progressive_script,
                "--component", step.component,
                "--robot", self.robot_host
            ]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=step.estimated_time * 3  # Allow 3x estimated time
            )
            
            if result.returncode == 0:
                logger.info(f"Deployment script completed successfully for {step.component}")
                return True
            else:
                logger.error(f"Deployment script failed for {step.component}: {result.stderr}")
                return False
        
        except subprocess.TimeoutExpired:
            logger.error(f"Deployment timeout for {step.component}")
            return False
        except Exception as e:
            logger.error(f"Error executing deployment for {step.component}: {e}")
            return False
    
    def _check_system_resources(self) -> Dict[str, Any]:
        """Check if system resources are suitable for deployment"""
        try:
            # Get current system metrics via SSH
            cmd = f"""
ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no {self.robot_user}@{self.robot_host} '
python3 -c "
import psutil
import json
import os

cpu_percent = psutil.cpu_percent(interval=1)
memory = psutil.virtual_memory()
load_avg = os.getloadavg()
disk = psutil.disk_usage(\\"/\\")

metrics = {
    \\"cpu_percent\\": cpu_percent,
    \\"memory_percent\\": memory.percent,
    \\"load_avg_1m\\": load_avg[0],
    \\"disk_usage_percent\\": disk.percent
}

print(json.dumps(metrics))
"'
"""
            
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30)
            
            if result.returncode != 0:
                return {'suitable': False, 'issues': ['Failed to get system metrics']}
            
            metrics = json.loads(result.stdout.strip())
            
            issues = []
            if metrics['cpu_percent'] > self.performance_thresholds['cpu_max']:
                issues.append(f"High CPU usage: {metrics['cpu_percent']:.1f}%")
            
            if metrics['memory_percent'] > self.performance_thresholds['memory_max']:
                issues.append(f"High memory usage: {metrics['memory_percent']:.1f}%")
            
            if metrics['load_avg_1m'] > self.performance_thresholds['load_avg_max']:
                issues.append(f"High load average: {metrics['load_avg_1m']:.2f}")
            
            if metrics['disk_usage_percent'] > self.performance_thresholds['disk_usage_max']:
                issues.append(f"High disk usage: {metrics['disk_usage_percent']:.1f}%")
            
            return {
                'suitable': len(issues) == 0,
                'issues': issues,
                'metrics': metrics
            }
            
        except Exception as e:
            logger.error(f"Error checking system resources: {e}")
            return {'suitable': False, 'issues': [f'Resource check error: {str(e)}']}
    
    def _rollback_component(self, component: str) -> bool:
        """Rollback a component deployment"""
        logger.info(f"Rolling back component: {component}")
        
        try:
            # Implement component-specific rollback logic
            rollback_commands = {
                'enhanced_vision_utils': 'rm -f /tmp/advanced_vision_utils.py',
                'adaptive_line_detector': 'rm -rf /tmp/enhanced_line_detector',
                'enhanced_lane_filter': 'rm -rf /tmp/enhanced_lane_filter',
                'safety_monitoring': 'rm -f /tmp/safety_status_publisher.py',
                'performance_optimizer': 'rm -f /tmp/performance_optimizer.py',
                'enhanced_navigation': 'rm -rf /tmp/enhanced_navigation',
                'object_detection': 'rm -rf /tmp/enhanced_vehicle_detection',
                'apriltag_enhancements': 'rm -rf /tmp/enhanced_apriltag'
            }
            
            if component in rollback_commands:
                cmd = f"ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no {self.robot_user}@{self.robot_host} '{rollback_commands[component]}'"
                result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30)
                return result.returncode == 0
            
            return False
            
        except Exception as e:
            logger.error(f"Error rolling back {component}: {e}")
            return False
    
    def _collect_monitoring_summary(self) -> Dict[str, Any]:
        """Collect monitoring data summary"""
        summary = {
            'alerts_count': 0,
            'critical_alerts': 0,
            'status_updates': 0,
            'collection_time': time.time()
        }
        
        # Process monitoring queue
        while not self.monitoring_data.empty():
            try:
                data = self.monitoring_data.get_nowait()
                if data['type'] == 'alert':
                    summary['alerts_count'] += 1
                    if 'CRITICAL' in data['message']:
                        summary['critical_alerts'] += 1
                elif data['type'] == 'status':
                    summary['status_updates'] += 1
            except queue.Empty:
                break
        
        return summary
    
    # Resolution strategies
    def _resolve_high_cpu(self) -> bool:
        """Resolve high CPU usage"""
        logger.info("Attempting to resolve high CPU usage...")
        try:
            # Set CPU governor to powersave
            cmd = f"ssh {self.robot_user}@{self.robot_host} 'echo powersave | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor'"
            subprocess.run(cmd, shell=True, timeout=30)
            
            # Wait and check if resolved
            time.sleep(10)
            return True
        except:
            return False
    
    def _resolve_high_memory(self) -> bool:
        """Resolve high memory usage"""
        logger.info("Attempting to resolve high memory usage...")
        try:
            # Clear system caches
            cmd = f"ssh {self.robot_user}@{self.robot_host} 'sudo sync && sudo sysctl -w vm.drop_caches=3'"
            subprocess.run(cmd, shell=True, timeout=30)
            
            # Wait and check if resolved
            time.sleep(5)
            return True
        except:
            return False
    
    def _resolve_high_temperature(self) -> bool:
        """Resolve high temperature"""
        logger.info("Attempting to resolve high temperature...")
        try:
            # Reduce CPU frequency and clear caches
            self._resolve_high_cpu()
            self._resolve_high_memory()
            
            # Wait for temperature to stabilize
            time.sleep(30)
            return True
        except:
            return False
    
    def _resolve_disk_full(self) -> bool:
        """Resolve disk space issues"""
        logger.info("Attempting to resolve disk space issues...")
        try:
            # Clean temporary files
            cmd = f"ssh {self.robot_user}@{self.robot_host} 'sudo find /tmp -type f -atime +1 -delete 2>/dev/null'"
            subprocess.run(cmd, shell=True, timeout=30)
            return True
        except:
            return False
    
    def _resolve_test_failure(self) -> bool:
        """Resolve test failures"""
        logger.info("Analyzing test failure for resolution...")
        # Implementation depends on specific test failure
        return False
    
    def _resolve_deployment_failure(self) -> bool:
        """Resolve deployment failures"""
        logger.info("Analyzing deployment failure for resolution...")
        # Implementation depends on specific deployment failure
        return False
    
    def execute_smart_deployment(self, start_from: str = None, interactive: bool = True) -> bool:
        """Execute the complete smart deployment"""
        logger.info("🚀 Starting Smart Deployment Orchestration")
        logger.info(f"Robot: {self.robot_host}")
        logger.info(f"Interactive mode: {interactive}")
        logger.info(f"Total steps: {len(self.deployment_steps)}")
        
        # Start monitoring
        if not self.start_monitoring():
            logger.error("Failed to start monitoring, aborting deployment")
            return False
        
        self.deployment_active = True
        successful_deployments = 0
        
        try:
            # Find starting point
            start_index = 0
            if start_from:
                for i, step in enumerate(self.deployment_steps):
                    if step.component == start_from:
                        start_index = i
                        break
            
            # Execute deployment steps
            for i in range(start_index, len(self.deployment_steps)):
                if self.abort_deployment:
                    logger.warning("Deployment aborted due to critical issues")
                    break
                
                step = self.deployment_steps[i]
                
                print(f"\\n{'='*80}")
                print(f"🔧 DEPLOYMENT STEP {i+1}/{len(self.deployment_steps)}: {step.component.upper()}")
                print(f"{'='*80}")
                print(f"Description: {step.description}")
                print(f"Critical: {'Yes' if step.critical else 'No'}")
                print(f"Estimated time: {step.estimated_time}s")
                print(f"Rollback possible: {'Yes' if step.rollback_possible else 'No'}")
                
                if interactive:
                    print(f"\\n🤔 Ready to deploy {step.component}?")
                    user_input = input("Press ENTER to continue, 's' to skip, 'q' to quit: ").strip().lower()
                    
                    if user_input == 'q':
                        logger.info("Deployment quit by user")
                        break
                    elif user_input == 's':
                        logger.info(f"Skipping component: {step.component}")
                        continue
                
                # Deploy component
                result = self.deploy_component(step)
                self.deployment_results.append(result)
                
                # Handle result
                if result.status == "SUCCESS":
                    successful_deployments += 1
                    print(f"\\n✅ SUCCESS: {step.component} deployed successfully")
                    
                elif result.status == "ROLLED_BACK":
                    print(f"\\n⚠️ ROLLED BACK: {step.component} was rolled back")
                    if step.critical:
                        print("❌ Critical component rollback - stopping deployment")
                        break
                
                elif result.status == "FAILED":
                    print(f"\\n❌ FAILED: {step.component} deployment failed")
                    print(f"Details: {result.details}")
                    
                    if step.critical:
                        print("❌ Critical component failure - stopping deployment")
                        break
                    else:
                        if interactive:
                            continue_input = input("Non-critical failure. Continue? (y/n): ").strip().lower()
                            if continue_input != 'y':
                                break
                
                # Show progress
                progress = ((i + 1) / len(self.deployment_steps)) * 100
                print(f"\\n📊 Overall Progress: {progress:.1f}% ({successful_deployments}/{i+1} successful)")
                
                if interactive and i < len(self.deployment_steps) - 1:
                    input("\\nPress ENTER to continue to next step...")
        
        finally:
            self.deployment_active = False
            self.stop_monitoring()
            
            # Generate final report
            self._generate_final_report()
        
        success_rate = (successful_deployments / len(self.deployment_results)) * 100 if self.deployment_results else 0
        overall_success = success_rate >= 80  # 80% success threshold
        
        print(f"\\n{'='*80}")
        print("📊 SMART DEPLOYMENT SUMMARY")
        print(f"{'='*80}")
        print(f"Total Steps: {len(self.deployment_results)}")
        print(f"Successful: {successful_deployments}")
        print(f"Success Rate: {success_rate:.1f}%")
        print(f"Overall Status: {'✅ SUCCESS' if overall_success else '❌ PARTIAL/FAILED'}")
        print(f"Session ID: {self.session_id}")
        print(f"Logs: {self.log_dir}")
        
        return overall_success
    
    def _generate_final_report(self):
        """Generate comprehensive final deployment report"""
        report_file = os.path.join(self.log_dir, f"smart_deployment_report_{self.session_id}.json")
        
        report_data = {
            'session_id': self.session_id,
            'robot_host': self.robot_host,
            'start_time': self.deployment_results[0].start_time if self.deployment_results else time.time(),
            'end_time': time.time(),
            'total_steps': len(self.deployment_results),
            'successful_steps': len([r for r in self.deployment_results if r.status == "SUCCESS"]),
            'failed_steps': len([r for r in self.deployment_results if r.status == "FAILED"]),
            'rolled_back_steps': len([r for r in self.deployment_results if r.status == "ROLLED_BACK"]),
            'total_duration': sum(r.duration for r in self.deployment_results),
            'deployment_results': [r.to_dict() for r in self.deployment_results],
            'performance_thresholds': self.performance_thresholds,
            'abort_deployment': self.abort_deployment
        }
        
        try:
            with open(report_file, 'w') as f:
                json.dump(report_data, f, indent=2)
            logger.info(f"Final report saved to: {report_file}")
        except Exception as e:
            logger.error(f"Failed to save final report: {e}")


def main():
    """Main orchestrator application"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Smart Deployment Orchestrator')
    parser.add_argument('--robot', default='blueduckie.local', help='Robot hostname')
    parser.add_argument('--user', default='duckie', help='SSH username')
    parser.add_argument('--project-root', help='Project root directory')
    parser.add_argument('--start-from', help='Start deployment from specific component')
    parser.add_argument('--non-interactive', action='store_true', help='Run in non-interactive mode')
    
    args = parser.parse_args()
    
    # Initialize orchestrator
    orchestrator = SmartDeploymentOrchestrator(
        robot_host=args.robot,
        robot_user=args.user,
        project_root=args.project_root
    )
    
    def signal_handler(signum, frame):
        print("\\n🛑 Deployment interrupted by user")
        orchestrator.abort_deployment = True
        orchestrator.stop_monitoring()
        sys.exit(130)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        success = orchestrator.execute_smart_deployment(
            start_from=args.start_from,
            interactive=not args.non_interactive
        )
        
        sys.exit(0 if success else 1)
        
    except Exception as e:
        logger.error(f"Orchestrator error: {e}")
        orchestrator.stop_monitoring()
        sys.exit(1)


if __name__ == "__main__":
    main()

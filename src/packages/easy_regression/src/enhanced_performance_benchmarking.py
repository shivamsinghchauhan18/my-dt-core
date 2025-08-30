#!/usr/bin/env python3
"""
Enhanced Performance Benchmarking for Advanced Autonomous Duckietown System

This module extends the existing easy_regression framework with advanced performance
benchmarking capabilities including timing requirements, resource monitoring,
and comprehensive test data generation.
"""

import time
import psutil
import numpy as np
import json
import os
import sys
from datetime import datetime
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, asdict
from pathlib import Path
import threading
import queue
import logging

# Configure logging for comprehensive test execution monitoring
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [BENCHMARK] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


@dataclass
class PerformanceMetrics:
    """Performance metrics data structure."""
    test_name: str
    execution_time: float
    cpu_usage_percent: float
    memory_usage_mb: float
    peak_memory_mb: float
    fps: Optional[float] = None
    latency_ms: Optional[float] = None
    throughput: Optional[float] = None
    success_rate: float = 1.0
    error_count: int = 0
    timestamp: str = ""
    
    def __post_init__(self):
        if not self.timestamp:
            self.timestamp = datetime.now().isoformat()


@dataclass
class TimingRequirement:
    """Timing requirement specification."""
    test_name: str
    max_execution_time: float
    max_latency_ms: Optional[float] = None
    min_fps: Optional[float] = None
    max_cpu_percent: Optional[float] = None
    max_memory_mb: Optional[float] = None


class ResourceMonitor:
    """Real-time resource monitoring during test execution."""
    
    def __init__(self, sampling_interval: float = 0.1):
        self.sampling_interval = sampling_interval
        self.monitoring = False
        self.metrics_queue = queue.Queue()
        self.monitor_thread = None
        
        logger.info(f"ResourceMonitor initialized with {sampling_interval}s sampling interval")
    
    def start_monitoring(self):
        """Start resource monitoring in background thread."""
        if self.monitoring:
            logger.warning("Resource monitoring already active")
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_resources)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        logger.info("Resource monitoring started")
    
    def stop_monitoring(self) -> Dict[str, float]:
        """Stop monitoring and return aggregated metrics."""
        if not self.monitoring:
            logger.warning("Resource monitoring not active")
            return {}
        
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        
        # Collect all metrics from queue
        cpu_samples = []
        memory_samples = []
        
        while not self.metrics_queue.empty():
            try:
                sample = self.metrics_queue.get_nowait()
                cpu_samples.append(sample['cpu_percent'])
                memory_samples.append(sample['memory_mb'])
            except queue.Empty:
                break
        
        if not cpu_samples:
            logger.warning("No resource samples collected")
            return {}
        
        metrics = {
            'avg_cpu_percent': np.mean(cpu_samples),
            'max_cpu_percent': np.max(cpu_samples),
            'avg_memory_mb': np.mean(memory_samples),
            'peak_memory_mb': np.max(memory_samples),
            'sample_count': len(cpu_samples)
        }
        
        logger.info(f"Resource monitoring stopped. Collected {len(cpu_samples)} samples")
        logger.info(f"CPU: avg={metrics['avg_cpu_percent']:.1f}%, max={metrics['max_cpu_percent']:.1f}%")
        logger.info(f"Memory: avg={metrics['avg_memory_mb']:.1f}MB, peak={metrics['peak_memory_mb']:.1f}MB")
        
        return metrics
    
    def _monitor_resources(self):
        """Background thread function for resource monitoring."""
        process = psutil.Process()
        
        while self.monitoring:
            try:
                cpu_percent = process.cpu_percent()
                memory_info = process.memory_info()
                memory_mb = memory_info.rss / 1024 / 1024
                
                sample = {
                    'timestamp': time.time(),
                    'cpu_percent': cpu_percent,
                    'memory_mb': memory_mb
                }
                
                self.metrics_queue.put(sample)
                time.sleep(self.sampling_interval)
                
            except Exception as e:
                logger.error(f"Error in resource monitoring: {e}")
                break


class EnhancedPerformanceBenchmark:
    """Enhanced performance benchmarking with timing requirements and resource monitoring."""
    
    def __init__(self, output_dir: str = "benchmark_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        self.timing_requirements: Dict[str, TimingRequirement] = {}
        self.benchmark_results: List[PerformanceMetrics] = []
        self.resource_monitor = ResourceMonitor()
        
        logger.info(f"EnhancedPerformanceBenchmark initialized with output dir: {self.output_dir}")
    
    def add_timing_requirement(self, requirement: TimingRequirement):
        """Add timing requirement for a test."""
        self.timing_requirements[requirement.test_name] = requirement
        logger.info(f"Added timing requirement for {requirement.test_name}: "
                   f"max_time={requirement.max_execution_time}s")
    
    def benchmark_function(self, test_name: str, func, *args, **kwargs) -> PerformanceMetrics:
        """Benchmark a function with comprehensive performance monitoring."""
        logger.info(f"Starting benchmark for {test_name}")
        
        # Start resource monitoring
        self.resource_monitor.start_monitoring()
        
        # Record start time and initial memory
        start_time = time.time()
        start_memory = psutil.Process().memory_info().rss / 1024 / 1024
        
        error_count = 0
        result = None
        
        try:
            # Execute function
            result = func(*args, **kwargs)
            logger.info(f"Function {test_name} executed successfully")
            
        except Exception as e:
            error_count = 1
            logger.error(f"Function {test_name} failed with error: {e}")
        
        # Record end time
        end_time = time.time()
        execution_time = end_time - start_time
        
        # Stop resource monitoring and get metrics
        resource_metrics = self.resource_monitor.stop_monitoring()
        
        # Calculate performance metrics
        metrics = PerformanceMetrics(
            test_name=test_name,
            execution_time=execution_time,
            cpu_usage_percent=resource_metrics.get('avg_cpu_percent', 0.0),
            memory_usage_mb=resource_metrics.get('avg_memory_mb', start_memory),
            peak_memory_mb=resource_metrics.get('peak_memory_mb', start_memory),
            success_rate=1.0 - error_count,
            error_count=error_count
        )
        
        # Calculate additional metrics if applicable
        if hasattr(result, '__len__') and execution_time > 0:
            metrics.throughput = len(result) / execution_time
        
        if execution_time > 0:
            metrics.fps = 1.0 / execution_time
            metrics.latency_ms = execution_time * 1000
        
        self.benchmark_results.append(metrics)
        
        logger.info(f"Benchmark completed for {test_name}:")
        logger.info(f"  Execution time: {execution_time:.3f}s")
        logger.info(f"  CPU usage: {metrics.cpu_usage_percent:.1f}%")
        logger.info(f"  Memory usage: {metrics.memory_usage_mb:.1f}MB")
        logger.info(f"  Success rate: {metrics.success_rate:.2%}")
        
        # Check timing requirements
        self._check_timing_requirements(metrics)
        
        return metrics
    
    def benchmark_pipeline(self, test_name: str, pipeline_steps: List[Tuple[str, callable, tuple, dict]]) -> PerformanceMetrics:
        """Benchmark a multi-step pipeline with detailed step timing."""
        logger.info(f"Starting pipeline benchmark for {test_name} with {len(pipeline_steps)} steps")
        
        self.resource_monitor.start_monitoring()
        start_time = time.time()
        
        step_times = []
        total_errors = 0
        pipeline_result = None
        
        for i, (step_name, func, args, kwargs) in enumerate(pipeline_steps):
            step_start = time.time()
            
            try:
                logger.info(f"  Executing step {i+1}/{len(pipeline_steps)}: {step_name}")
                step_result = func(*args, **kwargs)
                step_time = time.time() - step_start
                step_times.append((step_name, step_time))
                
                logger.info(f"  Step {step_name} completed in {step_time:.3f}s")
                
                # Pass result to next step if applicable
                if i < len(pipeline_steps) - 1:
                    next_step = pipeline_steps[i + 1]
                    if len(next_step[2]) == 0:  # No args, use previous result
                        pipeline_steps[i + 1] = (next_step[0], next_step[1], (step_result,), next_step[3])
                
                pipeline_result = step_result
                
            except Exception as e:
                total_errors += 1
                step_time = time.time() - step_start
                step_times.append((step_name, step_time))
                logger.error(f"  Step {step_name} failed after {step_time:.3f}s: {e}")
        
        total_time = time.time() - start_time
        resource_metrics = self.resource_monitor.stop_monitoring()
        
        # Log step timing breakdown
        logger.info(f"Pipeline {test_name} step timing breakdown:")
        for step_name, step_time in step_times:
            percentage = (step_time / total_time) * 100
            logger.info(f"  {step_name}: {step_time:.3f}s ({percentage:.1f}%)")
        
        metrics = PerformanceMetrics(
            test_name=test_name,
            execution_time=total_time,
            cpu_usage_percent=resource_metrics.get('avg_cpu_percent', 0.0),
            memory_usage_mb=resource_metrics.get('avg_memory_mb', 0.0),
            peak_memory_mb=resource_metrics.get('peak_memory_mb', 0.0),
            success_rate=1.0 - (total_errors / len(pipeline_steps)),
            error_count=total_errors
        )
        
        if total_time > 0:
            metrics.fps = 1.0 / total_time
            metrics.latency_ms = total_time * 1000
        
        self.benchmark_results.append(metrics)
        
        logger.info(f"Pipeline benchmark completed for {test_name}:")
        logger.info(f"  Total time: {total_time:.3f}s")
        logger.info(f"  Success rate: {metrics.success_rate:.2%}")
        
        self._check_timing_requirements(metrics)
        
        return metrics
    
    def _check_timing_requirements(self, metrics: PerformanceMetrics):
        """Check if metrics meet timing requirements."""
        requirement = self.timing_requirements.get(metrics.test_name)
        if not requirement:
            return
        
        logger.info(f"Checking timing requirements for {metrics.test_name}")
        
        violations = []
        
        if metrics.execution_time > requirement.max_execution_time:
            violations.append(f"Execution time {metrics.execution_time:.3f}s > {requirement.max_execution_time}s")
        
        if requirement.max_latency_ms and metrics.latency_ms and metrics.latency_ms > requirement.max_latency_ms:
            violations.append(f"Latency {metrics.latency_ms:.1f}ms > {requirement.max_latency_ms}ms")
        
        if requirement.min_fps and metrics.fps and metrics.fps < requirement.min_fps:
            violations.append(f"FPS {metrics.fps:.1f} < {requirement.min_fps}")
        
        if requirement.max_cpu_percent and metrics.cpu_usage_percent > requirement.max_cpu_percent:
            violations.append(f"CPU usage {metrics.cpu_usage_percent:.1f}% > {requirement.max_cpu_percent}%")
        
        if requirement.max_memory_mb and metrics.peak_memory_mb > requirement.max_memory_mb:
            violations.append(f"Memory usage {metrics.peak_memory_mb:.1f}MB > {requirement.max_memory_mb}MB")
        
        if violations:
            logger.warning(f"Timing requirement violations for {metrics.test_name}:")
            for violation in violations:
                logger.warning(f"  - {violation}")
        else:
            logger.info(f"All timing requirements met for {metrics.test_name}")
    
    def generate_performance_report(self) -> Dict[str, Any]:
        """Generate comprehensive performance report."""
        if not self.benchmark_results:
            logger.warning("No benchmark results available for report generation")
            return {}
        
        logger.info("Generating comprehensive performance report")
        
        # Calculate aggregate statistics
        execution_times = [m.execution_time for m in self.benchmark_results]
        cpu_usages = [m.cpu_usage_percent for m in self.benchmark_results]
        memory_usages = [m.peak_memory_mb for m in self.benchmark_results]
        success_rates = [m.success_rate for m in self.benchmark_results]
        
        report = {
            'summary': {
                'total_tests': len(self.benchmark_results),
                'avg_execution_time': np.mean(execution_times),
                'max_execution_time': np.max(execution_times),
                'avg_cpu_usage': np.mean(cpu_usages),
                'max_cpu_usage': np.max(cpu_usages),
                'avg_memory_usage': np.mean(memory_usages),
                'peak_memory_usage': np.max(memory_usages),
                'overall_success_rate': np.mean(success_rates),
                'total_errors': sum(m.error_count for m in self.benchmark_results)
            },
            'detailed_results': [asdict(m) for m in self.benchmark_results],
            'timing_requirements': {name: asdict(req) for name, req in self.timing_requirements.items()},
            'requirement_violations': self._get_requirement_violations(),
            'generated_at': datetime.now().isoformat()
        }
        
        # Save report to file
        report_file = self.output_dir / f"performance_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        logger.info(f"Performance report saved to {report_file}")
        logger.info(f"Report summary:")
        logger.info(f"  Total tests: {report['summary']['total_tests']}")
        logger.info(f"  Average execution time: {report['summary']['avg_execution_time']:.3f}s")
        logger.info(f"  Overall success rate: {report['summary']['overall_success_rate']:.2%}")
        logger.info(f"  Total errors: {report['summary']['total_errors']}")
        
        return report
    
    def _get_requirement_violations(self) -> List[Dict[str, Any]]:
        """Get list of timing requirement violations."""
        violations = []
        
        for metrics in self.benchmark_results:
            requirement = self.timing_requirements.get(metrics.test_name)
            if not requirement:
                continue
            
            test_violations = []
            
            if metrics.execution_time > requirement.max_execution_time:
                test_violations.append({
                    'type': 'execution_time',
                    'actual': metrics.execution_time,
                    'required': requirement.max_execution_time
                })
            
            if requirement.max_latency_ms and metrics.latency_ms and metrics.latency_ms > requirement.max_latency_ms:
                test_violations.append({
                    'type': 'latency',
                    'actual': metrics.latency_ms,
                    'required': requirement.max_latency_ms
                })
            
            if requirement.min_fps and metrics.fps and metrics.fps < requirement.min_fps:
                test_violations.append({
                    'type': 'fps',
                    'actual': metrics.fps,
                    'required': requirement.min_fps
                })
            
            if test_violations:
                violations.append({
                    'test_name': metrics.test_name,
                    'violations': test_violations
                })
        
        return violations


class TestDataGenerator:
    """Enhanced test data generation for consistent testing scenarios."""
    
    def __init__(self, output_dir: str = "test_data"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        logger.info(f"TestDataGenerator initialized with output dir: {self.output_dir}")
    
    def generate_synthetic_camera_data(self, num_frames: int = 100, width: int = 640, height: int = 480) -> List[np.ndarray]:
        """Generate synthetic camera frames for testing."""
        logger.info(f"Generating {num_frames} synthetic camera frames ({width}x{height})")
        
        frames = []
        for i in range(num_frames):
            # Create frame with varying patterns
            frame = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
            
            # Add some structured elements (lanes, objects)
            if i % 10 == 0:  # Every 10th frame has lane markings
                cv2.line(frame, (width//4, height), (width//4, height//2), (255, 255, 255), 3)
                cv2.line(frame, (3*width//4, height), (3*width//4, height//2), (255, 255, 255), 3)
            
            if i % 20 == 0:  # Every 20th frame has objects
                cv2.rectangle(frame, (width//2-20, height//2-20), (width//2+20, height//2+20), (0, 255, 0), -1)
            
            frames.append(frame)
        
        # Save sample frames
        sample_dir = self.output_dir / "synthetic_camera_frames"
        sample_dir.mkdir(exist_ok=True)
        
        for i in [0, num_frames//4, num_frames//2, 3*num_frames//4, num_frames-1]:
            cv2.imwrite(str(sample_dir / f"frame_{i:04d}.jpg"), frames[i])
        
        logger.info(f"Generated {len(frames)} synthetic camera frames")
        logger.info(f"Sample frames saved to {sample_dir}")
        
        return frames
    
    def generate_apriltag_scenarios(self, num_scenarios: int = 50) -> List[Dict[str, Any]]:
        """Generate AprilTag detection test scenarios."""
        logger.info(f"Generating {num_scenarios} AprilTag test scenarios")
        
        scenarios = []
        for i in range(num_scenarios):
            scenario = {
                'scenario_id': i,
                'tag_id': np.random.randint(1, 100),
                'distance': np.random.uniform(0.3, 3.0),  # 30cm to 3m
                'angle': np.random.uniform(-45, 45),  # -45 to +45 degrees
                'lighting': np.random.choice(['bright', 'normal', 'dim']),
                'occlusion': np.random.uniform(0.0, 0.3),  # 0-30% occlusion
                'noise_level': np.random.uniform(0.0, 0.1),  # 0-10% noise
                'expected_detection': True if np.random.random() > 0.1 else False
            }
            scenarios.append(scenario)
        
        # Save scenarios to file
        scenarios_file = self.output_dir / "apriltag_scenarios.json"
        with open(scenarios_file, 'w') as f:
            json.dump(scenarios, f, indent=2)
        
        logger.info(f"Generated {len(scenarios)} AprilTag scenarios")
        logger.info(f"Scenarios saved to {scenarios_file}")
        
        return scenarios
    
    def generate_lane_detection_scenarios(self, num_scenarios: int = 50) -> List[Dict[str, Any]]:
        """Generate lane detection test scenarios."""
        logger.info(f"Generating {num_scenarios} lane detection test scenarios")
        
        scenarios = []
        for i in range(num_scenarios):
            scenario = {
                'scenario_id': i,
                'lane_type': np.random.choice(['straight', 'curved_left', 'curved_right', 'intersection']),
                'lane_width': np.random.uniform(0.3, 0.6),  # 30-60cm
                'marking_quality': np.random.choice(['good', 'faded', 'broken']),
                'lighting': np.random.choice(['bright', 'normal', 'dim', 'shadows']),
                'weather': np.random.choice(['clear', 'overcast', 'light_rain']),
                'surface': np.random.choice(['smooth', 'textured', 'worn']),
                'curvature': np.random.uniform(-0.1, 0.1) if 'curved' in scenario.get('lane_type', '') else 0.0,
                'expected_detection_confidence': np.random.uniform(0.6, 0.95)
            }
            scenarios.append(scenario)
        
        # Save scenarios to file
        scenarios_file = self.output_dir / "lane_detection_scenarios.json"
        with open(scenarios_file, 'w') as f:
            json.dump(scenarios, f, indent=2)
        
        logger.info(f"Generated {len(scenarios)} lane detection scenarios")
        logger.info(f"Scenarios saved to {scenarios_file}")
        
        return scenarios
    
    def generate_object_detection_scenarios(self, num_scenarios: int = 50) -> List[Dict[str, Any]]:
        """Generate object detection test scenarios."""
        logger.info(f"Generating {num_scenarios} object detection test scenarios")
        
        scenarios = []
        object_types = ['duckiebot', 'duckie', 'cone', 'sign', 'barrier']
        
        for i in range(num_scenarios):
            num_objects = np.random.randint(0, 5)  # 0-4 objects per scenario
            objects = []
            
            for j in range(num_objects):
                obj = {
                    'object_id': j,
                    'type': np.random.choice(object_types),
                    'distance': np.random.uniform(0.5, 5.0),
                    'lateral_offset': np.random.uniform(-1.0, 1.0),
                    'size': np.random.uniform(0.1, 0.5),
                    'occlusion': np.random.uniform(0.0, 0.4),
                    'expected_confidence': np.random.uniform(0.6, 0.95)
                }
                objects.append(obj)
            
            scenario = {
                'scenario_id': i,
                'objects': objects,
                'lighting': np.random.choice(['bright', 'normal', 'dim']),
                'background_complexity': np.random.choice(['simple', 'moderate', 'complex']),
                'motion_blur': np.random.uniform(0.0, 0.2),
                'expected_detection_count': len(objects)
            }
            scenarios.append(scenario)
        
        # Save scenarios to file
        scenarios_file = self.output_dir / "object_detection_scenarios.json"
        with open(scenarios_file, 'w') as f:
            json.dump(scenarios, f, indent=2)
        
        logger.info(f"Generated {len(scenarios)} object detection scenarios")
        logger.info(f"Scenarios saved to {scenarios_file}")
        
        return scenarios


def create_default_timing_requirements() -> List[TimingRequirement]:
    """Create default timing requirements for advanced autonomous system components."""
    logger.info("Creating default timing requirements for system components")
    
    requirements = [
        # Lane detection requirements
        TimingRequirement(
            test_name="lane_detection_pipeline",
            max_execution_time=0.05,  # 50ms for 20 FPS
            max_latency_ms=50,
            min_fps=20,
            max_cpu_percent=30,
            max_memory_mb=500
        ),
        
        # AprilTag detection requirements
        TimingRequirement(
            test_name="apriltag_detection_pipeline",
            max_execution_time=0.1,  # 100ms
            max_latency_ms=100,
            min_fps=10,
            max_cpu_percent=40,
            max_memory_mb=300
        ),
        
        # Object detection requirements
        TimingRequirement(
            test_name="yolo_object_detection",
            max_execution_time=0.1,  # 100ms
            max_latency_ms=100,
            min_fps=10,
            max_cpu_percent=60,
            max_memory_mb=1000
        ),
        
        # Lane control requirements
        TimingRequirement(
            test_name="mpc_lane_control",
            max_execution_time=0.02,  # 20ms for 50 Hz control
            max_latency_ms=20,
            min_fps=50,
            max_cpu_percent=25,
            max_memory_mb=200
        ),
        
        # Safety monitoring requirements
        TimingRequirement(
            test_name="safety_monitoring",
            max_execution_time=0.01,  # 10ms for 100 Hz monitoring
            max_latency_ms=10,
            min_fps=100,
            max_cpu_percent=15,
            max_memory_mb=100
        ),
        
        # End-to-end pipeline requirements
        TimingRequirement(
            test_name="end_to_end_pipeline",
            max_execution_time=0.2,  # 200ms total latency
            max_latency_ms=200,
            min_fps=5,
            max_cpu_percent=80,
            max_memory_mb=2000
        )
    ]
    
    logger.info(f"Created {len(requirements)} default timing requirements")
    return requirements


if __name__ == "__main__":
    # Example usage and testing
    logger.info("Testing Enhanced Performance Benchmarking")
    
    # Create benchmark instance
    benchmark = EnhancedPerformanceBenchmark()
    
    # Add timing requirements
    for req in create_default_timing_requirements():
        benchmark.add_timing_requirement(req)
    
    # Example benchmark function
    def example_processing_function(data_size: int = 1000):
        """Example function to benchmark."""
        data = np.random.random((data_size, data_size))
        result = np.fft.fft2(data)
        return result
    
    # Run benchmark
    metrics = benchmark.benchmark_function("example_processing", example_processing_function, 500)
    
    # Generate test data
    data_generator = TestDataGenerator()
    camera_frames = data_generator.generate_synthetic_camera_data(10)
    apriltag_scenarios = data_generator.generate_apriltag_scenarios(10)
    
    # Generate performance report
    report = benchmark.generate_performance_report()
    
    logger.info("Enhanced Performance Benchmarking test completed successfully")
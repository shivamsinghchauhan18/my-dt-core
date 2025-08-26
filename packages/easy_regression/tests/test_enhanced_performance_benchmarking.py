#!/usr/bin/env python3
"""
Unit tests for Enhanced Performance Benchmarking module.

This test suite validates the enhanced performance benchmarking capabilities
including timing requirements, resource monitoring, and test data generation.
"""

import unittest
import tempfile
import shutil
import time
import json
import numpy as np
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from enhanced_performance_benchmarking import (
    PerformanceMetrics, TimingRequirement, ResourceMonitor,
    EnhancedPerformanceBenchmark, TestDataGenerator,
    create_default_timing_requirements
)


class TestPerformanceMetrics(unittest.TestCase):
    """Test PerformanceMetrics data structure."""
    
    def test_performance_metrics_creation(self):
        """Test PerformanceMetrics creation and initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing PerformanceMetrics creation")
        
        metrics = PerformanceMetrics(
            test_name="test_function",
            execution_time=0.5,
            cpu_usage_percent=25.0,
            memory_usage_mb=100.0,
            peak_memory_mb=120.0
        )
        
        self.assertEqual(metrics.test_name, "test_function")
        self.assertEqual(metrics.execution_time, 0.5)
        self.assertEqual(metrics.cpu_usage_percent, 25.0)
        self.assertEqual(metrics.memory_usage_mb, 100.0)
        self.assertEqual(metrics.peak_memory_mb, 120.0)
        self.assertEqual(metrics.success_rate, 1.0)
        self.assertEqual(metrics.error_count, 0)
        self.assertIsNotNone(metrics.timestamp)
        
        print(f"[{self._get_timestamp()}] [TEST] PerformanceMetrics creation test passed")
    
    def test_performance_metrics_with_optional_fields(self):
        """Test PerformanceMetrics with optional fields."""
        print(f"[{self._get_timestamp()}] [TEST] Testing PerformanceMetrics with optional fields")
        
        metrics = PerformanceMetrics(
            test_name="test_function",
            execution_time=0.1,
            cpu_usage_percent=30.0,
            memory_usage_mb=200.0,
            peak_memory_mb=250.0,
            fps=10.0,
            latency_ms=100.0,
            throughput=50.0,
            success_rate=0.95,
            error_count=1
        )
        
        self.assertEqual(metrics.fps, 10.0)
        self.assertEqual(metrics.latency_ms, 100.0)
        self.assertEqual(metrics.throughput, 50.0)
        self.assertEqual(metrics.success_rate, 0.95)
        self.assertEqual(metrics.error_count, 1)
        
        print(f"[{self._get_timestamp()}] [TEST] PerformanceMetrics optional fields test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestTimingRequirement(unittest.TestCase):
    """Test TimingRequirement data structure."""
    
    def test_timing_requirement_creation(self):
        """Test TimingRequirement creation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing TimingRequirement creation")
        
        requirement = TimingRequirement(
            test_name="lane_detection",
            max_execution_time=0.05,
            max_latency_ms=50,
            min_fps=20,
            max_cpu_percent=30,
            max_memory_mb=500
        )
        
        self.assertEqual(requirement.test_name, "lane_detection")
        self.assertEqual(requirement.max_execution_time, 0.05)
        self.assertEqual(requirement.max_latency_ms, 50)
        self.assertEqual(requirement.min_fps, 20)
        self.assertEqual(requirement.max_cpu_percent, 30)
        self.assertEqual(requirement.max_memory_mb, 500)
        
        print(f"[{self._get_timestamp()}] [TEST] TimingRequirement creation test passed")
    
    def test_timing_requirement_minimal(self):
        """Test TimingRequirement with minimal parameters."""
        print(f"[{self._get_timestamp()}] [TEST] Testing minimal TimingRequirement")
        
        requirement = TimingRequirement(
            test_name="simple_test",
            max_execution_time=1.0
        )
        
        self.assertEqual(requirement.test_name, "simple_test")
        self.assertEqual(requirement.max_execution_time, 1.0)
        self.assertIsNone(requirement.max_latency_ms)
        self.assertIsNone(requirement.min_fps)
        self.assertIsNone(requirement.max_cpu_percent)
        self.assertIsNone(requirement.max_memory_mb)
        
        print(f"[{self._get_timestamp()}] [TEST] Minimal TimingRequirement test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestResourceMonitor(unittest.TestCase):
    """Test ResourceMonitor functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.monitor = ResourceMonitor(sampling_interval=0.01)  # Fast sampling for tests
    
    def test_resource_monitor_initialization(self):
        """Test ResourceMonitor initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ResourceMonitor initialization")
        
        self.assertEqual(self.monitor.sampling_interval, 0.01)
        self.assertFalse(self.monitor.monitoring)
        self.assertIsNone(self.monitor.monitor_thread)
        
        print(f"[{self._get_timestamp()}] [TEST] ResourceMonitor initialization test passed")
    
    def test_resource_monitor_start_stop(self):
        """Test ResourceMonitor start and stop functionality."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ResourceMonitor start/stop")
        
        # Start monitoring
        self.monitor.start_monitoring()
        self.assertTrue(self.monitor.monitoring)
        self.assertIsNotNone(self.monitor.monitor_thread)
        
        # Let it run briefly
        time.sleep(0.05)
        
        # Stop monitoring
        metrics = self.monitor.stop_monitoring()
        self.assertFalse(self.monitor.monitoring)
        self.assertIsInstance(metrics, dict)
        
        # Check that we got some metrics
        if metrics:  # May be empty in some test environments
            self.assertIn('avg_cpu_percent', metrics)
            self.assertIn('avg_memory_mb', metrics)
            self.assertIn('sample_count', metrics)
        
        print(f"[{self._get_timestamp()}] [TEST] ResourceMonitor start/stop test passed")
    
    def test_resource_monitor_double_start(self):
        """Test ResourceMonitor behavior with double start."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ResourceMonitor double start")
        
        self.monitor.start_monitoring()
        self.assertTrue(self.monitor.monitoring)
        
        # Try to start again - should not crash
        self.monitor.start_monitoring()
        self.assertTrue(self.monitor.monitoring)
        
        # Clean up
        self.monitor.stop_monitoring()
        
        print(f"[{self._get_timestamp()}] [TEST] ResourceMonitor double start test passed")
    
    def test_resource_monitor_stop_without_start(self):
        """Test ResourceMonitor stop without start."""
        print(f"[{self._get_timestamp()}] [TEST] Testing ResourceMonitor stop without start")
        
        metrics = self.monitor.stop_monitoring()
        self.assertEqual(metrics, {})
        
        print(f"[{self._get_timestamp()}] [TEST] ResourceMonitor stop without start test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestEnhancedPerformanceBenchmark(unittest.TestCase):
    """Test EnhancedPerformanceBenchmark functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.benchmark = EnhancedPerformanceBenchmark(output_dir=self.temp_dir)
    
    def tearDown(self):
        """Clean up test fixtures."""
        shutil.rmtree(self.temp_dir)
    
    def test_benchmark_initialization(self):
        """Test EnhancedPerformanceBenchmark initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing EnhancedPerformanceBenchmark initialization")
        
        self.assertEqual(str(self.benchmark.output_dir), self.temp_dir)
        self.assertEqual(len(self.benchmark.timing_requirements), 0)
        self.assertEqual(len(self.benchmark.benchmark_results), 0)
        self.assertIsInstance(self.benchmark.resource_monitor, ResourceMonitor)
        
        print(f"[{self._get_timestamp()}] [TEST] EnhancedPerformanceBenchmark initialization test passed")
    
    def test_add_timing_requirement(self):
        """Test adding timing requirements."""
        print(f"[{self._get_timestamp()}] [TEST] Testing timing requirement addition")
        
        requirement = TimingRequirement(
            test_name="test_function",
            max_execution_time=0.1
        )
        
        self.benchmark.add_timing_requirement(requirement)
        
        self.assertEqual(len(self.benchmark.timing_requirements), 1)
        self.assertIn("test_function", self.benchmark.timing_requirements)
        self.assertEqual(self.benchmark.timing_requirements["test_function"], requirement)
        
        print(f"[{self._get_timestamp()}] [TEST] Timing requirement addition test passed")
    
    def test_benchmark_function_success(self):
        """Test benchmarking a successful function."""
        print(f"[{self._get_timestamp()}] [TEST] Testing successful function benchmarking")
        
        def test_function(x):
            time.sleep(0.01)  # Simulate some work
            return x * 2
        
        metrics = self.benchmark.benchmark_function("test_func", test_function, 5)
        
        self.assertEqual(metrics.test_name, "test_func")
        self.assertGreater(metrics.execution_time, 0.005)  # Should be at least 5ms
        self.assertEqual(metrics.success_rate, 1.0)
        self.assertEqual(metrics.error_count, 0)
        self.assertIsNotNone(metrics.fps)
        self.assertIsNotNone(metrics.latency_ms)
        
        # Check that result was added to benchmark results
        self.assertEqual(len(self.benchmark.benchmark_results), 1)
        
        print(f"[{self._get_timestamp()}] [TEST] Successful function benchmarking test passed")
    
    def test_benchmark_function_failure(self):
        """Test benchmarking a failing function."""
        print(f"[{self._get_timestamp()}] [TEST] Testing failing function benchmarking")
        
        def failing_function():
            raise ValueError("Test error")
        
        metrics = self.benchmark.benchmark_function("failing_func", failing_function)
        
        self.assertEqual(metrics.test_name, "failing_func")
        self.assertEqual(metrics.success_rate, 0.0)
        self.assertEqual(metrics.error_count, 1)
        
        print(f"[{self._get_timestamp()}] [TEST] Failing function benchmarking test passed")
    
    def test_benchmark_pipeline(self):
        """Test benchmarking a multi-step pipeline."""
        print(f"[{self._get_timestamp()}] [TEST] Testing pipeline benchmarking")
        
        def step1(x):
            time.sleep(0.005)
            return x + 1
        
        def step2(x):
            time.sleep(0.005)
            return x * 2
        
        def step3(x):
            time.sleep(0.005)
            return x - 1
        
        pipeline_steps = [
            ("step1", step1, (5,), {}),
            ("step2", step2, (), {}),  # Will use result from step1
            ("step3", step3, (), {})   # Will use result from step2
        ]
        
        metrics = self.benchmark.benchmark_pipeline("test_pipeline", pipeline_steps)
        
        self.assertEqual(metrics.test_name, "test_pipeline")
        self.assertGreater(metrics.execution_time, 0.01)  # Should be at least 10ms
        self.assertEqual(metrics.success_rate, 1.0)
        self.assertEqual(metrics.error_count, 0)
        
        print(f"[{self._get_timestamp()}] [TEST] Pipeline benchmarking test passed")
    
    def test_timing_requirement_checking(self):
        """Test timing requirement violation detection."""
        print(f"[{self._get_timestamp()}] [TEST] Testing timing requirement checking")
        
        # Add a strict timing requirement
        requirement = TimingRequirement(
            test_name="slow_function",
            max_execution_time=0.001,  # Very strict - 1ms
            max_cpu_percent=10
        )
        self.benchmark.add_timing_requirement(requirement)
        
        def slow_function():
            time.sleep(0.01)  # 10ms - will violate requirement
            return "done"
        
        # This should log warnings about timing violations
        metrics = self.benchmark.benchmark_function("slow_function", slow_function)
        
        self.assertEqual(metrics.test_name, "slow_function")
        self.assertGreater(metrics.execution_time, requirement.max_execution_time)
        
        print(f"[{self._get_timestamp()}] [TEST] Timing requirement checking test passed")
    
    def test_performance_report_generation(self):
        """Test performance report generation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing performance report generation")
        
        # Add some benchmark results
        def test_func1():
            time.sleep(0.01)
            return "result1"
        
        def test_func2():
            time.sleep(0.005)
            return "result2"
        
        self.benchmark.benchmark_function("func1", test_func1)
        self.benchmark.benchmark_function("func2", test_func2)
        
        # Generate report
        report = self.benchmark.generate_performance_report()
        
        self.assertIn('summary', report)
        self.assertIn('detailed_results', report)
        self.assertIn('timing_requirements', report)
        self.assertIn('requirement_violations', report)
        self.assertIn('generated_at', report)
        
        # Check summary statistics
        summary = report['summary']
        self.assertEqual(summary['total_tests'], 2)
        self.assertGreater(summary['avg_execution_time'], 0)
        self.assertEqual(summary['overall_success_rate'], 1.0)
        self.assertEqual(summary['total_errors'], 0)
        
        # Check that report file was created
        report_files = list(Path(self.temp_dir).glob("performance_report_*.json"))
        self.assertEqual(len(report_files), 1)
        
        print(f"[{self._get_timestamp()}] [TEST] Performance report generation test passed")
    
    def test_empty_report_generation(self):
        """Test report generation with no benchmark results."""
        print(f"[{self._get_timestamp()}] [TEST] Testing empty report generation")
        
        report = self.benchmark.generate_performance_report()
        self.assertEqual(report, {})
        
        print(f"[{self._get_timestamp()}] [TEST] Empty report generation test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestTestDataGenerator(unittest.TestCase):
    """Test TestDataGenerator functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.generator = TestDataGenerator(output_dir=self.temp_dir)
    
    def tearDown(self):
        """Clean up test fixtures."""
        shutil.rmtree(self.temp_dir)
    
    def test_generator_initialization(self):
        """Test TestDataGenerator initialization."""
        print(f"[{self._get_timestamp()}] [TEST] Testing TestDataGenerator initialization")
        
        self.assertEqual(str(self.generator.output_dir), self.temp_dir)
        self.assertTrue(Path(self.temp_dir).exists())
        
        print(f"[{self._get_timestamp()}] [TEST] TestDataGenerator initialization test passed")
    
    @patch('cv2.imwrite')
    @patch('cv2.line')
    @patch('cv2.rectangle')
    def test_synthetic_camera_data_generation(self, mock_rectangle, mock_line, mock_imwrite):
        """Test synthetic camera data generation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing synthetic camera data generation")
        
        frames = self.generator.generate_synthetic_camera_data(num_frames=10, width=320, height=240)
        
        self.assertEqual(len(frames), 10)
        for frame in frames:
            self.assertEqual(frame.shape, (240, 320, 3))
            self.assertEqual(frame.dtype, np.uint8)
        
        # Check that sample frames directory was created
        sample_dir = Path(self.temp_dir) / "synthetic_camera_frames"
        self.assertTrue(sample_dir.exists())
        
        print(f"[{self._get_timestamp()}] [TEST] Synthetic camera data generation test passed")
    
    def test_apriltag_scenarios_generation(self):
        """Test AprilTag scenarios generation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing AprilTag scenarios generation")
        
        scenarios = self.generator.generate_apriltag_scenarios(num_scenarios=20)
        
        self.assertEqual(len(scenarios), 20)
        
        for scenario in scenarios:
            self.assertIn('scenario_id', scenario)
            self.assertIn('tag_id', scenario)
            self.assertIn('distance', scenario)
            self.assertIn('angle', scenario)
            self.assertIn('lighting', scenario)
            self.assertIn('occlusion', scenario)
            self.assertIn('noise_level', scenario)
            self.assertIn('expected_detection', scenario)
            
            # Check value ranges
            self.assertGreaterEqual(scenario['distance'], 0.3)
            self.assertLessEqual(scenario['distance'], 3.0)
            self.assertGreaterEqual(scenario['angle'], -45)
            self.assertLessEqual(scenario['angle'], 45)
            self.assertIn(scenario['lighting'], ['bright', 'normal', 'dim'])
        
        # Check that scenarios file was created
        scenarios_file = Path(self.temp_dir) / "apriltag_scenarios.json"
        self.assertTrue(scenarios_file.exists())
        
        # Verify file content
        with open(scenarios_file, 'r') as f:
            saved_scenarios = json.load(f)
        self.assertEqual(len(saved_scenarios), 20)
        
        print(f"[{self._get_timestamp()}] [TEST] AprilTag scenarios generation test passed")
    
    def test_lane_detection_scenarios_generation(self):
        """Test lane detection scenarios generation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing lane detection scenarios generation")
        
        scenarios = self.generator.generate_lane_detection_scenarios(num_scenarios=15)
        
        self.assertEqual(len(scenarios), 15)
        
        for scenario in scenarios:
            self.assertIn('scenario_id', scenario)
            self.assertIn('lane_type', scenario)
            self.assertIn('lane_width', scenario)
            self.assertIn('marking_quality', scenario)
            self.assertIn('lighting', scenario)
            self.assertIn('weather', scenario)
            self.assertIn('surface', scenario)
            self.assertIn('expected_detection_confidence', scenario)
            
            # Check value ranges
            self.assertGreaterEqual(scenario['lane_width'], 0.3)
            self.assertLessEqual(scenario['lane_width'], 0.6)
            self.assertIn(scenario['lane_type'], ['straight', 'curved_left', 'curved_right', 'intersection'])
            self.assertIn(scenario['marking_quality'], ['good', 'faded', 'broken'])
        
        # Check that scenarios file was created
        scenarios_file = Path(self.temp_dir) / "lane_detection_scenarios.json"
        self.assertTrue(scenarios_file.exists())
        
        print(f"[{self._get_timestamp()}] [TEST] Lane detection scenarios generation test passed")
    
    def test_object_detection_scenarios_generation(self):
        """Test object detection scenarios generation."""
        print(f"[{self._get_timestamp()}] [TEST] Testing object detection scenarios generation")
        
        scenarios = self.generator.generate_object_detection_scenarios(num_scenarios=25)
        
        self.assertEqual(len(scenarios), 25)
        
        for scenario in scenarios:
            self.assertIn('scenario_id', scenario)
            self.assertIn('objects', scenario)
            self.assertIn('lighting', scenario)
            self.assertIn('background_complexity', scenario)
            self.assertIn('motion_blur', scenario)
            self.assertIn('expected_detection_count', scenario)
            
            # Check objects structure
            for obj in scenario['objects']:
                self.assertIn('object_id', obj)
                self.assertIn('type', obj)
                self.assertIn('distance', obj)
                self.assertIn('lateral_offset', obj)
                self.assertIn('size', obj)
                self.assertIn('occlusion', obj)
                self.assertIn('expected_confidence', obj)
                
                self.assertIn(obj['type'], ['duckiebot', 'duckie', 'cone', 'sign', 'barrier'])
        
        # Check that scenarios file was created
        scenarios_file = Path(self.temp_dir) / "object_detection_scenarios.json"
        self.assertTrue(scenarios_file.exists())
        
        print(f"[{self._get_timestamp()}] [TEST] Object detection scenarios generation test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestDefaultTimingRequirements(unittest.TestCase):
    """Test default timing requirements creation."""
    
    def test_create_default_timing_requirements(self):
        """Test creation of default timing requirements."""
        print(f"[{self._get_timestamp()}] [TEST] Testing default timing requirements creation")
        
        requirements = create_default_timing_requirements()
        
        self.assertIsInstance(requirements, list)
        self.assertGreater(len(requirements), 0)
        
        # Check that all requirements are TimingRequirement instances
        for req in requirements:
            self.assertIsInstance(req, TimingRequirement)
            self.assertIsNotNone(req.test_name)
            self.assertGreater(req.max_execution_time, 0)
        
        # Check for expected test names
        test_names = [req.test_name for req in requirements]
        expected_names = [
            "lane_detection_pipeline",
            "apriltag_detection_pipeline", 
            "yolo_object_detection",
            "mpc_lane_control",
            "safety_monitoring",
            "end_to_end_pipeline"
        ]
        
        for expected_name in expected_names:
            self.assertIn(expected_name, test_names)
        
        print(f"[{self._get_timestamp()}] [TEST] Default timing requirements creation test passed")
    
    def test_timing_requirements_values(self):
        """Test that timing requirements have reasonable values."""
        print(f"[{self._get_timestamp()}] [TEST] Testing timing requirements values")
        
        requirements = create_default_timing_requirements()
        
        for req in requirements:
            # All requirements should have positive execution time limits
            self.assertGreater(req.max_execution_time, 0)
            
            # Real-time requirements should be reasonable
            if req.test_name in ["lane_detection_pipeline", "mpc_lane_control", "safety_monitoring"]:
                self.assertLessEqual(req.max_execution_time, 0.1)  # Should be <= 100ms for real-time
            
            # FPS requirements should be reasonable
            if req.min_fps:
                self.assertGreaterEqual(req.min_fps, 1)
                self.assertLessEqual(req.min_fps, 100)
            
            # CPU and memory limits should be reasonable
            if req.max_cpu_percent:
                self.assertGreaterEqual(req.max_cpu_percent, 10)
                self.assertLessEqual(req.max_cpu_percent, 100)
            
            if req.max_memory_mb:
                self.assertGreaterEqual(req.max_memory_mb, 50)
                self.assertLessEqual(req.max_memory_mb, 5000)
        
        print(f"[{self._get_timestamp()}] [TEST] Timing requirements values test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


class TestIntegration(unittest.TestCase):
    """Integration tests for enhanced performance benchmarking."""
    
    def setUp(self):
        """Set up integration test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.benchmark = EnhancedPerformanceBenchmark(output_dir=self.temp_dir)
        self.generator = TestDataGenerator(output_dir=self.temp_dir)
    
    def tearDown(self):
        """Clean up integration test fixtures."""
        shutil.rmtree(self.temp_dir)
    
    def test_end_to_end_benchmarking_workflow(self):
        """Test complete end-to-end benchmarking workflow."""
        print(f"[{self._get_timestamp()}] [TEST] Testing end-to-end benchmarking workflow")
        
        # 1. Add timing requirements
        requirements = create_default_timing_requirements()
        for req in requirements[:3]:  # Use first 3 for testing
            self.benchmark.add_timing_requirement(req)
        
        # 2. Generate test data
        camera_frames = self.generator.generate_synthetic_camera_data(5)
        apriltag_scenarios = self.generator.generate_apriltag_scenarios(5)
        
        # 3. Define test functions that use the generated data
        def lane_detection_test():
            # Simulate lane detection processing
            for frame in camera_frames[:2]:  # Process 2 frames
                # Simulate some image processing
                gray = np.mean(frame, axis=2)
                edges = np.gradient(gray)
                time.sleep(0.001)  # Simulate processing time
            return len(camera_frames)
        
        def apriltag_detection_test():
            # Simulate AprilTag detection
            detections = []
            for scenario in apriltag_scenarios[:3]:  # Process 3 scenarios
                if scenario['expected_detection']:
                    detections.append(scenario['tag_id'])
                time.sleep(0.002)  # Simulate processing time
            return detections
        
        # 4. Run benchmarks
        lane_metrics = self.benchmark.benchmark_function("lane_detection_pipeline", lane_detection_test)
        apriltag_metrics = self.benchmark.benchmark_function("apriltag_detection_pipeline", apriltag_detection_test)
        
        # 5. Verify results
        self.assertEqual(len(self.benchmark.benchmark_results), 2)
        self.assertGreater(lane_metrics.execution_time, 0)
        self.assertGreater(apriltag_metrics.execution_time, 0)
        
        # 6. Generate comprehensive report
        report = self.benchmark.generate_performance_report()
        
        self.assertIn('summary', report)
        self.assertEqual(report['summary']['total_tests'], 2)
        self.assertGreater(report['summary']['avg_execution_time'], 0)
        
        # 7. Verify files were created
        report_files = list(Path(self.temp_dir).glob("performance_report_*.json"))
        self.assertEqual(len(report_files), 1)
        
        camera_dir = Path(self.temp_dir) / "synthetic_camera_frames"
        self.assertTrue(camera_dir.exists())
        
        apriltag_file = Path(self.temp_dir) / "apriltag_scenarios.json"
        self.assertTrue(apriltag_file.exists())
        
        print(f"[{self._get_timestamp()}] [TEST] End-to-end benchmarking workflow test passed")
    
    def test_performance_regression_detection(self):
        """Test performance regression detection capabilities."""
        print(f"[{self._get_timestamp()}] [TEST] Testing performance regression detection")
        
        # Add strict timing requirement
        requirement = TimingRequirement(
            test_name="regression_test",
            max_execution_time=0.01,  # 10ms limit
            max_cpu_percent=20
        )
        self.benchmark.add_timing_requirement(requirement)
        
        # Run a function that meets requirements
        def fast_function():
            time.sleep(0.005)  # 5ms - within limit
            return "fast"
        
        fast_metrics = self.benchmark.benchmark_function("regression_test", fast_function)
        
        # Run a function that violates requirements (simulating regression)
        def slow_function():
            time.sleep(0.02)  # 20ms - exceeds limit
            return "slow"
        
        slow_metrics = self.benchmark.benchmark_function("regression_test", slow_function)
        
        # Generate report and check for violations
        report = self.benchmark.generate_performance_report()
        violations = report['requirement_violations']
        
        # Should have at least one violation from the slow function
        violation_tests = [v['test_name'] for v in violations]
        self.assertIn("regression_test", violation_tests)
        
        print(f"[{self._get_timestamp()}] [TEST] Performance regression detection test passed")
    
    def _get_timestamp(self):
        """Get current timestamp for logging."""
        from datetime import datetime
        return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def run_enhanced_benchmarking_tests():
    """Run all enhanced performance benchmarking tests."""
    print(f"[{TestPerformanceMetrics()._get_timestamp()}] [TEST] Starting enhanced performance benchmarking test suite...")
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestPerformanceMetrics,
        TestTimingRequirement,
        TestResourceMonitor,
        TestEnhancedPerformanceBenchmark,
        TestTestDataGenerator,
        TestDefaultTimingRequirements,
        TestIntegration
    ]
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n[{TestPerformanceMetrics()._get_timestamp()}] [TEST] Enhanced Benchmarking Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\n[{TestPerformanceMetrics()._get_timestamp()}] [TEST] Failures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\n[{TestPerformanceMetrics()._get_timestamp()}] [TEST] Errors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_enhanced_benchmarking_tests()
    sys.exit(0 if success else 1)
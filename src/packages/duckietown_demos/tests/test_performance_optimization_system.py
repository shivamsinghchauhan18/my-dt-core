#!/usr/bin/env python3
"""
Test Suite for Performance Optimization System

Comprehensive tests for enhanced performance profiler, dynamic quality adjuster,
performance alerting system, and resource optimization utilities.

Author: Duckietown
"""

import unittest
import rospy
import time
import json
import threading
from unittest.mock import Mock, patch, MagicMock
import numpy as np

# ROS messages
from std_msgs.msg import String, Float32, Bool

# Import the modules to test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from enhanced_performance_profiler import EnhancedPerformanceProfiler, PerformanceProfile, SystemBottleneck
from performance_alerting_system import PerformanceAlertingSystem, PerformanceAlert, AlertSeverity, AlertCategory
from performance_data_aggregator import PerformanceDataAggregator, ComponentPerformanceData, AggregatedPerformanceData

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'image_processing', 'src'))
from dynamic_quality_adjuster import DynamicQualityAdjuster, QualityLevel, ResourcePressure, QualitySettings
from resource_optimization_utils import ResourceOptimizationUtils, ResourceUsage, OptimizationResult, OptimizationStrategy


class TestEnhancedPerformanceProfiler(unittest.TestCase):
    """Test cases for Enhanced Performance Profiler"""
    
    def setUp(self):
        """Set up test fixtures"""
        rospy.init_node('test_performance_profiler', anonymous=True)
        
        # Mock the profiler to avoid ROS node conflicts
        with patch('enhanced_performance_profiler.rospy.init_node'):
            self.profiler = EnhancedPerformanceProfiler()
    
    def test_component_registration(self):
        """Test component registration functionality"""
        component_name = "test_component"
        priority = 7
        performance_targets = {
            'target_fps': 20.0,
            'max_latency_ms': 200.0,
            'max_cpu_percent': 25.0,
            'max_memory_mb': 512.0
        }
        
        result = self.profiler.register_component(component_name, priority, performance_targets)
        
        self.assertTrue(result)
        self.assertIn(component_name, self.profiler.registered_components)
        self.assertEqual(self.profiler.registered_components[component_name]['priority'], priority)
        self.assertEqual(self.profiler.component_priorities[component_name], priority)
    
    def test_performance_profile_creation(self):
        """Test performance profile creation"""
        perf_data = {
            'component_name': 'test_component',
            'cpu_usage_percent': 50.0,
            'memory_usage_mb': 256.0,
            'processing_time_ms': 100.0,
            'fps': 15.0,
            'queue_depth': 5,
            'error_count': 2,
            'latency_samples': [90, 100, 110, 120, 95]
        }
        
        # Register component first
        self.profiler.register_component('test_component', 5)
        
        profile = self.profiler._create_performance_profile(perf_data)
        
        self.assertIsInstance(profile, PerformanceProfile)
        self.assertEqual(profile.component_name, 'test_component')
        self.assertEqual(profile.cpu_usage_percent, 50.0)
        self.assertEqual(profile.memory_usage_mb, 256.0)
        self.assertEqual(profile.fps, 15.0)
        self.assertGreater(profile.resource_efficiency, 0.0)
        self.assertLessEqual(profile.resource_efficiency, 1.0)
    
    def test_bottleneck_analysis(self):
        """Test bottleneck analysis functionality"""
        component_name = "test_component"
        
        # Create mock profiles with high CPU usage
        profiles = []
        for i in range(5):
            profile = PerformanceProfile(
                component_name=component_name,
                timestamp=time.time() - i,
                cpu_usage_percent=95.0,  # High CPU usage
                memory_usage_mb=256.0,
                processing_time_ms=200.0,
                fps=8.0,  # Low FPS
                queue_depth=15,  # High queue depth
                error_count=0,
                latency_p50_ms=200.0,
                latency_p95_ms=350.0,
                latency_p99_ms=400.0,
                throughput_ops_per_sec=8.0,
                resource_efficiency=0.3,
                bottleneck_indicators={'cpu_pressure': 0.95},
                optimization_opportunities=['reduce_cpu_usage']
            )
            profiles.append(profile)
        
        bottlenecks = self.profiler._analyze_component_bottlenecks(component_name, profiles)
        
        self.assertGreater(len(bottlenecks), 0)
        
        # Should detect CPU bottleneck
        cpu_bottleneck = next((b for b in bottlenecks if b.bottleneck_type == "cpu"), None)
        self.assertIsNotNone(cpu_bottleneck)
        self.assertEqual(cpu_bottleneck.affected_components, [component_name])
        self.assertGreater(cpu_bottleneck.severity, 0.8)
    
    def test_system_bottleneck_analysis(self):
        """Test system-level bottleneck analysis"""
        # Mock system metrics with high resource usage
        system_metrics = []
        for i in range(10):
            metrics = {
                'timestamp': time.time() - i,
                'cpu': {'percent': 92.0},  # High CPU
                'memory': {'percent': 88.0},  # High memory
                'temperature_celsius': 78.0  # High temperature
            }
            system_metrics.append(metrics)
        
        self.profiler.system_metrics_history.extend(system_metrics)
        
        bottlenecks = self.profiler._analyze_system_bottlenecks()
        
        self.assertGreater(len(bottlenecks), 0)
        
        # Should detect system CPU bottleneck
        system_bottleneck = next((b for b in bottlenecks if b.bottleneck_type == "system_cpu"), None)
        self.assertIsNotNone(system_bottleneck)
        self.assertGreater(system_bottleneck.severity, 0.8)


class TestDynamicQualityAdjuster(unittest.TestCase):
    """Test cases for Dynamic Quality Adjuster"""
    
    def setUp(self):
        """Set up test fixtures"""
        with patch('dynamic_quality_adjuster.rospy.init_node'):
            self.adjuster = DynamicQualityAdjuster()
    
    def test_quality_level_enum(self):
        """Test quality level enumeration"""
        self.assertEqual(QualityLevel.ULTRA_LOW.value, 0.2)
        self.assertEqual(QualityLevel.LOW.value, 0.4)
        self.assertEqual(QualityLevel.MEDIUM.value, 0.6)
        self.assertEqual(QualityLevel.HIGH.value, 0.8)
        self.assertEqual(QualityLevel.ULTRA_HIGH.value, 1.0)
    
    def test_resource_pressure_calculation(self):
        """Test resource pressure level calculation"""
        # High pressure scenario
        high_pressure_metrics = ResourceMetrics(
            cpu_percent=95.0,
            memory_percent=90.0,
            memory_available_mb=100.0,
            temperature_celsius=82.0,
            processing_fps=8.0,
            target_fps=20.0,
            latency_ms=400.0,
            queue_depth=15,
            timestamp=time.time()
        )
        
        pressure = high_pressure_metrics.get_pressure_level(raspberry_pi_mode=True)
        self.assertEqual(pressure, ResourcePressure.CRITICAL)
        
        # Low pressure scenario
        low_pressure_metrics = ResourceMetrics(
            cpu_percent=30.0,
            memory_percent=40.0,
            memory_available_mb=1500.0,
            temperature_celsius=55.0,
            processing_fps=22.0,
            target_fps=20.0,
            latency_ms=80.0,
            queue_depth=2,
            timestamp=time.time()
        )
        
        pressure = low_pressure_metrics.get_pressure_level(raspberry_pi_mode=True)
        self.assertEqual(pressure, ResourcePressure.NONE)
    
    def test_quality_settings_generation(self):
        """Test quality settings generation for different levels"""
        ultra_low_settings = self.adjuster._get_default_quality_settings(QualityLevel.ULTRA_LOW)
        ultra_high_settings = self.adjuster._get_default_quality_settings(QualityLevel.ULTRA_HIGH)
        
        # Ultra low should have lower quality settings
        self.assertLess(ultra_low_settings.resolution_scale, ultra_high_settings.resolution_scale)
        self.assertLess(ultra_low_settings.compression_quality, ultra_high_settings.compression_quality)
        self.assertGreater(ultra_low_settings.frame_skip_rate, ultra_high_settings.frame_skip_rate)
        self.assertFalse(ultra_low_settings.enable_multi_scale)
        self.assertTrue(ultra_high_settings.enable_multi_scale)
    
    def test_quality_adjustment_assessment(self):
        """Test quality adjustment assessment logic"""
        # High pressure scenario should trigger quality reduction
        high_pressure_metrics = ResourceMetrics(
            cpu_percent=95.0,
            memory_percent=90.0,
            memory_available_mb=100.0,
            temperature_celsius=82.0,
            processing_fps=8.0,
            target_fps=20.0,
            latency_ms=400.0,
            queue_depth=15,
            timestamp=time.time()
        )
        
        pressure = high_pressure_metrics.get_pressure_level(raspberry_pi_mode=True)
        adjustment_needed, target_quality = self.adjuster._assess_quality_adjustment(high_pressure_metrics, pressure)
        
        self.assertTrue(adjustment_needed)
        self.assertEqual(target_quality, QualityLevel.ULTRA_LOW)
    
    def test_component_registration(self):
        """Test component registration for quality adjustment"""
        component_name = "test_component"
        priority = 8
        
        def mock_callback(settings):
            pass
        
        result = self.adjuster.register_component(component_name, mock_callback, priority)
        
        self.assertTrue(result)
        self.assertIn(component_name, self.adjuster.registered_components)
        self.assertIn(component_name, self.adjuster.quality_callbacks)
        self.assertEqual(self.adjuster.registered_components[component_name]['priority'], priority)


class TestPerformanceAlertingSystem(unittest.TestCase):
    """Test cases for Performance Alerting System"""
    
    def setUp(self):
        """Set up test fixtures"""
        with patch('performance_alerting_system.rospy.init_node'):
            self.alerting_system = PerformanceAlertingSystem()
    
    def test_alert_threshold_violation_check(self):
        """Test alert threshold violation checking"""
        from performance_alerting_system import AlertThreshold
        
        threshold = AlertThreshold(
            metric_name='cpu_percent',
            warning_threshold=75.0,
            critical_threshold=90.0,
            emergency_threshold=98.0,
            comparison_operator='>',
            duration_seconds=5.0,
            enabled=True
        )
        
        # Test warning violation
        warning_severity = threshold.check_violation(80.0, 6.0)
        self.assertEqual(warning_severity, AlertSeverity.WARNING)
        
        # Test critical violation
        critical_severity = threshold.check_violation(95.0, 6.0)
        self.assertEqual(critical_severity, AlertSeverity.CRITICAL)
        
        # Test emergency violation
        emergency_severity = threshold.check_violation(99.0, 6.0)
        self.assertEqual(emergency_severity, AlertSeverity.EMERGENCY)
        
        # Test insufficient duration
        no_violation = threshold.check_violation(95.0, 2.0)
        self.assertIsNone(no_violation)
    
    def test_alert_creation(self):
        """Test alert creation functionality"""
        alert = PerformanceAlert(
            alert_id="test_alert_001",
            timestamp=time.time(),
            severity=AlertSeverity.CRITICAL,
            category=AlertCategory.PERFORMANCE,
            component="test_component",
            title="Test Alert",
            description="This is a test alert",
            metrics={'cpu_percent': 95.0},
            threshold_violated="cpu_percent > 90.0",
            recommended_actions=["Reduce CPU usage", "Optimize algorithms"],
            escalation_level=0,
            acknowledged=False,
            resolved=False,
            resolution_time=None
        )
        
        self.assertEqual(alert.severity, AlertSeverity.CRITICAL)
        self.assertEqual(alert.category, AlertCategory.PERFORMANCE)
        self.assertFalse(alert.acknowledged)
        self.assertFalse(alert.resolved)
        self.assertEqual(len(alert.recommended_actions), 2)
    
    def test_threshold_alert_creation(self):
        """Test threshold-based alert creation"""
        from performance_alerting_system import AlertThreshold
        
        threshold = AlertThreshold(
            metric_name='memory_percent',
            warning_threshold=70.0,
            critical_threshold=85.0,
            emergency_threshold=95.0,
            comparison_operator='>',
            duration_seconds=10.0,
            enabled=True
        )
        
        alert = self.alerting_system._create_threshold_alert(
            alert_id="memory_test_001",
            component="test_component",
            metric_name="memory_percent",
            value=88.0,
            threshold=threshold,
            severity=AlertSeverity.CRITICAL
        )
        
        self.assertEqual(alert.severity, AlertSeverity.CRITICAL)
        self.assertEqual(alert.component, "test_component")
        self.assertIn("memory_percent", alert.title.lower())
        self.assertGreater(len(alert.recommended_actions), 0)
    
    def test_recommended_actions_generation(self):
        """Test recommended actions generation"""
        cpu_actions = self.alerting_system._get_recommended_actions("cpu_percent", AlertSeverity.CRITICAL)
        memory_actions = self.alerting_system._get_recommended_actions("memory_percent", AlertSeverity.WARNING)
        temperature_actions = self.alerting_system._get_recommended_actions("temperature_celsius", AlertSeverity.EMERGENCY)
        
        self.assertGreater(len(cpu_actions), 0)
        self.assertGreater(len(memory_actions), 0)
        self.assertGreater(len(temperature_actions), 0)
        
        # Critical CPU alerts should have more aggressive actions
        self.assertGreater(len(cpu_actions), 3)
        
        # Emergency temperature alerts should include shutdown considerations
        self.assertTrue(any("shutdown" in action.lower() for action in temperature_actions))


class TestResourceOptimizationUtils(unittest.TestCase):
    """Test cases for Resource Optimization Utils"""
    
    def setUp(self):
        """Set up test fixtures"""
        with patch('resource_optimization_utils.rospy.get_param', return_value=True):
            self.optimizer = ResourceOptimizationUtils()
    
    def test_resource_usage_collection(self):
        """Test resource usage data collection"""
        usage = self.optimizer.get_current_resource_usage()
        
        self.assertIsInstance(usage, ResourceUsage)
        self.assertGreaterEqual(usage.cpu_percent, 0.0)
        self.assertGreaterEqual(usage.memory_rss_mb, 0.0)
        self.assertGreaterEqual(usage.memory_percent, 0.0)
        self.assertGreater(usage.timestamp, 0)
    
    def test_memory_pool_functionality(self):
        """Test memory pool functionality"""
        buffer_size = 1024
        pool_size = 5
        
        from resource_optimization_utils import MemoryPool
        pool = MemoryPool(buffer_size, pool_size)
        
        # Test buffer allocation
        buffer1 = pool.get_buffer()
        self.assertIsNotNone(buffer1)
        self.assertEqual(buffer1.size, buffer_size)
        
        buffer2 = pool.get_buffer()
        self.assertIsNotNone(buffer2)
        
        # Test buffer return
        pool.return_buffer(buffer1)
        stats = pool.get_stats()
        self.assertEqual(stats['used_buffers'], 1)  # Only buffer2 is still used
    
    def test_memory_optimization(self):
        """Test memory optimization functionality"""
        before_usage = self.optimizer.get_current_resource_usage()
        
        # Perform memory optimization
        result = self.optimizer.optimize_memory_usage(aggressive=False)
        
        self.assertIsInstance(result, OptimizationResult)
        self.assertEqual(result.strategy, OptimizationStrategy.MEMORY_POOLING)
        self.assertIsInstance(result.success, bool)
        self.assertGreaterEqual(result.duration_seconds, 0.0)
    
    def test_cpu_optimization(self):
        """Test CPU optimization functionality"""
        before_usage = self.optimizer.get_current_resource_usage()
        
        # Perform CPU optimization
        result = self.optimizer.optimize_cpu_usage()
        
        self.assertIsInstance(result, OptimizationResult)
        self.assertEqual(result.strategy, OptimizationStrategy.PROCESS_PRIORITY)
        self.assertIsInstance(result.success, bool)
        self.assertGreaterEqual(result.duration_seconds, 0.0)
    
    def test_image_optimization(self):
        """Test image processing optimization"""
        # Create test image
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Test optimization with different quality factors
        optimized_high = self.optimizer.optimize_image_processing(test_image, quality_factor=1.0)
        optimized_low = self.optimizer.optimize_image_processing(test_image, quality_factor=0.5)
        
        self.assertIsInstance(optimized_high, np.ndarray)
        self.assertIsInstance(optimized_low, np.ndarray)
        
        # Low quality should result in smaller or processed image
        # (exact comparison depends on optimization strategy)
        self.assertGreaterEqual(optimized_high.size, optimized_low.size)
    
    def test_buffer_management(self):
        """Test optimized buffer management"""
        buffer_size = 1024
        
        # Get buffer
        buffer = self.optimizer.get_buffer(buffer_size)
        self.assertIsNotNone(buffer)
        self.assertGreaterEqual(buffer.size, buffer_size)
        
        # Return buffer
        self.optimizer.return_buffer(buffer)
        
        # Should not raise any exceptions


class TestPerformanceDataAggregator(unittest.TestCase):
    """Test cases for Performance Data Aggregator"""
    
    def setUp(self):
        """Set up test fixtures"""
        with patch('performance_data_aggregator.rospy.init_node'):
            self.aggregator = PerformanceDataAggregator()
    
    def test_component_performance_data_creation(self):
        """Test component performance data creation"""
        data = ComponentPerformanceData(
            component_name="test_component",
            timestamp=time.time(),
            fps=15.0,
            avg_latency_ms=120.0,
            max_latency_ms=200.0,
            cpu_usage_percent=45.0,
            memory_usage_mb=256.0,
            queue_depth=3,
            error_count=1,
            processing_time_ms=100.0
        )
        
        self.assertEqual(data.component_name, "test_component")
        self.assertEqual(data.fps, 15.0)
        self.assertEqual(data.cpu_usage_percent, 45.0)
        
        # Test serialization
        data_dict = data.to_dict()
        self.assertIsInstance(data_dict, dict)
        self.assertEqual(data_dict['component_name'], "test_component")
    
    def test_system_efficiency_calculation(self):
        """Test system efficiency calculation"""
        # Create mock component data
        components = {
            'component1': ComponentPerformanceData(
                component_name="component1",
                timestamp=time.time(),
                fps=18.0,  # Good FPS
                avg_latency_ms=100.0,  # Good latency
                max_latency_ms=150.0,
                cpu_usage_percent=20.0,  # Low CPU
                memory_usage_mb=128.0,  # Low memory
                queue_depth=2,
                error_count=0,  # No errors
                processing_time_ms=80.0
            ),
            'component2': ComponentPerformanceData(
                component_name="component2",
                timestamp=time.time(),
                fps=8.0,  # Poor FPS
                avg_latency_ms=300.0,  # High latency
                max_latency_ms=500.0,
                cpu_usage_percent=80.0,  # High CPU
                memory_usage_mb=512.0,  # High memory
                queue_depth=10,
                error_count=3,  # Errors
                processing_time_ms=250.0
            )
        }
        
        efficiency = self.aggregator._calculate_system_efficiency(components)
        
        self.assertGreaterEqual(efficiency, 0.0)
        self.assertLessEqual(efficiency, 1.0)
        # Should be moderate efficiency due to mixed performance
        self.assertGreater(efficiency, 0.2)
        self.assertLess(efficiency, 0.8)
    
    def test_resource_pressure_determination(self):
        """Test resource pressure determination"""
        # High pressure scenario
        high_pressure = self.aggregator._determine_resource_pressure(
            total_cpu=180.0,  # High total CPU
            total_memory=1500.0,  # High total memory
            efficiency=0.3  # Low efficiency
        )
        
        self.assertIn(high_pressure, ["HIGH", "CRITICAL"])
        
        # Low pressure scenario
        low_pressure = self.aggregator._determine_resource_pressure(
            total_cpu=50.0,  # Low total CPU
            total_memory=300.0,  # Low total memory
            efficiency=0.9  # High efficiency
        )
        
        self.assertIn(low_pressure, ["NONE", "LOW"])


class TestSystemIntegration(unittest.TestCase):
    """Integration tests for the complete performance optimization system"""
    
    def setUp(self):
        """Set up integration test fixtures"""
        # Mock ROS initialization for all components
        self.patches = [
            patch('enhanced_performance_profiler.rospy.init_node'),
            patch('dynamic_quality_adjuster.rospy.init_node'),
            patch('performance_alerting_system.rospy.init_node'),
            patch('performance_data_aggregator.rospy.init_node'),
            patch('resource_optimization_utils.rospy.get_param', return_value=True)
        ]
        
        for p in self.patches:
            p.start()
    
    def tearDown(self):
        """Clean up patches"""
        for p in self.patches:
            p.stop()
    
    def test_end_to_end_performance_monitoring(self):
        """Test end-to-end performance monitoring workflow"""
        # Create components
        profiler = EnhancedPerformanceProfiler()
        aggregator = PerformanceDataAggregator()
        alerting_system = PerformanceAlertingSystem()
        quality_adjuster = DynamicQualityAdjuster()
        
        # Register test component
        component_name = "integration_test_component"
        profiler.register_component(component_name, priority=7)
        
        def mock_quality_callback(settings):
            pass
        
        quality_adjuster.register_component(component_name, mock_quality_callback, priority=7)
        
        # Simulate performance data
        perf_data = {
            'component_name': component_name,
            'cpu_usage_percent': 85.0,  # High CPU usage
            'memory_usage_mb': 512.0,
            'processing_time_ms': 150.0,
            'fps': 12.0,  # Low FPS
            'queue_depth': 8,
            'error_count': 1,
            'latency_samples': [140, 150, 160, 170, 145]
        }
        
        # Create performance profile
        profile = profiler._create_performance_profile(perf_data)
        
        # Verify profile creation
        self.assertIsInstance(profile, PerformanceProfile)
        self.assertEqual(profile.component_name, component_name)
        self.assertLess(profile.resource_efficiency, 0.8)  # Should indicate poor efficiency
        
        # Test bottleneck detection
        profiles = [profile] * 5  # Simulate consistent poor performance
        bottlenecks = profiler._analyze_component_bottlenecks(component_name, profiles)
        
        self.assertGreater(len(bottlenecks), 0)
        
        # Test alert generation
        cpu_bottleneck = next((b for b in bottlenecks if b.bottleneck_type == "cpu"), None)
        if cpu_bottleneck:
            self.assertGreater(cpu_bottleneck.severity, 0.7)
    
    def test_quality_adjustment_workflow(self):
        """Test quality adjustment workflow"""
        quality_adjuster = DynamicQualityAdjuster()
        
        # Register mock component
        adjustment_applied = {'value': False, 'settings': None}
        
        def mock_quality_callback(settings):
            adjustment_applied['value'] = True
            adjustment_applied['settings'] = settings
        
        quality_adjuster.register_component("test_component", mock_quality_callback, priority=5)
        
        # Simulate high resource pressure
        from dynamic_quality_adjuster import ResourceMetrics
        high_pressure_metrics = ResourceMetrics(
            cpu_percent=92.0,
            memory_percent=88.0,
            memory_available_mb=150.0,
            temperature_celsius=79.0,
            processing_fps=9.0,
            target_fps=20.0,
            latency_ms=350.0,
            queue_depth=12,
            timestamp=time.time()
        )
        
        pressure = high_pressure_metrics.get_pressure_level(raspberry_pi_mode=True)
        self.assertEqual(pressure, ResourcePressure.CRITICAL)
        
        # Test quality adjustment assessment
        adjustment_needed, target_quality = quality_adjuster._assess_quality_adjustment(
            high_pressure_metrics, pressure
        )
        
        self.assertTrue(adjustment_needed)
        self.assertEqual(target_quality, QualityLevel.ULTRA_LOW)
    
    def test_resource_optimization_integration(self):
        """Test resource optimization integration"""
        optimizer = ResourceOptimizationUtils()
        
        # Test comprehensive monitoring and optimization
        result = optimizer.monitor_and_optimize()
        
        self.assertIsInstance(result, dict)
        self.assertIn('optimizations_applied', result)
        self.assertIn('current_usage', result)
        self.assertIn('optimization_statistics', result)
        
        # Test statistics
        stats = optimizer.get_optimization_statistics()
        self.assertIsInstance(stats, dict)
        self.assertIn('total_optimizations', stats)
        self.assertIn('successful_optimizations', stats)
        self.assertIn('memory_pools', stats)


if __name__ == '__main__':
    # Initialize ROS for testing
    rospy.init_node('test_performance_optimization_system', anonymous=True)
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test cases
    test_suite.addTest(unittest.makeSuite(TestEnhancedPerformanceProfiler))
    test_suite.addTest(unittest.makeSuite(TestDynamicQualityAdjuster))
    test_suite.addTest(unittest.makeSuite(TestPerformanceAlertingSystem))
    test_suite.addTest(unittest.makeSuite(TestResourceOptimizationUtils))
    test_suite.addTest(unittest.makeSuite(TestPerformanceDataAggregator))
    test_suite.addTest(unittest.makeSuite(TestSystemIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"Performance Optimization System Test Results")
    print(f"{'='*60}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\nFailures:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\nErrors:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    print(f"{'='*60}")
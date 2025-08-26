#!/usr/bin/env python3

import unittest
import time
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the src directory to the path so we can import the performance optimizer
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Mock dependencies before importing
sys.modules['rospy'] = Mock()
sys.modules['psutil'] = Mock()

from performance_optimizer import PerformanceOptimizer, ResourceMetrics, ModulePerformance, OptimizationStrategy


class TestPerformanceOptimization(unittest.TestCase):
    """Test suite for performance optimization functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock rospy
        self.rospy_patcher = patch('performance_optimizer.rospy')
        self.mock_rospy = self.rospy_patcher.start()
        self.mock_rospy.loginfo = Mock()
        self.mock_rospy.logdebug = Mock()
        self.mock_rospy.logwarn = Mock()
        self.mock_rospy.logerr = Mock()
        self.mock_rospy.get_param = Mock(return_value=80.0)
        self.mock_rospy.Duration = Mock()
        self.mock_rospy.Timer = Mock()
        self.mock_rospy.Rate = Mock(return_value=Mock(sleep=Mock()))
        self.mock_rospy.is_shutdown = Mock(return_value=False)
        
        # Mock psutil
        self.psutil_patcher = patch('performance_optimizer.psutil')
        self.mock_psutil = self.psutil_patcher.start()
        self.mock_psutil.cpu_percent.return_value = 50.0
        self.mock_psutil.cpu_count.return_value = 4
        self.mock_psutil.virtual_memory.return_value = Mock(percent=60.0, available=1024*1024*1024)
        self.mock_psutil.disk_io_counters.return_value = Mock(read_bytes=1024*1024, write_bytes=512*1024)
        self.mock_psutil.net_io_counters.return_value = Mock(bytes_sent=1024, bytes_recv=2048)
        self.mock_psutil.Process.return_value = Mock(
            cpu_percent=Mock(return_value=10.0),
            memory_info=Mock(return_value=Mock(rss=100*1024*1024))
        )
        
        # Create performance optimizer instance
        self.optimizer = PerformanceOptimizer("test_node")
        
        # Stop the monitoring thread for testing
        self.optimizer.monitoring_active = False
    
    def tearDown(self):
        """Clean up test fixtures"""
        self.optimizer.shutdown()
        self.rospy_patcher.stop()
        self.psutil_patcher.stop()
    
    def test_optimizer_initialization(self):
        """Test performance optimizer initialization"""
        print("\n[TEST] Testing performance optimizer initialization...")
        
        self.assertIsNotNone(self.optimizer)
        self.assertEqual(self.optimizer.node_name, "test_node")
        self.assertGreater(self.optimizer.cpu_warning_threshold, 0)
        self.assertGreater(self.optimizer.memory_warning_threshold, 0)
        self.assertGreater(self.optimizer.target_fps, 0)
        
        # Check data structures are initialized
        self.assertIsInstance(self.optimizer.registered_modules, dict)
        self.assertIsInstance(self.optimizer.module_performance, dict)
        self.assertIsInstance(self.optimizer.quality_levels, dict)
        
        print("✓ Performance optimizer initialization test passed")
    
    def test_module_registration(self):
        """Test module registration and unregistration"""
        print("\n[TEST] Testing module registration...")
        
        # Test successful registration
        callbacks = {
            'reduce_quality': lambda params: None,
            'skip_frames': lambda params: None
        }
        
        success = self.optimizer.register_module(
            module_name="test_module",
            priority=7,
            optimization_callbacks=callbacks
        )
        
        self.assertTrue(success)
        self.assertIn("test_module", self.optimizer.registered_modules)
        self.assertEqual(self.optimizer.module_priorities["test_module"], 7)
        self.assertEqual(self.optimizer.quality_levels["test_module"], 1.0)
        
        # Test unregistration
        success = self.optimizer.unregister_module("test_module")
        self.assertTrue(success)
        self.assertNotIn("test_module", self.optimizer.registered_modules)
        
        # Test unregistering non-existent module
        success = self.optimizer.unregister_module("non_existent")
        self.assertFalse(success)
        
        print("✓ Module registration test passed")
    
    def test_resource_metrics_collection(self):
        """Test resource metrics collection"""
        print("\n[TEST] Testing resource metrics collection...")
        
        metrics = self.optimizer._collect_resource_metrics()
        
        self.assertIsInstance(metrics, ResourceMetrics)
        self.assertGreaterEqual(metrics.cpu_percent, 0)
        self.assertGreaterEqual(metrics.memory_percent, 0)
        self.assertGreaterEqual(metrics.memory_available_mb, 0)
        self.assertIsInstance(metrics.timestamp, float)
        
        print("✓ Resource metrics collection test passed")
    
    def test_performance_update(self):
        """Test module performance updates"""
        print("\n[TEST] Testing performance updates...")
        
        # Register a module first
        self.optimizer.register_module("test_module", priority=5)
        
        # Update performance
        self.optimizer.update_module_performance(
            module_name="test_module",
            processing_time=0.05,
            fps=20.0,
            queue_size=2,
            error_count=0
        )
        
        # Check performance was recorded
        self.assertIn("test_module", self.optimizer.module_performance)
        performance_history = self.optimizer.module_performance["test_module"]
        self.assertGreater(len(performance_history), 0)
        
        latest_performance = performance_history[-1]
        self.assertEqual(latest_performance.module_name, "test_module")
        self.assertEqual(latest_performance.fps, 20.0)
        self.assertEqual(latest_performance.processing_time, 0.05)
        
        print("✓ Performance update test passed")
    
    def test_optimization_need_assessment(self):
        """Test optimization need assessment"""
        print("\n[TEST] Testing optimization need assessment...")
        
        # Test with normal metrics - should not need optimization
        normal_metrics = ResourceMetrics(
            cpu_percent=50.0,
            memory_percent=60.0,
            memory_available_mb=1024.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        
        need_optimization = self.optimizer._assess_optimization_need(normal_metrics)
        self.assertFalse(need_optimization)
        
        # Test with high CPU - should need optimization
        high_cpu_metrics = ResourceMetrics(
            cpu_percent=90.0,
            memory_percent=60.0,
            memory_available_mb=1024.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        
        need_optimization = self.optimizer._assess_optimization_need(high_cpu_metrics)
        self.assertTrue(need_optimization)
        
        print("✓ Optimization need assessment test passed")
    
    def test_optimization_strategy_selection(self):
        """Test optimization strategy selection"""
        print("\n[TEST] Testing optimization strategy selection...")
        
        # Create test performance data
        performance = ModulePerformance(
            module_name="test_module",
            processing_time=0.1,
            fps=15.0,  # Below target
            cpu_usage=20.0,
            memory_usage_mb=100.0,
            queue_size=3,
            error_count=0,
            timestamp=time.time()
        )
        
        # Test with high CPU metrics
        high_cpu_metrics = ResourceMetrics(
            cpu_percent=98.0,  # Critical level
            memory_percent=60.0,
            memory_available_mb=1024.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        
        strategy = self.optimizer._select_optimization_strategy("test_module", performance, high_cpu_metrics)
        self.assertIn(strategy, [OptimizationStrategy.SKIP_FRAMES, OptimizationStrategy.REDUCE_QUALITY])
        
        # Test with high memory metrics
        high_memory_metrics = ResourceMetrics(
            cpu_percent=50.0,
            memory_percent=98.0,  # Critical level
            memory_available_mb=50.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        
        strategy = self.optimizer._select_optimization_strategy("test_module", performance, high_memory_metrics)
        self.assertEqual(strategy, OptimizationStrategy.CACHE_RESULTS)
        
        print("✓ Optimization strategy selection test passed")
    
    def test_optimization_execution(self):
        """Test optimization execution"""
        print("\n[TEST] Testing optimization execution...")
        
        # Register module with callback
        callback_called = False
        def test_callback(params):
            nonlocal callback_called
            callback_called = True
        
        callbacks = {'reduce_quality': test_callback}
        self.optimizer.register_module("test_module", priority=5, optimization_callbacks=callbacks)
        
        # Create test data
        performance = ModulePerformance(
            module_name="test_module",
            processing_time=0.1,
            fps=15.0,
            cpu_usage=20.0,
            memory_usage_mb=100.0,
            queue_size=3,
            error_count=0,
            timestamp=time.time()
        )
        
        metrics = ResourceMetrics(
            cpu_percent=90.0,
            memory_percent=60.0,
            memory_available_mb=1024.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        
        # Execute optimization
        success = self.optimizer._execute_optimization(
            "test_module",
            OptimizationStrategy.REDUCE_QUALITY,
            performance,
            metrics
        )
        
        self.assertTrue(success)
        self.assertTrue(callback_called)
        self.assertIn("test_module", self.optimizer.active_optimizations)
        self.assertLess(self.optimizer.quality_levels["test_module"], 1.0)
        
        print("✓ Optimization execution test passed")
    
    def test_quality_level_management(self):
        """Test quality level management"""
        print("\n[TEST] Testing quality level management...")
        
        # Register module
        self.optimizer.register_module("test_module", priority=5)
        
        # Test manual quality setting
        success = self.optimizer.set_module_quality("test_module", 0.7)
        self.assertTrue(success)
        self.assertEqual(self.optimizer.quality_levels["test_module"], 0.7)
        
        # Test quality bounds
        success = self.optimizer.set_module_quality("test_module", 1.5)  # Above max
        self.assertTrue(success)
        self.assertEqual(self.optimizer.quality_levels["test_module"], 1.0)
        
        success = self.optimizer.set_module_quality("test_module", -0.1)  # Below min
        self.assertTrue(success)
        self.assertEqual(self.optimizer.quality_levels["test_module"], 0.1)
        
        # Test setting quality for non-existent module
        success = self.optimizer.set_module_quality("non_existent", 0.5)
        self.assertFalse(success)
        
        print("✓ Quality level management test passed")
    
    def test_optimization_reset(self):
        """Test optimization reset functionality"""
        print("\n[TEST] Testing optimization reset...")
        
        # Register module and apply some optimizations
        self.optimizer.register_module("test_module", priority=5)
        self.optimizer.quality_levels["test_module"] = 0.6
        self.optimizer.frame_skip_counters["test_module"] = 2
        self.optimizer.module_load_factors["test_module"] = 0.8
        
        # Reset optimizations
        success = self.optimizer.reset_module_optimizations("test_module")
        self.assertTrue(success)
        
        # Check values are reset
        self.assertEqual(self.optimizer.quality_levels["test_module"], 1.0)
        self.assertEqual(self.optimizer.frame_skip_counters["test_module"], 0)
        self.assertEqual(self.optimizer.module_load_factors["test_module"], 1.0)
        
        # Test reset for non-existent module
        success = self.optimizer.reset_module_optimizations("non_existent")
        self.assertFalse(success)
        
        print("✓ Optimization reset test passed")
    
    def test_resource_violation_detection(self):
        """Test resource violation detection"""
        print("\n[TEST] Testing resource violation detection...")
        
        initial_violations = self.optimizer.resource_violations
        
        # Test with normal metrics
        normal_metrics = ResourceMetrics(
            cpu_percent=50.0,
            memory_percent=60.0,
            memory_available_mb=1024.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        
        self.optimizer._check_resource_violations(normal_metrics)
        self.assertEqual(self.optimizer.resource_violations, initial_violations)
        
        # Test with high resource usage
        high_usage_metrics = ResourceMetrics(
            cpu_percent=98.0,  # Critical
            memory_percent=97.0,  # Critical
            memory_available_mb=50.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        
        self.optimizer._check_resource_violations(high_usage_metrics)
        self.assertGreater(self.optimizer.resource_violations, initial_violations)
        
        print("✓ Resource violation detection test passed")
    
    def test_optimization_statistics(self):
        """Test optimization statistics collection"""
        print("\n[TEST] Testing optimization statistics...")
        
        # Register module and add some performance data
        self.optimizer.register_module("test_module", priority=5)
        
        # Add resource history
        metrics = ResourceMetrics(
            cpu_percent=70.0,
            memory_percent=65.0,
            memory_available_mb=1024.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        self.optimizer.resource_history.append(metrics)
        
        # Add performance history
        performance = ModulePerformance(
            module_name="test_module",
            processing_time=0.05,
            fps=20.0,
            cpu_usage=15.0,
            memory_usage_mb=80.0,
            queue_size=1,
            error_count=0,
            timestamp=time.time()
        )
        self.optimizer.module_performance["test_module"].append(performance)
        
        # Get statistics
        stats = self.optimizer.get_optimization_statistics()
        
        # Verify statistics structure
        self.assertIn('total_optimizations', stats)
        self.assertIn('successful_optimizations', stats)
        self.assertIn('success_rate', stats)
        self.assertIn('resource_violations', stats)
        self.assertIn('average_cpu_usage', stats)
        self.assertIn('average_memory_usage', stats)
        self.assertIn('module_statistics', stats)
        
        # Verify module statistics
        self.assertIn('test_module', stats['module_statistics'])
        module_stats = stats['module_statistics']['test_module']
        self.assertIn('average_fps', module_stats)
        self.assertIn('current_quality', module_stats)
        
        print("✓ Optimization statistics test passed")
    
    def test_load_balancing_optimization(self):
        """Test load balancing optimization"""
        print("\n[TEST] Testing load balancing optimization...")
        
        # Register multiple modules with different priorities
        self.optimizer.register_module("high_priority", priority=9)
        self.optimizer.register_module("low_priority", priority=3)
        
        # Add performance data showing high queue for low priority module
        high_queue_performance = ModulePerformance(
            module_name="low_priority",
            processing_time=0.2,
            fps=10.0,
            cpu_usage=30.0,
            memory_usage_mb=150.0,
            queue_size=8,  # High queue
            error_count=0,
            timestamp=time.time()
        )
        
        self.optimizer.module_performance["low_priority"].append(high_queue_performance)
        
        # Test strategy selection
        metrics = ResourceMetrics(
            cpu_percent=70.0,
            memory_percent=65.0,
            memory_available_mb=1024.0,
            disk_io_read_mb=10.0,
            disk_io_write_mb=5.0,
            network_bytes_sent=1024,
            network_bytes_recv=2048,
            timestamp=time.time()
        )
        
        strategy = self.optimizer._select_optimization_strategy("low_priority", high_queue_performance, metrics)
        self.assertEqual(strategy, OptimizationStrategy.LOAD_BALANCE)
        
        # Execute load balancing
        initial_load_factor = self.optimizer.module_load_factors["low_priority"]
        success = self.optimizer._execute_optimization("low_priority", strategy, high_queue_performance, metrics)
        
        self.assertTrue(success)
        self.assertLess(self.optimizer.module_load_factors["low_priority"], initial_load_factor)
        
        print("✓ Load balancing optimization test passed")


def run_performance_optimization_tests():
    """Run all performance optimization tests"""
    print("=" * 70)
    print("RUNNING PERFORMANCE OPTIMIZATION TESTS")
    print("=" * 70)
    
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestPerformanceOptimization)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "=" * 70)
    print("PERFORMANCE OPTIMIZATION TEST SUMMARY")
    print("=" * 70)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print("\nFAILURES:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print("\nERRORS:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    success = len(result.failures) == 0 and len(result.errors) == 0
    print(f"\nOverall result: {'PASS' if success else 'FAIL'}")
    
    return success


if __name__ == '__main__':
    run_performance_optimization_tests()
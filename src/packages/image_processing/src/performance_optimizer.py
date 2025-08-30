#!/usr/bin/env python3

import time
import threading
import psutil
import os
from typing import Dict, List, Optional, Any, Callable, Tuple
from dataclasses import dataclass, asdict
from enum import Enum
import rospy
from collections import deque, defaultdict
import numpy as np


class ResourceType(Enum):
    """Types of system resources to monitor"""
    CPU = "cpu"
    MEMORY = "memory"
    GPU = "gpu"
    DISK_IO = "disk_io"
    NETWORK = "network"


class OptimizationStrategy(Enum):
    """Performance optimization strategies"""
    REDUCE_QUALITY = "reduce_quality"
    SKIP_FRAMES = "skip_frames"
    LOAD_BALANCE = "load_balance"
    CACHE_RESULTS = "cache_results"
    PARALLEL_PROCESSING = "parallel_processing"
    ADAPTIVE_RATE = "adaptive_rate"


@dataclass
class ResourceMetrics:
    """System resource metrics"""
    cpu_percent: float
    memory_percent: float
    memory_available_mb: float
    disk_io_read_mb: float
    disk_io_write_mb: float
    network_bytes_sent: int
    network_bytes_recv: int
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ModulePerformance:
    """Performance metrics for a processing module"""
    module_name: str
    processing_time: float
    fps: float
    cpu_usage: float
    memory_usage_mb: float
    queue_size: int
    error_count: int
    timestamp: float
    quality_level: float = 1.0
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class OptimizationAction:
    """Represents an optimization action taken"""
    strategy: OptimizationStrategy
    module_name: str
    parameters: Dict[str, Any]
    expected_benefit: float
    timestamp: float
    success: bool = False
    actual_benefit: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class PerformanceOptimizer:
    """
    Dynamic resource allocation and performance optimization system.
    Monitors system resources and automatically adjusts processing parameters
    to maintain optimal performance under varying load conditions.
    """
    
    def __init__(self, node_name: str):
        self.node_name = node_name
        
        # Configuration parameters
        self.cpu_warning_threshold = rospy.get_param("~cpu_warning_threshold", 80.0)
        self.cpu_critical_threshold = rospy.get_param("~cpu_critical_threshold", 95.0)
        self.memory_warning_threshold = rospy.get_param("~memory_warning_threshold", 80.0)
        self.memory_critical_threshold = rospy.get_param("~memory_critical_threshold", 95.0)
        self.target_fps = rospy.get_param("~target_fps", 20.0)
        self.min_fps = rospy.get_param("~min_fps", 10.0)
        self.optimization_interval = rospy.get_param("~optimization_interval", 2.0)  # seconds
        
        # Resource monitoring
        self.resource_history: deque = deque(maxlen=100)
        self.module_performance: Dict[str, deque] = defaultdict(lambda: deque(maxlen=50))
        self.registered_modules: Dict[str, Dict[str, Any]] = {}
        
        # Load balancing
        self.module_priorities: Dict[str, int] = {}
        self.module_load_factors: Dict[str, float] = {}
        self.processing_queues: Dict[str, int] = {}
        
        # Optimization state
        self.active_optimizations: Dict[str, OptimizationAction] = {}
        self.optimization_history: List[OptimizationAction] = []
        self.quality_levels: Dict[str, float] = {}
        self.frame_skip_counters: Dict[str, int] = {}
        
        # Performance statistics
        self.total_optimizations = 0
        self.successful_optimizations = 0
        self.resource_violations = 0
        self.performance_improvements = 0.0
        
        # Threading
        self.monitoring_active = True
        self.optimization_lock = threading.Lock()
        
        # Start monitoring thread
        self.monitoring_thread = threading.Thread(target=self._monitoring_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        # Optimization timer
        self.optimization_timer = rospy.Timer(
            rospy.Duration(self.optimization_interval),
            self._optimization_callback
        )
        
        rospy.loginfo(f"[{self.node_name}] PerformanceOptimizer initialized")
        rospy.loginfo(f"[{self.node_name}] CPU thresholds: {self.cpu_warning_threshold}%/{self.cpu_critical_threshold}%")
        rospy.loginfo(f"[{self.node_name}] Memory thresholds: {self.memory_warning_threshold}%/{self.memory_critical_threshold}%")
        rospy.loginfo(f"[{self.node_name}] Target FPS: {self.target_fps}, Min FPS: {self.min_fps}")
        rospy.loginfo(f"[{self.node_name}] Optimization interval: {self.optimization_interval}s")
    
    def register_module(self, module_name: str, priority: int = 5, 
                       optimization_callbacks: Dict[str, Callable] = None) -> bool:
        """
        Register a processing module for optimization.
        
        Args:
            module_name: Name of the module
            priority: Priority level (1-10, higher = more important)
            optimization_callbacks: Dictionary of optimization callbacks
            
        Returns:
            bool: True if registration successful
        """
        with self.optimization_lock:
            if optimization_callbacks is None:
                optimization_callbacks = {}
            
            self.registered_modules[module_name] = {
                'priority': priority,
                'callbacks': optimization_callbacks,
                'registration_time': time.time(),
                'optimization_count': 0,
                'last_optimization': None
            }
            
            # Initialize module state
            self.module_priorities[module_name] = priority
            self.module_load_factors[module_name] = 1.0
            self.processing_queues[module_name] = 0
            self.quality_levels[module_name] = 1.0
            self.frame_skip_counters[module_name] = 0
            
            rospy.loginfo(f"[{self.node_name}] Registered module '{module_name}' with priority {priority}")
            rospy.logdebug(f"[{self.node_name}] Available optimization callbacks: {list(optimization_callbacks.keys())}")
            
            return True
    
    def unregister_module(self, module_name: str) -> bool:
        """Unregister a processing module"""
        with self.optimization_lock:
            if module_name in self.registered_modules:
                del self.registered_modules[module_name]
                
                # Clean up module state
                self.module_priorities.pop(module_name, None)
                self.module_load_factors.pop(module_name, None)
                self.processing_queues.pop(module_name, None)
                self.quality_levels.pop(module_name, None)
                self.frame_skip_counters.pop(module_name, None)
                self.active_optimizations.pop(module_name, None)
                
                rospy.loginfo(f"[{self.node_name}] Unregistered module '{module_name}'")
                return True
            
            return False
    
    def update_module_performance(self, module_name: str, processing_time: float, 
                                fps: float, queue_size: int = 0, error_count: int = 0) -> None:
        """
        Update performance metrics for a module.
        
        Args:
            module_name: Name of the module
            processing_time: Processing time in seconds
            fps: Current frames per second
            queue_size: Current processing queue size
            error_count: Number of errors since last update
        """
        if module_name not in self.registered_modules:
            rospy.logwarn(f"[{self.node_name}] Performance update for unregistered module: {module_name}")
            return
        
        current_time = time.time()
        
        # Estimate CPU and memory usage for this module
        try:
            process = psutil.Process()
            cpu_usage = process.cpu_percent(interval=None) / psutil.cpu_count()
            memory_usage_mb = process.memory_info().rss / 1024 / 1024
        except:
            cpu_usage = 0.0
            memory_usage_mb = 0.0
        
        performance = ModulePerformance(
            module_name=module_name,
            processing_time=processing_time,
            fps=fps,
            cpu_usage=cpu_usage,
            memory_usage_mb=memory_usage_mb,
            queue_size=queue_size,
            error_count=error_count,
            timestamp=current_time,
            quality_level=self.quality_levels.get(module_name, 1.0)
        )
        
        self.module_performance[module_name].append(performance)
        self.processing_queues[module_name] = queue_size
        
        rospy.logdebug(f"[{self.node_name}] Performance update for '{module_name}': "
                      f"FPS={fps:.1f}, Time={processing_time:.3f}s, Queue={queue_size}")
        
        # Real-time monitoring logs
        rospy.logdebug(f"[{self.node_name}] Module performance event: {module_name}, "
                      f"FPS={fps:.1f}, CPU={cpu_usage:.1f}%, Memory={memory_usage_mb:.1f}MB")
    
    def _monitoring_loop(self):
        """Main resource monitoring loop"""
        rate = rospy.Rate(1.0)  # 1 Hz monitoring
        
        while self.monitoring_active and not rospy.is_shutdown():
            try:
                current_time = time.time()
                
                # Collect system resource metrics
                metrics = self._collect_resource_metrics()
                self.resource_history.append(metrics)
                
                # Check for resource violations
                self._check_resource_violations(metrics)
                
                # Log resource monitoring events
                rospy.logdebug(f"[{self.node_name}] Resource monitoring: "
                              f"CPU={metrics.cpu_percent:.1f}%, Memory={metrics.memory_percent:.1f}%")
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"[{self.node_name}] Resource monitoring error: {e}")
                rate.sleep()
    
    def _collect_resource_metrics(self) -> ResourceMetrics:
        """Collect current system resource metrics"""
        try:
            # CPU metrics
            cpu_percent = psutil.cpu_percent(interval=0.1)
            
            # Memory metrics
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            memory_available_mb = memory.available / 1024 / 1024
            
            # Disk I/O metrics
            disk_io = psutil.disk_io_counters()
            disk_io_read_mb = disk_io.read_bytes / 1024 / 1024 if disk_io else 0.0
            disk_io_write_mb = disk_io.write_bytes / 1024 / 1024 if disk_io else 0.0
            
            # Network metrics
            network_io = psutil.net_io_counters()
            network_bytes_sent = network_io.bytes_sent if network_io else 0
            network_bytes_recv = network_io.bytes_recv if network_io else 0
            
            return ResourceMetrics(
                cpu_percent=cpu_percent,
                memory_percent=memory_percent,
                memory_available_mb=memory_available_mb,
                disk_io_read_mb=disk_io_read_mb,
                disk_io_write_mb=disk_io_write_mb,
                network_bytes_sent=network_bytes_sent,
                network_bytes_recv=network_bytes_recv,
                timestamp=time.time()
            )
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error collecting resource metrics: {e}")
            return ResourceMetrics(0, 0, 0, 0, 0, 0, 0, time.time())
    
    def _check_resource_violations(self, metrics: ResourceMetrics):
        """Check for resource threshold violations"""
        violations = []
        
        if metrics.cpu_percent > self.cpu_critical_threshold:
            violations.append(f"CPU critical: {metrics.cpu_percent:.1f}%")
        elif metrics.cpu_percent > self.cpu_warning_threshold:
            violations.append(f"CPU warning: {metrics.cpu_percent:.1f}%")
        
        if metrics.memory_percent > self.memory_critical_threshold:
            violations.append(f"Memory critical: {metrics.memory_percent:.1f}%")
        elif metrics.memory_percent > self.memory_warning_threshold:
            violations.append(f"Memory warning: {metrics.memory_percent:.1f}%")
        
        if violations:
            self.resource_violations += 1
            for violation in violations:
                rospy.logwarn(f"[{self.node_name}] Resource violation: {violation}")
            
            # Real-time monitoring
            rospy.logwarn(f"[{self.node_name}] Resource violation event: {len(violations)} violations detected")
    
    def _optimization_callback(self, event):
        """Periodic optimization callback"""
        if not self.monitoring_active:
            return
        
        try:
            with self.optimization_lock:
                current_time = time.time()
                
                # Get current resource state
                if not self.resource_history:
                    return
                
                latest_metrics = self.resource_history[-1]
                
                rospy.logdebug(f"[{self.node_name}] Running optimization cycle")
                rospy.logdebug(f"[{self.node_name}] Current resources: CPU={latest_metrics.cpu_percent:.1f}%, "
                              f"Memory={latest_metrics.memory_percent:.1f}%")
                
                # Determine if optimization is needed
                optimization_needed = self._assess_optimization_need(latest_metrics)
                
                if optimization_needed:
                    rospy.loginfo(f"[{self.node_name}] Optimization needed - analyzing modules")
                    
                    # Analyze module performance and apply optimizations
                    optimizations_applied = self._apply_optimizations(latest_metrics)
                    
                    if optimizations_applied > 0:
                        rospy.loginfo(f"[{self.node_name}] Applied {optimizations_applied} optimizations")
                        self.total_optimizations += optimizations_applied
                        
                        # Real-time monitoring
                        rospy.loginfo(f"[{self.node_name}] Optimization event: {optimizations_applied} actions applied")
                    else:
                        rospy.logdebug(f"[{self.node_name}] No optimizations applied this cycle")
                else:
                    rospy.logdebug(f"[{self.node_name}] System performance within acceptable limits")
                
                # Log performance optimization events
                rospy.logdebug(f"[{self.node_name}] Optimization cycle completed: "
                              f"Total optimizations={self.total_optimizations}, "
                              f"Success rate={self.successful_optimizations/max(1, self.total_optimizations):.2%}")
                
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Optimization callback error: {e}")
    
    def _assess_optimization_need(self, metrics: ResourceMetrics) -> bool:
        """Assess if optimization is needed based on current metrics"""
        # Check CPU usage
        if metrics.cpu_percent > self.cpu_warning_threshold:
            rospy.logdebug(f"[{self.node_name}] CPU optimization needed: {metrics.cpu_percent:.1f}%")
            return True
        
        # Check memory usage
        if metrics.memory_percent > self.memory_warning_threshold:
            rospy.logdebug(f"[{self.node_name}] Memory optimization needed: {metrics.memory_percent:.1f}%")
            return True
        
        # Check module performance
        for module_name, performance_history in self.module_performance.items():
            if not performance_history:
                continue
            
            latest_performance = performance_history[-1]
            
            # Check if FPS is below target
            if latest_performance.fps < self.min_fps:
                rospy.logdebug(f"[{self.node_name}] FPS optimization needed for '{module_name}': {latest_performance.fps:.1f}")
                return True
            
            # Check if processing queue is growing
            if latest_performance.queue_size > 10:
                rospy.logdebug(f"[{self.node_name}] Queue optimization needed for '{module_name}': {latest_performance.queue_size}")
                return True
        
        return False
    
    def _apply_optimizations(self, metrics: ResourceMetrics) -> int:
        """Apply performance optimizations based on current state"""
        optimizations_applied = 0
        
        # Sort modules by priority (lower priority modules get optimized first)
        sorted_modules = sorted(
            self.registered_modules.items(),
            key=lambda x: x[1]['priority']
        )
        
        for module_name, module_info in sorted_modules:
            if module_name not in self.module_performance:
                continue
            
            performance_history = self.module_performance[module_name]
            if not performance_history:
                continue
            
            latest_performance = performance_history[-1]
            
            # Determine optimization strategy
            strategy = self._select_optimization_strategy(module_name, latest_performance, metrics)
            
            if strategy:
                success = self._execute_optimization(module_name, strategy, latest_performance, metrics)
                if success:
                    optimizations_applied += 1
                    self.successful_optimizations += 1
                    
                    rospy.loginfo(f"[{self.node_name}] Applied {strategy.value} optimization to '{module_name}'")
                    
                    # Real-time monitoring
                    rospy.logdebug(f"[{self.node_name}] Optimization applied: {module_name} -> {strategy.value}")
        
        return optimizations_applied
    
    def _select_optimization_strategy(self, module_name: str, performance: ModulePerformance, 
                                    metrics: ResourceMetrics) -> Optional[OptimizationStrategy]:
        """Select appropriate optimization strategy for a module"""
        
        # High CPU usage - reduce quality or skip frames
        if metrics.cpu_percent > self.cpu_critical_threshold:
            if performance.fps < self.min_fps:
                return OptimizationStrategy.SKIP_FRAMES
            else:
                return OptimizationStrategy.REDUCE_QUALITY
        
        # High memory usage - cache management
        if metrics.memory_percent > self.memory_critical_threshold:
            return OptimizationStrategy.CACHE_RESULTS
        
        # High queue size - load balancing (check before FPS)
        if performance.queue_size > 5:
            return OptimizationStrategy.LOAD_BALANCE
        
        # Low FPS - adaptive rate control
        if performance.fps < self.target_fps * 0.8:
            return OptimizationStrategy.ADAPTIVE_RATE
        
        # Moderate resource usage - try parallel processing
        if (metrics.cpu_percent > self.cpu_warning_threshold and 
            metrics.cpu_percent < self.cpu_critical_threshold):
            return OptimizationStrategy.PARALLEL_PROCESSING
        
        return None
    
    def _execute_optimization(self, module_name: str, strategy: OptimizationStrategy,
                            performance: ModulePerformance, metrics: ResourceMetrics) -> bool:
        """Execute a specific optimization strategy"""
        
        try:
            parameters = {}
            expected_benefit = 0.0
            
            if strategy == OptimizationStrategy.REDUCE_QUALITY:
                # Reduce quality level
                current_quality = self.quality_levels[module_name]
                new_quality = max(0.5, current_quality * 0.8)
                self.quality_levels[module_name] = new_quality
                parameters = {'quality_level': new_quality}
                expected_benefit = (current_quality - new_quality) * 20.0  # Estimated CPU reduction
                
                rospy.loginfo(f"[{self.node_name}] Reducing quality for '{module_name}': {current_quality:.2f} -> {new_quality:.2f}")
            
            elif strategy == OptimizationStrategy.SKIP_FRAMES:
                # Increase frame skip counter
                current_skip = self.frame_skip_counters[module_name]
                new_skip = min(3, current_skip + 1)
                self.frame_skip_counters[module_name] = new_skip
                parameters = {'frame_skip': new_skip}
                expected_benefit = new_skip * 10.0  # Estimated CPU reduction
                
                rospy.loginfo(f"[{self.node_name}] Increasing frame skip for '{module_name}': {current_skip} -> {new_skip}")
            
            elif strategy == OptimizationStrategy.ADAPTIVE_RATE:
                # Adjust processing rate based on performance
                target_rate = max(self.min_fps, performance.fps * 1.2)
                parameters = {'target_rate': target_rate}
                expected_benefit = 5.0
                
                rospy.loginfo(f"[{self.node_name}] Adjusting rate for '{module_name}': target={target_rate:.1f} FPS")
            
            elif strategy == OptimizationStrategy.LOAD_BALANCE:
                # Adjust load factor
                current_factor = self.module_load_factors[module_name]
                new_factor = max(0.5, current_factor * 0.9)
                self.module_load_factors[module_name] = new_factor
                parameters = {'load_factor': new_factor}
                expected_benefit = (current_factor - new_factor) * 15.0
                
                rospy.loginfo(f"[{self.node_name}] Load balancing for '{module_name}': {current_factor:.2f} -> {new_factor:.2f}")
            
            else:
                # Generic optimization
                parameters = {'strategy': strategy.value}
                expected_benefit = 5.0
                
                rospy.loginfo(f"[{self.node_name}] Applying {strategy.value} to '{module_name}'")
            
            # Record optimization action
            action = OptimizationAction(
                strategy=strategy,
                module_name=module_name,
                parameters=parameters,
                expected_benefit=expected_benefit,
                timestamp=time.time(),
                success=True
            )
            
            self.active_optimizations[module_name] = action
            self.optimization_history.append(action)
            
            # Execute callback if available
            module_info = self.registered_modules[module_name]
            callback = module_info['callbacks'].get(strategy.value)
            
            if callback:
                try:
                    callback(parameters)
                    rospy.logdebug(f"[{self.node_name}] Executed optimization callback for '{module_name}'")
                except Exception as e:
                    rospy.logerr(f"[{self.node_name}] Optimization callback error for '{module_name}': {e}")
                    action.success = False
                    return False
            
            # Update module optimization count
            module_info['optimization_count'] += 1
            module_info['last_optimization'] = time.time()
            
            # Log optimization actions with detailed debug prints
            rospy.logdebug(f"[{self.node_name}] Optimization action executed:")
            rospy.logdebug(f"[{self.node_name}] Module: {module_name}")
            rospy.logdebug(f"[{self.node_name}] Strategy: {strategy.value}")
            rospy.logdebug(f"[{self.node_name}] Parameters: {parameters}")
            rospy.logdebug(f"[{self.node_name}] Expected benefit: {expected_benefit:.1f}")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Optimization execution event: {module_name}, "
                          f"Strategy={strategy.value}, Benefit={expected_benefit:.1f}")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Optimization execution error: {e}")
            return False
    
    def get_optimization_statistics(self) -> Dict[str, Any]:
        """Get performance optimization statistics"""
        with self.optimization_lock:
            current_time = time.time()
            
            # Calculate average resource usage
            if self.resource_history:
                recent_metrics = list(self.resource_history)[-10:]  # Last 10 measurements
                avg_cpu = sum(m.cpu_percent for m in recent_metrics) / len(recent_metrics)
                avg_memory = sum(m.memory_percent for m in recent_metrics) / len(recent_metrics)
            else:
                avg_cpu = 0.0
                avg_memory = 0.0
            
            # Module statistics
            module_stats = {}
            for module_name, performance_history in self.module_performance.items():
                if performance_history:
                    recent_performance = list(performance_history)[-5:]  # Last 5 measurements
                    avg_fps = sum(p.fps for p in recent_performance) / len(recent_performance)
                    avg_processing_time = sum(p.processing_time for p in recent_performance) / len(recent_performance)
                    
                    module_stats[module_name] = {
                        'average_fps': avg_fps,
                        'average_processing_time': avg_processing_time,
                        'current_quality': self.quality_levels.get(module_name, 1.0),
                        'frame_skip_count': self.frame_skip_counters.get(module_name, 0),
                        'load_factor': self.module_load_factors.get(module_name, 1.0),
                        'optimization_count': self.registered_modules.get(module_name, {}).get('optimization_count', 0)
                    }
            
            return {
                'total_optimizations': self.total_optimizations,
                'successful_optimizations': self.successful_optimizations,
                'success_rate': self.successful_optimizations / max(1, self.total_optimizations),
                'resource_violations': self.resource_violations,
                'average_cpu_usage': avg_cpu,
                'average_memory_usage': avg_memory,
                'registered_modules': len(self.registered_modules),
                'active_optimizations': len(self.active_optimizations),
                'module_statistics': module_stats,
                'optimization_history_length': len(self.optimization_history)
            }
    
    def get_current_quality_settings(self) -> Dict[str, float]:
        """Get current quality settings for all modules"""
        return dict(self.quality_levels)
    
    def set_module_quality(self, module_name: str, quality_level: float) -> bool:
        """Manually set quality level for a module"""
        if module_name not in self.registered_modules:
            return False
        
        quality_level = max(0.1, min(1.0, quality_level))
        self.quality_levels[module_name] = quality_level
        
        rospy.loginfo(f"[{self.node_name}] Manual quality adjustment for '{module_name}': {quality_level:.2f}")
        
        return True
    
    def reset_module_optimizations(self, module_name: str) -> bool:
        """Reset all optimizations for a specific module"""
        if module_name not in self.registered_modules:
            return False
        
        with self.optimization_lock:
            self.quality_levels[module_name] = 1.0
            self.frame_skip_counters[module_name] = 0
            self.module_load_factors[module_name] = 1.0
            self.active_optimizations.pop(module_name, None)
            
            rospy.loginfo(f"[{self.node_name}] Reset optimizations for module '{module_name}'")
            
            return True
    
    def shutdown(self):
        """Shutdown the performance optimizer"""
        rospy.loginfo(f"[{self.node_name}] PerformanceOptimizer shutting down")
        
        self.monitoring_active = False
        
        if hasattr(self, 'monitoring_thread') and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=2.0)
        
        if hasattr(self, 'optimization_timer'):
            self.optimization_timer.shutdown()
        
        # Log final statistics
        stats = self.get_optimization_statistics()
        rospy.loginfo(f"[{self.node_name}] Final optimization statistics:")
        rospy.loginfo(f"[{self.node_name}] Total optimizations: {stats['total_optimizations']}")
        rospy.loginfo(f"[{self.node_name}] Success rate: {stats['success_rate']:.2%}")
        rospy.loginfo(f"[{self.node_name}] Resource violations: {stats['resource_violations']}")
        rospy.loginfo(f"[{self.node_name}] Registered modules: {stats['registered_modules']}")
        
        rospy.loginfo(f"[{self.node_name}] PerformanceOptimizer shutdown complete")
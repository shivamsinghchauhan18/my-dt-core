#!/usr/bin/env python3
"""
Resource Optimization Utilities for Enhanced Autonomous Navigation

Provides comprehensive resource optimization utilities for memory usage,
CPU utilization, and system performance on Raspberry Pi deployment.

Author: Duckietown
"""

import rospy
import time
import threading
import gc
import os
import psutil
import mmap
from typing import Dict, List, Optional, Any, Tuple, Callable, Union
from collections import deque, defaultdict
from dataclasses import dataclass, asdict
from enum import Enum
import numpy as np
import cv2

# Memory management
try:
    import resource
    RESOURCE_MODULE_AVAILABLE = True
except ImportError:
    RESOURCE_MODULE_AVAILABLE = False


class OptimizationStrategy(Enum):
    """Resource optimization strategies"""
    MEMORY_POOLING = "memory_pooling"
    GARBAGE_COLLECTION = "garbage_collection"
    BUFFER_MANAGEMENT = "buffer_management"
    CPU_AFFINITY = "cpu_affinity"
    PROCESS_PRIORITY = "process_priority"
    CACHE_OPTIMIZATION = "cache_optimization"
    MEMORY_MAPPING = "memory_mapping"
    LAZY_LOADING = "lazy_loading"


@dataclass
class ResourceUsage:
    """Current resource usage metrics"""
    timestamp: float
    cpu_percent: float
    memory_rss_mb: float
    memory_vms_mb: float
    memory_percent: float
    memory_available_mb: float
    open_files: int
    threads: int
    gpu_memory_mb: Optional[float]
    temperature_celsius: Optional[float]
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class OptimizationResult:
    """Result of an optimization operation"""
    strategy: OptimizationStrategy
    timestamp: float
    success: bool
    before_usage: ResourceUsage
    after_usage: ResourceUsage
    improvement_percent: float
    description: str
    duration_seconds: float
    
    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data['strategy'] = self.strategy.value
        return data


class MemoryPool:
    """Memory pool for efficient buffer management"""
    
    def __init__(self, buffer_size: int, pool_size: int = 10):
        self.buffer_size = buffer_size
        self.pool_size = pool_size
        self.available_buffers = deque()
        self.used_buffers = set()
        self.lock = threading.Lock()
        
        # Pre-allocate buffers
        for _ in range(pool_size):
            buffer = np.empty(buffer_size, dtype=np.uint8)
            self.available_buffers.append(buffer)
        
        rospy.logdebug(f"MemoryPool initialized: {pool_size} buffers of {buffer_size} bytes")
    
    def get_buffer(self) -> Optional[np.ndarray]:
        """Get a buffer from the pool"""
        with self.lock:
            if self.available_buffers:
                buffer = self.available_buffers.popleft()
                self.used_buffers.add(id(buffer))
                return buffer
            else:
                # Pool exhausted, create new buffer
                rospy.logwarn("Memory pool exhausted, creating new buffer")
                buffer = np.empty(self.buffer_size, dtype=np.uint8)
                self.used_buffers.add(id(buffer))
                return buffer
    
    def return_buffer(self, buffer: np.ndarray):
        """Return a buffer to the pool"""
        with self.lock:
            buffer_id = id(buffer)
            if buffer_id in self.used_buffers:
                self.used_buffers.remove(buffer_id)
                if len(self.available_buffers) < self.pool_size:
                    self.available_buffers.append(buffer)
                # If pool is full, let buffer be garbage collected
    
    def get_stats(self) -> Dict[str, int]:
        """Get memory pool statistics"""
        with self.lock:
            return {
                'available_buffers': len(self.available_buffers),
                'used_buffers': len(self.used_buffers),
                'total_capacity': self.pool_size,
                'buffer_size': self.buffer_size
            }


class ResourceOptimizationUtils:
    """
    Comprehensive resource optimization utilities for enhanced autonomous navigation.
    Provides memory management, CPU optimization, and system performance tuning.
    """
    
    def __init__(self):
        self.node_name = "resource_optimization_utils"
        
        # Configuration
        self.raspberry_pi_mode = rospy.get_param("~raspberry_pi_mode", True)
        self.optimization_interval = rospy.get_param("~optimization_interval", 30.0)  # seconds
        self.memory_threshold_mb = rospy.get_param("~memory_threshold_mb", 1024 if self.raspberry_pi_mode else 2048)
        self.cpu_threshold_percent = rospy.get_param("~cpu_threshold_percent", 80.0)
        self.enable_aggressive_optimization = rospy.get_param("~enable_aggressive_optimization", self.raspberry_pi_mode)
        
        # Memory pools for different buffer sizes
        self.memory_pools: Dict[int, MemoryPool] = {}
        self._initialize_memory_pools()
        
        # Resource monitoring
        self.resource_history: deque = deque(maxlen=100)
        self.optimization_history: List[OptimizationResult] = []
        
        # Optimization state
        self.optimization_active = True
        self.optimization_lock = threading.Lock()
        self.last_gc_time = 0.0
        self.last_optimization_time = 0.0
        
        # CPU affinity and process management
        self.process = psutil.Process()
        self.original_nice = self.process.nice()
        self.cpu_count = psutil.cpu_count()
        
        # Cache management
        self.image_cache: Dict[str, Any] = {}
        self.cache_max_size = rospy.get_param("~cache_max_size", 50)
        self.cache_access_times: Dict[str, float] = {}
        
        # Statistics
        self.total_optimizations = 0
        self.successful_optimizations = 0
        self.memory_saved_mb = 0.0
        self.cpu_time_saved_seconds = 0.0
        
        rospy.loginfo(f"[{self.node_name}] Resource Optimization Utils initialized")
        rospy.loginfo(f"[{self.node_name}] Raspberry Pi mode: {self.raspberry_pi_mode}")
        rospy.loginfo(f"[{self.node_name}] Memory threshold: {self.memory_threshold_mb} MB")
        rospy.loginfo(f"[{self.node_name}] CPU threshold: {self.cpu_threshold_percent}%")
        rospy.loginfo(f"[{self.node_name}] Aggressive optimization: {self.enable_aggressive_optimization}")
        rospy.loginfo(f"[{self.node_name}] CPU count: {self.cpu_count}")
    
    def _initialize_memory_pools(self):
        """Initialize memory pools for common buffer sizes"""
        # Common image buffer sizes
        buffer_sizes = [
            640 * 480 * 3,      # VGA RGB
            320 * 240 * 3,      # QVGA RGB
            160 * 120 * 3,      # Thumbnail RGB
            640 * 480,          # VGA grayscale
            320 * 240,          # QVGA grayscale
            1024 * 1024,        # 1MB general purpose
            512 * 512,          # 512KB general purpose
            256 * 256,          # 256KB general purpose
        ]
        
        pool_sizes = [5, 5, 10, 5, 5, 3, 5, 10]  # Adjust pool sizes based on expected usage
        
        for buffer_size, pool_size in zip(buffer_sizes, pool_sizes):
            self.memory_pools[buffer_size] = MemoryPool(buffer_size, pool_size)
        
        rospy.loginfo(f"[{self.node_name}] Initialized {len(self.memory_pools)} memory pools")
    
    def get_current_resource_usage(self) -> ResourceUsage:
        """Get current system resource usage"""
        try:
            current_time = time.time()
            
            # CPU usage
            cpu_percent = self.process.cpu_percent()
            
            # Memory usage
            memory_info = self.process.memory_info()
            memory_rss_mb = memory_info.rss / 1024 / 1024
            memory_vms_mb = memory_info.vms / 1024 / 1024
            
            # System memory
            system_memory = psutil.virtual_memory()
            memory_percent = system_memory.percent
            memory_available_mb = system_memory.available / 1024 / 1024
            
            # Process info
            open_files = len(self.process.open_files())
            threads = self.process.num_threads()
            
            # GPU memory (if available)
            gpu_memory_mb = self._get_gpu_memory_usage()
            
            # Temperature (Raspberry Pi)
            temperature_celsius = None
            if self.raspberry_pi_mode:
                temperature_celsius = self._get_cpu_temperature()
            
            return ResourceUsage(
                timestamp=current_time,
                cpu_percent=cpu_percent,
                memory_rss_mb=memory_rss_mb,
                memory_vms_mb=memory_vms_mb,
                memory_percent=memory_percent,
                memory_available_mb=memory_available_mb,
                open_files=open_files,
                threads=threads,
                gpu_memory_mb=gpu_memory_mb,
                temperature_celsius=temperature_celsius
            )
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error getting resource usage: {e}")
            return ResourceUsage(time.time(), 0, 0, 0, 0, 0, 0, 0, None, None)
    
    def _get_gpu_memory_usage(self) -> Optional[float]:
        """Get GPU memory usage if available"""
        try:
            # Try to get GPU memory usage (implementation depends on GPU type)
            # This is a placeholder - actual implementation would depend on the specific GPU
            return None
        except:
            return None
    
    def _get_cpu_temperature(self) -> Optional[float]:
        """Get CPU temperature on Raspberry Pi"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_str = f.read().strip()
                return float(temp_str) / 1000.0  # Convert to Celsius
        except:
            return None
    
    def optimize_memory_usage(self, aggressive: bool = False) -> OptimizationResult:
        """Optimize memory usage through various strategies"""
        before_usage = self.get_current_resource_usage()
        start_time = time.time()
        
        try:
            rospy.logdebug(f"[{self.node_name}] Starting memory optimization (aggressive={aggressive})")
            
            # Strategy 1: Garbage collection
            if aggressive or (time.time() - self.last_gc_time) > 60.0:
                gc.collect()
                self.last_gc_time = time.time()
                rospy.logdebug(f"[{self.node_name}] Performed garbage collection")
            
            # Strategy 2: Clear image cache if memory pressure is high
            if before_usage.memory_percent > 80.0 or aggressive:
                self._optimize_image_cache()
                rospy.logdebug(f"[{self.node_name}] Optimized image cache")
            
            # Strategy 3: Memory pool optimization
            self._optimize_memory_pools()
            
            # Strategy 4: System-level memory optimization
            if aggressive and self.raspberry_pi_mode:
                self._system_memory_optimization()
            
            after_usage = self.get_current_resource_usage()
            duration = time.time() - start_time
            
            # Calculate improvement
            memory_improvement = before_usage.memory_rss_mb - after_usage.memory_rss_mb
            improvement_percent = (memory_improvement / max(1.0, before_usage.memory_rss_mb)) * 100.0
            
            result = OptimizationResult(
                strategy=OptimizationStrategy.MEMORY_POOLING,
                timestamp=start_time,
                success=improvement_percent > 0,
                before_usage=before_usage,
                after_usage=after_usage,
                improvement_percent=improvement_percent,
                description=f"Memory optimization: {memory_improvement:.1f}MB freed",
                duration_seconds=duration
            )
            
            if result.success:
                self.memory_saved_mb += memory_improvement
                self.successful_optimizations += 1
            
            self.total_optimizations += 1
            self.optimization_history.append(result)
            
            rospy.loginfo(f"[{self.node_name}] Memory optimization completed: "
                         f"{improvement_percent:.1f}% improvement, {memory_improvement:.1f}MB freed")
            
            # Comprehensive logging
            rospy.logdebug(f"[{self.node_name}] Memory optimization details:")
            rospy.logdebug(f"[{self.node_name}] - Before: {before_usage.memory_rss_mb:.1f}MB RSS")
            rospy.logdebug(f"[{self.node_name}] - After: {after_usage.memory_rss_mb:.1f}MB RSS")
            rospy.logdebug(f"[{self.node_name}] - Duration: {duration:.2f}s")
            rospy.logdebug(f"[{self.node_name}] - Aggressive mode: {aggressive}")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Memory optimization event: "
                          f"Improvement={improvement_percent:.1f}%, Freed={memory_improvement:.1f}MB")
            
            return result
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Memory optimization error: {e}")
            
            after_usage = self.get_current_resource_usage()
            duration = time.time() - start_time
            
            return OptimizationResult(
                strategy=OptimizationStrategy.MEMORY_POOLING,
                timestamp=start_time,
                success=False,
                before_usage=before_usage,
                after_usage=after_usage,
                improvement_percent=0.0,
                description=f"Memory optimization failed: {str(e)}",
                duration_seconds=duration
            )
    
    def optimize_cpu_usage(self, target_cpu_percent: Optional[float] = None) -> OptimizationResult:
        """Optimize CPU usage through process management and affinity"""
        before_usage = self.get_current_resource_usage()
        start_time = time.time()
        
        if target_cpu_percent is None:
            target_cpu_percent = self.cpu_threshold_percent
        
        try:
            rospy.logdebug(f"[{self.node_name}] Starting CPU optimization (target={target_cpu_percent}%)")
            
            # Strategy 1: Adjust process priority
            if before_usage.cpu_percent > target_cpu_percent:
                current_nice = self.process.nice()
                if current_nice < 10:  # Don't make it too low priority
                    new_nice = min(10, current_nice + 2)
                    self.process.nice(new_nice)
                    rospy.logdebug(f"[{self.node_name}] Adjusted process nice: {current_nice} -> {new_nice}")
            elif before_usage.cpu_percent < target_cpu_percent * 0.7:
                # CPU usage is low, can increase priority
                current_nice = self.process.nice()
                if current_nice > self.original_nice:
                    new_nice = max(self.original_nice, current_nice - 1)
                    self.process.nice(new_nice)
                    rospy.logdebug(f"[{self.node_name}] Increased process priority: {current_nice} -> {new_nice}")
            
            # Strategy 2: CPU affinity optimization (Raspberry Pi)
            if self.raspberry_pi_mode and self.cpu_count > 1:
                self._optimize_cpu_affinity()
            
            # Strategy 3: Thread management
            self._optimize_thread_usage()
            
            time.sleep(0.1)  # Allow changes to take effect
            
            after_usage = self.get_current_resource_usage()
            duration = time.time() - start_time
            
            # Calculate improvement
            cpu_improvement = before_usage.cpu_percent - after_usage.cpu_percent
            improvement_percent = (cpu_improvement / max(1.0, before_usage.cpu_percent)) * 100.0
            
            result = OptimizationResult(
                strategy=OptimizationStrategy.PROCESS_PRIORITY,
                timestamp=start_time,
                success=improvement_percent > 0,
                before_usage=before_usage,
                after_usage=after_usage,
                improvement_percent=improvement_percent,
                description=f"CPU optimization: {cpu_improvement:.1f}% reduction",
                duration_seconds=duration
            )
            
            if result.success:
                self.cpu_time_saved_seconds += cpu_improvement * duration / 100.0
                self.successful_optimizations += 1
            
            self.total_optimizations += 1
            self.optimization_history.append(result)
            
            rospy.loginfo(f"[{self.node_name}] CPU optimization completed: "
                         f"{improvement_percent:.1f}% improvement, {cpu_improvement:.1f}% reduction")
            
            # Comprehensive logging
            rospy.logdebug(f"[{self.node_name}] CPU optimization details:")
            rospy.logdebug(f"[{self.node_name}] - Before: {before_usage.cpu_percent:.1f}% CPU")
            rospy.logdebug(f"[{self.node_name}] - After: {after_usage.cpu_percent:.1f}% CPU")
            rospy.logdebug(f"[{self.node_name}] - Target: {target_cpu_percent:.1f}% CPU")
            rospy.logdebug(f"[{self.node_name}] - Duration: {duration:.2f}s")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] CPU optimization event: "
                          f"Improvement={improvement_percent:.1f}%, Reduction={cpu_improvement:.1f}%")
            
            return result
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] CPU optimization error: {e}")
            
            after_usage = self.get_current_resource_usage()
            duration = time.time() - start_time
            
            return OptimizationResult(
                strategy=OptimizationStrategy.PROCESS_PRIORITY,
                timestamp=start_time,
                success=False,
                before_usage=before_usage,
                after_usage=after_usage,
                improvement_percent=0.0,
                description=f"CPU optimization failed: {str(e)}",
                duration_seconds=duration
            )
    
    def _optimize_image_cache(self):
        """Optimize image cache by removing least recently used items"""
        if len(self.image_cache) <= self.cache_max_size // 2:
            return
        
        current_time = time.time()
        
        # Sort by access time (least recently used first)
        sorted_items = sorted(
            self.cache_access_times.items(),
            key=lambda x: x[1]
        )
        
        # Remove oldest half of cache
        items_to_remove = len(sorted_items) // 2
        for i in range(items_to_remove):
            key = sorted_items[i][0]
            if key in self.image_cache:
                del self.image_cache[key]
            if key in self.cache_access_times:
                del self.cache_access_times[key]
        
        rospy.logdebug(f"[{self.node_name}] Removed {items_to_remove} items from image cache")
    
    def _optimize_memory_pools(self):
        """Optimize memory pools by adjusting sizes based on usage"""
        for buffer_size, pool in self.memory_pools.items():
            stats = pool.get_stats()
            
            # If pool is frequently exhausted, consider increasing size
            if stats['available_buffers'] == 0 and stats['used_buffers'] > stats['total_capacity']:
                rospy.logdebug(f"[{self.node_name}] Memory pool {buffer_size} is exhausted")
            
            # If pool has too many unused buffers, consider reducing size
            elif stats['available_buffers'] > stats['total_capacity'] * 0.8:
                rospy.logdebug(f"[{self.node_name}] Memory pool {buffer_size} has excess capacity")
    
    def _system_memory_optimization(self):
        """System-level memory optimization for Raspberry Pi"""
        try:
            # Drop caches (requires root privileges)
            # This is typically done by writing to /proc/sys/vm/drop_caches
            # For safety, we'll just log what could be done
            rospy.logdebug(f"[{self.node_name}] System memory optimization (would require root privileges)")
            
            # Force garbage collection of all generations
            for generation in range(3):
                gc.collect(generation)
            
        except Exception as e:
            rospy.logdebug(f"[{self.node_name}] System memory optimization limited: {e}")
    
    def _optimize_cpu_affinity(self):
        """Optimize CPU affinity for better performance"""
        try:
            current_affinity = self.process.cpu_affinity()
            
            # For Raspberry Pi, try to use specific cores for better performance
            if self.cpu_count == 4:  # Raspberry Pi 4
                # Use cores 2 and 3 for compute-intensive tasks
                optimal_affinity = [2, 3]
            elif self.cpu_count == 2:  # Raspberry Pi 3
                # Use both cores
                optimal_affinity = [0, 1]
            else:
                optimal_affinity = list(range(self.cpu_count))
            
            if set(current_affinity) != set(optimal_affinity):
                self.process.cpu_affinity(optimal_affinity)
                rospy.logdebug(f"[{self.node_name}] CPU affinity optimized: {current_affinity} -> {optimal_affinity}")
            
        except Exception as e:
            rospy.logdebug(f"[{self.node_name}] CPU affinity optimization failed: {e}")
    
    def _optimize_thread_usage(self):
        """Optimize thread usage"""
        try:
            thread_count = self.process.num_threads()
            
            # Log thread usage for monitoring
            rospy.logdebug(f"[{self.node_name}] Current thread count: {thread_count}")
            
            # If too many threads, consider thread pool optimization
            if thread_count > self.cpu_count * 4:
                rospy.logwarn(f"[{self.node_name}] High thread count detected: {thread_count}")
            
        except Exception as e:
            rospy.logdebug(f"[{self.node_name}] Thread optimization failed: {e}")
    
    def get_buffer(self, size: int) -> Optional[np.ndarray]:
        """Get an optimized buffer from memory pool"""
        # Find the best matching pool
        best_pool_size = None
        for pool_size in self.memory_pools.keys():
            if pool_size >= size:
                if best_pool_size is None or pool_size < best_pool_size:
                    best_pool_size = pool_size
        
        if best_pool_size is not None:
            buffer = self.memory_pools[best_pool_size].get_buffer()
            if buffer is not None:
                # Return a view of the requested size
                return buffer[:size].view()
        
        # No suitable pool found, create new buffer
        rospy.logdebug(f"[{self.node_name}] Creating new buffer of size {size}")
        return np.empty(size, dtype=np.uint8)
    
    def return_buffer(self, buffer: np.ndarray):
        """Return a buffer to the appropriate memory pool"""
        buffer_size = buffer.size
        
        # Find matching pool
        for pool_size, pool in self.memory_pools.items():
            if pool_size == buffer_size:
                pool.return_buffer(buffer)
                return
        
        # No matching pool, buffer will be garbage collected
        rospy.logdebug(f"[{self.node_name}] Buffer of size {buffer_size} returned to GC")
    
    def cache_image(self, key: str, image: np.ndarray, max_age_seconds: float = 300.0):
        """Cache an image with automatic cleanup"""
        current_time = time.time()
        
        # Clean up old cache entries
        keys_to_remove = []
        for cache_key, access_time in self.cache_access_times.items():
            if current_time - access_time > max_age_seconds:
                keys_to_remove.append(cache_key)
        
        for cache_key in keys_to_remove:
            if cache_key in self.image_cache:
                del self.image_cache[cache_key]
            if cache_key in self.cache_access_times:
                del self.cache_access_times[cache_key]
        
        # Add new image to cache
        if len(self.image_cache) >= self.cache_max_size:
            self._optimize_image_cache()
        
        self.image_cache[key] = image.copy()
        self.cache_access_times[key] = current_time
        
        rospy.logdebug(f"[{self.node_name}] Cached image: {key}, cache size: {len(self.image_cache)}")
    
    def get_cached_image(self, key: str) -> Optional[np.ndarray]:
        """Get a cached image"""
        if key in self.image_cache:
            self.cache_access_times[key] = time.time()
            return self.image_cache[key]
        return None
    
    def optimize_image_processing(self, image: np.ndarray, target_size: Optional[Tuple[int, int]] = None,
                                quality_factor: float = 1.0) -> np.ndarray:
        """Optimize image processing based on available resources"""
        current_usage = self.get_current_resource_usage()
        
        # Determine optimization level based on resource usage
        if current_usage.memory_percent > 85 or current_usage.cpu_percent > 90:
            optimization_level = 3  # Aggressive
        elif current_usage.memory_percent > 75 or current_usage.cpu_percent > 80:
            optimization_level = 2  # Moderate
        elif current_usage.memory_percent > 65 or current_usage.cpu_percent > 70:
            optimization_level = 1  # Light
        else:
            optimization_level = 0  # None
        
        optimized_image = image
        
        # Apply optimizations based on level
        if optimization_level >= 1:
            # Light optimization: reduce quality slightly
            if quality_factor > 0.8:
                quality_factor *= 0.9
        
        if optimization_level >= 2:
            # Moderate optimization: resize if needed
            if target_size is None and image.shape[0] > 480:
                scale_factor = 480.0 / image.shape[0]
                new_width = int(image.shape[1] * scale_factor)
                target_size = (480, new_width)
        
        if optimization_level >= 3:
            # Aggressive optimization: significant resize
            if target_size is None:
                target_size = (240, 320)  # Quarter VGA
            quality_factor *= 0.8
        
        # Apply resizing if needed
        if target_size is not None and (image.shape[0] != target_size[0] or image.shape[1] != target_size[1]):
            optimized_image = cv2.resize(image, (target_size[1], target_size[0]), 
                                       interpolation=cv2.INTER_LINEAR if optimization_level < 3 else cv2.INTER_NEAREST)
        
        # Apply quality reduction if needed
        if quality_factor < 1.0:
            # Simulate quality reduction through slight blurring
            kernel_size = max(1, int((1.0 - quality_factor) * 5))
            if kernel_size > 1:
                optimized_image = cv2.GaussianBlur(optimized_image, (kernel_size, kernel_size), 0)
        
        rospy.logdebug(f"[{self.node_name}] Image optimization: level={optimization_level}, "
                      f"quality={quality_factor:.2f}, size={optimized_image.shape}")
        
        return optimized_image
    
    def monitor_and_optimize(self) -> Dict[str, Any]:
        """Monitor resources and apply optimizations as needed"""
        current_usage = self.get_current_resource_usage()
        self.resource_history.append(current_usage)
        
        optimizations_applied = []
        
        # Check if optimization is needed
        current_time = time.time()
        if (current_time - self.last_optimization_time) < self.optimization_interval:
            return {'optimizations_applied': [], 'current_usage': current_usage.to_dict()}
        
        # Memory optimization
        if (current_usage.memory_percent > 80 or 
            current_usage.memory_rss_mb > self.memory_threshold_mb):
            
            aggressive = current_usage.memory_percent > 90
            result = self.optimize_memory_usage(aggressive=aggressive)
            optimizations_applied.append(result.to_dict())
            
            rospy.loginfo(f"[{self.node_name}] Memory optimization triggered: "
                         f"{current_usage.memory_percent:.1f}% usage")
        
        # CPU optimization
        if current_usage.cpu_percent > self.cpu_threshold_percent:
            result = self.optimize_cpu_usage()
            optimizations_applied.append(result.to_dict())
            
            rospy.loginfo(f"[{self.node_name}] CPU optimization triggered: "
                         f"{current_usage.cpu_percent:.1f}% usage")
        
        # Temperature-based optimization (Raspberry Pi)
        if (self.raspberry_pi_mode and current_usage.temperature_celsius and 
            current_usage.temperature_celsius > 75.0):
            
            # Thermal throttling optimization
            result = self.optimize_cpu_usage(target_cpu_percent=60.0)
            optimizations_applied.append(result.to_dict())
            
            rospy.logwarn(f"[{self.node_name}] Thermal optimization triggered: "
                         f"{current_usage.temperature_celsius:.1f}°C")
        
        self.last_optimization_time = current_time
        
        # Comprehensive logging
        if optimizations_applied:
            rospy.logdebug(f"[{self.node_name}] Resource optimization cycle completed:")
            rospy.logdebug(f"[{self.node_name}] - Optimizations applied: {len(optimizations_applied)}")
            rospy.logdebug(f"[{self.node_name}] - Memory usage: {current_usage.memory_percent:.1f}%")
            rospy.logdebug(f"[{self.node_name}] - CPU usage: {current_usage.cpu_percent:.1f}%")
            if current_usage.temperature_celsius:
                rospy.logdebug(f"[{self.node_name}] - Temperature: {current_usage.temperature_celsius:.1f}°C")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Resource monitoring event: "
                      f"Memory={current_usage.memory_percent:.1f}%, "
                      f"CPU={current_usage.cpu_percent:.1f}%, "
                      f"Optimizations={len(optimizations_applied)}")
        
        return {
            'optimizations_applied': optimizations_applied,
            'current_usage': current_usage.to_dict(),
            'optimization_statistics': self.get_optimization_statistics()
        }
    
    def get_optimization_statistics(self) -> Dict[str, Any]:
        """Get comprehensive optimization statistics"""
        current_time = time.time()
        
        # Memory pool statistics
        pool_stats = {}
        for buffer_size, pool in self.memory_pools.items():
            pool_stats[str(buffer_size)] = pool.get_stats()
        
        # Recent resource usage
        recent_usage = None
        if self.resource_history:
            recent_usage = self.resource_history[-1].to_dict()
        
        return {
            'timestamp': current_time,
            'total_optimizations': self.total_optimizations,
            'successful_optimizations': self.successful_optimizations,
            'success_rate': self.successful_optimizations / max(1, self.total_optimizations),
            'memory_saved_mb': self.memory_saved_mb,
            'cpu_time_saved_seconds': self.cpu_time_saved_seconds,
            'memory_pools': pool_stats,
            'image_cache_size': len(self.image_cache),
            'cache_max_size': self.cache_max_size,
            'recent_resource_usage': recent_usage,
            'optimization_history_length': len(self.optimization_history),
            'raspberry_pi_mode': self.raspberry_pi_mode,
            'aggressive_optimization': self.enable_aggressive_optimization
        }
    
    def reset_optimizations(self):
        """Reset all optimizations to default state"""
        try:
            # Reset process priority
            self.process.nice(self.original_nice)
            
            # Reset CPU affinity to all cores
            if self.cpu_count > 1:
                self.process.cpu_affinity(list(range(self.cpu_count)))
            
            # Clear caches
            self.image_cache.clear()
            self.cache_access_times.clear()
            
            # Force garbage collection
            gc.collect()
            
            rospy.loginfo(f"[{self.node_name}] All optimizations reset to default state")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error resetting optimizations: {e}")
    
    def shutdown(self):
        """Shutdown resource optimization utilities"""
        rospy.loginfo(f"[{self.node_name}] Resource Optimization Utils shutting down")
        
        self.optimization_active = False
        
        # Reset optimizations
        self.reset_optimizations()
        
        # Log final statistics
        stats = self.get_optimization_statistics()
        rospy.loginfo(f"[{self.node_name}] Final optimization statistics:")
        rospy.loginfo(f"[{self.node_name}] Total optimizations: {stats['total_optimizations']}")
        rospy.loginfo(f"[{self.node_name}] Success rate: {stats['success_rate']:.2%}")
        rospy.loginfo(f"[{self.node_name}] Memory saved: {stats['memory_saved_mb']:.1f} MB")
        rospy.loginfo(f"[{self.node_name}] CPU time saved: {stats['cpu_time_saved_seconds']:.1f} seconds")
        rospy.loginfo(f"[{self.node_name}] Image cache size: {stats['image_cache_size']}")
        
        rospy.loginfo(f"[{self.node_name}] Resource Optimization Utils shutdown complete")


# Convenience functions for easy integration
def get_optimized_buffer(size: int) -> Optional[np.ndarray]:
    """Get an optimized buffer (convenience function)"""
    global _resource_optimizer
    if '_resource_optimizer' not in globals():
        _resource_optimizer = ResourceOptimizationUtils()
    return _resource_optimizer.get_buffer(size)


def return_optimized_buffer(buffer: np.ndarray):
    """Return an optimized buffer (convenience function)"""
    global _resource_optimizer
    if '_resource_optimizer' not in globals():
        _resource_optimizer = ResourceOptimizationUtils()
    _resource_optimizer.return_buffer(buffer)


def optimize_image_for_processing(image: np.ndarray, target_size: Optional[Tuple[int, int]] = None,
                                quality_factor: float = 1.0) -> np.ndarray:
    """Optimize image for processing (convenience function)"""
    global _resource_optimizer
    if '_resource_optimizer' not in globals():
        _resource_optimizer = ResourceOptimizationUtils()
    return _resource_optimizer.optimize_image_processing(image, target_size, quality_factor)


def monitor_and_optimize_resources() -> Dict[str, Any]:
    """Monitor and optimize system resources (convenience function)"""
    global _resource_optimizer
    if '_resource_optimizer' not in globals():
        _resource_optimizer = ResourceOptimizationUtils()
    return _resource_optimizer.monitor_and_optimize()


if __name__ == '__main__':
    # Test the resource optimization utilities
    rospy.init_node('resource_optimization_test')
    
    optimizer = ResourceOptimizationUtils()
    
    try:
        # Test resource monitoring
        usage = optimizer.get_current_resource_usage()
        rospy.loginfo(f"Current resource usage: {usage.to_dict()}")
        
        # Test memory optimization
        result = optimizer.optimize_memory_usage()
        rospy.loginfo(f"Memory optimization result: {result.to_dict()}")
        
        # Test CPU optimization
        result = optimizer.optimize_cpu_usage()
        rospy.loginfo(f"CPU optimization result: {result.to_dict()}")
        
        # Test monitoring and optimization
        monitor_result = optimizer.monitor_and_optimize()
        rospy.loginfo(f"Monitor and optimize result: {monitor_result}")
        
        rospy.loginfo("Resource optimization utilities test completed successfully")
        
    except Exception as e:
        rospy.logerr(f"Resource optimization test error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        optimizer.shutdown()
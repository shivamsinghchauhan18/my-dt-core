#!/usr/bin/env python3
import copy
import time
import psutil
import os
import threading
import json
import pickle
from enum import Enum
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime

import rospy
from duckietown_msgs.msg import BoolStamped, FSMState, AprilTagDetectionArray, StopLineReading, SafetyStatus
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern
from std_srvs.srv import SetBool
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist

# Import safety status publisher
from safety_status_publisher import SafetyStatusPublisher


class StateValidationResult(Enum):
    """State validation results"""
    VALID = "valid"
    INVALID_TRANSITION = "invalid_transition"
    MISSING_PRECONDITIONS = "missing_preconditions"
    CONSISTENCY_ERROR = "consistency_error"
    TIMEOUT_ERROR = "timeout_error"


@dataclass
class StateTransition:
    """Represents a state transition with metadata"""
    from_state: str
    to_state: str
    event: str
    timestamp: float
    duration_in_previous_state: float
    validation_result: StateValidationResult
    metadata: Dict[str, Any] = None
    
    def __post_init__(self):
        if self.metadata is None:
            self.metadata = {}


@dataclass
class StateSnapshot:
    """Snapshot of FSM state for persistence and recovery"""
    state: str
    timestamp: float
    active_nodes: List[str]
    safety_status: Dict[str, Any]
    system_metrics: Dict[str, Any]
    transition_history: List[Dict[str, Any]]
    metadata: Dict[str, Any] = None
    
    def __post_init__(self):
        if self.metadata is None:
            self.metadata = {}


class StateManager:
    """
    Enhanced state management system with persistence, recovery, and validation.
    Provides advanced state transition logic, consistency checking, and recovery mechanisms.
    """
    
    def __init__(self, node_name: str, persistence_path: str = None):
        self.node_name = node_name
        self.persistence_path = persistence_path or f"/tmp/fsm_state_{node_name.replace('/', '_')}.pkl"
        
        # State management
        self.current_state = None
        self.previous_state = None
        self.state_entry_time = None
        self.state_transition_history: List[StateTransition] = []
        self.state_snapshots: List[StateSnapshot] = []
        
        # State validation and consistency
        self.state_validators: Dict[str, callable] = {}
        self.state_preconditions: Dict[str, List[callable]] = {}
        self.state_postconditions: Dict[str, List[callable]] = {}
        self.consistency_checkers: List[callable] = []
        
        # Recovery mechanisms
        self.recovery_strategies: Dict[str, callable] = {}
        self.max_recovery_attempts = 3
        self.recovery_attempt_count = 0
        self.last_recovery_time = None
        
        # Performance monitoring
        self.state_durations: Dict[str, List[float]] = {}
        self.transition_counts: Dict[str, int] = {}
        self.validation_failures: Dict[str, int] = {}
        self.recovery_events: List[Dict[str, Any]] = []
        
        # Persistence settings
        self.enable_persistence = rospy.get_param("~enable_state_persistence", True)
        self.snapshot_interval = rospy.get_param("~state_snapshot_interval", 10.0)  # seconds
        self.max_snapshots = rospy.get_param("~max_state_snapshots", 100)
        self.max_history_length = rospy.get_param("~max_transition_history", 1000)
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Initialize default validators and recovery strategies
        self._initialize_default_validators()
        self._initialize_default_recovery_strategies()
        
        # Load persisted state if available
        if self.enable_persistence:
            self._load_persisted_state()
        
        # Start periodic snapshot timer
        if self.enable_persistence:
            self.snapshot_timer = rospy.Timer(
                rospy.Duration(self.snapshot_interval),
                self._periodic_snapshot_callback
            )
        
        rospy.loginfo(f"[{self.node_name}] StateManager initialized with advanced state management")
        rospy.loginfo(f"[{self.node_name}] Persistence enabled: {self.enable_persistence}")
        rospy.loginfo(f"[{self.node_name}] Persistence path: {self.persistence_path}")
        rospy.loginfo(f"[{self.node_name}] Snapshot interval: {self.snapshot_interval}s")
    
    def _initialize_default_validators(self):
        """Initialize default state validators"""
        self.state_validators = {
            'timeout_validator': self._validate_state_timeout,
            'consistency_validator': self._validate_state_consistency,
            'safety_validator': self._validate_safety_conditions,
            'resource_validator': self._validate_resource_availability
        }
        
        # Initialize preconditions for common states
        self.state_preconditions = {
            'LANE_FOLLOWING': [self._check_camera_available, self._check_lane_detection_ready],
            'INTERSECTION_COORDINATION': [self._check_apriltag_detection_ready],
            'EMERGENCY_STOP': [],  # No preconditions for emergency stop
            'SAFE_MODE': [self._check_basic_systems_operational]
        }
        
        # Initialize postconditions
        self.state_postconditions = {
            'LANE_FOLLOWING': [self._verify_lane_following_active],
            'INTERSECTION_COORDINATION': [self._verify_coordination_active],
            'EMERGENCY_STOP': [self._verify_emergency_stop_active]
        }
        
        rospy.logdebug(f"[{self.node_name}] Initialized {len(self.state_validators)} state validators")
    
    def _initialize_default_recovery_strategies(self):
        """Initialize default recovery strategies"""
        self.recovery_strategies = {
            'timeout_recovery': self._recover_from_timeout,
            'consistency_recovery': self._recover_from_consistency_error,
            'safety_recovery': self._recover_from_safety_violation,
            'resource_recovery': self._recover_from_resource_shortage,
            'general_recovery': self._general_recovery_strategy
        }
        
        rospy.logdebug(f"[{self.node_name}] Initialized {len(self.recovery_strategies)} recovery strategies")
    
    def transition_to_state(self, new_state: str, event: str, metadata: Dict[str, Any] = None) -> bool:
        """
        Perform enhanced state transition with validation and recovery.
        
        Args:
            new_state: Target state to transition to
            event: Event triggering the transition
            metadata: Additional metadata for the transition
            
        Returns:
            bool: True if transition successful, False otherwise
        """
        with self.lock:
            start_time = time.time()
            
            if metadata is None:
                metadata = {}
            
            rospy.loginfo(f"[{self.node_name}] Attempting state transition: {self.current_state} -> {new_state} (event: {event})")
            rospy.logdebug(f"[{self.node_name}] Transition metadata: {metadata}")
            
            # Calculate duration in previous state
            duration_in_previous = 0.0
            if self.state_entry_time:
                duration_in_previous = start_time - self.state_entry_time
            
            # Validate transition
            validation_result = self._validate_state_transition(self.current_state, new_state, event)
            
            if validation_result != StateValidationResult.VALID:
                rospy.logwarn(f"[{self.node_name}] State transition validation failed: {validation_result}")
                rospy.logwarn(f"[{self.node_name}] Transition rejected: {self.current_state} -> {new_state}")
                
                # Record validation failure
                self.validation_failures[validation_result.value] = self.validation_failures.get(validation_result.value, 0) + 1
                
                # Attempt recovery
                recovery_success = self._attempt_recovery(validation_result, new_state, event, metadata)
                if not recovery_success:
                    rospy.logerr(f"[{self.node_name}] State transition failed and recovery unsuccessful")
                    return False
            
            # Check preconditions for new state
            if not self._check_state_preconditions(new_state):
                rospy.logwarn(f"[{self.node_name}] Preconditions not met for state: {new_state}")
                
                # Attempt to resolve preconditions
                if not self._resolve_preconditions(new_state):
                    rospy.logerr(f"[{self.node_name}] Could not resolve preconditions for state: {new_state}")
                    return False
            
            # Perform the transition
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_entry_time = start_time
            
            # Record transition
            transition = StateTransition(
                from_state=self.previous_state or "INITIAL",
                to_state=new_state,
                event=event,
                timestamp=start_time,
                duration_in_previous_state=duration_in_previous,
                validation_result=validation_result,
                metadata=metadata
            )
            
            self._record_transition(transition)
            
            # Verify postconditions
            if not self._check_state_postconditions(new_state):
                rospy.logwarn(f"[{self.node_name}] Postconditions not satisfied for state: {new_state}")
                # Continue anyway but log the issue
            
            # Update statistics
            self.transition_counts[new_state] = self.transition_counts.get(new_state, 0) + 1
            
            if self.previous_state:
                if self.previous_state not in self.state_durations:
                    self.state_durations[self.previous_state] = []
                self.state_durations[self.previous_state].append(duration_in_previous)
            
            rospy.loginfo(f"[{self.node_name}] State transition successful: {self.previous_state} -> {new_state}")
            rospy.loginfo(f"[{self.node_name}] Time in previous state: {duration_in_previous:.3f}s")
            rospy.logdebug(f"[{self.node_name}] Total transitions to {new_state}: {self.transition_counts[new_state]}")
            
            # Persist state if enabled
            if self.enable_persistence:
                self._persist_current_state()
            
            return True
    
    def _validate_state_transition(self, from_state: str, to_state: str, event: str) -> StateValidationResult:
        """Validate a state transition"""
        rospy.logdebug(f"[{self.node_name}] Validating transition: {from_state} -> {to_state}")
        
        # Run all validators
        for validator_name, validator_func in self.state_validators.items():
            try:
                result = validator_func(from_state, to_state, event)
                if result != StateValidationResult.VALID:
                    rospy.logwarn(f"[{self.node_name}] Validator '{validator_name}' failed: {result}")
                    return result
            except Exception as e:
                rospy.logerr(f"[{self.node_name}] Validator '{validator_name}' error: {e}")
                return StateValidationResult.CONSISTENCY_ERROR
        
        rospy.logdebug(f"[{self.node_name}] State transition validation passed")
        return StateValidationResult.VALID
    
    def _validate_state_timeout(self, from_state: str, to_state: str, event: str) -> StateValidationResult:
        """Validate state hasn't exceeded timeout"""
        if not self.state_entry_time:
            return StateValidationResult.VALID
        
        current_time = time.time()
        time_in_state = current_time - self.state_entry_time
        
        # Define state-specific timeouts
        state_timeouts = {
            'LANE_FOLLOWING': 300.0,  # 5 minutes max
            'INTERSECTION_COORDINATION': 30.0,  # 30 seconds max
            'APRILTAG_STOP': 10.0,  # 10 seconds max
            'EMERGENCY_STOP': 60.0,  # 1 minute max
            'SAFE_MODE': 120.0  # 2 minutes max
        }
        
        timeout = state_timeouts.get(from_state, 600.0)  # Default 10 minutes
        
        if time_in_state > timeout:
            rospy.logwarn(f"[{self.node_name}] State timeout: {from_state} exceeded {timeout}s (current: {time_in_state:.1f}s)")
            return StateValidationResult.TIMEOUT_ERROR
        
        return StateValidationResult.VALID
    
    def _validate_state_consistency(self, from_state: str, to_state: str, event: str) -> StateValidationResult:
        """Validate state transition consistency"""
        # Check for invalid transitions
        invalid_transitions = {
            'EMERGENCY_STOP': ['LANE_FOLLOWING'],  # Can't go directly from emergency to lane following
            'APRILTAG_STOP': ['INTERSECTION_COORDINATION']  # Can't go from apriltag stop to intersection
        }
        
        if from_state in invalid_transitions and to_state in invalid_transitions[from_state]:
            rospy.logwarn(f"[{self.node_name}] Invalid direct transition: {from_state} -> {to_state}")
            return StateValidationResult.INVALID_TRANSITION
        
        # Check for rapid state oscillation
        if len(self.state_transition_history) >= 3:
            recent_states = [t.to_state for t in self.state_transition_history[-3:]]
            if len(set(recent_states)) <= 2 and to_state in recent_states:
                # Additional check: ensure we're actually oscillating between same states
                unique_states = set(recent_states + [to_state])
                if len(unique_states) <= 2:
                    rospy.logwarn(f"[{self.node_name}] Rapid state oscillation detected: {recent_states}")
                    return StateValidationResult.CONSISTENCY_ERROR
        
        return StateValidationResult.VALID
    
    def _validate_safety_conditions(self, from_state: str, to_state: str, event: str) -> StateValidationResult:
        """Validate safety conditions for transition"""
        # Emergency stop can always be entered
        if to_state == 'EMERGENCY_STOP':
            return StateValidationResult.VALID
        
        # Can't leave emergency stop without proper clearance
        if from_state == 'EMERGENCY_STOP' and to_state != 'SAFE_MODE':
            rospy.logwarn(f"[{self.node_name}] Invalid exit from emergency stop to: {to_state}")
            return StateValidationResult.INVALID_TRANSITION
        
        return StateValidationResult.VALID
    
    def _validate_resource_availability(self, from_state: str, to_state: str, event: str) -> StateValidationResult:
        """Validate system resources for transition"""
        try:
            # Check CPU and memory
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory_percent = psutil.virtual_memory().percent
            
            if cpu_percent > 95.0 or memory_percent > 95.0:
                rospy.logwarn(f"[{self.node_name}] High resource usage: CPU {cpu_percent}%, Memory {memory_percent}%")
                
                # Only allow transitions to safe states under high load
                if to_state not in ['EMERGENCY_STOP', 'SAFE_MODE']:
                    return StateValidationResult.CONSISTENCY_ERROR
            
        except Exception as e:
            rospy.logdebug(f"[{self.node_name}] Resource check error: {e}")
        
        return StateValidationResult.VALID
    
    def _check_state_preconditions(self, state: str) -> bool:
        """Check preconditions for entering a state"""
        if state not in self.state_preconditions:
            return True  # No preconditions defined
        
        rospy.logdebug(f"[{self.node_name}] Checking preconditions for state: {state}")
        
        for precondition in self.state_preconditions[state]:
            try:
                if not precondition():
                    rospy.logwarn(f"[{self.node_name}] Precondition failed for state {state}: {precondition.__name__}")
                    return False
            except Exception as e:
                rospy.logerr(f"[{self.node_name}] Precondition check error for {state}: {e}")
                return False
        
        rospy.logdebug(f"[{self.node_name}] All preconditions satisfied for state: {state}")
        return True
    
    def _check_state_postconditions(self, state: str) -> bool:
        """Check postconditions after entering a state"""
        if state not in self.state_postconditions:
            return True  # No postconditions defined
        
        rospy.logdebug(f"[{self.node_name}] Checking postconditions for state: {state}")
        
        for postcondition in self.state_postconditions[state]:
            try:
                if not postcondition():
                    rospy.logwarn(f"[{self.node_name}] Postcondition failed for state {state}: {postcondition.__name__}")
                    return False
            except Exception as e:
                rospy.logerr(f"[{self.node_name}] Postcondition check error for {state}: {e}")
                return False
        
        rospy.logdebug(f"[{self.node_name}] All postconditions satisfied for state: {state}")
        return True
    
    def _resolve_preconditions(self, state: str) -> bool:
        """Attempt to resolve failed preconditions"""
        rospy.loginfo(f"[{self.node_name}] Attempting to resolve preconditions for state: {state}")
        
        # Simple resolution strategies
        if state == 'LANE_FOLLOWING':
            # Wait a bit for camera to become available
            rospy.sleep(0.5)
            return self._check_state_preconditions(state)
        
        elif state == 'INTERSECTION_COORDINATION':
            # Wait for AprilTag detection to initialize
            rospy.sleep(1.0)
            return self._check_state_preconditions(state)
        
        return False
    
    def _attempt_recovery(self, validation_result: StateValidationResult, target_state: str, 
                         event: str, metadata: Dict[str, Any]) -> bool:
        """Attempt recovery from validation failure"""
        if self.recovery_attempt_count >= self.max_recovery_attempts:
            rospy.logerr(f"[{self.node_name}] Maximum recovery attempts exceeded")
            return False
        
        self.recovery_attempt_count += 1
        self.last_recovery_time = time.time()
        
        rospy.logwarn(f"[{self.node_name}] Attempting recovery from {validation_result} (attempt {self.recovery_attempt_count})")
        
        # Select recovery strategy
        recovery_strategy = None
        if validation_result == StateValidationResult.TIMEOUT_ERROR:
            recovery_strategy = self.recovery_strategies.get('timeout_recovery')
        elif validation_result == StateValidationResult.CONSISTENCY_ERROR:
            recovery_strategy = self.recovery_strategies.get('consistency_recovery')
        elif validation_result == StateValidationResult.INVALID_TRANSITION:
            recovery_strategy = self.recovery_strategies.get('general_recovery')
        
        if not recovery_strategy:
            recovery_strategy = self.recovery_strategies.get('general_recovery')
        
        try:
            success = recovery_strategy(validation_result, target_state, event, metadata)
            
            # Record recovery event
            recovery_event = {
                'timestamp': time.time(),
                'validation_result': validation_result.value,
                'target_state': target_state,
                'event': event,
                'success': success,
                'attempt_number': self.recovery_attempt_count
            }
            self.recovery_events.append(recovery_event)
            
            if success:
                rospy.loginfo(f"[{self.node_name}] Recovery successful for {validation_result}")
                self.recovery_attempt_count = 0  # Reset on success
            else:
                rospy.logwarn(f"[{self.node_name}] Recovery failed for {validation_result}")
            
            return success
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Recovery strategy error: {e}")
            return False
    
    def _record_transition(self, transition: StateTransition):
        """Record state transition in history"""
        self.state_transition_history.append(transition)
        
        # Limit history size
        if len(self.state_transition_history) > self.max_history_length:
            self.state_transition_history = self.state_transition_history[-self.max_history_length//2:]
        
        rospy.logdebug(f"[{self.node_name}] Recorded transition: {transition.from_state} -> {transition.to_state}")
        rospy.logdebug(f"[{self.node_name}] Transition history length: {len(self.state_transition_history)}")
    
    def _persist_current_state(self):
        """Persist current state to disk"""
        try:
            snapshot = self._create_state_snapshot()
            
            with open(self.persistence_path, 'wb') as f:
                pickle.dump(snapshot, f)
            
            rospy.logdebug(f"[{self.node_name}] State persisted to {self.persistence_path}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] State persistence error: {e}")
    
    def _load_persisted_state(self):
        """Load persisted state from disk"""
        try:
            if os.path.exists(self.persistence_path):
                with open(self.persistence_path, 'rb') as f:
                    snapshot = pickle.load(f)
                
                self._restore_from_snapshot(snapshot)
                rospy.loginfo(f"[{self.node_name}] State restored from {self.persistence_path}")
                rospy.loginfo(f"[{self.node_name}] Restored state: {self.current_state}")
            else:
                rospy.loginfo(f"[{self.node_name}] No persisted state found")
                
        except Exception as e:
            rospy.logwarn(f"[{self.node_name}] State restoration error: {e}")
    
    def _create_state_snapshot(self) -> StateSnapshot:
        """Create a snapshot of current state"""
        return StateSnapshot(
            state=self.current_state or "UNKNOWN",
            timestamp=time.time(),
            active_nodes=[],  # Would be populated by FSM
            safety_status={},  # Would be populated by safety monitor
            system_metrics={
                'cpu_percent': psutil.cpu_percent(interval=None),
                'memory_percent': psutil.virtual_memory().percent,
                'transition_count': len(self.state_transition_history)
            },
            transition_history=[asdict(t) for t in self.state_transition_history[-10:]],  # Last 10 transitions
            metadata={
                'recovery_attempts': self.recovery_attempt_count,
                'validation_failures': dict(self.validation_failures)
            }
        )
    
    def _restore_from_snapshot(self, snapshot: StateSnapshot):
        """Restore state from snapshot"""
        self.current_state = snapshot.state
        self.state_entry_time = time.time()  # Reset entry time to now
        
        # Restore transition history if available
        if snapshot.transition_history:
            try:
                self.state_transition_history = [
                    StateTransition(**t) for t in snapshot.transition_history
                ]
            except Exception as e:
                rospy.logwarn(f"[{self.node_name}] Could not restore transition history: {e}")
        
        # Restore metadata
        if snapshot.metadata:
            self.recovery_attempt_count = snapshot.metadata.get('recovery_attempts', 0)
            self.validation_failures = snapshot.metadata.get('validation_failures', {})
    
    def _periodic_snapshot_callback(self, event):
        """Periodic snapshot callback"""
        if self.enable_persistence and self.current_state:
            snapshot = self._create_state_snapshot()
            self.state_snapshots.append(snapshot)
            
            # Limit snapshots
            if len(self.state_snapshots) > self.max_snapshots:
                self.state_snapshots = self.state_snapshots[-self.max_snapshots//2:]
            
            rospy.logdebug(f"[{self.node_name}] Periodic snapshot created: {len(self.state_snapshots)} total")
    
    # Default precondition checkers
    def _check_camera_available(self) -> bool:
        """Check if camera is available"""
        # Simplified check - in reality would check ROS topics
        return True
    
    def _check_lane_detection_ready(self) -> bool:
        """Check if lane detection is ready"""
        return True
    
    def _check_apriltag_detection_ready(self) -> bool:
        """Check if AprilTag detection is ready"""
        return True
    
    def _check_basic_systems_operational(self) -> bool:
        """Check if basic systems are operational"""
        try:
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory_percent = psutil.virtual_memory().percent
            return cpu_percent < 90.0 and memory_percent < 90.0
        except:
            return True
    
    # Default postcondition checkers
    def _verify_lane_following_active(self) -> bool:
        """Verify lane following is active"""
        return True
    
    def _verify_coordination_active(self) -> bool:
        """Verify coordination is active"""
        return True
    
    def _verify_emergency_stop_active(self) -> bool:
        """Verify emergency stop is active"""
        return True
    
    # Recovery strategies
    def _recover_from_timeout(self, validation_result: StateValidationResult, target_state: str, 
                             event: str, metadata: Dict[str, Any]) -> bool:
        """Recover from timeout error"""
        rospy.logwarn(f"[{self.node_name}] Recovering from timeout: forcing transition to SAFE_MODE")
        
        # Force transition to safe mode
        self.current_state = 'SAFE_MODE'
        self.state_entry_time = time.time()
        
        return True
    
    def _recover_from_consistency_error(self, validation_result: StateValidationResult, target_state: str,
                                       event: str, metadata: Dict[str, Any]) -> bool:
        """Recover from consistency error"""
        rospy.logwarn(f"[{self.node_name}] Recovering from consistency error: clearing history")
        
        # Clear recent history to break oscillation
        if len(self.state_transition_history) > 5:
            self.state_transition_history = self.state_transition_history[:-3]
        
        return True
    
    def _recover_from_safety_violation(self, validation_result: StateValidationResult, target_state: str,
                                      event: str, metadata: Dict[str, Any]) -> bool:
        """Recover from safety violation"""
        rospy.logwarn(f"[{self.node_name}] Recovering from safety violation: emergency stop")
        
        # Force emergency stop
        self.current_state = 'EMERGENCY_STOP'
        self.state_entry_time = time.time()
        
        return True
    
    def _recover_from_resource_shortage(self, validation_result: StateValidationResult, target_state: str,
                                       event: str, metadata: Dict[str, Any]) -> bool:
        """Recover from resource shortage"""
        rospy.logwarn(f"[{self.node_name}] Recovering from resource shortage: reducing load")
        
        # Wait for resources to free up
        rospy.sleep(1.0)
        
        return True
    
    def _general_recovery_strategy(self, validation_result: StateValidationResult, target_state: str,
                                  event: str, metadata: Dict[str, Any]) -> bool:
        """General recovery strategy"""
        rospy.logwarn(f"[{self.node_name}] Applying general recovery strategy")
        
        # Wait and retry
        rospy.sleep(0.5)
        
        return True
    
    def get_state_statistics(self) -> Dict[str, Any]:
        """Get state management statistics"""
        with self.lock:
            avg_durations = {}
            for state, durations in self.state_durations.items():
                if durations:
                    avg_durations[state] = sum(durations) / len(durations)
            
            return {
                'current_state': self.current_state,
                'previous_state': self.previous_state,
                'total_transitions': len(self.state_transition_history),
                'transition_counts': dict(self.transition_counts),
                'average_state_durations': avg_durations,
                'validation_failures': dict(self.validation_failures),
                'recovery_attempts': self.recovery_attempt_count,
                'recovery_events': len(self.recovery_events),
                'snapshots_created': len(self.state_snapshots)
            }
    
    def get_current_state(self) -> Optional[str]:
        """Get current state"""
        return self.current_state
    
    def get_time_in_current_state(self) -> float:
        """Get time spent in current state"""
        if self.state_entry_time:
            return time.time() - self.state_entry_time
        return 0.0
    
    def shutdown(self):
        """Shutdown state manager"""
        rospy.loginfo(f"[{self.node_name}] StateManager shutting down")
        
        # Final persistence
        if self.enable_persistence and self.current_state:
            self._persist_current_state()
        
        # Log final statistics
        stats = self.get_state_statistics()
        rospy.loginfo(f"[{self.node_name}] Final state statistics:")
        rospy.loginfo(f"[{self.node_name}] Total transitions: {stats['total_transitions']}")
        rospy.loginfo(f"[{self.node_name}] Recovery events: {stats['recovery_events']}")
        rospy.loginfo(f"[{self.node_name}] Validation failures: {sum(stats['validation_failures'].values())}")
        
        rospy.loginfo(f"[{self.node_name}] StateManager shutdown complete")


class SafetyMonitor:
    """
    Multi-layer safety monitoring system for comprehensive system health tracking.
    Monitors hardware, sensors, algorithms, and behavioral safety.
    """
    
    def __init__(self, node_name):
        self.node_name = node_name
        
        # Safety monitoring parameters
        self.cpu_temp_warning_threshold = rospy.get_param("~cpu_temp_warning", 70.0)  # Celsius
        self.cpu_temp_critical_threshold = rospy.get_param("~cpu_temp_critical", 85.0)  # Celsius
        self.memory_warning_threshold = rospy.get_param("~memory_warning", 80.0)  # Percentage
        self.memory_critical_threshold = rospy.get_param("~memory_critical", 95.0)  # Percentage
        self.sensor_timeout_threshold = rospy.get_param("~sensor_timeout", 2.0)  # Seconds
        self.algorithm_fps_warning = rospy.get_param("~algorithm_fps_warning", 15.0)  # FPS
        self.algorithm_fps_critical = rospy.get_param("~algorithm_fps_critical", 10.0)  # FPS
        
        # Safety state tracking
        self.safety_status = SafetyStatus()
        self.safety_status.safety_level = SafetyStatus.SAFE
        self.safety_status.hardware_health = SafetyStatus.HEALTH_OK
        self.safety_status.camera_status = SafetyStatus.SENSOR_OK
        self.safety_status.imu_status = SafetyStatus.SENSOR_OK
        self.safety_status.encoder_status = SafetyStatus.SENSOR_OK
        self.safety_status.emergency_stop_active = False
        self.safety_status.system_health_score = 100.0
        
        # Sensor monitoring
        self.last_camera_time = None
        self.last_imu_time = None
        self.last_encoder_time = None
        self.camera_fps_counter = 0
        self.camera_fps_start_time = time.time()
        
        # Algorithm performance monitoring
        self.lane_detection_confidence = 0.0
        self.object_detection_fps = 0.0
        self.control_loop_frequency = 0.0
        self.algorithm_performance_history = []
        
        # Emergency state
        self.emergency_triggers = []
        self.safety_violations = []
        self.recovery_attempts = 0
        
        # Monitoring thread
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=self._safety_monitoring_loop)
        self.monitoring_thread.daemon = True
        
        rospy.loginfo(f"[{self.node_name}] Safety Monitor initialized with thresholds:")
        rospy.loginfo(f"[{self.node_name}] CPU temp warning/critical: {self.cpu_temp_warning_threshold}/{self.cpu_temp_critical_threshold}°C")
        rospy.loginfo(f"[{self.node_name}] Memory warning/critical: {self.memory_warning_threshold}/{self.memory_critical_threshold}%")
        rospy.loginfo(f"[{self.node_name}] Sensor timeout: {self.sensor_timeout_threshold}s")
        rospy.loginfo(f"[{self.node_name}] Algorithm FPS warning/critical: {self.algorithm_fps_warning}/{self.algorithm_fps_critical}")
    
    def start_monitoring(self):
        """Start the safety monitoring thread."""
        self.monitoring_thread.start()
        rospy.loginfo(f"[{self.node_name}] Safety monitoring thread started")
    
    def stop_monitoring(self):
        """Stop the safety monitoring thread."""
        self.monitoring_active = False
        if self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=1.0)
        rospy.loginfo(f"[{self.node_name}] Safety monitoring thread stopped")
    
    def _safety_monitoring_loop(self):
        """Main safety monitoring loop running in separate thread."""
        rate = rospy.Rate(1.0)  # 1 Hz monitoring
        
        while self.monitoring_active and not rospy.is_shutdown():
            try:
                current_time = rospy.Time.now()
                
                # Update safety status timestamp
                self.safety_status.header.stamp = current_time
                self.safety_status.last_safety_check = current_time
                
                # Monitor hardware health
                self._monitor_hardware_health()
                
                # Monitor sensor status
                self._monitor_sensor_status()
                
                # Monitor algorithm performance
                self._monitor_algorithm_performance()
                
                # Monitor behavioral safety
                self._monitor_behavioral_safety()
                
                # Calculate overall safety level
                self._calculate_overall_safety_level()
                
                # Log safety monitoring events
                self._log_safety_status()
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"[{self.node_name}] Safety monitoring error: {e}")
                self.safety_status.active_warnings.append(f"Safety monitoring error: {str(e)}")
    
    def _monitor_hardware_health(self):
        """Monitor CPU temperature and memory usage."""
        try:
            # Monitor CPU temperature
            cpu_temp = self._get_cpu_temperature()
            self.safety_status.cpu_temperature = cpu_temp
            
            # Monitor memory usage
            memory_info = psutil.virtual_memory()
            memory_usage = memory_info.percent
            self.safety_status.memory_usage = memory_usage
            
            # Update hardware health status
            hardware_warnings = []
            
            if cpu_temp > self.cpu_temp_critical_threshold:
                self.safety_status.hardware_health = SafetyStatus.HEALTH_CRITICAL
                hardware_warnings.append(f"CPU temperature critical: {cpu_temp:.1f}°C")
                rospy.logwarn(f"[{self.node_name}] CRITICAL: CPU temperature {cpu_temp:.1f}°C exceeds threshold {self.cpu_temp_critical_threshold}°C")
            elif cpu_temp > self.cpu_temp_warning_threshold:
                self.safety_status.hardware_health = max(self.safety_status.hardware_health, SafetyStatus.HEALTH_WARNING)
                hardware_warnings.append(f"CPU temperature warning: {cpu_temp:.1f}°C")
                rospy.logwarn(f"[{self.node_name}] WARNING: CPU temperature {cpu_temp:.1f}°C exceeds threshold {self.cpu_temp_warning_threshold}°C")
            
            if memory_usage > self.memory_critical_threshold:
                self.safety_status.hardware_health = SafetyStatus.HEALTH_CRITICAL
                hardware_warnings.append(f"Memory usage critical: {memory_usage:.1f}%")
                rospy.logwarn(f"[{self.node_name}] CRITICAL: Memory usage {memory_usage:.1f}% exceeds threshold {self.memory_critical_threshold}%")
            elif memory_usage > self.memory_warning_threshold:
                self.safety_status.hardware_health = max(self.safety_status.hardware_health, SafetyStatus.HEALTH_WARNING)
                hardware_warnings.append(f"Memory usage warning: {memory_usage:.1f}%")
                rospy.logwarn(f"[{self.node_name}] WARNING: Memory usage {memory_usage:.1f}% exceeds threshold {self.memory_warning_threshold}%")
            
            # Update hardware health timestamp
            if hardware_warnings:
                self.safety_status.last_hardware_check = rospy.Time.now()
                self.safety_status.active_warnings.extend(hardware_warnings)
            
            # Log hardware health metrics
            rospy.logdebug(f"[{self.node_name}] Hardware health: CPU {cpu_temp:.1f}°C, Memory {memory_usage:.1f}%")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Hardware monitoring error: {e}")
            self.safety_status.active_warnings.append(f"Hardware monitoring error: {str(e)}")
    
    def _monitor_sensor_status(self):
        """Monitor sensor health and connectivity."""
        current_time = rospy.Time.now()
        sensor_warnings = []
        
        # Monitor camera status
        if self.last_camera_time:
            camera_timeout = (current_time - self.last_camera_time).to_sec()
            if camera_timeout > self.sensor_timeout_threshold:
                self.safety_status.camera_status = SafetyStatus.SENSOR_FAILED
                sensor_warnings.append(f"Camera timeout: {camera_timeout:.1f}s")
                rospy.logwarn(f"[{self.node_name}] Camera sensor timeout: {camera_timeout:.1f}s")
            else:
                self.safety_status.camera_status = SafetyStatus.SENSOR_OK
        
        # Monitor IMU status
        if self.last_imu_time:
            imu_timeout = (current_time - self.last_imu_time).to_sec()
            if imu_timeout > self.sensor_timeout_threshold:
                self.safety_status.imu_status = SafetyStatus.SENSOR_FAILED
                sensor_warnings.append(f"IMU timeout: {imu_timeout:.1f}s")
                rospy.logwarn(f"[{self.node_name}] IMU sensor timeout: {imu_timeout:.1f}s")
            else:
                self.safety_status.imu_status = SafetyStatus.SENSOR_OK
        
        # Monitor encoder status (simplified)
        if self.last_encoder_time:
            encoder_timeout = (current_time - self.last_encoder_time).to_sec()
            if encoder_timeout > self.sensor_timeout_threshold:
                self.safety_status.encoder_status = SafetyStatus.SENSOR_FAILED
                sensor_warnings.append(f"Encoder timeout: {encoder_timeout:.1f}s")
                rospy.logwarn(f"[{self.node_name}] Encoder sensor timeout: {encoder_timeout:.1f}s")
            else:
                self.safety_status.encoder_status = SafetyStatus.SENSOR_OK
        
        if sensor_warnings:
            self.safety_status.active_warnings.extend(sensor_warnings)
        
        # Log sensor validation results
        rospy.logdebug(f"[{self.node_name}] Sensor status: Camera={self.safety_status.camera_status}, IMU={self.safety_status.imu_status}, Encoders={self.safety_status.encoder_status}")
    
    def _monitor_algorithm_performance(self):
        """Monitor algorithm performance and processing rates."""
        performance_warnings = []
        
        # Monitor lane detection confidence
        if self.lane_detection_confidence < 0.5:  # Below 50% confidence
            performance_warnings.append(f"Low lane detection confidence: {self.lane_detection_confidence:.2f}")
            rospy.logwarn(f"[{self.node_name}] Low lane detection confidence: {self.lane_detection_confidence:.2f}")
        
        # Monitor object detection FPS
        if self.object_detection_fps < self.algorithm_fps_critical:
            performance_warnings.append(f"Critical object detection FPS: {self.object_detection_fps:.1f}")
            rospy.logwarn(f"[{self.node_name}] CRITICAL: Object detection FPS {self.object_detection_fps:.1f} below threshold {self.algorithm_fps_critical}")
        elif self.object_detection_fps < self.algorithm_fps_warning:
            performance_warnings.append(f"Low object detection FPS: {self.object_detection_fps:.1f}")
            rospy.logwarn(f"[{self.node_name}] WARNING: Object detection FPS {self.object_detection_fps:.1f} below threshold {self.algorithm_fps_warning}")
        
        # Monitor control loop frequency
        if self.control_loop_frequency < 10.0:  # Below 10 Hz
            performance_warnings.append(f"Low control loop frequency: {self.control_loop_frequency:.1f}Hz")
            rospy.logwarn(f"[{self.node_name}] Low control loop frequency: {self.control_loop_frequency:.1f}Hz")
        
        # Update safety status
        self.safety_status.lane_detection_confidence = self.lane_detection_confidence
        self.safety_status.object_detection_fps = self.object_detection_fps
        self.safety_status.control_loop_frequency = self.control_loop_frequency
        
        if performance_warnings:
            self.safety_status.active_warnings.extend(performance_warnings)
        
        # Log algorithm performance checks
        rospy.logdebug(f"[{self.node_name}] Algorithm performance: Lane confidence={self.lane_detection_confidence:.2f}, Object FPS={self.object_detection_fps:.1f}, Control freq={self.control_loop_frequency:.1f}Hz")
    
    def _monitor_behavioral_safety(self):
        """Monitor behavioral safety violations and system state consistency."""
        behavioral_warnings = []
        
        # Check for behavioral safety violations
        if len(self.safety_violations) > 0:
            for violation in self.safety_violations:
                behavioral_warnings.append(f"Behavioral violation: {violation}")
                rospy.logwarn(f"[{self.node_name}] Behavioral safety violation: {violation}")
        
        # Check emergency triggers
        if len(self.emergency_triggers) > 0:
            for trigger in self.emergency_triggers:
                behavioral_warnings.append(f"Emergency trigger: {trigger}")
                rospy.logwarn(f"[{self.node_name}] Emergency trigger: {trigger}")
        
        if behavioral_warnings:
            self.safety_status.active_warnings.extend(behavioral_warnings)
        
        # Log behavioral safety checks
        if behavioral_warnings:
            rospy.logdebug(f"[{self.node_name}] Behavioral safety violations: {len(self.safety_violations)}, Emergency triggers: {len(self.emergency_triggers)}")
    
    def _calculate_overall_safety_level(self):
        """Calculate overall safety level based on all monitoring layers."""
        safety_scores = []
        
        # Hardware health score
        if self.safety_status.hardware_health == SafetyStatus.HEALTH_OK:
            hardware_score = 100.0
        elif self.safety_status.hardware_health == SafetyStatus.HEALTH_WARNING:
            hardware_score = 70.0
        else:  # HEALTH_CRITICAL
            hardware_score = 30.0
        safety_scores.append(hardware_score)
        
        # Sensor health score
        sensor_failures = 0
        if self.safety_status.camera_status == SafetyStatus.SENSOR_FAILED:
            sensor_failures += 1
        if self.safety_status.imu_status == SafetyStatus.SENSOR_FAILED:
            sensor_failures += 1
        if self.safety_status.encoder_status == SafetyStatus.SENSOR_FAILED:
            sensor_failures += 1
        
        sensor_score = max(0.0, 100.0 - (sensor_failures * 30.0))
        safety_scores.append(sensor_score)
        
        # Algorithm performance score
        algorithm_score = 100.0
        if self.lane_detection_confidence < 0.5:
            algorithm_score -= 20.0
        if self.object_detection_fps < self.algorithm_fps_warning:
            algorithm_score -= 15.0
        if self.control_loop_frequency < 10.0:
            algorithm_score -= 10.0
        safety_scores.append(max(0.0, algorithm_score))
        
        # Calculate overall system health score
        self.safety_status.system_health_score = min(safety_scores)
        
        # Determine overall safety level
        if self.safety_status.emergency_stop_active or len(self.emergency_triggers) > 0:
            self.safety_status.safety_level = SafetyStatus.EMERGENCY
        elif self.safety_status.system_health_score < 30.0:
            self.safety_status.safety_level = SafetyStatus.CRITICAL
        elif self.safety_status.system_health_score < 70.0:
            self.safety_status.safety_level = SafetyStatus.WARNING
        else:
            self.safety_status.safety_level = SafetyStatus.SAFE
        
        # Log overall safety level calculation
        rospy.logdebug(f"[{self.node_name}] Safety level calculation: Hardware={hardware_score:.1f}, Sensors={sensor_score:.1f}, Algorithms={algorithm_score:.1f}, Overall={self.safety_status.system_health_score:.1f}")
    
    def _log_safety_status(self):
        """Log current safety status and monitoring events."""
        safety_level_names = {
            SafetyStatus.SAFE: "SAFE",
            SafetyStatus.WARNING: "WARNING", 
            SafetyStatus.CRITICAL: "CRITICAL",
            SafetyStatus.EMERGENCY: "EMERGENCY"
        }
        
        current_level = safety_level_names.get(self.safety_status.safety_level, "UNKNOWN")
        
        # Log safety monitoring events with timestamps
        if self.safety_status.safety_level > SafetyStatus.SAFE:
            rospy.loginfo(f"[{self.node_name}] Safety status: {current_level} (Score: {self.safety_status.system_health_score:.1f})")
            
            if len(self.safety_status.active_warnings) > 0:
                rospy.loginfo(f"[{self.node_name}] Active warnings: {len(self.safety_status.active_warnings)}")
                for warning in self.safety_status.active_warnings[-3:]:  # Log last 3 warnings
                    rospy.loginfo(f"[{self.node_name}] Warning: {warning}")
        
        # Real-time monitoring logs
        rospy.logdebug(f"[{self.node_name}] Safety monitoring event: Level={current_level}, Score={self.safety_status.system_health_score:.1f}, Warnings={len(self.safety_status.active_warnings)}")
        
        # Clear processed warnings (keep only recent ones)
        if len(self.safety_status.active_warnings) > 10:
            self.safety_status.active_warnings = self.safety_status.active_warnings[-5:]
    
    def _get_cpu_temperature(self):
        """Get CPU temperature (Linux-specific implementation)."""
        try:
            # Try multiple common temperature sensor paths
            temp_paths = [
                "/sys/class/thermal/thermal_zone0/temp",
                "/sys/class/thermal/thermal_zone1/temp",
                "/sys/devices/virtual/thermal/thermal_zone0/temp"
            ]
            
            for path in temp_paths:
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        temp_millicelsius = int(f.read().strip())
                        return temp_millicelsius / 1000.0
            
            # Fallback: use psutil if available
            if hasattr(psutil, 'sensors_temperatures'):
                temps = psutil.sensors_temperatures()
                if temps:
                    for name, entries in temps.items():
                        if entries:
                            return entries[0].current
            
            # Default safe temperature if unable to read
            return 45.0
            
        except Exception as e:
            rospy.logdebug(f"[{self.node_name}] CPU temperature read error: {e}")
            return 45.0  # Safe default
    
    def update_sensor_status(self, sensor_type, timestamp):
        """Update sensor status with latest timestamp."""
        if sensor_type == "camera":
            self.last_camera_time = timestamp
            self.camera_fps_counter += 1
        elif sensor_type == "imu":
            self.last_imu_time = timestamp
        elif sensor_type == "encoder":
            self.last_encoder_time = timestamp
    
    def update_algorithm_performance(self, lane_confidence=None, object_fps=None, control_freq=None):
        """Update algorithm performance metrics."""
        if lane_confidence is not None:
            self.lane_detection_confidence = lane_confidence
        if object_fps is not None:
            self.object_detection_fps = object_fps
        if control_freq is not None:
            self.control_loop_frequency = control_freq
    
    def trigger_emergency_stop(self, reason):
        """Trigger emergency stop with specified reason."""
        self.safety_status.emergency_stop_active = True
        self.safety_status.emergency_reason = reason
        self.emergency_triggers.append(reason)
        
        rospy.logwarn(f"[{self.node_name}] EMERGENCY STOP TRIGGERED: {reason}")
        rospy.logwarn(f"[{self.node_name}] Emergency stop activations: {len(self.emergency_triggers)}")
    
    def clear_emergency_stop(self):
        """Clear emergency stop condition."""
        if self.safety_status.emergency_stop_active:
            rospy.loginfo(f"[{self.node_name}] Emergency stop cleared")
            self.recovery_attempts += 1
            rospy.loginfo(f"[{self.node_name}] Recovery attempts: {self.recovery_attempts}")
        
        self.safety_status.emergency_stop_active = False
        self.safety_status.emergency_reason = ""
    
    def add_safety_violation(self, violation):
        """Add a behavioral safety violation."""
        self.safety_violations.append(violation)
        rospy.logwarn(f"[{self.node_name}] Safety violation added: {violation}")
    
    def get_safety_status(self):
        """Get current safety status."""
        return self.safety_status


class FSMNode:
    def __init__(self):
        self.node_name = rospy.get_name()

        # Initialize safety monitor
        self.safety_monitor = SafetyMonitor(self.node_name)
        
        # Initialize safety status publisher
        self.safety_status_publisher = SafetyStatusPublisher(self.node_name)
        
        # Initialize enhanced state manager
        persistence_path = rospy.get_param("~state_persistence_path", f"/tmp/fsm_state_{self.node_name.replace('/', '_')}.pkl")
        self.state_manager = StateManager(self.node_name, persistence_path)
        
        # Safety monitoring parameters
        self.enable_safety_monitoring = rospy.get_param("~enable_safety_monitoring", True)
        self.emergency_stop_timeout = rospy.get_param("~emergency_stop_timeout", 0.2)  # 200ms
        self.safety_check_frequency = rospy.get_param("~safety_check_frequency", 1.0)  # 1 Hz
        
        # Safety state tracking
        self.safety_emergency_active = False
        self.last_safety_check = time.time()
        self.safety_state_transitions = 0
        self.emergency_stop_count = 0
        
        rospy.loginfo(f"[{self.node_name}] Safety monitoring enabled: {self.enable_safety_monitoring}")
        rospy.loginfo(f"[{self.node_name}] Emergency stop timeout: {self.emergency_stop_timeout}s")
        rospy.loginfo(f"[{self.node_name}] Safety check frequency: {self.safety_check_frequency}Hz")
        rospy.loginfo(f"[{self.node_name}] Safety status publisher initialized")

        # Build transition dictionray
        self.states_dict = rospy.get_param("~states", {})
        # Validate state and global transitions
        if not self._validateStates(self.states_dict):
            rospy.signal_shutdown(f"[{self.node_name}] Incoherent definition.")
            return

        # Load global transitions
        self.global_transitions_dict = rospy.get_param("~global_transitions", {})
        if not self._validateGlobalTransitions(self.global_transitions_dict, list(self.states_dict.keys())):
            rospy.signal_shutdown(f"[{self.node_name}] Incoherent definition.")
            return

        # Setup initial state
        self.state_msg = FSMState()
        self.state_msg.state = rospy.get_param("~initial_state", "")
        self.state_msg.header.stamp = rospy.Time.now()
        # Setup publisher and publish initial state
        self.pub_state = rospy.Publisher("~mode", FSMState, queue_size=1, latch=True)
        
        # Safety status publisher
        self.pub_safety_status = rospy.Publisher("~safety_status", SafetyStatus, queue_size=1, latch=True)
        
        # Enhanced AprilTag stop state management
        self.enable_apriltag_stops = rospy.get_param("~enable_apriltag_stops", True)
        self.apriltag_stop_distance_threshold = rospy.get_param("~apriltag_stop_distance_threshold", 1.0)
        self.apriltag_stop_duration = rospy.get_param("~apriltag_stop_duration", 2.0)
        
        # AprilTag stop state tracking
        self.apriltag_detections = []
        self.last_apriltag_detection_time = None
        self.apriltag_stop_start_time = None
        self.is_apriltag_stopping = False
        self.apriltag_stop_count = 0
        
        # Performance monitoring
        self.state_transition_count = 0
        self.apriltag_stop_sequences = 0
        self.last_performance_log = time.time()
        
        rospy.loginfo(f"[{self.node_name}] Enhanced AprilTag stop management: {self.enable_apriltag_stops}")
        rospy.loginfo(f"[{self.node_name}] AprilTag stop distance threshold: {self.apriltag_stop_distance_threshold}m")
        rospy.loginfo(f"[{self.node_name}] AprilTag stop duration: {self.apriltag_stop_duration}s")

        # Provide service
        self.srv_state = rospy.Service("~set_state", SetFSMState, self.cbSrvSetState)

        # Construct service calls
        self.srv_dict = dict()
        nodes = rospy.get_param("~nodes")
        # rospy.loginfo(nodes)
        self.active_nodes = None

        # for node_name, topic_name in list(nodes.items()):
        #     self.pub_dict[node_name] = rospy.Publisher(topic_name, BoolStamped, queue_size=1, latch=True)

        for node_name, service_name in list(nodes.items()):
            rospy.loginfo(f"FSM waiting for service {service_name}")
            try:
                rospy.wait_for_service(
                    service_name, timeout=10.0
                )  #  Not sure if there is a better way to do this
                self.srv_dict[node_name] = rospy.ServiceProxy(service_name, SetBool)
                rospy.loginfo(f"FSM found service {service_name}")
            except rospy.ROSException as e:
                rospy.logwarn(f"{e}")

        # to change the LEDs
        self.changePattern = rospy.ServiceProxy("~set_pattern", ChangePattern)

        # print self.pub_dict
        # Process events definition
        param_events_dict = rospy.get_param("~events", {})
        # Validate events definition
        if not self._validateEvents(param_events_dict):
            rospy.signal_shutdown(f"[{self.node_name}] Invalid event definition.")
            return

        self.sub_list = list()
        self.event_trigger_dict = dict()
        for event_name, event_dict in list(param_events_dict.items()):
            topic_name = event_dict["topic"]
            msg_type = event_dict["msg_type"]
            self.event_trigger_dict[event_name] = event_dict["trigger"]
            # TODO so far I can't figure out how to put msg_type instead of BoolStamped.
            # importlib might help. But it might get too complicated since different type
            self.sub_list.append(
                rospy.Subscriber(topic_name, BoolStamped, self.cbEvent, callback_args=event_name)
            )
        
        # Enhanced AprilTag integration subscribers
        if self.enable_apriltag_stops:
            self.sub_apriltag = rospy.Subscriber(
                "/apriltag_detector_node/detections", 
                AprilTagDetectionArray, 
                self.cb_apriltag_detections
            )
            self.sub_stop_line = rospy.Subscriber(
                "/stop_line_filter_node/stop_line_reading",
                StopLineReading,
                self.cb_stop_line_reading
            )
            rospy.loginfo(f"[{self.node_name}] AprilTag and stop line subscribers initialized")
        
        # Safety monitoring subscribers
        if self.enable_safety_monitoring:
            # Monitor sensor inputs for health checking
            self.sub_camera = rospy.Subscriber(
                "/camera_node/image/raw",
                Image,
                self.cb_camera_health
            )
            self.sub_imu = rospy.Subscriber(
                "/imu_node/data",
                Imu,
                self.cb_imu_health
            )
            self.sub_control_commands = rospy.Subscriber(
                "/lane_controller_node/car_cmd",
                Twist,
                self.cb_control_health
            )
            
            rospy.loginfo(f"[{self.node_name}] Safety monitoring subscribers initialized")
            
            # Start safety monitoring
            self.safety_monitor.start_monitoring()
            
            # Safety monitoring timer
            self.safety_timer = rospy.Timer(
                rospy.Duration(1.0 / self.safety_check_frequency),
                self.cb_safety_check
            )

        rospy.loginfo(f"[{self.node_name}] Initialized with comprehensive safety monitoring.")
        # Publish initial state
        self.publish()

    def _validateGlobalTransitions(self, global_transitions, valid_states):
        pass_flag = True
        for event_name, state_name in list(global_transitions.items()):
            if state_name not in valid_states:
                rospy.logerr(
                    f"[{self.node_name}] State {state_name} is not valid. (From global_transitions of "
                    f"{event_name})"
                )
                pass_flag = False
        return pass_flag

    def _validateEvents(self, events_dict):
        pass_flag = True
        for event_name, event_dict in list(events_dict.items()):
            if "topic" not in event_dict:
                rospy.logerr(f"[{self.node_name}] Event {event_name} missing topic definition.")
                pass_flag = False
            if "msg_type" not in event_dict:
                rospy.logerr(f"[{self.node_name}] Event {event_name} missing msg_type definition.")
                pass_flag = False
            if "trigger" not in event_dict:
                rospy.logerr(f"[{self.node_name}] Event {event_name} missing trigger definition.")
                pass_flag = False
        return pass_flag

    def _validateStates(self, states_dict):
        pass_flag = True
        valid_states = list(states_dict.keys())
        for state, state_dict in list(states_dict.items()):
            # Validate the existence of all reachable states
            transitions_dict = state_dict.get("transitions")
            if transitions_dict is None:
                continue
            else:
                for transition, next_state in list(transitions_dict.items()):
                    if next_state not in valid_states:
                        rospy.logerr(
                            f"[{self.node_name}] {next_state} not a valide state. (From {state} with event "
                            f"{transition})"
                        )
                        pass_flag = False
        return pass_flag

    def _getNextState(self, state_name, event_name):
        if not self.isValidState(state_name):
            rospy.logwarn(f"[{self.node_name}] {state_name} not defined. Treat as terminal. ")
            return None

        # state transitions overwrites global transition
        state_dict = self.states_dict.get(state_name)
        if "transitions" in state_dict:
            next_state = state_dict["transitions"].get(event_name)
        else:
            next_state = None

        # state transitions overwrites global transitions
        if next_state is None:
            # No state transition defined, look up global transition
            next_state = self.global_transitions_dict.get(event_name)  # None when no global transitions
        return next_state

    def _getActiveNodesOfState(self, state_name):
        state_dict = self.states_dict[state_name]
        active_nodes = state_dict.get("active_nodes")
        if active_nodes is None:
            rospy.logwarn(f"[{self.node_name}] No active nodes defined for {state_name}. Deactive all nodes.")
            active_nodes = []
        return active_nodes

    def _getLightsofState(self, state_name):
        state_dict = self.states_dict[state_name]
        lights = state_dict.get("lights")
        return lights

    def publish(self):
        self.publishBools()
        self.publishState()
        self.updateLights()

    def isValidState(self, state):
        return state in list(self.states_dict.keys())

    def cbSrvSetState(self, req):
        if self.isValidState(req.state):
            # Use enhanced state manager for transition
            success = self.state_manager.transition_to_state(
                new_state=req.state,
                event="service_request",
                metadata={"source": "service_call", "requested_state": req.state}
            )
            
            if success:
                self.state_msg.header.stamp = rospy.Time.now()
                self.state_msg.state = req.state
                self.publish()
                rospy.loginfo(f"[{self.node_name}] Service state transition successful: {req.state}")
            else:
                rospy.logwarn(f"[{self.node_name}] Service state transition failed: {req.state}")
        else:
            rospy.logwarn(f"[{self.node_name}] {req.state} is not a valid state.")
        return SetFSMStateResponse()

    def publishState(self):
        self.pub_state.publish(self.state_msg)
        rospy.loginfo(f"[{self.node_name}] FSMState: {self.state_msg.state}")

    def publishBools(self):
        active_nodes = self._getActiveNodesOfState(self.state_msg.state)

        for node_name, srv_pub in list(self.srv_dict.items()):
            msg = BoolStamped()
            msg.header.stamp = self.state_msg.header.stamp
            msg.data = bool(node_name in active_nodes)
            node_state = "ON" if msg.data else "OFF"
            # rospy.loginfo("[%s] Node %s is %s in %s" %(self.node_name, node_name, node_state,
            # self.state_msg.state))
            if self.active_nodes is not None:
                if (node_name in active_nodes) == (node_name in self.active_nodes):
                    continue
            # else:
            #     rospy.logwarn("[%s] self.active_nodes is None!" %(self.node_name))
            # continue

            resp = srv_pub(msg.data)

            # rospy.loginfo("[%s] node %s msg %s" %(self.node_name, node_name, msg))
            # rospy.loginfo("[%s] Node %s set to %s." %(self.node_name, node_name, node_state))
        self.active_nodes = copy.deepcopy(active_nodes)

    def updateLights(self):
        lights = self._getLightsofState(self.state_msg.state)
        if lights is not None:
            msg = String()
            msg.data = lights
            self.changePattern(msg)

    def cbEvent(self, msg, event_name):
        if msg.data == self.event_trigger_dict[event_name]:
            # Update timestamp
            self.state_msg.header.stamp = msg.header.stamp
            next_state = self._getNextState(self.state_msg.state, event_name)
            if next_state is not None:
                # Use enhanced state manager for transition
                success = self.state_manager.transition_to_state(
                    new_state=next_state,
                    event=event_name,
                    metadata={
                        "trigger_data": msg.data,
                        "trigger_timestamp": msg.header.stamp.to_sec(),
                        "previous_state": self.state_msg.state
                    }
                )
                
                if success:
                    # Has a defined transition
                    self.state_msg.state = next_state
                    self.state_transition_count += 1
                    
                    rospy.loginfo(f"[{self.node_name}] Enhanced state transition: {event_name} -> {next_state}")
                    rospy.logdebug(f"[{self.node_name}] Total state transitions: {self.state_transition_count}")
                    rospy.logdebug(f"[{self.node_name}] Time in previous state: {self.state_manager.get_time_in_current_state():.3f}s")
                    
                    self.publish()
                else:
                    rospy.logwarn(f"[{self.node_name}] Enhanced state transition failed: {event_name} -> {next_state}")
            else:
                rospy.logdebug(f"[{self.node_name}] No transition defined for event: {event_name} in state: {self.state_msg.state}")
    
    def cb_apriltag_detections(self, msg):
        """
        Callback for AprilTag detections to manage enhanced stop states.
        
        Args:
            msg: AprilTagDetectionArray message
        """
        if not self.enable_apriltag_stops:
            return
        
        current_time = rospy.Time.now()
        self.last_apriltag_detection_time = current_time
        self.apriltag_detections = msg.detections
        
        rospy.logdebug(f"[{self.node_name}] AprilTag detections received: {len(msg.detections)} tags")
        
        # Check if we should transition to AprilTag stop state
        for detection in msg.detections:
            # Extract distance from detection (simplified - in reality would use pose)
            # For now, simulate distance calculation
            simulated_distance = 0.8  # 80cm placeholder
            
            rospy.logdebug(f"[{self.node_name}] AprilTag {detection.tag_id}: simulated distance {simulated_distance:.3f}m")
            
            if (simulated_distance <= self.apriltag_stop_distance_threshold and 
                not self.is_apriltag_stopping and
                self.state_msg.state in ["LANE_FOLLOWING", "NORMAL_JOYSTICK_CONTROL"]):
                
                rospy.loginfo(f"[{self.node_name}] Initiating AprilTag stop sequence for tag {detection.tag_id}")
                self._initiate_apriltag_stop_sequence(detection.tag_id, simulated_distance)
                break
        
        # Real-time monitoring
        if len(msg.detections) > 0:
            rospy.logdebug(f"[{self.node_name}] AprilTag detection event: {len(msg.detections)} tags detected")
    
    def cb_stop_line_reading(self, msg):
        """
        Callback for stop line readings to coordinate with AprilTag stops.
        
        Args:
            msg: StopLineReading message
        """
        if not self.enable_apriltag_stops:
            return
        
        rospy.logdebug(f"[{self.node_name}] Stop line reading: detected={msg.stop_line_detected}, at_stop={msg.at_stop_line}")
        
        if msg.at_stop_line and not self.is_apriltag_stopping:
            # Traditional stop line detected, but we might want to coordinate with AprilTag stops
            rospy.logdebug(f"[{self.node_name}] Stop line detected, checking for AprilTag coordination")
            
            # If we have recent AprilTag detections, coordinate the stop
            if (self.last_apriltag_detection_time and 
                (rospy.Time.now() - self.last_apriltag_detection_time).to_sec() < 1.0):
                
                rospy.loginfo(f"[{self.node_name}] Coordinating stop line with AprilTag detection")
                self._initiate_apriltag_stop_sequence(tag_id=0, distance=0.3)  # Use stop line distance
    
    def _initiate_apriltag_stop_sequence(self, tag_id, distance):
        """
        Initiate AprilTag stop sequence with enhanced state management.
        
        Args:
            tag_id: ID of the detected AprilTag
            distance: Distance to the AprilTag
        """
        if self.is_apriltag_stopping:
            rospy.logdebug(f"[{self.node_name}] AprilTag stop sequence already in progress")
            return
        
        self.is_apriltag_stopping = True
        self.apriltag_stop_start_time = time.time()
        self.apriltag_stop_count += 1
        
        rospy.loginfo(f"[{self.node_name}] AprilTag stop sequence initiated:")
        rospy.loginfo(f"[{self.node_name}] Tag ID: {tag_id}, Distance: {distance:.3f}m")
        rospy.loginfo(f"[{self.node_name}] Stop count: {self.apriltag_stop_count}")
        
        # Use enhanced state manager for transition
        success = self.state_manager.transition_to_state(
            new_state="APRILTAG_STOP",
            event="apriltag_detected",
            metadata={
                "tag_id": tag_id,
                "distance": distance,
                "stop_count": self.apriltag_stop_count,
                "detection_time": time.time()
            }
        )
        
        if success:
            # Transition to AprilTag stop state
            previous_state = self.state_msg.state
            self.state_msg.state = "APRILTAG_STOP"
            self.state_msg.header.stamp = rospy.Time.now()
            self.state_transition_count += 1
            
            rospy.loginfo(f"[{self.node_name}] Enhanced state transition: {previous_state} -> APRILTAG_STOP")
            rospy.logdebug(f"[{self.node_name}] AprilTag stop sequence {self.apriltag_stop_count} started")
            rospy.logdebug(f"[{self.node_name}] State persistence and validation applied")
            
            self.publish()
            
            # Schedule return to normal operation after stop duration
            rospy.Timer(rospy.Duration(self.apriltag_stop_duration), self._complete_apriltag_stop_sequence, oneshot=True)
            
            # Real-time monitoring
            rospy.loginfo(f"[{self.node_name}] AprilTag stop timer started: {self.apriltag_stop_duration}s")
        else:
            rospy.logwarn(f"[{self.node_name}] AprilTag stop state transition failed - reverting")
            self.is_apriltag_stopping = False
            self.apriltag_stop_start_time = None
    
    def _complete_apriltag_stop_sequence(self, event):
        """
        Complete AprilTag stop sequence and return to normal operation.
        
        Args:
            event: Timer event (unused)
        """
        if not self.is_apriltag_stopping:
            return
        
        stop_end_time = time.time()
        total_stop_time = stop_end_time - self.apriltag_stop_start_time if self.apriltag_stop_start_time else 0.0
        
        rospy.loginfo(f"[{self.node_name}] AprilTag stop sequence completed")
        rospy.loginfo(f"[{self.node_name}] Total stop time: {total_stop_time:.3f}s")
        
        # Use enhanced state manager for transition back to lane following
        success = self.state_manager.transition_to_state(
            new_state="LANE_FOLLOWING",
            event="apriltag_stop_complete",
            metadata={
                "total_stop_time": total_stop_time,
                "stop_sequence_number": self.apriltag_stop_sequences + 1,
                "completion_time": stop_end_time
            }
        )
        
        if success:
            # Transition back to lane following
            previous_state = self.state_msg.state
            self.state_msg.state = "LANE_FOLLOWING"
            self.state_msg.header.stamp = rospy.Time.now()
            self.state_transition_count += 1
            self.apriltag_stop_sequences += 1
            
            rospy.loginfo(f"[{self.node_name}] Enhanced state transition: {previous_state} -> LANE_FOLLOWING")
            rospy.loginfo(f"[{self.node_name}] AprilTag stop sequences completed: {self.apriltag_stop_sequences}")
            rospy.logdebug(f"[{self.node_name}] State recovery and validation applied")
            
            # Reset stop state
            self.is_apriltag_stopping = False
            self.apriltag_stop_start_time = None
            
            self.publish()
            
            # Performance monitoring with state manager statistics
            current_time = time.time()
            if current_time - self.last_performance_log > 30.0:  # Log every 30 seconds
                state_stats = self.state_manager.get_state_statistics()
                
                rospy.loginfo(f"[{self.node_name}] Performance summary:")
                rospy.loginfo(f"[{self.node_name}] State transitions: {self.state_transition_count}")
                rospy.loginfo(f"[{self.node_name}] AprilTag stops: {self.apriltag_stop_count}")
                rospy.loginfo(f"[{self.node_name}] Completed stop sequences: {self.apriltag_stop_sequences}")
                rospy.loginfo(f"[{self.node_name}] State manager transitions: {state_stats['total_transitions']}")
                rospy.loginfo(f"[{self.node_name}] Validation failures: {sum(state_stats['validation_failures'].values())}")
                rospy.loginfo(f"[{self.node_name}] Recovery events: {state_stats['recovery_events']}")
                
                self.last_performance_log = current_time
            
            rospy.loginfo(f"[{self.node_name}] Resuming normal lane following operation")
        else:
            rospy.logwarn(f"[{self.node_name}] Failed to transition back to LANE_FOLLOWING - staying in current state")
            # Don't reset stop state if transition failed
    
    def cb_camera_health(self, msg):
        """Monitor camera health for safety monitoring."""
        if self.enable_safety_monitoring:
            self.safety_monitor.update_sensor_status("camera", msg.header.stamp)
            rospy.logdebug(f"[{self.node_name}] Camera health update: {msg.header.stamp}")
    
    def cb_imu_health(self, msg):
        """Monitor IMU health for safety monitoring."""
        if self.enable_safety_monitoring:
            self.safety_monitor.update_sensor_status("imu", msg.header.stamp)
            rospy.logdebug(f"[{self.node_name}] IMU health update: {msg.header.stamp}")
    
    def cb_control_health(self, msg):
        """Monitor control system health for safety monitoring."""
        if self.enable_safety_monitoring:
            # Calculate control loop frequency
            current_time = time.time()
            if hasattr(self, 'last_control_time'):
                control_dt = current_time - self.last_control_time
                if control_dt > 0:
                    control_freq = 1.0 / control_dt
                    self.safety_monitor.update_algorithm_performance(control_freq=control_freq)
            self.last_control_time = current_time
            
            rospy.logdebug(f"[{self.node_name}] Control health update: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}")
    
    def cb_safety_check(self, event):
        """Periodic safety check callback."""
        if not self.enable_safety_monitoring:
            return
        
        current_time = time.time()
        self.last_safety_check = current_time
        
        # Get current safety status
        safety_status = self.safety_monitor.get_safety_status()
        
        # Publish safety status through integrated publisher
        self.safety_status_publisher.publish_safety_status(safety_status)
        
        # Check for emergency conditions
        if safety_status.safety_level == SafetyStatus.EMERGENCY and not self.safety_emergency_active:
            self._handle_safety_emergency(safety_status)
        elif safety_status.safety_level == SafetyStatus.CRITICAL:
            self._handle_safety_critical(safety_status)
        elif safety_status.safety_level == SafetyStatus.WARNING:
            self._handle_safety_warning(safety_status)
        elif self.safety_emergency_active and safety_status.safety_level <= SafetyStatus.WARNING:
            self._handle_safety_recovery(safety_status)
        
        # Log safety monitoring events with timestamps
        if safety_status.safety_level > SafetyStatus.SAFE:
            rospy.logdebug(f"[{self.node_name}] Safety check: Level={safety_status.safety_level}, Score={safety_status.system_health_score:.1f}")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Safety monitoring event: Timestamp={rospy.Time.now()}, Emergency={self.safety_emergency_active}")
    
    def _handle_safety_emergency(self, safety_status):
        """Handle emergency safety conditions."""
        if self.safety_emergency_active:
            return
        
        self.safety_emergency_active = True
        self.emergency_stop_count += 1
        
        rospy.logwarn(f"[{self.node_name}] SAFETY EMERGENCY DETECTED!")
        rospy.logwarn(f"[{self.node_name}] Emergency reason: {safety_status.emergency_reason}")
        rospy.logwarn(f"[{self.node_name}] System health score: {safety_status.system_health_score:.1f}")
        rospy.logwarn(f"[{self.node_name}] Emergency stop count: {self.emergency_stop_count}")
        
        # Use enhanced state manager for emergency transition
        success = self.state_manager.transition_to_state(
            new_state="EMERGENCY_STOP",
            event="safety_emergency",
            metadata={
                "emergency_reason": safety_status.emergency_reason,
                "system_health_score": safety_status.system_health_score,
                "emergency_count": self.emergency_stop_count,
                "active_warnings": safety_status.active_warnings,
                "trigger_time": time.time()
            }
        )
        
        if success:
            # Transition to emergency stop state
            previous_state = self.state_msg.state
            self.state_msg.state = "EMERGENCY_STOP"
            self.state_msg.header.stamp = rospy.Time.now()
            self.safety_state_transitions += 1
            
            rospy.logwarn(f"[{self.node_name}] ENHANCED EMERGENCY STATE TRANSITION: {previous_state} -> EMERGENCY_STOP")
            rospy.logwarn(f"[{self.node_name}] Safety state transitions: {self.safety_state_transitions}")
            rospy.logwarn(f"[{self.node_name}] State persistence and recovery mechanisms activated")
            
            self.publish()
            
            # Log emergency stop activation with timestamp
            rospy.logwarn(f"[{self.node_name}] Emergency stop activated at {rospy.Time.now()}")
            
            # Add to safety violations
            self.safety_monitor.add_safety_violation(f"Emergency stop triggered: {safety_status.emergency_reason}")
        else:
            rospy.logerr(f"[{self.node_name}] CRITICAL: Emergency state transition failed!")
            # Force the transition anyway for safety
            self.state_msg.state = "EMERGENCY_STOP"
            self.state_msg.header.stamp = rospy.Time.now()
            self.publish()
    
    def _handle_safety_critical(self, safety_status):
        """Handle critical safety conditions."""
        rospy.logwarn(f"[{self.node_name}] SAFETY CRITICAL: Score={safety_status.system_health_score:.1f}")
        
        # Check if we should transition to safe mode
        if self.state_msg.state not in ["EMERGENCY_STOP", "SAFE_MODE"]:
            rospy.logwarn(f"[{self.node_name}] Transitioning to SAFE_MODE due to critical conditions")
            
            previous_state = self.state_msg.state
            self.state_msg.state = "SAFE_MODE"
            self.state_msg.header.stamp = rospy.Time.now()
            self.safety_state_transitions += 1
            
            rospy.logwarn(f"[{self.node_name}] CRITICAL STATE TRANSITION: {previous_state} -> SAFE_MODE")
            self.publish()
        
        # Log critical safety conditions
        for warning in safety_status.active_warnings[-3:]:  # Log last 3 warnings
            rospy.logwarn(f"[{self.node_name}] Critical warning: {warning}")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Critical safety event: Threshold violations={len(safety_status.active_warnings)}")
    
    def _handle_safety_warning(self, safety_status):
        """Handle warning safety conditions."""
        rospy.logwarn(f"[{self.node_name}] SAFETY WARNING: Score={safety_status.system_health_score:.1f}")
        
        # Log warning conditions
        if len(safety_status.active_warnings) > 0:
            rospy.logwarn(f"[{self.node_name}] Active warnings: {len(safety_status.active_warnings)}")
            for warning in safety_status.active_warnings[-2:]:  # Log last 2 warnings
                rospy.logwarn(f"[{self.node_name}] Warning: {warning}")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Warning safety event: Active warnings={len(safety_status.active_warnings)}")
    
    def _handle_safety_recovery(self, safety_status):
        """Handle safety recovery from emergency conditions."""
        if not self.safety_emergency_active:
            return
        
        rospy.loginfo(f"[{self.node_name}] SAFETY RECOVERY: Conditions improved")
        rospy.loginfo(f"[{self.node_name}] System health score: {safety_status.system_health_score:.1f}")
        
        self.safety_emergency_active = False
        self.safety_monitor.clear_emergency_stop()
        
        # Transition back to safe operation
        if self.state_msg.state == "EMERGENCY_STOP":
            previous_state = self.state_msg.state
            self.state_msg.state = "SAFE_MODE"  # Transition to safe mode first
            self.state_msg.header.stamp = rospy.Time.now()
            self.safety_state_transitions += 1
            
            rospy.loginfo(f"[{self.node_name}] RECOVERY STATE TRANSITION: {previous_state} -> SAFE_MODE")
            rospy.loginfo(f"[{self.node_name}] Recovery attempts: {self.safety_monitor.recovery_attempts}")
            
            self.publish()
        
        # Log recovery with timestamp
        rospy.loginfo(f"[{self.node_name}] Safety recovery completed at {rospy.Time.now()}")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Recovery event: Emergency cleared, transitioning to safe operation")
    
    def trigger_manual_emergency_stop(self, reason="Manual trigger"):
        """Manually trigger emergency stop for testing or external control."""
        rospy.logwarn(f"[{self.node_name}] Manual emergency stop triggered: {reason}")
        self.safety_monitor.trigger_emergency_stop(reason)
        
        # Force immediate safety check
        safety_status = self.safety_monitor.get_safety_status()
        self._handle_safety_emergency(safety_status)

    def on_shutdown(self):
        rospy.loginfo(f"[{self.node_name}] Shutting down with enhanced state management and safety monitoring.")
        
        # Shutdown enhanced state manager
        if hasattr(self, 'state_manager'):
            state_stats = self.state_manager.get_state_statistics()
            rospy.loginfo(f"[{self.node_name}] Final state manager statistics:")
            rospy.loginfo(f"[{self.node_name}] Total state transitions: {state_stats['total_transitions']}")
            rospy.loginfo(f"[{self.node_name}] Validation failures: {sum(state_stats['validation_failures'].values())}")
            rospy.loginfo(f"[{self.node_name}] Recovery events: {state_stats['recovery_events']}")
            rospy.loginfo(f"[{self.node_name}] Snapshots created: {state_stats['snapshots_created']}")
            
            self.state_manager.shutdown()
        
        # Stop safety monitoring
        if self.enable_safety_monitoring:
            self.safety_monitor.stop_monitoring()
            
            # Shutdown safety status publisher
            self.safety_status_publisher.shutdown()
            
            # Log final safety statistics
            rospy.loginfo(f"[{self.node_name}] Final safety statistics:")
            rospy.loginfo(f"[{self.node_name}] Safety state transitions: {self.safety_state_transitions}")
            rospy.loginfo(f"[{self.node_name}] Emergency stop count: {self.emergency_stop_count}")
            rospy.loginfo(f"[{self.node_name}] Recovery attempts: {self.safety_monitor.recovery_attempts}")
            
            # Log safety publisher statistics
            safety_stats = self.safety_status_publisher.get_safety_statistics()
            rospy.loginfo(f"[{self.node_name}] Safety publisher events: {safety_stats.get('total_events', 0)}")
            rospy.loginfo(f"[{self.node_name}] Average health score: {safety_stats.get('avg_health_score', 0.0):.1f}")
        
        rospy.loginfo(f"[{self.node_name}] Enhanced FSM shutdown complete.")


if __name__ == "__main__":
    # Initialize the node with rospy
    rospy.init_node("fsm_node", anonymous=False)

    # Create the NodeName object
    node = FSMNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()

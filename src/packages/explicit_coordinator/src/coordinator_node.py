#!/usr/bin/env python3

from random import random
import rospy
from duckietown_msgs.msg import (
    CoordinationClearance,
    FSMState,
    BoolStamped,
    Twist2DStamped,
    AprilTagsWithInfos,
)
from duckietown_msgs.msg import SignalsDetection, CoordinationSignal, MaintenanceState
from std_msgs.msg import String
from time import time
from enum import Enum
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass
import threading
import psutil
import os

UNKNOWN = "UNKNOWN"


class BehaviorPriority(Enum):
    """Priority levels for behavior arbitration"""
    EMERGENCY_STOP = 1000
    SAFETY_CRITICAL = 900
    OBSTACLE_AVOIDANCE = 800
    APRILTAG_STOP = 700
    LANE_CHANGE = 600
    INTERSECTION_COORDINATION = 500
    LANE_FOLLOWING = 400
    IDLE = 100


@dataclass
class BehaviorRequest:
    """Represents a behavior request with priority and metadata"""
    behavior_id: str
    priority: int
    command: Any
    timestamp: float
    source_node: str
    confidence: float = 1.0
    metadata: Dict[str, Any] = None
    
    def __post_init__(self):
        if self.metadata is None:
            self.metadata = {}


@dataclass
class ArbitrationResult:
    """Result of behavior arbitration process"""
    selected_behavior: Optional[BehaviorRequest]
    rejected_behaviors: List[BehaviorRequest]
    arbitration_time: float
    conflict_resolution_applied: bool
    reason: str


class BehaviorArbitrator:
    """Advanced behavior arbitration system with priority-based selection and conflict resolution"""
    
    def __init__(self, node_name: str):
        self.node_name = node_name
        self.registered_behaviors: Dict[str, Dict] = {}
        self.active_requests: List[BehaviorRequest] = []
        self.arbitration_history: List[ArbitrationResult] = []
        self.conflict_resolution_strategies: Dict[str, Callable] = {}
        self.lock = threading.Lock()
        
        # Performance monitoring
        self.arbitration_count = 0
        self.total_arbitration_time = 0.0
        self.conflict_count = 0
        
        # Initialize default conflict resolution strategies
        self._initialize_conflict_resolution()
        
        rospy.loginfo(f"[{self.node_name}] BehaviorArbitrator initialized with advanced arbitration capabilities")
    
    def _initialize_conflict_resolution(self):
        """Initialize default conflict resolution strategies"""
        self.conflict_resolution_strategies = {
            'priority_based': self._resolve_by_priority,
            'confidence_weighted': self._resolve_by_confidence,
            'temporal_priority': self._resolve_by_temporal_priority,
            'safety_first': self._resolve_safety_first
        }
        rospy.logdebug(f"[{self.node_name}] Initialized {len(self.conflict_resolution_strategies)} conflict resolution strategies")
    
    def register_behavior(self, behavior_id: str, priority: int, source_node: str, 
                         metadata: Dict[str, Any] = None) -> bool:
        """Register a behavior with the arbitrator"""
        with self.lock:
            if metadata is None:
                metadata = {}
                
            registration_info = {
                'priority': priority,
                'source_node': source_node,
                'metadata': metadata,
                'registration_time': time(),
                'request_count': 0,
                'last_selected': None
            }
            
            self.registered_behaviors[behavior_id] = registration_info
            
            rospy.loginfo(f"[{self.node_name}] Registered behavior '{behavior_id}' with priority {priority} from node '{source_node}'")
            rospy.logdebug(f"[{self.node_name}] Behavior registration metadata: {metadata}")
            
            return True
    
    def unregister_behavior(self, behavior_id: str) -> bool:
        """Unregister a behavior from the arbitrator"""
        with self.lock:
            if behavior_id in self.registered_behaviors:
                del self.registered_behaviors[behavior_id]
                # Remove any pending requests for this behavior
                self.active_requests = [req for req in self.active_requests if req.behavior_id != behavior_id]
                rospy.loginfo(f"[{self.node_name}] Unregistered behavior '{behavior_id}'")
                return True
            else:
                rospy.logwarn(f"[{self.node_name}] Attempted to unregister unknown behavior '{behavior_id}'")
                return False
    
    def submit_behavior_request(self, request: BehaviorRequest) -> bool:
        """Submit a behavior request for arbitration"""
        with self.lock:
            # Validate behavior is registered
            if request.behavior_id not in self.registered_behaviors:
                rospy.logwarn(f"[{self.node_name}] Rejected request for unregistered behavior '{request.behavior_id}'")
                return False
            
            # Update behavior statistics
            self.registered_behaviors[request.behavior_id]['request_count'] += 1
            
            # Add to active requests
            self.active_requests.append(request)
            
            rospy.logdebug(f"[{self.node_name}] Submitted behavior request: {request.behavior_id} (priority: {request.priority}, confidence: {request.confidence})")
            rospy.logdebug(f"[{self.node_name}] Active requests count: {len(self.active_requests)}")
            
            return True
    
    def arbitrate_behaviors(self, strategy: str = 'safety_first') -> ArbitrationResult:
        """Perform behavior arbitration using specified strategy"""
        start_time = time()
        
        with self.lock:
            self.arbitration_count += 1
            
            rospy.logdebug(f"[{self.node_name}] Starting behavior arbitration (attempt #{self.arbitration_count}) with strategy '{strategy}'")
            rospy.logdebug(f"[{self.node_name}] Active requests: {[req.behavior_id for req in self.active_requests]}")
            
            if not self.active_requests:
                result = ArbitrationResult(
                    selected_behavior=None,
                    rejected_behaviors=[],
                    arbitration_time=time() - start_time,
                    conflict_resolution_applied=False,
                    reason="No active behavior requests"
                )
                rospy.logdebug(f"[{self.node_name}] Arbitration completed: No active requests")
                return result
            
            # Apply conflict resolution strategy
            if strategy not in self.conflict_resolution_strategies:
                rospy.logwarn(f"[{self.node_name}] Unknown arbitration strategy '{strategy}', using 'safety_first'")
                strategy = 'safety_first'
            
            selected, rejected, conflict_applied = self.conflict_resolution_strategies[strategy](self.active_requests)
            
            # Update statistics
            if conflict_applied:
                self.conflict_count += 1
            
            if selected:
                self.registered_behaviors[selected.behavior_id]['last_selected'] = time()
                rospy.loginfo(f"[{self.node_name}] Arbitration selected behavior '{selected.behavior_id}' (priority: {selected.priority})")
                rospy.logdebug(f"[{self.node_name}] Selected behavior confidence: {selected.confidence}, source: {selected.source_node}")
            
            if rejected:
                rejected_ids = [req.behavior_id for req in rejected]
                rospy.logdebug(f"[{self.node_name}] Arbitration rejected behaviors: {rejected_ids}")
            
            # Clear active requests after arbitration
            self.active_requests.clear()
            
            arbitration_time = time() - start_time
            self.total_arbitration_time += arbitration_time
            
            result = ArbitrationResult(
                selected_behavior=selected,
                rejected_behaviors=rejected,
                arbitration_time=arbitration_time,
                conflict_resolution_applied=conflict_applied,
                reason=f"Strategy '{strategy}' applied successfully"
            )
            
            # Store in history (keep last 100 results)
            self.arbitration_history.append(result)
            if len(self.arbitration_history) > 100:
                self.arbitration_history.pop(0)
            
            rospy.logdebug(f"[{self.node_name}] Arbitration completed in {arbitration_time:.4f}s")
            
            return result
    
    def _resolve_by_priority(self, requests: List[BehaviorRequest]) -> tuple:
        """Resolve conflicts by priority only"""
        if not requests:
            return None, [], False
        
        # Sort by priority (higher number = higher priority)
        sorted_requests = sorted(requests, key=lambda x: x.priority, reverse=True)
        selected = sorted_requests[0]
        rejected = sorted_requests[1:]
        
        conflict_applied = len(requests) > 1
        
        rospy.logdebug(f"[{self.node_name}] Priority-based resolution: selected '{selected.behavior_id}' over {len(rejected)} others")
        
        return selected, rejected, conflict_applied
    
    def _resolve_by_confidence(self, requests: List[BehaviorRequest]) -> tuple:
        """Resolve conflicts by confidence-weighted priority"""
        if not requests:
            return None, [], False
        
        # Calculate weighted scores (priority * confidence)
        for req in requests:
            req.weighted_score = req.priority * req.confidence
        
        sorted_requests = sorted(requests, key=lambda x: x.weighted_score, reverse=True)
        selected = sorted_requests[0]
        rejected = sorted_requests[1:]
        
        conflict_applied = len(requests) > 1
        
        rospy.logdebug(f"[{self.node_name}] Confidence-weighted resolution: selected '{selected.behavior_id}' (score: {selected.weighted_score:.2f})")
        
        return selected, rejected, conflict_applied
    
    def _resolve_by_temporal_priority(self, requests: List[BehaviorRequest]) -> tuple:
        """Resolve conflicts considering temporal aspects"""
        if not requests:
            return None, [], False
        
        current_time = time()
        
        # Boost priority for time-critical requests
        for req in requests:
            age = current_time - req.timestamp
            # Boost emergency behaviors that are recent
            if req.priority >= BehaviorPriority.SAFETY_CRITICAL.value and age < 1.0:
                req.temporal_score = req.priority * 1.5
            else:
                req.temporal_score = req.priority * max(0.5, 1.0 - age * 0.1)
        
        sorted_requests = sorted(requests, key=lambda x: x.temporal_score, reverse=True)
        selected = sorted_requests[0]
        rejected = sorted_requests[1:]
        
        conflict_applied = len(requests) > 1
        
        rospy.logdebug(f"[{self.node_name}] Temporal priority resolution: selected '{selected.behavior_id}' (temporal score: {selected.temporal_score:.2f})")
        
        return selected, rejected, conflict_applied
    
    def _resolve_safety_first(self, requests: List[BehaviorRequest]) -> tuple:
        """Resolve conflicts with safety as the primary concern"""
        if not requests:
            return None, [], False
        
        # Separate safety-critical from normal requests
        safety_critical = [req for req in requests if req.priority >= BehaviorPriority.SAFETY_CRITICAL.value]
        normal_requests = [req for req in requests if req.priority < BehaviorPriority.SAFETY_CRITICAL.value]
        
        if safety_critical:
            # Among safety-critical, use confidence-weighted selection
            selected, rejected_safety, _ = self._resolve_by_confidence(safety_critical)
            rejected = rejected_safety + normal_requests
            rospy.loginfo(f"[{self.node_name}] Safety-first resolution: selected safety-critical behavior '{selected.behavior_id}'")
        else:
            # No safety-critical requests, use normal arbitration
            selected, rejected, _ = self._resolve_by_confidence(normal_requests)
            rospy.logdebug(f"[{self.node_name}] Safety-first resolution: no safety-critical requests, selected '{selected.behavior_id if selected else 'None'}'")
        
        conflict_applied = len(requests) > 1
        
        return selected, rejected, conflict_applied
    
    def get_arbitration_statistics(self) -> Dict[str, Any]:
        """Get arbitration performance statistics"""
        with self.lock:
            avg_arbitration_time = self.total_arbitration_time / max(1, self.arbitration_count)
            
            stats = {
                'total_arbitrations': self.arbitration_count,
                'total_conflicts': self.conflict_count,
                'average_arbitration_time': avg_arbitration_time,
                'registered_behaviors': len(self.registered_behaviors),
                'active_requests': len(self.active_requests),
                'conflict_rate': self.conflict_count / max(1, self.arbitration_count),
                'behavior_statistics': {}
            }
            
            # Add per-behavior statistics
            for behavior_id, info in self.registered_behaviors.items():
                stats['behavior_statistics'][behavior_id] = {
                    'priority': info['priority'],
                    'request_count': info['request_count'],
                    'last_selected': info['last_selected'],
                    'source_node': info['source_node']
                }
            
            return stats
    
    def update_behavior_priority(self, behavior_id: str, new_priority: int) -> bool:
        """Update the priority of a registered behavior"""
        with self.lock:
            if behavior_id in self.registered_behaviors:
                old_priority = self.registered_behaviors[behavior_id]['priority']
                self.registered_behaviors[behavior_id]['priority'] = new_priority
                rospy.loginfo(f"[{self.node_name}] Updated behavior '{behavior_id}' priority from {old_priority} to {new_priority}")
                return True
            else:
                rospy.logwarn(f"[{self.node_name}] Cannot update priority for unregistered behavior '{behavior_id}'")
                return False


class State:
    INTERSECTION_PLANNING = "INTERSECTION_PLANNING"
    LANE_FOLLOWING = "LANE_FOLLOWING"
    AT_STOP_CLEARING = "AT_STOP_CLEARING"
    SACRIFICE = "SACRIFICE"
    SOLVING_UNKNOWN = "SOLVING_UNKNOWN"
    GO = "GO"
    KEEP_CALM = "KEEP_CALM"
    TL_SENSING = "TL_SENSING"
    INTERSECTION_CONTROL = "INTERSECTION_CONTROL"
    AT_STOP_CLEARING_AND_PRIORITY = "AT_STOP_CLEARING_AND_PRIORITY"
    SACRIFICE_FOR_PRIORITY = "SACRIFICE_FOR_PRIORITY"
    OBSTACLE_ALERT = "OBSTACLE_ALERT"
    OBSTACLE_STOP = "OBSTACLE_STOP"


class VehicleCoordinator:
    """The Vehicle Coordination Module for Duckiebot with Advanced Behavior Arbitration"""

    T_MAX_RANDOM = 5.0  # seconds
    T_CROSS = 6.0  # seconds
    T_SENSE = 2.0  # seconds
    T_UNKNOWN = 1.0  # seconds
    T_MIN_RANDOM = 2.0  # seconds
    T_KEEP_CALM = 4.0  # seconds

    def __init__(self):

        self.node = rospy.init_node("veh_coordinator", anonymous=True)

        # We communicate that the coordination mode has started
        rospy.loginfo("The Enhanced Coordination Mode with Behavior Arbitration has Started")

        self.active = True

        # Determine the state of the bot
        self.state = State.INTERSECTION_PLANNING
        self.last_state_transition = time()
        self.random_delay = 0
        self.priority = False

        # Node name
        self.node_name = rospy.get_name()

        # Initialize flag
        self.intersection_go_published = False

        # Parameters
        self.traffic_light_intersection = UNKNOWN

        self.use_priority_protocol = True
        if rospy.get_param("~use_priority_protocol") == False:
            self.use_priority_protocol = False

        self.tl_timeout = 120
        rospy.set_param("~tl_timeout", self.tl_timeout)
        
        # Initialize advanced behavior arbitration system
        self.behavior_arbitrator = BehaviorArbitrator(self.node_name)
        self._initialize_behavior_registration()
        
        # Performance monitoring
        self.coordination_performance = {
            'total_decisions': 0,
            'arbitration_failures': 0,
            'conflict_resolutions': 0,
            'average_decision_time': 0.0,
            'last_performance_log': time()
        }

        # Initialize detection
        self.traffic_light = UNKNOWN
        self.right_veh = UNKNOWN
        self.opposite_veh = UNKNOWN

        # Initialize mode
        self.mode = "INTERSECTION_PLANNING"

        # Subscriptions
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)

        rospy.Subscriber("~mode", FSMState, lambda msg: self.set("mode", msg.state))
        rospy.Subscriber("~apriltags_out", AprilTagsWithInfos, self.set_traffic_light)
        rospy.Subscriber("~signals_detection", SignalsDetection, self.process_signals_detection)
        rospy.Subscriber("~maintenance_state", MaintenanceState, self.cbMaintenanceState)

        # Initialize clearance to go
        self.clearance_to_go = CoordinationClearance.NA

        # Set the light to be off
        self.roof_light = CoordinationSignal.OFF

        # Publishing
        self.clearance_to_go_pub = rospy.Publisher("~clearance_to_go", CoordinationClearance, queue_size=10)
        self.pub_intersection_go = rospy.Publisher("~intersection_go", BoolStamped, queue_size=1)
        self.pub_coord_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.roof_light_pub = rospy.Publisher("~change_color_pattern", String, queue_size=10)
        self.coordination_state_pub = rospy.Publisher("~coordination_state", String, queue_size=10)

        # Update param timer
        rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)
        
        # Performance monitoring timer
        rospy.Timer(rospy.Duration.from_sec(10.0), self.log_performance_metrics)

        while not rospy.is_shutdown():
            self.enhanced_coordination_loop()
            rospy.sleep(0.1)
    
    def _initialize_behavior_registration(self):
        """Initialize and register all coordination behaviors with the arbitrator"""
        rospy.loginfo(f"[{self.node_name}] Initializing behavior registration for enhanced coordination")
        
        # Register intersection coordination behaviors
        behaviors_to_register = [
            ("intersection_planning", BehaviorPriority.INTERSECTION_COORDINATION.value, "Planning intersection approach"),
            ("at_stop_clearing", BehaviorPriority.INTERSECTION_COORDINATION.value, "Clearing intersection stop"),
            ("traffic_light_sensing", BehaviorPriority.INTERSECTION_COORDINATION.value, "Sensing traffic light state"),
            ("intersection_go", BehaviorPriority.INTERSECTION_COORDINATION.value, "Executing intersection crossing"),
            ("sacrifice_behavior", BehaviorPriority.INTERSECTION_COORDINATION.value - 50, "Yielding to other vehicles"),
            ("keep_calm_behavior", BehaviorPriority.INTERSECTION_COORDINATION.value, "Maintaining calm state"),
            ("priority_coordination", BehaviorPriority.INTERSECTION_COORDINATION.value + 100, "Priority vehicle coordination"),
            ("emergency_coordination", BehaviorPriority.EMERGENCY_STOP.value, "Emergency coordination override")
        ]
        
        for behavior_id, priority, description in behaviors_to_register:
            metadata = {
                'description': description,
                'coordination_type': 'intersection',
                'can_be_interrupted': behavior_id not in ['emergency_coordination'],
                'requires_clearance': behavior_id in ['intersection_go', 'at_stop_clearing']
            }
            
            success = self.behavior_arbitrator.register_behavior(
                behavior_id=behavior_id,
                priority=priority,
                source_node=self.node_name,
                metadata=metadata
            )
            
            if success:
                rospy.logdebug(f"[{self.node_name}] Successfully registered behavior '{behavior_id}' with priority {priority}")
            else:
                rospy.logwarn(f"[{self.node_name}] Failed to register behavior '{behavior_id}'")
        
        rospy.loginfo(f"[{self.node_name}] Completed behavior registration: {len(behaviors_to_register)} behaviors registered")
    
    def enhanced_coordination_loop(self):
        """Enhanced coordination loop with behavior arbitration"""
        if not self.active:
            return
        
        start_time = time()
        
        # Collect current coordination requests
        coordination_requests = self._generate_coordination_requests()
        
        # Submit requests to arbitrator
        for request in coordination_requests:
            self.behavior_arbitrator.submit_behavior_request(request)
        
        # Perform arbitration
        arbitration_result = self.behavior_arbitrator.arbitrate_behaviors(strategy='safety_first')
        
        # Execute selected behavior
        if arbitration_result.selected_behavior:
            self._execute_coordination_behavior(arbitration_result.selected_behavior)
            rospy.logdebug(f"[{self.node_name}] Executed behavior: {arbitration_result.selected_behavior.behavior_id}")
        else:
            rospy.logdebug(f"[{self.node_name}] No behavior selected, maintaining current state")
        
        # Log conflicts and performance
        if arbitration_result.conflict_resolution_applied:
            self.coordination_performance['conflict_resolutions'] += 1
            rejected_ids = [req.behavior_id for req in arbitration_result.rejected_behaviors]
            rospy.logdebug(f"[{self.node_name}] Conflict resolved: selected '{arbitration_result.selected_behavior.behavior_id}' over {rejected_ids}")
        
        # Update performance metrics
        decision_time = time() - start_time
        self.coordination_performance['total_decisions'] += 1
        self.coordination_performance['average_decision_time'] = (
            (self.coordination_performance['average_decision_time'] * (self.coordination_performance['total_decisions'] - 1) + decision_time) /
            self.coordination_performance['total_decisions']
        )
        
        # Continue with original coordination logic
        if self.traffic_light_intersection != UNKNOWN:
            self.reconsider()
        self.publish_topics()
    
    def _generate_coordination_requests(self) -> List[BehaviorRequest]:
        """Generate behavior requests based on current coordination state"""
        requests = []
        current_time = time()
        
        rospy.logdebug(f"[{self.node_name}] Generating coordination requests for state: {self.state}")
        
        # Generate requests based on current state and conditions
        if self.state == State.INTERSECTION_PLANNING:
            if self.mode == "INTERSECTION_COORDINATION":
                request = BehaviorRequest(
                    behavior_id="intersection_planning",
                    priority=BehaviorPriority.INTERSECTION_COORDINATION.value,
                    command={"action": "plan_intersection", "traffic_light": self.traffic_light_intersection},
                    timestamp=current_time,
                    source_node=self.node_name,
                    confidence=0.9,
                    metadata={"state": self.state, "mode": self.mode}
                )
                requests.append(request)
                rospy.logdebug(f"[{self.node_name}] Generated intersection planning request")
        
        elif self.state == State.AT_STOP_CLEARING or self.state == State.AT_STOP_CLEARING_AND_PRIORITY:
            priority_boost = 100 if self.priority else 0
            request = BehaviorRequest(
                behavior_id="at_stop_clearing",
                priority=BehaviorPriority.INTERSECTION_COORDINATION.value + priority_boost,
                command={"action": "clear_stop", "priority": self.priority},
                timestamp=current_time,
                source_node=self.node_name,
                confidence=0.95,
                metadata={"state": self.state, "priority": self.priority}
            )
            requests.append(request)
            rospy.logdebug(f"[{self.node_name}] Generated stop clearing request (priority: {self.priority})")
        
        elif self.state == State.TL_SENSING:
            request = BehaviorRequest(
                behavior_id="traffic_light_sensing",
                priority=BehaviorPriority.INTERSECTION_COORDINATION.value,
                command={"action": "sense_traffic_light", "timeout": self.tl_timeout},
                timestamp=current_time,
                source_node=self.node_name,
                confidence=0.8,
                metadata={"state": self.state, "traffic_light": self.traffic_light}
            )
            requests.append(request)
            rospy.logdebug(f"[{self.node_name}] Generated traffic light sensing request")
        
        elif self.state == State.GO:
            request = BehaviorRequest(
                behavior_id="intersection_go",
                priority=BehaviorPriority.INTERSECTION_COORDINATION.value + 50,
                command={"action": "execute_go", "clearance": True},
                timestamp=current_time,
                source_node=self.node_name,
                confidence=1.0,
                metadata={"state": self.state, "clearance_to_go": self.clearance_to_go}
            )
            requests.append(request)
            rospy.logdebug(f"[{self.node_name}] Generated intersection go request")
        
        elif self.state in [State.SACRIFICE, State.SACRIFICE_FOR_PRIORITY]:
            request = BehaviorRequest(
                behavior_id="sacrifice_behavior",
                priority=BehaviorPriority.INTERSECTION_COORDINATION.value - 50,
                command={"action": "sacrifice", "delay": self.random_delay},
                timestamp=current_time,
                source_node=self.node_name,
                confidence=0.7,
                metadata={"state": self.state, "random_delay": self.random_delay}
            )
            requests.append(request)
            rospy.logdebug(f"[{self.node_name}] Generated sacrifice behavior request")
        
        elif self.state == State.KEEP_CALM:
            request = BehaviorRequest(
                behavior_id="keep_calm_behavior",
                priority=BehaviorPriority.INTERSECTION_COORDINATION.value,
                command={"action": "keep_calm", "duration": self.T_KEEP_CALM},
                timestamp=current_time,
                source_node=self.node_name,
                confidence=0.8,
                metadata={"state": self.state, "priority": self.priority}
            )
            requests.append(request)
            rospy.logdebug(f"[{self.node_name}] Generated keep calm behavior request")
        
        # Check for emergency conditions
        if self._detect_emergency_conditions():
            emergency_request = BehaviorRequest(
                behavior_id="emergency_coordination",
                priority=BehaviorPriority.EMERGENCY_STOP.value,
                command={"action": "emergency_stop", "reason": "coordination_emergency"},
                timestamp=current_time,
                source_node=self.node_name,
                confidence=1.0,
                metadata={"emergency": True, "original_state": self.state}
            )
            requests.append(emergency_request)
            rospy.logwarn(f"[{self.node_name}] Generated emergency coordination request")
        
        rospy.logdebug(f"[{self.node_name}] Generated {len(requests)} coordination requests")
        return requests
    
    def _execute_coordination_behavior(self, behavior_request: BehaviorRequest):
        """Execute the selected coordination behavior"""
        behavior_id = behavior_request.behavior_id
        command = behavior_request.command
        
        rospy.logdebug(f"[{self.node_name}] Executing coordination behavior: {behavior_id}")
        rospy.logdebug(f"[{self.node_name}] Behavior command: {command}")
        
        try:
            if behavior_id == "intersection_planning":
                self._execute_intersection_planning(command)
            elif behavior_id == "at_stop_clearing":
                self._execute_stop_clearing(command)
            elif behavior_id == "traffic_light_sensing":
                self._execute_traffic_light_sensing(command)
            elif behavior_id == "intersection_go":
                self._execute_intersection_go(command)
            elif behavior_id == "sacrifice_behavior":
                self._execute_sacrifice_behavior(command)
            elif behavior_id == "keep_calm_behavior":
                self._execute_keep_calm_behavior(command)
            elif behavior_id == "emergency_coordination":
                self._execute_emergency_coordination(command)
            else:
                rospy.logwarn(f"[{self.node_name}] Unknown behavior execution request: {behavior_id}")
            
            rospy.logdebug(f"[{self.node_name}] Successfully executed behavior: {behavior_id}")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error executing behavior '{behavior_id}': {str(e)}")
            self.coordination_performance['arbitration_failures'] += 1
    
    def _execute_intersection_planning(self, command: Dict[str, Any]):
        """Execute intersection planning behavior"""
        rospy.logdebug(f"[{self.node_name}] Executing intersection planning with traffic light: {command.get('traffic_light', 'unknown')}")
        # Original intersection planning logic is handled in reconsider()
    
    def _execute_stop_clearing(self, command: Dict[str, Any]):
        """Execute stop clearing behavior"""
        priority = command.get('priority', False)
        rospy.logdebug(f"[{self.node_name}] Executing stop clearing behavior (priority: {priority})")
        # Original stop clearing logic is handled in reconsider()
    
    def _execute_traffic_light_sensing(self, command: Dict[str, Any]):
        """Execute traffic light sensing behavior"""
        timeout = command.get('timeout', self.tl_timeout)
        rospy.logdebug(f"[{self.node_name}] Executing traffic light sensing with timeout: {timeout}s")
        # Original traffic light sensing logic is handled in reconsider()
    
    def _execute_intersection_go(self, command: Dict[str, Any]):
        """Execute intersection go behavior"""
        clearance = command.get('clearance', False)
        rospy.logdebug(f"[{self.node_name}] Executing intersection go behavior (clearance: {clearance})")
        # Original go logic is handled in reconsider()
    
    def _execute_sacrifice_behavior(self, command: Dict[str, Any]):
        """Execute sacrifice behavior"""
        delay = command.get('delay', self.random_delay)
        rospy.logdebug(f"[{self.node_name}] Executing sacrifice behavior with delay: {delay:.2f}s")
        # Original sacrifice logic is handled in reconsider()
    
    def _execute_keep_calm_behavior(self, command: Dict[str, Any]):
        """Execute keep calm behavior"""
        duration = command.get('duration', self.T_KEEP_CALM)
        rospy.logdebug(f"[{self.node_name}] Executing keep calm behavior for duration: {duration}s")
        # Original keep calm logic is handled in reconsider()
    
    def _execute_emergency_coordination(self, command: Dict[str, Any]):
        """Execute emergency coordination behavior"""
        reason = command.get('reason', 'unknown')
        rospy.logwarn(f"[{self.node_name}] Executing emergency coordination: {reason}")
        
        # Override current state and execute emergency stop
        self.clearance_to_go = CoordinationClearance.WAIT
        self.roof_light = CoordinationSignal.OFF
        
        # Publish emergency stop command
        car_cmd_msg = Twist2DStamped(v=0.0, omega=0.0)
        car_cmd_msg.header.stamp = rospy.Time.now()
        self.pub_coord_cmd.publish(car_cmd_msg)
        
        rospy.logwarn(f"[{self.node_name}] Emergency coordination executed: vehicle stopped")
    
    def _detect_emergency_conditions(self) -> bool:
        """Detect emergency conditions requiring immediate coordination override"""
        # Check for system resource constraints
        try:
            cpu_percent = psutil.cpu_percent(interval=None)
            memory_percent = psutil.virtual_memory().percent
            
            if cpu_percent > 95.0 or memory_percent > 95.0:
                rospy.logwarn(f"[{self.node_name}] High resource usage detected: CPU {cpu_percent}%, Memory {memory_percent}%")
                return True
        except Exception as e:
            rospy.logdebug(f"[{self.node_name}] Could not check system resources: {str(e)}")
        
        # Check for coordination timeout conditions
        if self.time_at_current_state() > 30.0:  # 30 seconds in same state
            rospy.logwarn(f"[{self.node_name}] Coordination timeout detected: {self.time_at_current_state():.1f}s in state {self.state}")
            return True
        
        return False
    
    def log_performance_metrics(self, event):
        """Log coordination performance metrics"""
        current_time = time()
        
        if current_time - self.coordination_performance['last_performance_log'] >= 10.0:
            # Get arbitrator statistics
            arbitrator_stats = self.behavior_arbitrator.get_arbitration_statistics()
            
            rospy.loginfo(f"[{self.node_name}] === Coordination Performance Metrics ===")
            rospy.loginfo(f"[{self.node_name}] Total decisions: {self.coordination_performance['total_decisions']}")
            rospy.loginfo(f"[{self.node_name}] Average decision time: {self.coordination_performance['average_decision_time']:.4f}s")
            rospy.loginfo(f"[{self.node_name}] Conflict resolutions: {self.coordination_performance['conflict_resolutions']}")
            rospy.loginfo(f"[{self.node_name}] Arbitration failures: {self.coordination_performance['arbitration_failures']}")
            rospy.loginfo(f"[{self.node_name}] Total arbitrations: {arbitrator_stats['total_arbitrations']}")
            rospy.loginfo(f"[{self.node_name}] Conflict rate: {arbitrator_stats['conflict_rate']:.2%}")
            rospy.loginfo(f"[{self.node_name}] Registered behaviors: {arbitrator_stats['registered_behaviors']}")
            
            self.coordination_performance['last_performance_log'] = current_time

    def loop(self):
        """Original coordination loop for backward compatibility"""
        if not self.active:
            return

        if self.traffic_light_intersection != UNKNOWN:
            self.reconsider()
        self.publish_topics()

    def cbMaintenanceState(self, msg):
        if msg.state == "WAY_TO_MAINTENANCE" and self.use_priority_protocol:
            self.priority = True
            rospy.loginfo(f"[{self.node_name}] Granted priority rights on intersections.")
        else:
            self.priority = False

    def set_traffic_light(self, msg):
        # Save old traffic light
        traffic_light_old = self.traffic_light_intersection
        # New traffic light
        # TODO: only consider two closest signs
        for item in msg.infos:
            if item.traffic_sign_type == 17:
                self.traffic_light_intersection = True
                break
            else:
                self.traffic_light_intersection = False
        # If different from the one before, restart from lane following
        if traffic_light_old != self.traffic_light_intersection:
            self.set_state(State.INTERSECTION_PLANNING)

        # Print result
        # if self.traffic_light_intersection != UNKNOWN:
        #     #TODO if tl but can't see the led's for too long, switch to april tag intersection
        #     # Print
        #     if self.traffic_light_intersection:
        #         rospy.loginfo('[%s] Intersection with traffic light' %(self.node_name))
        #     else:
        #         rospy.loginfo('[%s] Intersection without traffic light' %(self.node_name))

    def set_state(self, state):
        # Update only when changing state
        if self.state != state:
            rospy.loginfo(f"[{self.node_name}] Transitioned from {self.state} to {state}")
            self.last_state_transition = time()
            self.state = state

        # Set roof light
        if self.state == State.AT_STOP_CLEARING:
            # self.reset_signals_detection()
            self.roof_light = CoordinationSignal.SIGNAL_A
        elif self.state == State.AT_STOP_CLEARING_AND_PRIORITY:
            self.roof_light = CoordinationSignal.SIGNAL_PRIORITY
            # Publish LEDs - priority interrupt
            # self.roof_light_pub.publish(self.roof_light)
        elif self.state == State.SACRIFICE_FOR_PRIORITY:
            self.roof_light = CoordinationSignal.SIGNAL_SACRIFICE_FOR_PRIORITY
            # Publish LEDs - priority interrupt
            # self.roof_light_pub.publish(self.roof_light)
        elif self.state == State.SACRIFICE:
            self.roof_light = CoordinationSignal.OFF
        elif self.state == State.KEEP_CALM:
            if self.priority:
                self.roof_light = CoordinationSignal.SIGNAL_PRIORITY
            else:
                self.roof_light = CoordinationSignal.SIGNAL_A
        elif self.state == State.GO and not self.traffic_light_intersection:
            self.roof_light = CoordinationSignal.SIGNAL_GREEN
        elif self.state == State.INTERSECTION_PLANNING or self.state == State.TL_SENSING:
            self.roof_light = CoordinationSignal.OFF

    #    rospy.logdebug('[coordination_node] Transitioned to state' + self.state)

    # Define the time at this current state
    def time_at_current_state(self):
        return time() - self.last_state_transition

    def set(self, name, value):

        self.__dict__[name] = value

        # Initialization of the state and of the type of intersection
        if name == "mode":
            if value == "JOYSTICK_CONTROL" or value == "INTERSECTION_COORDINATION":
                self.set_state(State.INTERSECTION_PLANNING)
                self.traffic_light_intersection = UNKNOWN

    # Definition of each signal detection
    def process_signals_detection(self, msg):
        self.set("traffic_light", msg.traffic_light_state)
        self.set("right_veh", msg.right)
        self.set("opposite_veh", msg.front)

    # definition which resets everything we know
    def reset_signals_detection(self):
        self.traffic_light = UNKNOWN
        self.right_veh = UNKNOWN
        self.opposite_veh = UNKNOWN

    # publishing the topics
    def publish_topics(self):
        now = rospy.Time.now()

        # Publish LEDs
        self.roof_light_pub.publish(self.roof_light)

        # Clearance to go
        self.clearance_to_go_pub.publish(CoordinationClearance(status=self.clearance_to_go))

        # Publish intersection_go flag
        #        rospy.loginfo("clearance_to_go is "+str(self.clearance_to_go) + " and CoordinationClearance.GO is "+str(CoordinationClearance.GO))
        if self.clearance_to_go == CoordinationClearance.GO and not self.intersection_go_published:
            msg = BoolStamped()
            msg.header.stamp = now
            msg.data = True
            self.pub_intersection_go.publish(msg)
            self.intersection_go_published = True

            rospy.loginfo(f"[{self.node_name}] Go!")

        # Publish LEDs
        # self.roof_light_pub.publish(self.roof_light)

        car_cmd_msg = Twist2DStamped(v=0.0, omega=0.0)
        car_cmd_msg.header.stamp = now
        self.pub_coord_cmd.publish(car_cmd_msg)
        self.coordination_state_pub.publish(data=self.state)

    # definition of the loop
    def loop(self):

        if not self.active:
            return

        if self.traffic_light_intersection != UNKNOWN:
            self.reconsider()
        self.publish_topics()

    def reconsider(self):

        if self.state == State.INTERSECTION_PLANNING:
            if self.mode == "INTERSECTION_COORDINATION":
                # Reset detections
                self.reset_signals_detection()

                # Go to state (depending whether there is a traffic light)
                if self.traffic_light_intersection:
                    self.set_state(State.TL_SENSING)
                    self.begin_tl = time()

                else:
                    if self.priority:
                        self.set_state(State.AT_STOP_CLEARING_AND_PRIORITY)
                    else:
                        self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.AT_STOP_CLEARING_AND_PRIORITY:
            # First measurement not seen yet
            if self.right_veh == UNKNOWN or self.opposite_veh == UNKNOWN:
                self.random_delay = 1 + random() * self.T_UNKNOWN
                self.set_state(State.SOLVING_UNKNOWN)
            # Other cars with priority detected
            elif (
                self.right_veh == SignalsDetection.SIGNAL_PRIORITY
                or self.opposite_veh == SignalsDetection.SIGNAL_PRIORITY
            ):
                self.random_delay = self.T_MIN_RANDOM + random() * (self.T_MAX_RANDOM - self.T_MIN_RANDOM)
                self.set_state(State.SACRIFICE_FOR_PRIORITY)
                rospy.loginfo(
                    f"[{self.node_name}] Other vehicle are waiting as well. Will wait for "
                    f"{self.random_delay:.2f} s"
                )
            # No cars detected
            else:
                self.set_state(State.KEEP_CALM)

        elif self.state == State.AT_STOP_CLEARING:
            # First measurement not seen yet
            if self.right_veh == UNKNOWN or self.opposite_veh == UNKNOWN:
                self.random_delay = 1 + random() * self.T_UNKNOWN
                self.set_state(State.SOLVING_UNKNOWN)
            # Other cars with priority detected
            elif (
                self.right_veh == SignalsDetection.SIGNAL_PRIORITY
                or self.opposite_veh == SignalsDetection.SIGNAL_PRIORITY
            ):
                self.random_delay = self.T_MIN_RANDOM + random() * (self.T_MAX_RANDOM - self.T_MIN_RANDOM)
                self.set_state(State.SACRIFICE_FOR_PRIORITY)
                rospy.loginfo(
                    f"[{self.node_name}] Other vehicle are waiting as well. Will wait for "
                    f"{self.random_delay:.2f} s"
                )
            # Other cars with priority detected
            elif (
                self.right_veh == SignalsDetection.SIGNAL_SACRIFICE_FOR_PRIORITY
                or self.opposite_veh == SignalsDetection.SIGNAL_SACRIFICE_FOR_PRIORITY
            ):
                self.random_delay = self.T_MIN_RANDOM + random() * (self.T_MAX_RANDOM - self.T_MIN_RANDOM)
                self.set_state(State.SACRIFICE_FOR_PRIORITY)
                rospy.loginfo(
                    f"[{self.node_name}] Other vehicle are waiting as well. Will wait for "
                    f"{self.random_delay:.2f} s"
                )
            # Other cars  detected
            elif (
                self.right_veh == SignalsDetection.SIGNAL_A or self.opposite_veh == SignalsDetection.SIGNAL_A
            ):
                self.random_delay = self.T_MIN_RANDOM + random() * (self.T_MAX_RANDOM - self.T_MIN_RANDOM)
                self.set_state(State.SACRIFICE)
                rospy.loginfo(
                    f"[{self.node_name}] Other vehicle are waiting as well. Will wait for "
                    f"{self.random_delay:.2f} s"
                )
            # No cars detected
            else:
                self.set_state(State.KEEP_CALM)

        elif self.state == State.GO:
            self.clearance_to_go = CoordinationClearance.GO
            if self.mode == "INTERSECTION_PLANNING":
                self.set_state(State.INTERSECTION_PLANNING)

        elif self.state == State.SACRIFICE_FOR_PRIORITY:
            # Wait a random delay
            if self.time_at_current_state() > self.random_delay:
                if self.priority:
                    self.set_state(State.AT_STOP_CLEARING_AND_PRIORITY)
                else:
                    self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.SACRIFICE:
            # Wait a random delay
            if self.time_at_current_state() > self.random_delay:
                self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.KEEP_CALM:
            if self.priority:
                # Other cars with priority detected
                if (
                    self.right_veh == SignalsDetection.SIGNAL_PRIORITY
                    or self.opposite_veh == SignalsDetection.SIGNAL_PRIORITY
                ):
                    self.set_state(State.SACRIFICE_FOR_PRIORITY)
                # other cars not acknowledging my priority (just arrived)
                elif (
                    self.right_veh == SignalsDetection.SIGNAL_A
                    or self.opposite_veh == SignalsDetection.SIGNAL_A
                ):
                    self.set_state(State.KEEP_CALM)  # TODO: otherwise will go to else
                # No cars with priority detected
                else:
                    if self.time_at_current_state() > self.T_KEEP_CALM:
                        self.set_state(State.GO)
            else:
                # Other cars  detected
                if (
                    self.right_veh == SignalsDetection.SIGNAL_A
                    or self.opposite_veh == SignalsDetection.SIGNAL_A
                ):
                    self.set_state(State.SACRIFICE)
                    # No cars  detected
                else:
                    if self.time_at_current_state() > self.T_KEEP_CALM:
                        self.set_state(State.GO)

        elif self.state == State.SOLVING_UNKNOWN:
            if self.time_at_current_state() > self.random_delay:
                if self.priority:
                    self.set_state(State.AT_STOP_CLEARING_AND_PRIORITY)
                else:
                    self.set_state(State.AT_STOP_CLEARING)

        elif self.state == State.TL_SENSING:
            rospy.loginfo(
                "[%s] I have been waiting in traffic light for: %s", self.node_name, (time() - self.begin_tl)
            )
            if self.traffic_light == "traffic_light_go":
                rospy.loginfo("[%s] Traffic light is green. I have priority. GO!", self.node_name)
                self.set_state(State.GO)

            # If a tl intersection april tag is present but tl is switched off, wait until tl_timeout then use led coordination
            elif time() - self.begin_tl > self.tl_timeout:
                if self.priority:
                    self.set_state(State.AT_STOP_CLEARING_AND_PRIORITY)
                else:
                    self.set_state(State.AT_STOP_CLEARING)

        # If not GO, pusblish wait
        if self.state != State.GO:
            # Initialize intersection_go_published
            self.intersection_go_published = False
            # Publish wait
            self.clearance_to_go = CoordinationClearance.WAIT

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def updateParams(self, event):
        self.tl_timeout = rospy.get_param("~tl_timeout")

    # def onShutdown(self):
    #     rospy.loginfo("[CoordinatorNode] Shutdown.")
    #     self.clearance_to_go_pub.unregister()
    #     self.pub_intersection_go.unregister()
    #     self.pub_coord_cmd.unregister()
    #     self.roof_light_pub.unregister()
    #     self.coordination_state_pub.unregister()
    #


if __name__ == "__main__":
    car = VehicleCoordinator()
    #    rospy.on_shutdown(coordinator_node.onShutdown)
    rospy.spin()

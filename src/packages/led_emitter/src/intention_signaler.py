#!/usr/bin/env python3

import rospy
import time
import threading
from typing import Dict, Any, Optional, List
from enum import Enum
from dataclasses import dataclass

from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import String, Bool, ColorRGBA
from geometry_msgs.msg import Twist


class LaneChangeSignal(Enum):
    """Lane change signal types"""
    NONE = "none"
    LEFT_TURN = "left_turn"
    RIGHT_TURN = "right_turn"
    HAZARD = "hazard"
    ABORT = "abort"


class SignalPriority(Enum):
    """Signal priority levels"""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    EMERGENCY = 4


@dataclass
class SignalPattern:
    """LED signal pattern definition"""
    name: str
    color_list: List[str]
    frequency: float
    frequency_mask: List[int]
    duration: float
    priority: SignalPriority
    description: str


class IntentionSignaler:
    """
    Intention signaler for lane change communication using LED patterns.
    
    This system provides clear visual communication of lane change intentions
    through coordinated LED patterns, with abort mechanisms and FSM integration
    for safe lane change execution.
    """
    
    def __init__(self, led_emitter_node):
        """Initialize the intention signaler"""
        self.led_emitter = led_emitter_node
        self.node_name = led_emitter_node.node_name
        
        # Configuration parameters
        self.signaling_enabled = rospy.get_param("~lane_change_signaling_enabled", True)
        self.signal_duration = rospy.get_param("~lane_change_signal_duration", 3.0)  # seconds
        self.signal_frequency = rospy.get_param("~lane_change_signal_frequency", 2.0)  # Hz
        self.abort_signal_duration = rospy.get_param("~abort_signal_duration", 1.0)  # seconds
        self.signal_priority = rospy.get_param("~lane_change_signal_priority", SignalPriority.HIGH.value)
        
        # State tracking
        self.current_signal = LaneChangeSignal.NONE
        self.signal_active = False
        self.signal_start_time = None
        self.signal_timer = None
        self.abort_requested = False
        self.fsm_integration_active = False
        
        # Performance metrics
        self.performance_metrics = {
            'total_signals_sent': 0,
            'left_turn_signals': 0,
            'right_turn_signals': 0,
            'hazard_signals': 0,
            'abort_signals': 0,
            'signal_duration_total': 0.0,
            'average_signal_duration': 0.0,
            'fsm_integrations': 0,
            'last_update_time': time.time()
        }
        
        # Thread safety
        self.signal_lock = threading.Lock()
        
        # Define lane change signal patterns
        self.signal_patterns = self._initialize_signal_patterns()
        
        # FSM integration
        self.fsm_state_subscriber = rospy.Subscriber(
            "/fsm_node/mode", String, self.cb_fsm_state
        )
        
        # Lane change coordination
        self.lane_change_subscriber = rospy.Subscriber(
            "/navigation/lane_change_decision", String, self.cb_lane_change_decision
        )
        
        # Abort mechanism
        self.abort_subscriber = rospy.Subscriber(
            "/navigation/lane_change_abort", Bool, self.cb_lane_change_abort
        )
        
        # Signal status publisher
        self.signal_status_publisher = rospy.Publisher(
            "~lane_change_signal_status", String, queue_size=1
        )
        
        rospy.loginfo(f"[{self.node_name}] Intention Signaler initialized")
        rospy.loginfo(f"[{self.node_name}] Signaling enabled: {self.signaling_enabled}")
        rospy.loginfo(f"[{self.node_name}] Signal duration: {self.signal_duration}s")
        rospy.loginfo(f"[{self.node_name}] Signal frequency: {self.signal_frequency}Hz")
        rospy.loginfo(f"[{self.node_name}] Signal priority: {self.signal_priority}")
        rospy.loginfo(f"[{self.node_name}] Available signal patterns: {list(self.signal_patterns.keys())}")
    
    def _initialize_signal_patterns(self) -> Dict[str, SignalPattern]:
        """Initialize lane change signal patterns"""
        patterns = {}
        
        # Left turn signal pattern
        patterns["LEFT_TURN"] = SignalPattern(
            name="LEFT_TURN",
            color_list=["yellow", "yellow", "switchedoff", "switchedoff", "switchedoff"],
            frequency=self.signal_frequency,
            frequency_mask=[1, 1, 0, 0, 0],  # Blink left LEDs
            duration=self.signal_duration,
            priority=SignalPriority.HIGH,
            description="Left lane change intention signal"
        )
        
        # Right turn signal pattern
        patterns["RIGHT_TURN"] = SignalPattern(
            name="RIGHT_TURN",
            color_list=["switchedoff", "switchedoff", "switchedoff", "yellow", "yellow"],
            frequency=self.signal_frequency,
            frequency_mask=[0, 0, 0, 1, 1],  # Blink right LEDs
            duration=self.signal_duration,
            priority=SignalPriority.HIGH,
            description="Right lane change intention signal"
        )
        
        # Hazard signal pattern (both sides)
        patterns["HAZARD"] = SignalPattern(
            name="HAZARD",
            color_list=["yellow", "yellow", "yellow", "yellow", "yellow"],
            frequency=self.signal_frequency,
            frequency_mask=[1, 1, 1, 1, 1],  # Blink all LEDs
            duration=self.signal_duration * 2,  # Longer duration for hazard
            priority=SignalPriority.EMERGENCY,
            description="Hazard/emergency lane change signal"
        )
        
        # Abort signal pattern
        patterns["ABORT"] = SignalPattern(
            name="ABORT",
            color_list=["red", "red", "red", "red", "red"],
            frequency=4.0,  # Faster blinking for urgency
            frequency_mask=[1, 1, 1, 1, 1],  # Blink all LEDs
            duration=self.abort_signal_duration,
            priority=SignalPriority.EMERGENCY,
            description="Lane change abort signal"
        )
        
        # Confirmation signal pattern (lane change completed)
        patterns["CONFIRMATION"] = SignalPattern(
            name="CONFIRMATION",
            color_list=["green", "green", "green", "green", "green"],
            frequency=1.0,  # Slow blink
            frequency_mask=[1, 1, 1, 1, 1],  # Blink all LEDs
            duration=1.0,  # Short confirmation
            priority=SignalPriority.NORMAL,
            description="Lane change completion confirmation"
        )
        
        return patterns
    
    def cb_fsm_state(self, msg: String):
        """
        Callback for FSM state updates to coordinate with lane change signaling.
        
        Args:
            msg: FSM state message
        """
        if not self.signaling_enabled:
            return
        
        timestamp = time.time()
        fsm_state = msg.data
        
        rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] FSM state update: {fsm_state}")
        
        # Handle FSM integration for lane change states
        if "LANE_FOLLOWING" in fsm_state and self.signal_active:
            # Normal lane following - check if we should clear signals
            if self.current_signal in [LaneChangeSignal.LEFT_TURN, LaneChangeSignal.RIGHT_TURN]:
                rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Lane following resumed - clearing turn signals")
                self.clear_signal()
        
        elif "COORDINATION" in fsm_state:
            # Coordination state - may need hazard signals
            if not self.signal_active:
                rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Coordination state - activating hazard signals")
                self.signal_hazard()
        
        # Update FSM integration metrics
        self.fsm_integration_active = True
        self.performance_metrics['fsm_integrations'] += 1
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] FSM integration event: State={fsm_state}, Signal active={self.signal_active}")
    
    def cb_lane_change_decision(self, msg: String):
        """
        Callback for lane change decision messages.
        
        Args:
            msg: Lane change decision message
        """
        if not self.signaling_enabled:
            return
        
        timestamp = time.time()
        decision_data = msg.data
        
        rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Lane change decision received: {decision_data}")
        
        # Parse decision data
        if "CHANGE_LEFT" in decision_data:
            rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Signaling left lane change")
            self.signal_left_turn()
        elif "CHANGE_RIGHT" in decision_data:
            rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Signaling right lane change")
            self.signal_right_turn()
        elif "NO_CHANGE" in decision_data and self.signal_active:
            rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] No lane change needed - clearing signals")
            self.clear_signal()
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Lane change decision event: Decision={decision_data}, Signal={self.current_signal.value}")
    
    def cb_lane_change_abort(self, msg: Bool):
        """
        Callback for lane change abort messages.
        
        Args:
            msg: Abort message
        """
        if not self.signaling_enabled:
            return
        
        timestamp = time.time()
        abort_requested = msg.data
        
        if abort_requested:
            rospy.logwarn(f"[{self.node_name}] [{timestamp:.3f}] LANE CHANGE ABORT REQUESTED!")
            self.signal_abort()
        else:
            rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Lane change abort cleared")
            self.abort_requested = False
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Abort event: Requested={abort_requested}, Signal={self.current_signal.value}")
    
    def signal_left_turn(self):
        """Signal left lane change intention"""
        timestamp = time.time()
        
        rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Activating left turn signal")
        
        self._activate_signal(LaneChangeSignal.LEFT_TURN, "LEFT_TURN")
        
        # Update metrics
        self.performance_metrics['left_turn_signals'] += 1
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Left turn signal event: Duration={self.signal_duration}s, Frequency={self.signal_frequency}Hz")
    
    def signal_right_turn(self):
        """Signal right lane change intention"""
        timestamp = time.time()
        
        rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Activating right turn signal")
        
        self._activate_signal(LaneChangeSignal.RIGHT_TURN, "RIGHT_TURN")
        
        # Update metrics
        self.performance_metrics['right_turn_signals'] += 1
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Right turn signal event: Duration={self.signal_duration}s, Frequency={self.signal_frequency}Hz")
    
    def signal_hazard(self):
        """Signal hazard/emergency condition"""
        timestamp = time.time()
        
        rospy.logwarn(f"[{self.node_name}] [{timestamp:.3f}] Activating hazard signal")
        
        self._activate_signal(LaneChangeSignal.HAZARD, "HAZARD")
        
        # Update metrics
        self.performance_metrics['hazard_signals'] += 1
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Hazard signal event: Duration={self.signal_patterns['HAZARD'].duration}s")
    
    def signal_abort(self):
        """Signal lane change abort"""
        timestamp = time.time()
        
        rospy.logwarn(f"[{self.node_name}] [{timestamp:.3f}] Activating abort signal")
        
        self.abort_requested = True
        self._activate_signal(LaneChangeSignal.ABORT, "ABORT")
        
        # Update metrics
        self.performance_metrics['abort_signals'] += 1
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Abort signal event: Duration={self.abort_signal_duration}s")
    
    def signal_confirmation(self):
        """Signal lane change completion confirmation"""
        timestamp = time.time()
        
        rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Activating confirmation signal")
        
        self._activate_signal(LaneChangeSignal.NONE, "CONFIRMATION")  # Temporary signal
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Confirmation signal event: Duration=1.0s")
    
    def clear_signal(self):
        """Clear current lane change signal"""
        timestamp = time.time()
        
        with self.signal_lock:
            if self.signal_active:
                signal_duration = timestamp - self.signal_start_time if self.signal_start_time else 0.0
                
                rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Clearing lane change signal: {self.current_signal.value}")
                rospy.loginfo(f"[{self.node_name}] Signal duration: {signal_duration:.3f}s")
                
                # Cancel timer
                if self.signal_timer:
                    self.signal_timer.shutdown()
                    self.signal_timer = None
                
                # Clear signal state
                self.signal_active = False
                self.current_signal = LaneChangeSignal.NONE
                self.signal_start_time = None
                self.abort_requested = False
                
                # Return to normal LED pattern
                self.led_emitter.changePattern("CAR_DRIVING")
                
                # Update metrics
                self.performance_metrics['signal_duration_total'] += signal_duration
                if self.performance_metrics['total_signals_sent'] > 0:
                    self.performance_metrics['average_signal_duration'] = (
                        self.performance_metrics['signal_duration_total'] / 
                        self.performance_metrics['total_signals_sent']
                    )
                
                # Publish signal status
                self._publish_signal_status()
                
                rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Lane change signal cleared")
                
                # Real-time monitoring
                rospy.logdebug(f"[{self.node_name}] Signal clear event: Duration={signal_duration:.3f}s")
    
    def _activate_signal(self, signal_type: LaneChangeSignal, pattern_name: str):
        """
        Activate a lane change signal with the specified pattern.
        
        Args:
            signal_type: Type of signal to activate
            pattern_name: Name of the LED pattern to use
        """
        timestamp = time.time()
        
        with self.signal_lock:
            # Clear any existing signal
            if self.signal_active:
                self.clear_signal()
            
            # Activate new signal
            self.current_signal = signal_type
            self.signal_active = True
            self.signal_start_time = timestamp
            
            # Get pattern configuration
            pattern = self.signal_patterns.get(pattern_name)
            if not pattern:
                rospy.logerr(f"[{self.node_name}] Unknown signal pattern: {pattern_name}")
                return
            
            rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Activating signal pattern: {pattern_name}")
            rospy.loginfo(f"[{self.node_name}] Pattern description: {pattern.description}")
            rospy.loginfo(f"[{self.node_name}] Pattern duration: {pattern.duration}s")
            rospy.loginfo(f"[{self.node_name}] Pattern frequency: {pattern.frequency}Hz")
            rospy.loginfo(f"[{self.node_name}] Pattern priority: {pattern.priority.name}")
            
            # Apply the LED pattern
            self._apply_led_pattern(pattern)
            
            # Schedule automatic signal timeout
            self.signal_timer = rospy.Timer(
                rospy.Duration(pattern.duration),
                self._signal_timeout_callback,
                oneshot=True
            )
            
            # Update metrics
            self.performance_metrics['total_signals_sent'] += 1
            self.performance_metrics['last_update_time'] = timestamp
            
            # Publish signal status
            self._publish_signal_status()
            
            rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Signal activation completed: {pattern_name}")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Signal activation event: Pattern={pattern_name}, Duration={pattern.duration}s, Priority={pattern.priority.name}")
    
    def _apply_led_pattern(self, pattern: SignalPattern):
        """
        Apply LED pattern to the LED emitter.
        
        Args:
            pattern: Signal pattern to apply
        """
        timestamp = time.time()
        
        rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Applying LED pattern: {pattern.name}")
        rospy.loginfo(f"[{self.node_name}] Color list: {pattern.color_list}")
        rospy.loginfo(f"[{self.node_name}] Frequency mask: {pattern.frequency_mask}")
        
        # Create custom LED pattern
        try:
            # Update the LED emitter's protocol with our pattern
            protocol = self.led_emitter._LED_protocol
            protocol["signals"][f"LANE_CHANGE_{pattern.name}"] = {
                "color_mask": [1, 1, 1, 1, 1],  # All LEDs active
                "color_list": pattern.color_list,
                "frequency_mask": pattern.frequency_mask,
                "frequency": pattern.frequency,
            }
            
            # Update LED protocol
            self.led_emitter._LED_protocol = protocol
            rospy.set_param("~LED_protocol", protocol)
            
            # Change to the new pattern
            self.led_emitter.changePattern(f"LANE_CHANGE_{pattern.name}")
            
            rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] LED pattern applied successfully")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] LED pattern application event: Pattern={pattern.name}, Frequency={pattern.frequency}Hz")
            
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] [{timestamp:.3f}] Failed to apply LED pattern: {str(e)}")
            rospy.logerr(f"[{self.node_name}] Exception details: {type(e).__name__}")
    
    def _signal_timeout_callback(self, event):
        """Callback for signal timeout"""
        timestamp = time.time()
        
        rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Signal timeout reached for: {self.current_signal.value}")
        
        # Clear the signal unless it's an abort (which should be manually cleared)
        if self.current_signal != LaneChangeSignal.ABORT or not self.abort_requested:
            self.clear_signal()
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Signal timeout event: Signal={self.current_signal.value}, Abort requested={self.abort_requested}")
    
    def _publish_signal_status(self):
        """Publish current signal status"""
        timestamp = time.time()
        
        status_data = {
            'timestamp': timestamp,
            'signal_active': self.signal_active,
            'current_signal': self.current_signal.value,
            'signal_duration': timestamp - self.signal_start_time if self.signal_start_time else 0.0,
            'abort_requested': self.abort_requested,
            'fsm_integration_active': self.fsm_integration_active,
            'performance_metrics': self.performance_metrics
        }
        
        status_msg = String()
        status_msg.data = str(status_data)
        self.signal_status_publisher.publish(status_msg)
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Signal status published: Active={self.signal_active}, Signal={self.current_signal.value}")
    
    def get_signal_status(self) -> Dict[str, Any]:
        """Get current signal status"""
        timestamp = time.time()
        
        return {
            'signal_active': self.signal_active,
            'current_signal': self.current_signal.value,
            'signal_duration': timestamp - self.signal_start_time if self.signal_start_time else 0.0,
            'abort_requested': self.abort_requested,
            'fsm_integration_active': self.fsm_integration_active,
            'available_patterns': list(self.signal_patterns.keys()),
            'performance_metrics': self.performance_metrics
        }
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get performance metrics"""
        return {
            'total_signals_sent': self.performance_metrics['total_signals_sent'],
            'left_turn_signals': self.performance_metrics['left_turn_signals'],
            'right_turn_signals': self.performance_metrics['right_turn_signals'],
            'hazard_signals': self.performance_metrics['hazard_signals'],
            'abort_signals': self.performance_metrics['abort_signals'],
            'average_signal_duration': self.performance_metrics['average_signal_duration'],
            'fsm_integrations': self.performance_metrics['fsm_integrations'],
            'last_update_time': self.performance_metrics['last_update_time']
        }
    
    def reset_metrics(self):
        """Reset performance metrics"""
        self.performance_metrics = {
            'total_signals_sent': 0,
            'left_turn_signals': 0,
            'right_turn_signals': 0,
            'hazard_signals': 0,
            'abort_signals': 0,
            'signal_duration_total': 0.0,
            'average_signal_duration': 0.0,
            'fsm_integrations': 0,
            'last_update_time': time.time()
        }
        rospy.loginfo(f"[{self.node_name}] Intention signaler metrics reset")
    
    def shutdown(self):
        """Shutdown intention signaler"""
        timestamp = time.time()
        
        rospy.loginfo(f"[{self.node_name}] [{timestamp:.3f}] Shutting down Intention Signaler")
        
        # Clear any active signals
        if self.signal_active:
            self.clear_signal()
        
        # Cancel timers
        if self.signal_timer:
            self.signal_timer.shutdown()
        
        # Log final statistics
        metrics = self.get_performance_metrics()
        rospy.loginfo(f"[{self.node_name}] Intention signaler statistics:")
        rospy.loginfo(f"  Total signals sent: {metrics['total_signals_sent']}")
        rospy.loginfo(f"  Left turn signals: {metrics['left_turn_signals']}")
        rospy.loginfo(f"  Right turn signals: {metrics['right_turn_signals']}")
        rospy.loginfo(f"  Hazard signals: {metrics['hazard_signals']}")
        rospy.loginfo(f"  Abort signals: {metrics['abort_signals']}")
        rospy.loginfo(f"  Average signal duration: {metrics['average_signal_duration']:.2f}s")
        rospy.loginfo(f"  FSM integrations: {metrics['fsm_integrations']}")
        
        rospy.loginfo(f"[{self.node_name}] Intention Signaler shutdown complete")
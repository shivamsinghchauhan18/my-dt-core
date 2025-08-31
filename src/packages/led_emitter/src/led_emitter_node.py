#!/usr/bin/env python3
import time
import threading
import rospy

from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.srv import SetCustomLEDPatternResponse, ChangePatternResponse
from duckietown_msgs.msg import LEDPattern
from duckietown_enhanced_msgs.msg import SafetyStatus
from std_msgs.msg import ColorRGBA, Bool
from geometry_msgs.msg import Twist

from duckietown.dtros import DTROS, TopicType, NodeType


class EmergencyResponseSystem:
    """
    Emergency response system for LED-based warning patterns and motor control integration.
    Provides safety-specific LED patterns and emergency stop capabilities.
    """
    
    def __init__(self, led_emitter_node):
        self.led_emitter = led_emitter_node
        self.node_name = led_emitter_node.node_name
        
        # Emergency response parameters
        self.emergency_response_enabled = rospy.get_param("~emergency_response_enabled", True)
        self.emergency_pattern_priority = rospy.get_param("~emergency_pattern_priority", 100)
        self.warning_pattern_duration = rospy.get_param("~warning_pattern_duration", 5.0)  # seconds
        self.critical_pattern_duration = rospy.get_param("~critical_pattern_duration", 10.0)  # seconds
        self.emergency_motor_stop_timeout = rospy.get_param("~emergency_motor_stop_timeout", 0.2)  # seconds
        
        # Emergency state tracking
        self.current_emergency_level = SafetyStatus.SAFE
        self.emergency_pattern_active = False
        self.emergency_start_time = None
        self.last_safety_status = None
        self.motor_stop_active = False
        self.emergency_pattern_count = 0
        self.warning_pattern_count = 0
        self.critical_pattern_count = 0
        
        # Pattern timing control
        self.pattern_lock = threading.Lock()
        self.emergency_timer = None
        self.last_pattern_change = time.time()
        
        rospy.loginfo(f"[{self.node_name}] Emergency Response System initialized")
        rospy.loginfo(f"[{self.node_name}] Emergency response enabled: {self.emergency_response_enabled}")
        rospy.loginfo(f"[{self.node_name}] Emergency pattern priority: {self.emergency_pattern_priority}")
        rospy.loginfo(f"[{self.node_name}] Warning/Critical pattern durations: {self.warning_pattern_duration}s/{self.critical_pattern_duration}s")
        rospy.loginfo(f"[{self.node_name}] Emergency motor stop timeout: {self.emergency_motor_stop_timeout}s")
        
        # Motor control integration
        self.motor_stop_publisher = rospy.Publisher(
            "/emergency_stop", Bool, queue_size=1
        )
        self.motor_cmd_publisher = rospy.Publisher(
            "/car_cmd", Twist, queue_size=1
        )
        
        # Safety status subscriber
        self.safety_status_subscriber = rospy.Subscriber(
            "/fsm_node/safety_status", SafetyStatus, self.cb_safety_status
        )
        
        rospy.loginfo(f"[{self.node_name}] Emergency response motor control and safety monitoring initialized")
    
    def cb_safety_status(self, msg):
        """
        Callback for safety status updates to trigger appropriate emergency responses.
        
        Args:
            msg: SafetyStatus message
        """
        if not self.emergency_response_enabled:
            return
        
        self.last_safety_status = msg
        previous_level = self.current_emergency_level
        self.current_emergency_level = msg.safety_level
        
        rospy.logdebug(f"[{self.node_name}] Safety status update: Level={msg.safety_level}, Score={msg.system_health_score:.1f}")
        
        # Handle emergency level changes
        if msg.safety_level != previous_level:
            rospy.loginfo(f"[{self.node_name}] Emergency level change: {previous_level} -> {msg.safety_level}")
            self._handle_emergency_level_change(msg)
        
        # Handle specific emergency conditions
        if msg.safety_level == SafetyStatus.EMERGENCY:
            self._handle_emergency_response(msg)
        elif msg.safety_level == SafetyStatus.CRITICAL:
            self._handle_critical_response(msg)
        elif msg.safety_level == SafetyStatus.WARNING:
            self._handle_warning_response(msg)
        elif msg.safety_level == SafetyStatus.SAFE and self.emergency_pattern_active:
            self._handle_recovery_response(msg)
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Emergency response event: Level={msg.safety_level}, Pattern active={self.emergency_pattern_active}")
    
    def _handle_emergency_level_change(self, safety_status):
        """Handle emergency level changes with appropriate logging and pattern transitions."""
        current_time = time.time()
        self.last_pattern_change = current_time
        
        rospy.loginfo(f"[{self.node_name}] Emergency level change detected at {rospy.Time.now()}")
        rospy.loginfo(f"[{self.node_name}] System health score: {safety_status.system_health_score:.1f}")
        rospy.loginfo(f"[{self.node_name}] Active warnings: {len(safety_status.active_warnings)}")
        
        # Log pattern timing
        if self.emergency_start_time:
            pattern_duration = current_time - self.emergency_start_time
            rospy.loginfo(f"[{self.node_name}] Previous pattern duration: {pattern_duration:.3f}s")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Pattern change event: Timestamp={rospy.Time.now()}, Duration since last change={current_time - self.last_pattern_change:.3f}s")
    
    def _handle_emergency_response(self, safety_status):
        """Handle emergency safety conditions with immediate response."""
        if not self.emergency_pattern_active or self.current_emergency_level != SafetyStatus.EMERGENCY:
            self.emergency_pattern_count += 1
            self.emergency_start_time = time.time()
            
            rospy.logwarn(f"[{self.node_name}] EMERGENCY RESPONSE ACTIVATED!")
            rospy.logwarn(f"[{self.node_name}] Emergency reason: {safety_status.emergency_reason}")
            rospy.logwarn(f"[{self.node_name}] Emergency pattern activations: {self.emergency_pattern_count}")
            
            # Activate emergency LED pattern
            self._activate_emergency_pattern("EMERGENCY_STOP")
            
            # Execute emergency motor stop
            self._execute_emergency_motor_stop(safety_status.emergency_reason)
            
            # Log emergency response activation with timestamp
            rospy.logwarn(f"[{self.node_name}] Emergency response activated at {rospy.Time.now()}")
            rospy.logwarn(f"[{self.node_name}] Response time: {self.emergency_motor_stop_timeout}s")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Emergency response event: Motor stop active={self.motor_stop_active}, Pattern active={self.emergency_pattern_active}")
    
    def _handle_critical_response(self, safety_status):
        """Handle critical safety conditions with warning patterns."""
        if not self.emergency_pattern_active or self.current_emergency_level != SafetyStatus.CRITICAL:
            self.critical_pattern_count += 1
            self.emergency_start_time = time.time()
            
            rospy.logwarn(f"[{self.node_name}] CRITICAL RESPONSE ACTIVATED!")
            rospy.logwarn(f"[{self.node_name}] System health score: {safety_status.system_health_score:.1f}")
            rospy.logwarn(f"[{self.node_name}] Critical pattern activations: {self.critical_pattern_count}")
            
            # Activate critical LED pattern
            self._activate_emergency_pattern("CRITICAL_WARNING")
            
            # Schedule pattern duration
            self._schedule_pattern_timeout(self.critical_pattern_duration)
            
            # Log critical response activation
            rospy.logwarn(f"[{self.node_name}] Critical response activated at {rospy.Time.now()}")
            rospy.logwarn(f"[{self.node_name}] Pattern duration: {self.critical_pattern_duration}s")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Critical response event: Pattern duration={self.critical_pattern_duration}s, Health score={safety_status.system_health_score:.1f}")
    
    def _handle_warning_response(self, safety_status):
        """Handle warning safety conditions with attention patterns."""
        if not self.emergency_pattern_active or self.current_emergency_level != SafetyStatus.WARNING:
            self.warning_pattern_count += 1
            self.emergency_start_time = time.time()
            
            rospy.logwarn(f"[{self.node_name}] WARNING RESPONSE ACTIVATED!")
            rospy.logwarn(f"[{self.node_name}] System health score: {safety_status.system_health_score:.1f}")
            rospy.logwarn(f"[{self.node_name}] Warning pattern activations: {self.warning_pattern_count}")
            
            # Activate warning LED pattern
            self._activate_emergency_pattern("SAFETY_WARNING")
            
            # Schedule pattern duration
            self._schedule_pattern_timeout(self.warning_pattern_duration)
            
            # Log warning response activation
            rospy.logwarn(f"[{self.node_name}] Warning response activated at {rospy.Time.now()}")
            rospy.logwarn(f"[{self.node_name}] Pattern duration: {self.warning_pattern_duration}s")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Warning response event: Pattern duration={self.warning_pattern_duration}s, Active warnings={len(safety_status.active_warnings)}")
    
    def _handle_recovery_response(self, safety_status):
        """Handle recovery from emergency conditions."""
        if self.emergency_pattern_active:
            recovery_time = time.time() - self.emergency_start_time if self.emergency_start_time else 0.0
            
            rospy.loginfo(f"[{self.node_name}] EMERGENCY RECOVERY!")
            rospy.loginfo(f"[{self.node_name}] Recovery time: {recovery_time:.3f}s")
            rospy.loginfo(f"[{self.node_name}] System health score: {safety_status.system_health_score:.1f}")
            
            # Deactivate emergency patterns
            self._deactivate_emergency_pattern()
            
            # Clear motor stop if active
            if self.motor_stop_active:
                self._clear_emergency_motor_stop()
            
            # Log recovery with timestamp
            rospy.loginfo(f"[{self.node_name}] Emergency recovery completed at {rospy.Time.now()}")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Recovery event: Pattern deactivated, Motor stop cleared")
    
    def _activate_emergency_pattern(self, pattern_name):
        """Activate emergency LED pattern with priority override."""
        with self.pattern_lock:
            self.emergency_pattern_active = True
            
            rospy.loginfo(f"[{self.node_name}] Activating emergency pattern: {pattern_name}")
            rospy.loginfo(f"[{self.node_name}] Pattern priority: {self.emergency_pattern_priority}")
            
            # Change to emergency pattern
            self.led_emitter.changePattern(pattern_name)
            
            # Log pattern activation with timing
            rospy.logdebug(f"[{self.node_name}] Emergency pattern activated: {pattern_name} at {rospy.Time.now()}")
            
            # Real-time monitoring
            rospy.logdebug(f"[{self.node_name}] Pattern activation event: Name={pattern_name}, Priority={self.emergency_pattern_priority}")
    
    def _deactivate_emergency_pattern(self):
        """Deactivate emergency LED pattern and return to normal operation."""
        with self.pattern_lock:
            if self.emergency_pattern_active:
                pattern_duration = time.time() - self.emergency_start_time if self.emergency_start_time else 0.0
                
                rospy.loginfo(f"[{self.node_name}] Deactivating emergency pattern")
                rospy.loginfo(f"[{self.node_name}] Total pattern duration: {pattern_duration:.3f}s")
                
                self.emergency_pattern_active = False
                self.emergency_start_time = None
                
                # Cancel any active timer
                if self.emergency_timer:
                    self.emergency_timer.shutdown()
                    self.emergency_timer = None
                
                # Return to normal pattern
                self.led_emitter.changePattern("CAR_DRIVING")
                
                # Log pattern deactivation
                rospy.loginfo(f"[{self.node_name}] Emergency pattern deactivated at {rospy.Time.now()}")
                
                # Real-time monitoring
                rospy.logdebug(f"[{self.node_name}] Pattern deactivation event: Duration={pattern_duration:.3f}s")
    
    def _schedule_pattern_timeout(self, duration):
        """Schedule automatic pattern timeout for warning and critical patterns."""
        if self.emergency_timer:
            self.emergency_timer.shutdown()
        
        self.emergency_timer = rospy.Timer(
            rospy.Duration(duration),
            self._pattern_timeout_callback,
            oneshot=True
        )
        
        rospy.logdebug(f"[{self.node_name}] Pattern timeout scheduled: {duration}s")
    
    def _pattern_timeout_callback(self, event):
        """Callback for pattern timeout."""
        rospy.loginfo(f"[{self.node_name}] Emergency pattern timeout reached")
        
        # Only deactivate if we're not in emergency state
        if self.current_emergency_level != SafetyStatus.EMERGENCY:
            self._deactivate_emergency_pattern()
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Pattern timeout event: Emergency level={self.current_emergency_level}")
    
    def _execute_emergency_motor_stop(self, reason):
        """Execute emergency motor stop with immediate effect."""
        if self.motor_stop_active:
            return
        
        self.motor_stop_active = True
        
        rospy.logwarn(f"[{self.node_name}] EXECUTING EMERGENCY MOTOR STOP!")
        rospy.logwarn(f"[{self.node_name}] Stop reason: {reason}")
        rospy.logwarn(f"[{self.node_name}] Stop timeout: {self.emergency_motor_stop_timeout}s")
        
        # Publish emergency stop signal
        emergency_stop_msg = Bool()
        emergency_stop_msg.data = True
        self.motor_stop_publisher.publish(emergency_stop_msg)
        
        # Send zero velocity command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.motor_cmd_publisher.publish(stop_cmd)
        
        # Log motor stop command with timestamp
        rospy.logwarn(f"[{self.node_name}] Emergency motor stop executed at {rospy.Time.now()}")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Motor stop event: Reason={reason}, Timeout={self.emergency_motor_stop_timeout}s")
    
    def _clear_emergency_motor_stop(self):
        """Clear emergency motor stop condition."""
        if not self.motor_stop_active:
            return
        
        rospy.loginfo(f"[{self.node_name}] Clearing emergency motor stop")
        
        self.motor_stop_active = False
        
        # Publish emergency stop clear signal
        emergency_stop_msg = Bool()
        emergency_stop_msg.data = False
        self.motor_stop_publisher.publish(emergency_stop_msg)
        
        # Log motor stop clear
        rospy.loginfo(f"[{self.node_name}] Emergency motor stop cleared at {rospy.Time.now()}")
        
        # Real-time monitoring
        rospy.logdebug(f"[{self.node_name}] Motor stop clear event: Recovery completed")
    
    def get_emergency_status(self):
        """Get current emergency response status."""
        return {
            "emergency_level": self.current_emergency_level,
            "pattern_active": self.emergency_pattern_active,
            "motor_stop_active": self.motor_stop_active,
            "emergency_count": self.emergency_pattern_count,
            "warning_count": self.warning_pattern_count,
            "critical_count": self.critical_pattern_count
        }
    
    def shutdown(self):
        """Shutdown emergency response system."""
        rospy.loginfo(f"[{self.node_name}] Shutting down Emergency Response System")
        
        # Clear any active emergency patterns
        if self.emergency_pattern_active:
            self._deactivate_emergency_pattern()
        
        # Clear motor stop
        if self.motor_stop_active:
            self._clear_emergency_motor_stop()
        
        # Log final statistics
        rospy.loginfo(f"[{self.node_name}] Emergency response statistics:")
        rospy.loginfo(f"[{self.node_name}] Emergency patterns: {self.emergency_pattern_count}")
        rospy.loginfo(f"[{self.node_name}] Critical patterns: {self.critical_pattern_count}")
        rospy.loginfo(f"[{self.node_name}] Warning patterns: {self.warning_pattern_count}")
        
        rospy.loginfo(f"[{self.node_name}] Emergency Response System shutdown complete")


class LEDEmitterNode(DTROS):
    """Node for controlling LEDs.

    Publishes to the `~led_pattern` topic. If the absence of the FIFOs this should be remapped to the
     `led_driver_node/led_pattern` topic. The desired behavior is specified by
    the LED index (Duckiebots and watchtowers have multiple of these) and a pattern.
    A pattern is a combination of colors and blinking frequency.

    Duckiebots have 5 LEDs that are indexed and positioned as following:

    +------------------+------------------------------------------+
    | Index            | Position (rel. to direction of movement) |
    +==================+==========================================+
    | 0                | Front left                               |
    +------------------+------------------------------------------+
    | 1                | Rear left                                |
    +------------------+------------------------------------------+
    | 2                | Top / Front middle                       |
    +------------------+------------------------------------------+
    | 3                | Rear right                               |
    +------------------+------------------------------------------+
    | 4                | Front right                              |
    +------------------+------------------------------------------+

    A pattern is specified via 5 parameters:

    - its name

    - frequency: blinking frequency in Hz, should be set to 0 for a solid (non-blinking) behavior

    - color_list: a list of 5 colour names (see below), one for each LED ordered as above, or a single string with
      a single color name that would be applied to all LEDs

    - frequency_mask: a list of 5 binary flags (0 or 1) that specify which of the LEDs should be blinking,
      used only if the frequency is not 0. The LEDs with the flag set to 0, will maintain their solid color.

    The defaut patterns are defined in the `LED_protocol.yaml` configuration file for the node.

    Currently supported colors are: `green`, `red`, `blue`, `white`, `yellow`, `purple`, `cyan`,
    `pink`, `switchedoff`. More colors can be defined in the node's configuration file.

    Examples:

        To change the pattern to one of the predefined patterns (you can see them using `rosparam list`)
        use a variant of the following::

            rosservice call /HOSTNAME/led_emitter_node/set_pattern "pattern_name: {data: RED}"

        Other pre-defined patterns you can use are: `WHITE`, `GREEN`, `BLUE`, `LIGHT_OFF`, `CAR_SIGNAL_PRIORITY`,
        `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`, `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`, `CAR_SIGNAL_SACRIFICE_FOR_PRIORITY`,
        `CAR_DRIVING`.

        To add a custom pattern and switch to it use a variant of the following::

            rosservice call /HOSTNAME/led_emitter_node/set_custom_pattern "pattern: {color_list: ['green','yellow','pink','orange','blue'], color_mask: [1,1,1,1,1], frequency: 1.0, frequency_mask: [1,0,1,0,1]}"


    Configuration:
        ~LED_protocol (nested dictionary): Nested dictionary that describes the LED protocols (patterns). The
            default can be seen in the `LED_protocol.yaml` configuration file for the node.
        ~LED_scale (:obj:`float`): A scaling factor (between 0 and 1) that is applied to the colors in order
            to reduce the overall LED brightness, default is 0.8.
        ~channel_order (:obj:`str`): A string that controls the order in which the 3 color channels should be
            communicated to the LEDs. Should be one of 'RGB', `RBG`, `GBR`, `GRB`, `BGR`, `BRG`. Typically
            for a duckiebot this should be the default `RGB` and for traffic lights should be `GRB`, default is `RGB`.

    Publishers:
        ~led_pattern (:obj:`LEDPattern` message): Publishes the 5 LED values to be set by
        the LED driver

    Services:
        ~set_custom_pattern: Allows setting a custom protocol. Will be named `custom`. See an example of a call
            in :obj:`srvSetCustomLEDPattern`.

            input:

                pattern (:obj:`LEDPattern` message): The desired new LEDPattern

        ~set_pattern: Switch to a different pattern protocol.

            input:

                pattern_name (:obj:`String` message): The new pattern name, should match one of the patterns in
                   the `LED_protocol` parameter (or be `custom` if a custom pattern has been defined via a call to
                   the `~change_led` service.

    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LEDEmitterNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)

        self.LEDspattern = [[0.0, 0.0, 0.0]] * 5

        self.robot_type = rospy.get_param("~robot_type")

        # Add the node parameters to the parameters dictionary and load their default values
        self._LED_protocol = rospy.get_param("~LED_protocol")
        self._LED_scale = rospy.get_param("~LED_scale")
        self._channel_order = rospy.get_param("~channel_order")

        # Initialize LEDs to be off
        self.pattern = [[0, 0, 0]] * 5
        self.frequency_mask = [0] * 5
        self.current_pattern_name = "LIGHT_OFF"
        self.changePattern(self.current_pattern_name)
        
        # Initialize Emergency Response System
        self.emergency_response = EmergencyResponseSystem(self)
        
        # Initialize Intention Signaler for lane change communication
        from intention_signaler import IntentionSignaler
        self.intention_signaler = IntentionSignaler(self)
        
        rospy.loginfo(f"[{self.node_name}] LED Emitter with Emergency Response System and Intention Signaler initialized")

        # Initialize the timer
        self.frequency = 1.0 / self._LED_protocol["signals"]["CAR_SIGNAL_A"]["frequency"]
        self.is_on = False
    # Active switch (can be toggled by external logic if needed)
        self.switch = True
        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(self.frequency / 2.0), self._cycle_timer)

        # Publishers
        self.pub_leds = rospy.Publisher(
            "~led_pattern", LEDPattern, queue_size=1, dt_topic_type=TopicType.DRIVER
        )

        # Services
        self.srv_set_LED_ = rospy.Service(
            "~set_custom_pattern", SetCustomLEDPattern, self.srvSetCustomLEDPattern
        )
        self.srv_set_pattern_ = rospy.Service("~set_pattern", ChangePattern, self.srvSetPattern)

        # Scale intensity of the LEDs
        for name, c in self._LED_protocol["colors"].items():
            for i in range(3):
                c[i] = c[i] * self._LED_scale

        # Remap colors if robot does not have an RGB ordering
        if self._channel_order[self.robot_type] != "RGB":
            protocol = self._LED_protocol
            for name, col in self._LED_protocol["colors"].items():
                protocol["colors"][name] = self.remapColors(col)

            # update LED_protocol
            self._LED_protocol = protocol
            rospy.set_param("~LED_protocol", protocol)

            self.log("Colors remapped to " + str(self._channel_order[self.robot_type]))

        # Turn on the LEDs
        self.changePattern("WHITE")

        # Use DTROS logging helpers
        self.log("Initialized.")

    # --- Logging helpers (compat with existing calls) ---
    def log(self, msg, type: str = "info"):
        if type in ("err", "error"):
            self.logerr(msg)
        elif type in ("warn", "warning"):
            self.logwarn(msg)
        else:
            self.loginfo(msg)

    def srvSetCustomLEDPattern(self, req):
        """Service to set a custom pattern.

        Sets the LEDs to a custom pattern. The :obj:`LEDPattern` message from
        :obj:`duckietown_msgs` is used for that.

        Args:
            req (SetCustomLEDPatternRequest): the requested pattern

        """
        # Update the protocol
        protocol = self._LED_protocol
        protocol["signals"]["custom"] = {
            "color_mask": req.pattern.color_mask,
            "color_list": req.pattern.color_list,
            "frequency_mask": req.pattern.frequency_mask,
            "frequency": req.pattern.frequency,
        }
        # update LED_protocol
        self._LED_protocol = protocol
        rospy.set_param("~LED_protocol", protocol)

        self.log(
            "Custom pattern updated: "
            "color_mask: %s, " % str(self._LED_protocol["signals"]["custom"]["color_mask"])
            + "color_list: %s, " % str(self._LED_protocol["signals"]["custom"]["color_list"])
            + "frequency_mask: %s, " % str(self._LED_protocol["signals"]["custom"]["frequency_mask"])
            + "frequency: %s" % str(self._LED_protocol["signals"]["custom"]["frequency"])
        )

        # Perform the actual change
        self.changePattern("custom")
        # ---
        return SetCustomLEDPatternResponse()

    def _cycle_timer(self, event):
        """Timer.

        Calls updateLEDs according to the frequency of the current pattern.

        Args:
            event (TimerEvent): event generated by the timer.
        """
        self.updateLEDs()

    def updateLEDs(self):
        """Switches the LEDs to the requested signal.

        If the pattern is static, changes the color of LEDs according to
        the color specified in self.color_list. If a nonzero frequency is set,
        toggles on/off the LEDs specified on self.frequency_mask.
        """
        # Do nothing if inactive
        if not self.switch:
            return

        elif not self.frequency:
            # No oscillation
            for i in range(5):
                self.LEDspattern[i] = self.pattern[i]
        else:
            # Oscillate
            if self.is_on:
                for i in range(5):
                    if self.frequency_mask[i]:
                        self.LEDspattern[i] = [0.0, 0.0, 0.0]
                self.is_on = False

            else:
                for i in range(5):
                    self.LEDspattern[i] = self.pattern[i]
                self.is_on = True

        self.publishLEDs()

    def publishLEDs(self):
        LEDPattern_msg = LEDPattern()
        for i in range(5):
            rgba = ColorRGBA()
            rgba.r = self.LEDspattern[i][0]
            rgba.g = self.LEDspattern[i][1]
            rgba.b = self.LEDspattern[i][2]
            rgba.a = 1.0
            LEDPattern_msg.rgb_vals.append(rgba)
        self.pub_leds.publish(LEDPattern_msg)

    def srvSetPattern(self, msg):
        """Changes the current pattern according to the pattern name sent in the message.


        Args:
            msg (String): requested pattern name
        """
        self.changePattern(str(msg.pattern_name.data))
        return ChangePatternResponse()

    def changePattern(self, pattern_name):
        """Change the current LED pattern.

        Checks if the requested pattern is different from the current one,
        if so changes colors and frequency of LEDs accordingly and
        publishes the new current pattern. If the requested pattern name
        is not found, it will not change the pattern and will publish ROS
        Error log message.

        Args:
            pattern_name (string): Name of the wanted pattern

        """
        if pattern_name:
            # No need to change if we already have the right pattern, unless it is other, because
            # we might have updated its definition
            if self.current_pattern_name == pattern_name and pattern_name != "custom":
                return
            elif pattern_name.strip("'").strip('"') in self._LED_protocol["signals"]:
                self.current_pattern_name = pattern_name
            else:
                self.log(
                    "Pattern name %s not found in the list of patterns. Change of "
                    "pattern not executed." % pattern_name,
                    type="err",
                )
                self.log(self._LED_protocol["signals"], type="err")
                return

            # Extract the color from the protocol config file
            color_list = self._LED_protocol["signals"][pattern_name]["color_list"]

            if type(color_list) is str:
                self.pattern = [self._LED_protocol["colors"][color_list]] * 5
            else:
                if len(color_list) != 5:
                    self.log(
                        "The color list should be a string or a list of length 5. Change of "
                        "pattern not executed.",
                        type="err",
                    )
                    return

                self.pattern = [[0, 0, 0]] * 5
                for i in range(len(color_list)):
                    if isinstance(color_list[i], str):
                        self.pattern[i] = self._LED_protocol["colors"][color_list[i]]
                    elif isinstance(color_list[i], list) and len(color_list[i]) == 3:
                        self.pattern[i] = color_list[i]
                        self.pattern[i] = [max(0, min(c, 255)) for c in self.pattern[i]]
                    else:
                        self.log(
                            "LEDs color passed as RGB values must be expressed as lists of 3 "
                            "values from the range [0, 255].",
                            type="err",
                        )
                        return

            # Extract the frequency from the protocol
            self.frequency_mask = self._LED_protocol["signals"][pattern_name]["frequency_mask"]
            self.frequency = self._LED_protocol["signals"][pattern_name]["frequency"]

            # If static behavior, updated LEDs
            if self.frequency == 0:
                self.updateLEDs()

            # Anyway modify the frequency (to stop timer if static)
            self.changeFrequency()

            # Loginfo
            self.log("Pattern changed to (%r), cycle: %s " % (pattern_name, self.frequency))

    def changeFrequency(self):
        """Changes current frequency of LEDs

        Stops the current cycle_timer, and starts a new one with the
        frequency specified in self.frequency. If the frequency is zero,
        stops the callback timer.
        """
        if self.frequency == 0:
            self.cycle_timer.shutdown()

        else:
            try:
                self.cycle_timer.shutdown()
                # below, convert to hz
                d = 1.0 / (2.0 * self.frequency)
                self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(d), self._cycle_timer)

            except ValueError as e:
                self.frequency = None
                self.current_pattern_name = None

    def remapColors(self, color):
        """
        Remaps a color from RGB to the channel ordering currently set in the
        `channel_order` configuration parameter.

        Args:
            color (:obj:`list` of :obj:`float`): A color triplet

        Returns:
            :obj:`list` of :obj:`float`: The triplet with reordered channels

        """

        # Verify that the requested reordering is valid
        allowed_orderings = ["RGB", "RBG", "GBR", "GRB", "BGR", "BRG"]
        requested_ordering = self._channel_order[self.robot_type]
        if requested_ordering not in allowed_orderings:
            self.log(
                "The current channel order %s is not supported, use one of %s. "
                "The remapping was not performed." % (requested_ordering, str(allowed_orderings)),
                type="warn",
            )
            return color

        reordered_triplet = list()
        rgb_map = {"R": 0, "G": 1, "B": 2}
        for channel_color in requested_ordering:
            reordered_triplet.append(color[rgb_map[channel_color]])

        return reordered_triplet

    def on_shutdown(self):
        """Shutdown procedure.

        At shutdown, changes the LED pattern to `LIGHT_OFF` and cleans up emergency response system.
        """
        # Shutdown emergency response system first
        if hasattr(self, 'emergency_response'):
            self.emergency_response.shutdown()
        
        # Shutdown intention signaler
        if hasattr(self, 'intention_signaler'):
            self.intention_signaler.shutdown()
        
        # Turn off the lights when the node dies
        self.loginfo("Shutting down. Turning LEDs off.")
        self.changePattern("LIGHT_OFF")
        time.sleep(1)


if __name__ == "__main__":
    # Create the LEDEmitterNode object
    led_emitter_node = LEDEmitterNode(node_name="led_emitter")
    # Keep it spinning to keep the node alive
    rospy.spin()

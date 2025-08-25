#!/usr/bin/env python3
import copy
import time

import rospy
from duckietown_msgs.msg import BoolStamped, FSMState, AprilTagDetectionArray, StopLineReading
from duckietown_msgs.srv import SetFSMState, SetFSMStateResponse, ChangePattern
from std_srvs.srv import SetBool
from std_msgs.msg import String


class FSMNode:
    def __init__(self):
        self.node_name = rospy.get_name()

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

        rospy.loginfo(f"[{self.node_name}] Initialized.")
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
            self.state_msg.header.stamp = rospy.Time.now()
            self.state_msg.state = req.state
            self.publish()
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
                # Has a defined transition
                self.state_msg.state = next_state
                self.state_transition_count += 1
                
                rospy.loginfo(f"[{self.node_name}] State transition: {event_name} -> {next_state}")
                rospy.logdebug(f"[{self.node_name}] Total state transitions: {self.state_transition_count}")
                
                self.publish()
    
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
        
        # Transition to AprilTag stop state
        previous_state = self.state_msg.state
        self.state_msg.state = "APRILTAG_STOP"
        self.state_msg.header.stamp = rospy.Time.now()
        self.state_transition_count += 1
        
        rospy.loginfo(f"[{self.node_name}] State transition: {previous_state} -> APRILTAG_STOP")
        rospy.logdebug(f"[{self.node_name}] AprilTag stop sequence {self.apriltag_stop_count} started")
        
        self.publish()
        
        # Schedule return to normal operation after stop duration
        rospy.Timer(rospy.Duration(self.apriltag_stop_duration), self._complete_apriltag_stop_sequence, oneshot=True)
        
        # Real-time monitoring
        rospy.loginfo(f"[{self.node_name}] AprilTag stop timer started: {self.apriltag_stop_duration}s")
    
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
        
        # Transition back to lane following
        previous_state = self.state_msg.state
        self.state_msg.state = "LANE_FOLLOWING"
        self.state_msg.header.stamp = rospy.Time.now()
        self.state_transition_count += 1
        self.apriltag_stop_sequences += 1
        
        rospy.loginfo(f"[{self.node_name}] State transition: {previous_state} -> LANE_FOLLOWING")
        rospy.loginfo(f"[{self.node_name}] AprilTag stop sequences completed: {self.apriltag_stop_sequences}")
        
        # Reset stop state
        self.is_apriltag_stopping = False
        self.apriltag_stop_start_time = None
        
        self.publish()
        
        # Performance monitoring
        current_time = time.time()
        if current_time - self.last_performance_log > 30.0:  # Log every 30 seconds
            rospy.loginfo(f"[{self.node_name}] Performance summary:")
            rospy.loginfo(f"[{self.node_name}] State transitions: {self.state_transition_count}")
            rospy.loginfo(f"[{self.node_name}] AprilTag stops: {self.apriltag_stop_count}")
            rospy.loginfo(f"[{self.node_name}] Completed stop sequences: {self.apriltag_stop_sequences}")
            self.last_performance_log = current_time
        
        rospy.loginfo(f"[{self.node_name}] Resuming normal lane following operation")

    def on_shutdown(self):
        rospy.loginfo(f"[{self.node_name}] Shutting down.")


if __name__ == "__main__":
    # Initialize the node with rospy
    rospy.init_node("fsm_node", anonymous=False)

    # Create the NodeName object
    node = FSMNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()

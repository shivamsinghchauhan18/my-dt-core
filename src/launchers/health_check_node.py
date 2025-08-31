#!/usr/bin/env python3
"""
Health-check node: subscribes to a configurable list of topics and reports which ones received messages within a timeout.
Usage (rosrun):
  rosrun launchers health_check_node.py _veh:=blueduckie _topics:="['/blueduckie/camera_node/image/compressed','/blueduckie/line_detector_node/segment_list']" _timeout:=10

If _topics is omitted, a conservative default set is used based on _veh param.
"""
import rospy
import ast
from threading import Event


def parse_topics_param(param_name, default):
    try:
        raw = rospy.get_param(param_name)
    except KeyError:
        return default
    try:
        # param may be a YAML list or a string representation
        if isinstance(raw, list):
            return raw
        if isinstance(raw, str):
            return ast.literal_eval(raw)
    except Exception:
        pass
    return default


class TopicWatcher:
    def __init__(self, topics, timeout):
        self.topics = topics
        self.timeout = float(timeout)
        self.events = {t: Event() for t in topics}
        self.subs = []

    def _make_cb(self, topic):
        def cb(msg):
            if not self.events[topic].is_set():
                rospy.loginfo(f"[health_check] message received on: {topic}")
                self.events[topic].set()
        return cb

    def start(self):
        # Create subscribers with AnyMsg to accept any message type
        for t in self.topics:
            try:
                sub = rospy.Subscriber(t, rospy.AnyMsg, self._make_cb(t), queue_size=1)
                self.subs.append(sub)
            except Exception as e:
                rospy.logwarn(f"[health_check] Could not subscribe to {t}: {e}")
        rospy.loginfo(f"[health_check] Waiting up to {self.timeout}s for messages on {len(self.topics)} topics")
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if all(ev.is_set() for ev in self.events.values()):
                rospy.loginfo('[health_check] All topics received messages')
                return True
            if (rospy.Time.now().to_sec() - start) > self.timeout:
                break
            rate.sleep()
        # timed out
        missing = [t for t, ev in self.events.items() if not ev.is_set()]
        rospy.logwarn(f'[health_check] Timeout. Missing messages on {len(missing)} topics: {missing}')
        return False


if __name__ == '__main__':
    rospy.init_node('minimal_pipeline_health_check', anonymous=False)
    veh = rospy.get_param('~veh', rospy.get_param('~veh', 'blueduckie'))
    timeout = rospy.get_param('~timeout', 10)

    default_topics = [
        f'/{veh}/camera_node/image/compressed',
        f'/{veh}/camera_node/camera_info',
        f'/{veh}/line_detector_node/segment_list',
        f'/{veh}/lane_filter_node/lane_pose',
    ]

    topics = parse_topics_param('~topics', default_topics)

    watcher = TopicWatcher(topics, timeout)
    success = watcher.start()
    if success:
        rospy.loginfo('[health_check] SUCCESS: minimal pipeline topics are active')
        exit(0)
    else:
        rospy.logerr('[health_check] FAILURE: one or more topics did not receive messages')
        exit(2)

#!/usr/bin/env python3
"""
Enhanced AprilTag Detection Node
Extends basic AprilTag detection with precision stopping and distance estimation
"""

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String

class EnhancedAprilTagNode:
    def __init__(self):
        rospy.init_node('enhanced_apriltag_node', anonymous=False)
        
        # For now, subscribe to standard apriltag and republish enhanced
        self.sub_apriltag = rospy.Subscriber(
            '/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.apriltag_callback,
            queue_size=1
        )
        
        self.pub_enhanced = rospy.Publisher(
            '~enhanced_detections',
            AprilTagDetectionArray,
            queue_size=1
        )
        
        rospy.loginfo("Enhanced AprilTag node initialized")
    
    def apriltag_callback(self, msg):
        # For now, just republish - you can add enhancements here
        self.pub_enhanced.publish(msg)

if __name__ == '__main__':
    try:
        node = EnhancedAprilTagNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

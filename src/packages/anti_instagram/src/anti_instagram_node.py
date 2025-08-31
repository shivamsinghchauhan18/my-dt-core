#!/usr/bin/env python3

import rospy
from multiprocessing import Lock
from image_processing.anti_instagram import AntiInstagram
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import AntiInstagramThresholds
from duckietown_msgs.srv import SetBool, SetBoolResponse

from duckietown.dtros import DTROS, NodeType, TopicType


class AntiInstagramNode(DTROS):

    """

    Subscriber:
        ~uncorrected_image/compressed: The uncompressed image coming from the camera


    Publisher:
        ~

    """

    def __init__(self, node_name):

        super(AntiInstagramNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Read parameters
        self._interval = rospy.get_param("~interval")
        self._color_balance_percentage = rospy.get_param("~color_balance_scale")
        self._output_scale = rospy.get_param("~output_scale")

        # Initialize objects and data
        self.ai = AntiInstagram()
        self.bridge = CvBridge()
        self.image_msg = None
        self.mutex = Lock()

        # Active flag (controlled by ~switch service)
        self._active = True

        # Construct publisher
        self.pub = rospy.Publisher(
            "~thresholds", AntiInstagramThresholds, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

        # Construct subscriber
        self.uncorrected_image_subscriber = rospy.Subscriber(
            "~uncorrected_image/compressed",
            CompressedImage,
            self.store_image_msg,
            buff_size=10000000,
            queue_size=1,
        )

        # Initialize Timer
        rospy.Timer(rospy.Duration(self._interval), self.calculate_new_parameters)

        # Service to switch node on/off (used by FSM)
        self._srv_switch = rospy.Service("~switch", SetBool, self._cb_switch)

        # ---
        self.log("Initialized.")

    # Simple logging wrapper for compatibility with existing calls
    def log(self, msg, type: str = "info"):
        if type == "err" or type == "error":
            self.logerr(msg)
        elif type == "warn" or type == "warning":
            self.logwarn(msg)
        else:
            self.loginfo(msg)

    def store_image_msg(self, image_msg):
        with self.mutex:
            self.image_msg = image_msg

    def decode_image_msg(self):
        with self.mutex:
            try:
                image = self.bridge.compressed_imgmsg_to_cv2(self.image_msg, "bgr8")
            except ValueError as e:
                self.log(f"Anti_instagram cannot decode image: {e}")
                return None
        return image

    def calculate_new_parameters(self, event):
        if not self._active:
            return
        if self.image_msg is None:
            self.log("Waiting for first image!")
            return
        image = self.decode_image_msg()
        if image is None:
            return
        (lower_thresholds, higher_thresholds) = self.ai.calculate_color_balance_thresholds(
            image, self._output_scale, self._color_balance_percentage
        )

        # Publish parameters
        msg = AntiInstagramThresholds()
        msg.low = lower_thresholds
        msg.high = higher_thresholds
        self.pub.publish(msg)

    # --- Services ---
    def _cb_switch(self, req: SetBool):
        """Enable/disable this node.

        Args:
            req (duckietown_msgs/SetBool): req.data True to enable, False to disable

        Returns:
            duckietown_msgs/SetBoolResponse
        """
        self._active = bool(req.data)
        status = "enabled" if self._active else "disabled"
        self.log(f"Switch: {status}")
        return SetBoolResponse(success=True, message=f"AntiInstagramNode {status}")


if __name__ == "__main__":
    node = AntiInstagramNode(node_name="anti_instagram_node")
    rospy.spin()

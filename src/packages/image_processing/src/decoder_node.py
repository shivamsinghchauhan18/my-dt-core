#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage

# Try cv_bridge; fall back to pure OpenCV if unavailable or broken
HAVE_CV_BRIDGE = True
try:
    from cv_bridge import CvBridge
except Exception as e:
    HAVE_CV_BRIDGE = False
    rospy.logwarn("cv_bridge import failed: %s; will fall back to pure OpenCV path if enabled", e)

# Note: OpenCV (cv2) and numpy will be imported lazily only if the fallback path is used

from duckietown.dtros import DTROS, DTParam, NodeType, TopicType
from dt_class_utils import DTReminder


class DecoderNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name, node_type=NodeType.PERCEPTION)

        # parameters
        self.publish_freq = DTParam("~publish_freq", -1)
        # Force pure-OpenCV path (bypass cv_bridge). If false, use cv_bridge when available
        self.force_opencv = rospy.get_param("~force_opencv", False)

        # utility objects
        self.use_cv_bridge = (not self.force_opencv) and HAVE_CV_BRIDGE
        self.bridge = CvBridge() if self.use_cv_bridge else None
    # DTParam compatibility: in some distributions, .value is a callable
        self.reminder = DTReminder(frequency=self._get_publish_freq())

        # subscribers
        self.sub_img = rospy.Subscriber(
            "~image_in", CompressedImage, self.cb_image, queue_size=1, buff_size="10MB"
        )

        # publishers
        self.pub_img = rospy.Publisher(
            "~image_out",
            Image,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION,
            dt_healthy_freq=self._get_publish_freq(),
            dt_help="Raw image",
        )

    def cb_image(self, msg):
        # make sure this matters to somebody
        if not self.pub_img.anybody_listening():
            return
        # make sure the node is not switched off
        if not self.switch:
            return
        # make sure this is a good time to publish (always keep this as last check)
        if not self.reminder.is_time(frequency=self._get_publish_freq()):
            return
        # turn 'compressed image message' into 'raw image'
        if self.use_cv_bridge:
            with self.profiler("/cb/image/decode"):
                img = self.bridge.compressed_imgmsg_to_cv2(msg)
            # turn 'raw image' into 'raw image message'
            with self.profiler("/cb/image/serialize"):
                out_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        else:
            # Pure OpenCV fallback: decode JPEG/PNG and build Image message manually
            try:
                # Lazy import to avoid TLS/loader issues when not needed
                import numpy as np  # type: ignore
                import cv2  # type: ignore
            except Exception as e:
                rospy.logerr_throttle(5.0, "decoder_node: OpenCV fallback unavailable: %s", e)
                return
            with self.profiler("/cb/image/decode_opencv"):
                np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if img is None:
                    rospy.logwarn_throttle(5.0, "decoder_node: cv2.imdecode returned None")
                    return
            with self.profiler("/cb/image/serialize_opencv"):
                # Ensure BGR8 format
                if img.ndim == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                h, w = img.shape[:2]
                out_msg = Image()
                out_msg.height = h
                out_msg.width = w
                out_msg.encoding = "bgr8"
                out_msg.is_bigendian = 0
                out_msg.step = w * 3
                out_msg.data = img.tobytes()
        # maintain original header
        out_msg.header = msg.header
        # publish image
        self.pub_img.publish(out_msg)

    # --- Helpers ---------------------------------------------------------
    def _value_of(self, p, default=None):
        """Safely extract the numeric value from a DTParam or return default.

        DTParam.value can be a property or a callable depending on dtros version.
        """
        if p is None:
            return default
        v = getattr(p, "value", None)
        try:
            if callable(v):
                return v()
            if v is not None:
                return v
        except Exception:
            pass
        return default

    def _get_publish_freq(self):
        # default -1 means "no frequency constraint" for DTReminder/health checks
        return self._value_of(self.publish_freq, -1)


if __name__ == "__main__":
    node = DecoderNode("decoder_node")
    rospy.spin()

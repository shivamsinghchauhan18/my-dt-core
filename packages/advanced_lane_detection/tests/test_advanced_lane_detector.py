#!/usr/bin/env python3
import unittest
import numpy as np

# Patch ROS and message imports for unit testing
import sys
from unittest.mock import Mock
sys.modules['rospy'] = Mock()
sys.modules['cv2'] = __import__('cv2')
sys.modules['cv_bridge'] = __import__('cv_bridge')
sys.modules['sensor_msgs'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['duckietown_enhanced_msgs'] = Mock()
sys.modules['duckietown_enhanced_msgs.msg'] = Mock()

from types import SimpleNamespace

# Import the node
import importlib.util
spec = importlib.util.spec_from_file_location(
    'advanced_lane_detector',
    __file__.replace('tests/test_advanced_lane_detector.py', 'src/advanced_lane_detector.py')
)
ald = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ald)

class FakeImage:
    def __init__(self):
        self.header = SimpleNamespace()

class TestAdvancedLaneDetector(unittest.TestCase):
    def test_detects_centered_lane(self):
        node = ald.AdvancedLaneDetector()
        # Create a synthetic image: black background with a vertical white strip in center of ROI
        h, w = 240, 320
        img = np.zeros((h, w, 3), dtype=np.uint8)
        cv2 = __import__('cv2')
        cv2.rectangle(img, (w//2 - 5, int(0.6*h)), (w//2 + 5, h-1), (255,255,255), -1)

        # Mock CvBridge conversion
        ald.CvBridge.imgmsg_to_cv2 = lambda self, msg, desired_encoding=None: img

        # Capture published message
        published = []
        node.pub_pose = SimpleNamespace(publish=lambda m: published.append(m))

        node.cb_image(FakeImage())
        self.assertTrue(published, 'Should publish a pose')
        m = published[0]
        # Expect near zero lateral offset and small |phi|
        self.assertAlmostEqual(m.d, 0.0, delta=0.05)
        self.assertLess(abs(m.phi), 0.5)

if __name__ == '__main__':
    unittest.main()

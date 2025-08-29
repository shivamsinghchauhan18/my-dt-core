#!/usr/bin/env python3
import unittest
from unittest.mock import Mock

# Patch ROS and dependent msgs
import sys
sys.modules['rospy'] = Mock()
sys.modules['duckietown_msgs'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['duckietown_enhanced_msgs'] = __import__('types')
sys.modules['duckietown_enhanced_msgs'].msg = __import__('types')
sys.modules['geometry_msgs'] = Mock()
sys.modules['geometry_msgs.msg'] = Mock()

import importlib.util
spec = importlib.util.spec_from_file_location(
    'enhanced_apriltag_node',
    __file__.replace('tests/test_enhanced_apriltag_node.py', 'src/enhanced_apriltag_node.py')
)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

class MockCenter:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class MockDetection:
    def __init__(self, tag_id=1, size=40.0):
        self.tag_id = tag_id
        self.size = size
        self.center = MockCenter(160.0, 120.0)

class MockArray:
    def __init__(self, detections):
        self.detections = detections
        self.header = None

class TestEnhancedAprilTagNode(unittest.TestCase):
    def test_distance_estimation(self):
        node = mod.EnhancedAprilTagNode()
        node.fx = 500.0
        node.tag_size_m = 0.1

        d = node._estimate_distance(MockDetection(size=50.0))
        # Expected approx: (0.1 * 500) / 50 = 1.0 m
        self.assertAlmostEqual(d, 1.0, places=2)

    def test_publishes_object_array(self):
        node = mod.EnhancedAprilTagNode()
        published = []
        node.pub_enhanced = type('P', (), {'publish': lambda self, m: published.append(m)})()
        node.pub_passthrough = type('P', (), {'publish': lambda self, m: None})()

        msg = MockArray([MockDetection(tag_id=5, size=40.0)])
        node.apriltag_callback(msg)
        self.assertTrue(published, 'Should publish enhanced objects')

if __name__ == '__main__':
    unittest.main()

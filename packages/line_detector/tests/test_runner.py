#!/usr/bin/env python3

"""
Test runner for line detector tests without ROS dependencies.
This file mocks the necessary ROS components to allow unit testing.
"""

import sys
import os
import unittest
from unittest.mock import Mock, MagicMock

# Mock ROS modules before importing the actual code
sys.modules['rospy'] = Mock()
sys.modules['cv_bridge'] = Mock()
sys.modules['sensor_msgs'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['duckietown_msgs'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['line_detector'] = Mock()
sys.modules['image_processing'] = Mock()
sys.modules['image_processing.anti_instagram'] = Mock()
sys.modules['duckietown.dtros'] = Mock()

# Mock rospy functions
rospy = sys.modules['rospy']
rospy.logdebug = Mock()
rospy.loginfo = Mock()
rospy.logerr = Mock()
rospy.Time = Mock()
rospy.Time.now = Mock(return_value=123456789)

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Import test classes
from test_adaptive_threshold_detector import TestAdaptiveThresholdDetector
from test_temporal_consistency_filter import TestTemporalConsistencyFilter

if __name__ == '__main__':
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestAdaptiveThresholdDetector))
    suite.addTests(loader.loadTestsFromTestCase(TestTemporalConsistencyFilter))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)
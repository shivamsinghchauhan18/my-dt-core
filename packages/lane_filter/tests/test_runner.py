#!/usr/bin/env python3

"""
Test runner for lane filter tests without ROS dependencies.
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
sys.modules['lane_filter'] = Mock()
sys.modules['duckietown.dtros'] = Mock()
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Mock rospy functions
rospy = sys.modules['rospy']
rospy.logdebug = Mock()
rospy.loginfo = Mock()
rospy.logerr = Mock()
rospy.logwarn = Mock()
rospy.Time = Mock()
rospy.Time.now = Mock(return_value=123456789)

# Mock scipy modules (in case they're not available)
try:
    import scipy.optimize
    import scipy.interpolate
except ImportError:
    sys.modules['scipy'] = Mock()
    sys.modules['scipy.optimize'] = Mock()
    sys.modules['scipy.interpolate'] = Mock()
    scipy_mock = sys.modules['scipy']
    scipy_mock.optimize = Mock()
    scipy_mock.interpolate = Mock()
    scipy_mock.optimize.curve_fit = Mock()
    scipy_mock.interpolate.UnivariateSpline = Mock()

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Import test classes
from test_polynomial_curve_fitter import TestPolynomialCurveFitter

if __name__ == '__main__':
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestPolynomialCurveFitter))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)
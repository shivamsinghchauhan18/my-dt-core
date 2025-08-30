#!/usr/bin/env python3

"""
Test runner for enhanced lane detection integration tests.
This file runs comprehensive integration tests for the complete pipeline.
"""

import sys
import os
import unittest
from unittest.mock import Mock

# Mock ROS modules before importing the actual code
sys.modules['rospy'] = Mock()
sys.modules['cv_bridge'] = Mock()
sys.modules['sensor_msgs'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['duckietown_msgs'] = Mock()
sys.modules['duckietown_msgs.msg'] = Mock()
sys.modules['line_detector'] = Mock()
sys.modules['lane_filter'] = Mock()
sys.modules['image_processing'] = Mock()
sys.modules['image_processing.anti_instagram'] = Mock()
sys.modules['duckietown.dtros'] = Mock()
sys.modules['std_msgs'] = Mock()
sys.modules['std_msgs.msg'] = Mock()

# Mock scipy modules (in case they're not available)
try:
    import scipy.optimize
    import scipy.interpolate
except ImportError:
    sys.modules['scipy'] = Mock()
    sys.modules['scipy.optimize'] = Mock()
    sys.modules['scipy.interpolate'] = Mock()

# Import test classes
from test_enhanced_lane_detection_integration import TestEnhancedLaneDetectionIntegration

if __name__ == '__main__':
    print("Running Enhanced Lane Detection Integration Tests")
    print("=" * 60)
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add integration test cases
    suite.addTests(loader.loadTestsFromTestCase(TestEnhancedLaneDetectionIntegration))
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "=" * 60)
    print("Integration Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print("\nFailures:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback.split('AssertionError:')[-1].strip()}")
    
    if result.errors:
        print("\nErrors:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback.split('Exception:')[-1].strip()}")
    
    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)
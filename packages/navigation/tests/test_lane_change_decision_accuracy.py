#!/usr/bin/env python3

import unittest
import rospy
import time
import numpy as np
from unittest.mock import Mock, patch, MagicMock

# Import the modules to test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from lane_change_decision_engine import (
    LaneChangeDecisionEngine, LaneChangeDecision, LaneChangeReason,
    LaneState, GapAnalysis, DecisionCriteria, DecisionMatrix
)
from duckietown_msgs.msg import ObjectDetection
from geometry_msgs.msg import Point, Vector3


class TestLaneChangeDecisionAccuracy(unittest.TestCase):
    """
    Test suite for lane change decision engine accuracy.
    
    Tests the multi-criteria evaluation system, gap analysis, and decision
    making accuracy under various scenarios.
    """
    
    def setUp(self):
        """Set up test fixtures"""
        # Initialize ROS node for testing (mock if needed)
        if not rospy.get_node_uri():
            rospy.init_node('test_lane_change_decision', anonymous=True)
        
        # Create decision engine instance
        self.decision_engine = LaneChangeDecisionEngine()
        
        # Create test data
        self.create_test_data()
        
        print(f"[TestLaneChangeDecisionAccuracy] Test setup completed")
    
    def create_test_data(self):
        """Create test data for various scenarios"""
        
        # Test vehicle state
        self.test_vehicle_state = {
            'position': {'x': 0.0, 'y': 0.0},
            'velocity': {'x': 1.0, 'y': 0.0},
            'heading': 0.0,
            'timestamp': time.time()
        }
        
        # Test current lane state
        self.test_current_lane = LaneState(
            lane_id="current",
            is_current=True,
            is_available=True,
            width=0.6,
            center_line_offset=0.0,
            obstacles=[],
            gap_length=10.0,
            gap_start_distance=0.0,
            safety_score=0.8,
            timestamp=time.time()
        )
        
        # Test adjacent lanes
        self.test_adjacent_lanes = {
            "left": LaneState(
                lane_id="left",
                is_current=False,
                is_available=True,
                width=0.6,
                center_line_offset=-0.6,
                obstacles=[],
                gap_length=8.0,
                gap_start_distance=1.0,
                safety_score=0.9,
                timestamp=time.time()
            ),
            "right": LaneState(
                lane_id="right",
                is_current=False,
                is_available=True,
                width=0.6,
                center_line_offset=0.6,
                obstacles=[],
                gap_length=6.0,
                gap_start_distance=1.0,
                safety_score=0.7,
                timestamp=time.time()
            )
        }
        
        # Test obstacles
        self.test_obstacles = []
        
        print(f"[TestLaneChangeDecisionAccuracy] Test data created")
    
    def create_test_obstacle(self, class_name: str, distance: float, confidence: float = 0.8) -> ObjectDetection:
        """Create a test obstacle"""
        obstacle = ObjectDetection()
        obstacle.class_name = class_name
        obstacle.distance = distance
        obstacle.confidence = confidence
        return obstacle
    
    def test_decision_engine_initialization(self):
        """Test that the decision engine initializes correctly"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing decision engine initialization")
        
        # Check that engine is initialized
        self.assertIsNotNone(self.decision_engine)
        self.assertIsInstance(self.decision_engine.criteria, DecisionCriteria)
        self.assertIsInstance(self.decision_engine.decision_matrix, DecisionMatrix)
        
        # Check default criteria values
        self.assertGreater(self.decision_engine.criteria.minimum_gap_length, 0)
        self.assertGreater(self.decision_engine.criteria.minimum_safety_margin, 0)
        self.assertGreater(self.decision_engine.criteria.maximum_lateral_acceleration, 0)
        
        # Check decision matrix weights sum to 1.0
        total_weight = (
            self.decision_engine.decision_matrix.safety_weight +
            self.decision_engine.decision_matrix.efficiency_weight +
            self.decision_engine.decision_matrix.comfort_weight +
            self.decision_engine.decision_matrix.urgency_weight +
            self.decision_engine.decision_matrix.feasibility_weight
        )
        self.assertAlmostEqual(total_weight, 1.0, places=2)
        
        print(f"[TestLaneChangeDecisionAccuracy] Decision engine initialization test passed")
    
    def test_gap_analysis_clear_lane(self):
        """Test gap analysis with clear adjacent lane"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing gap analysis with clear lane")
        
        # Test with clear left lane
        clear_lane = self.test_adjacent_lanes["left"]
        clear_obstacles = []
        
        gap_analysis = self.decision_engine.perform_gap_analysis(
            clear_lane, self.test_vehicle_state, clear_obstacles
        )
        
        # Verify gap analysis results
        self.assertIsInstance(gap_analysis, GapAnalysis)
        self.assertTrue(gap_analysis.gap_available)
        self.assertGreater(gap_analysis.gap_length, self.decision_engine.criteria.minimum_gap_length)
        self.assertGreater(gap_analysis.confidence, 0.5)
        self.assertGreaterEqual(gap_analysis.safety_margin_front, 0.0)
        self.assertGreaterEqual(gap_analysis.safety_margin_rear, 0.0)
        
        print(f"[TestLaneChangeDecisionAccuracy] Gap analysis results:")
        print(f"  Gap available: {gap_analysis.gap_available}")
        print(f"  Gap length: {gap_analysis.gap_length:.2f}m")
        print(f"  Confidence: {gap_analysis.confidence:.3f}")
        print(f"  Safety margins: front={gap_analysis.safety_margin_front:.2f}m, rear={gap_analysis.safety_margin_rear:.2f}m")
        
        print(f"[TestLaneChangeDecisionAccuracy] Clear lane gap analysis test passed")
    
    def test_gap_analysis_blocked_lane(self):
        """Test gap analysis with blocked adjacent lane"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing gap analysis with blocked lane")
        
        # Create obstacles in the lane
        blocked_obstacles = [
            self.create_test_obstacle("duckiebot", 2.0, 0.9),
            self.create_test_obstacle("cone", 1.5, 0.8)
        ]
        
        blocked_lane = LaneState(
            lane_id="blocked",
            is_current=False,
            is_available=True,
            width=0.6,
            center_line_offset=-0.6,
            obstacles=blocked_obstacles,
            gap_length=2.0,  # Short gap
            gap_start_distance=0.5,
            safety_score=0.3,
            timestamp=time.time()
        )
        
        gap_analysis = self.decision_engine.perform_gap_analysis(
            blocked_lane, self.test_vehicle_state, blocked_obstacles
        )
        
        # Verify gap analysis results for blocked lane
        self.assertIsInstance(gap_analysis, GapAnalysis)
        # Gap might not be available due to obstacles
        self.assertLessEqual(gap_analysis.confidence, 0.8)  # Should have lower confidence
        
        print(f"[TestLaneChangeDecisionAccuracy] Blocked lane gap analysis results:")
        print(f"  Gap available: {gap_analysis.gap_available}")
        print(f"  Gap length: {gap_analysis.gap_length:.2f}m")
        print(f"  Confidence: {gap_analysis.confidence:.3f}")
        
        print(f"[TestLaneChangeDecisionAccuracy] Blocked lane gap analysis test passed")
    
    def test_decision_score_calculation(self):
        """Test decision score calculation with various scenarios"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing decision score calculation")
        
        # Test with clear lane and good gap
        clear_gap = GapAnalysis(
            gap_available=True,
            gap_length=8.0,
            gap_start_distance=1.0,
            gap_end_distance=9.0,
            leading_vehicle_distance=10.0,
            following_vehicle_distance=2.0,
            safety_margin_front=2.0,
            safety_margin_rear=1.0,
            confidence=0.9
        )
        
        score = self.decision_engine.calculate_decision_score(
            self.test_current_lane,
            self.test_adjacent_lanes["left"],
            clear_gap,
            self.test_vehicle_state,
            []
        )
        
        # Score should be reasonable for good conditions
        self.assertGreaterEqual(score, 0.0)
        self.assertLessEqual(score, 1.0)
        self.assertGreater(score, 0.3)  # Should be decent score for good conditions
        
        print(f"[TestLaneChangeDecisionAccuracy] Decision score for clear lane: {score:.3f}")
        
        # Test with poor gap
        poor_gap = GapAnalysis(
            gap_available=False,
            gap_length=2.0,
            gap_start_distance=0.5,
            gap_end_distance=2.5,
            leading_vehicle_distance=3.0,
            following_vehicle_distance=0.5,
            safety_margin_front=0.2,
            safety_margin_rear=0.1,
            confidence=0.2
        )
        
        poor_score = self.decision_engine.calculate_decision_score(
            self.test_current_lane,
            self.test_adjacent_lanes["right"],
            poor_gap,
            self.test_vehicle_state,
            []
        )
        
        # Poor conditions should result in lower score
        self.assertLess(poor_score, score)
        self.assertLess(poor_score, 0.5)
        
        print(f"[TestLaneChangeDecisionAccuracy] Decision score for poor gap: {poor_score:.3f}")
        
        print(f"[TestLaneChangeDecisionAccuracy] Decision score calculation test passed")
    
    def test_lane_change_decision_no_obstacles(self):
        """Test lane change decision with no obstacles"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing lane change decision with no obstacles")
        
        decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
            self.test_current_lane,
            self.test_adjacent_lanes,
            self.test_vehicle_state,
            []
        )
        
        # Verify decision results
        self.assertIsInstance(decision, LaneChangeDecision)
        self.assertIsInstance(reason, LaneChangeReason)
        self.assertGreaterEqual(confidence, 0.0)
        self.assertLessEqual(confidence, 1.0)
        
        # With no obstacles, should likely decide no change needed
        # (unless there's a clear efficiency benefit)
        print(f"[TestLaneChangeDecisionAccuracy] Decision results (no obstacles):")
        print(f"  Decision: {decision.value}")
        print(f"  Reason: {reason.value}")
        print(f"  Confidence: {confidence:.3f}")
        
        print(f"[TestLaneChangeDecisionAccuracy] No obstacles decision test passed")
    
    def test_lane_change_decision_with_blocking_obstacle(self):
        """Test lane change decision with blocking obstacle in current lane"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing lane change decision with blocking obstacle")
        
        # Add blocking obstacle to current lane
        blocking_obstacles = [
            self.create_test_obstacle("duckiebot", 1.5, 0.9)  # Close obstacle
        ]
        
        # Update current lane with obstacles
        current_lane_with_obstacle = LaneState(
            lane_id="current",
            is_current=True,
            is_available=True,
            width=0.6,
            center_line_offset=0.0,
            obstacles=blocking_obstacles,
            gap_length=1.0,  # Short gap due to obstacle
            gap_start_distance=0.0,
            safety_score=0.3,  # Lower safety score
            timestamp=time.time()
        )
        
        decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
            current_lane_with_obstacle,
            self.test_adjacent_lanes,
            self.test_vehicle_state,
            blocking_obstacles
        )
        
        # With blocking obstacle, should consider lane change
        print(f"[TestLaneChangeDecisionAccuracy] Decision results (blocking obstacle):")
        print(f"  Decision: {decision.value}")
        print(f"  Reason: {reason.value}")
        print(f"  Confidence: {confidence:.3f}")
        
        # Should either decide to change lanes or have obstacle avoidance reason
        if decision != LaneChangeDecision.NO_CHANGE:
            self.assertIn(reason, [LaneChangeReason.OBSTACLE_AVOIDANCE, LaneChangeReason.EMERGENCY_MANEUVER])
        
        print(f"[TestLaneChangeDecisionAccuracy] Blocking obstacle decision test passed")
    
    def test_decision_confidence_thresholds(self):
        """Test that decision confidence thresholds are respected"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing decision confidence thresholds")
        
        # Test with very low confidence scenario
        low_confidence_lane = LaneState(
            lane_id="low_confidence",
            is_current=False,
            is_available=True,
            width=0.6,
            center_line_offset=-0.6,
            obstacles=[self.create_test_obstacle("barrier", 1.0, 0.9)],  # Close barrier
            gap_length=1.5,  # Very short gap
            gap_start_distance=0.2,
            safety_score=0.1,  # Very low safety
            timestamp=time.time()
        )
        
        low_confidence_lanes = {"left": low_confidence_lane}
        
        decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
            self.test_current_lane,
            low_confidence_lanes,
            self.test_vehicle_state,
            []
        )
        
        # Should not recommend lane change with very low confidence
        if confidence < self.decision_engine.criteria.decision_confidence_threshold:
            self.assertEqual(decision, LaneChangeDecision.NO_CHANGE)
        
        print(f"[TestLaneChangeDecisionAccuracy] Low confidence scenario:")
        print(f"  Decision: {decision.value}")
        print(f"  Confidence: {confidence:.3f}")
        print(f"  Threshold: {self.decision_engine.criteria.decision_confidence_threshold}")
        
        print(f"[TestLaneChangeDecisionAccuracy] Confidence threshold test passed")
    
    def test_performance_metrics_tracking(self):
        """Test that performance metrics are tracked correctly"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing performance metrics tracking")
        
        # Reset metrics
        self.decision_engine.reset_metrics()
        initial_metrics = self.decision_engine.get_performance_metrics()
        
        # Perform several evaluations
        for i in range(5):
            decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
                self.test_current_lane,
                self.test_adjacent_lanes,
                self.test_vehicle_state,
                []
            )
        
        # Check that metrics were updated
        final_metrics = self.decision_engine.get_performance_metrics()
        
        self.assertGreater(final_metrics['total_evaluations'], initial_metrics['total_evaluations'])
        self.assertGreaterEqual(final_metrics['gap_analyses_performed'], final_metrics['total_evaluations'] * 2)  # At least 2 lanes analyzed per evaluation
        self.assertGreater(final_metrics['average_evaluation_time_ms'], 0)
        
        print(f"[TestLaneChangeDecisionAccuracy] Performance metrics:")
        print(f"  Total evaluations: {final_metrics['total_evaluations']}")
        print(f"  Gap analyses: {final_metrics['gap_analyses_performed']}")
        print(f"  Average evaluation time: {final_metrics['average_evaluation_time_ms']:.2f}ms")
        print(f"  Success rate: {final_metrics['success_rate']:.1f}%")
        
        print(f"[TestLaneChangeDecisionAccuracy] Performance metrics tracking test passed")
    
    def test_decision_history_tracking(self):
        """Test that decision history is tracked correctly"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing decision history tracking")
        
        # Reset metrics and history
        self.decision_engine.reset_metrics()
        
        # Perform several evaluations
        decisions_made = []
        for i in range(3):
            decision, reason, confidence = self.decision_engine.evaluate_lane_change_opportunity(
                self.test_current_lane,
                self.test_adjacent_lanes,
                self.test_vehicle_state,
                []
            )
            decisions_made.append((decision, reason, confidence))
        
        # Check decision history
        history = self.decision_engine.get_decision_history()
        
        self.assertGreater(len(history), 0)
        self.assertLessEqual(len(history), 3)  # Should have up to 3 decisions
        
        # Verify history format
        for timestamp, decision_str, confidence in history:
            self.assertIsInstance(timestamp, float)
            self.assertIsInstance(decision_str, str)
            self.assertIsInstance(confidence, float)
            self.assertGreaterEqual(confidence, 0.0)
            self.assertLessEqual(confidence, 1.0)
        
        print(f"[TestLaneChangeDecisionAccuracy] Decision history:")
        for i, (timestamp, decision_str, confidence) in enumerate(history):
            print(f"  Decision {i+1}: {decision_str} (confidence: {confidence:.3f})")
        
        print(f"[TestLaneChangeDecisionAccuracy] Decision history tracking test passed")
    
    def test_safety_score_calculations(self):
        """Test safety score calculations for different scenarios"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing safety score calculations")
        
        # Test safety score with good gap
        good_gap = GapAnalysis(
            gap_available=True,
            gap_length=10.0,
            gap_start_distance=2.0,
            gap_end_distance=12.0,
            leading_vehicle_distance=15.0,
            following_vehicle_distance=3.0,
            safety_margin_front=3.0,
            safety_margin_rear=2.0,
            confidence=0.95
        )
        
        safety_score_good = self.decision_engine.calculate_safety_score(
            good_gap, self.test_current_lane, self.test_adjacent_lanes["left"], []
        )
        
        # Test safety score with poor gap
        poor_gap = GapAnalysis(
            gap_available=False,
            gap_length=2.0,
            gap_start_distance=0.5,
            gap_end_distance=2.5,
            leading_vehicle_distance=3.0,
            following_vehicle_distance=0.5,
            safety_margin_front=0.1,
            safety_margin_rear=0.0,
            confidence=0.2
        )
        
        safety_score_poor = self.decision_engine.calculate_safety_score(
            poor_gap, self.test_current_lane, self.test_adjacent_lanes["right"], []
        )
        
        # Good gap should have higher safety score
        self.assertGreater(safety_score_good, safety_score_poor)
        self.assertGreaterEqual(safety_score_good, 0.0)
        self.assertLessEqual(safety_score_good, 1.0)
        self.assertGreaterEqual(safety_score_poor, 0.0)
        self.assertLessEqual(safety_score_poor, 1.0)
        
        print(f"[TestLaneChangeDecisionAccuracy] Safety scores:")
        print(f"  Good gap safety score: {safety_score_good:.3f}")
        print(f"  Poor gap safety score: {safety_score_poor:.3f}")
        
        print(f"[TestLaneChangeDecisionAccuracy] Safety score calculation test passed")
    
    def test_lateral_acceleration_estimation(self):
        """Test lateral acceleration estimation for lane changes"""
        print(f"\n[TestLaneChangeDecisionAccuracy] Testing lateral acceleration estimation")
        
        # Test with different velocities and gap lengths
        test_cases = [
            (1.0, 8.0),   # 1 m/s, 8m gap
            (2.0, 6.0),   # 2 m/s, 6m gap
            (0.5, 10.0),  # 0.5 m/s, 10m gap
        ]
        
        for velocity, gap_length in test_cases:
            lateral_accel = self.decision_engine.estimate_lateral_acceleration(velocity, gap_length)
            
            # Should be positive and reasonable
            self.assertGreater(lateral_accel, 0.0)
            self.assertLess(lateral_accel, 10.0)  # Should be reasonable value
            
            print(f"[TestLaneChangeDecisionAccuracy] Lateral acceleration:")
            print(f"  Velocity: {velocity:.1f} m/s, Gap: {gap_length:.1f}m -> Accel: {lateral_accel:.2f} m/s²")
        
        # Test edge cases
        infinite_accel = self.decision_engine.estimate_lateral_acceleration(0.0, 5.0)  # Zero velocity
        self.assertEqual(infinite_accel, float('inf'))
        
        infinite_accel2 = self.decision_engine.estimate_lateral_acceleration(1.0, 0.0)  # Zero gap
        self.assertEqual(infinite_accel2, float('inf'))
        
        print(f"[TestLaneChangeDecisionAccuracy] Lateral acceleration estimation test passed")


def run_lane_change_decision_tests():
    """Run all lane change decision accuracy tests"""
    print(f"\n{'='*60}")
    print(f"RUNNING LANE CHANGE DECISION ACCURACY TESTS")
    print(f"{'='*60}")
    
    # Create test suite
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestLaneChangeDecisionAccuracy)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout, buffer=False)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"LANE CHANGE DECISION ACCURACY TEST SUMMARY")
    print(f"{'='*60}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\nFailures:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback}")
    
    if result.errors:
        print(f"\nErrors:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    # Run the tests
    success = run_lane_change_decision_tests()
    
    if success:
        print(f"\n✅ All lane change decision accuracy tests passed!")
        exit(0)
    else:
        print(f"\n❌ Some lane change decision accuracy tests failed!")
        exit(1)
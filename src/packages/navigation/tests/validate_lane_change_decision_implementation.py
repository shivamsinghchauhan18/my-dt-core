#!/usr/bin/env python3

"""
Validation script for lane change decision engine implementation.
This script validates the implementation without requiring ROS environment.
"""

import sys
import os
import time
import traceback
from typing import List, Dict, Any

# Add the src directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

# Mock rospy for testing
class MockRospy:
    def loginfo(self, msg): print(f"[INFO] {msg}")
    def logwarn(self, msg): print(f"[WARN] {msg}")
    def logerr(self, msg): print(f"[ERROR] {msg}")
    def logdebug(self, msg): print(f"[DEBUG] {msg}")

sys.modules['rospy'] = MockRospy()

# Mock duckietown_msgs
class MockObjectDetection:
    def __init__(self):
        self.class_name = ""
        self.distance = 0.0
        self.confidence = 0.0

class MockDuckietownMsgs:
    class msg:
        ObjectDetection = MockObjectDetection

sys.modules['duckietown_msgs'] = MockDuckietownMsgs()
sys.modules['duckietown_msgs.msg'] = MockDuckietownMsgs.msg()

# Mock geometry_msgs
class MockPoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class MockVector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class MockGeometryMsgs:
    class msg:
        Point = MockPoint
        Vector3 = MockVector3

sys.modules['geometry_msgs'] = MockGeometryMsgs()
sys.modules['geometry_msgs.msg'] = MockGeometryMsgs.msg()

# Now import the actual implementation
try:
    from lane_change_decision_engine import (
        LaneChangeDecisionEngine, LaneChangeDecision, LaneChangeReason,
        LaneState, GapAnalysis, DecisionCriteria, DecisionMatrix
    )
    print("✅ Successfully imported lane change decision engine")
except Exception as e:
    print(f"❌ Failed to import lane change decision engine: {e}")
    traceback.print_exc()
    sys.exit(1)


def create_test_obstacle(class_name: str, distance: float, confidence: float = 0.8):
    """Create a test obstacle"""
    obstacle = MockObjectDetection()
    obstacle.class_name = class_name
    obstacle.distance = distance
    obstacle.confidence = confidence
    return obstacle


def create_test_lane_state(lane_id: str, is_current: bool = False, obstacles: List = None) -> LaneState:
    """Create a test lane state"""
    if obstacles is None:
        obstacles = []
    
    return LaneState(
        lane_id=lane_id,
        is_current=is_current,
        is_available=True,
        width=0.6,
        center_line_offset=0.0 if is_current else (-0.6 if "left" in lane_id else 0.6),
        obstacles=obstacles,
        gap_length=8.0,
        gap_start_distance=1.0,
        safety_score=0.8,
        timestamp=time.time()
    )


def create_test_vehicle_state() -> Dict[str, Any]:
    """Create test vehicle state"""
    return {
        'position': {'x': 0.0, 'y': 0.0},
        'velocity': {'x': 1.0, 'y': 0.0},
        'heading': 0.0,
        'timestamp': time.time()
    }


def validate_initialization():
    """Validate decision engine initialization"""
    print(f"\n{'='*50}")
    print(f"VALIDATING DECISION ENGINE INITIALIZATION")
    print(f"{'='*50}")
    
    try:
        engine = LaneChangeDecisionEngine()
        
        # Check basic attributes
        assert hasattr(engine, 'criteria'), "Missing criteria attribute"
        assert hasattr(engine, 'decision_matrix'), "Missing decision_matrix attribute"
        assert hasattr(engine, 'performance_metrics'), "Missing performance_metrics attribute"
        
        # Check criteria values
        assert engine.criteria.minimum_gap_length > 0, "Invalid minimum_gap_length"
        assert engine.criteria.minimum_safety_margin > 0, "Invalid minimum_safety_margin"
        assert engine.criteria.maximum_lateral_acceleration > 0, "Invalid maximum_lateral_acceleration"
        assert 0 <= engine.criteria.decision_confidence_threshold <= 1, "Invalid decision_confidence_threshold"
        
        # Check decision matrix weights
        total_weight = (
            engine.decision_matrix.safety_weight +
            engine.decision_matrix.efficiency_weight +
            engine.decision_matrix.comfort_weight +
            engine.decision_matrix.urgency_weight +
            engine.decision_matrix.feasibility_weight
        )
        assert abs(total_weight - 1.0) < 0.01, f"Decision matrix weights don't sum to 1.0: {total_weight}"
        
        print("✅ Decision engine initialization validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Decision engine initialization validation failed: {e}")
        traceback.print_exc()
        return False


def validate_gap_analysis():
    """Validate gap analysis functionality"""
    print(f"\n{'='*50}")
    print(f"VALIDATING GAP ANALYSIS")
    print(f"{'='*50}")
    
    try:
        engine = LaneChangeDecisionEngine()
        
        # Test with clear lane
        clear_lane = create_test_lane_state("test_lane", obstacles=[])
        vehicle_state = create_test_vehicle_state()
        
        gap_analysis = engine.perform_gap_analysis(clear_lane, vehicle_state, [])
        
        # Validate gap analysis structure
        assert isinstance(gap_analysis, GapAnalysis), "Invalid gap analysis type"
        assert hasattr(gap_analysis, 'gap_available'), "Missing gap_available"
        assert hasattr(gap_analysis, 'gap_length'), "Missing gap_length"
        assert hasattr(gap_analysis, 'confidence'), "Missing confidence"
        assert hasattr(gap_analysis, 'safety_margin_front'), "Missing safety_margin_front"
        assert hasattr(gap_analysis, 'safety_margin_rear'), "Missing safety_margin_rear"
        
        # Validate gap analysis values
        assert isinstance(gap_analysis.gap_available, bool), "gap_available should be boolean"
        assert gap_analysis.gap_length >= 0, "gap_length should be non-negative"
        assert 0 <= gap_analysis.confidence <= 1, "confidence should be between 0 and 1"
        assert gap_analysis.safety_margin_front >= 0, "safety_margin_front should be non-negative"
        assert gap_analysis.safety_margin_rear >= 0, "safety_margin_rear should be non-negative"
        
        print(f"✅ Gap analysis results:")
        print(f"   Gap available: {gap_analysis.gap_available}")
        print(f"   Gap length: {gap_analysis.gap_length:.2f}m")
        print(f"   Confidence: {gap_analysis.confidence:.3f}")
        print(f"   Safety margins: front={gap_analysis.safety_margin_front:.2f}m, rear={gap_analysis.safety_margin_rear:.2f}m")
        
        # Test with obstacles
        obstacles = [create_test_obstacle("duckiebot", 2.0, 0.9)]
        blocked_lane = create_test_lane_state("blocked_lane", obstacles=obstacles)
        
        gap_analysis_blocked = engine.perform_gap_analysis(blocked_lane, vehicle_state, obstacles)
        
        print(f"✅ Gap analysis with obstacles:")
        print(f"   Gap available: {gap_analysis_blocked.gap_available}")
        print(f"   Gap length: {gap_analysis_blocked.gap_length:.2f}m")
        print(f"   Confidence: {gap_analysis_blocked.confidence:.3f}")
        
        print("✅ Gap analysis validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Gap analysis validation failed: {e}")
        traceback.print_exc()
        return False


def validate_decision_scoring():
    """Validate decision scoring functionality"""
    print(f"\n{'='*50}")
    print(f"VALIDATING DECISION SCORING")
    print(f"{'='*50}")
    
    try:
        engine = LaneChangeDecisionEngine()
        
        # Create test data
        current_lane = create_test_lane_state("current", is_current=True)
        target_lane = create_test_lane_state("target")
        vehicle_state = create_test_vehicle_state()
        
        # Create good gap analysis
        good_gap = GapAnalysis(
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
        
        # Calculate decision score
        score = engine.calculate_decision_score(
            current_lane, target_lane, good_gap, vehicle_state, []
        )
        
        # Validate score
        assert isinstance(score, float), "Score should be float"
        assert 0 <= score <= 1, f"Score should be between 0 and 1, got {score}"
        
        print(f"✅ Decision score for good conditions: {score:.3f}")
        
        # Test individual score components
        safety_score = engine.calculate_safety_score(good_gap, current_lane, target_lane, [])
        efficiency_score = engine.calculate_efficiency_score(current_lane, target_lane, vehicle_state)
        comfort_score = engine.calculate_comfort_score(good_gap, vehicle_state)
        urgency_score = engine.calculate_urgency_score(current_lane, [])
        feasibility_score = engine.calculate_feasibility_score(good_gap, vehicle_state)
        
        print(f"✅ Score components:")
        print(f"   Safety: {safety_score:.3f}")
        print(f"   Efficiency: {efficiency_score:.3f}")
        print(f"   Comfort: {comfort_score:.3f}")
        print(f"   Urgency: {urgency_score:.3f}")
        print(f"   Feasibility: {feasibility_score:.3f}")
        
        # Validate individual scores
        for score_name, score_value in [
            ("Safety", safety_score),
            ("Efficiency", efficiency_score),
            ("Comfort", comfort_score),
            ("Urgency", urgency_score),
            ("Feasibility", feasibility_score)
        ]:
            assert 0 <= score_value <= 1, f"{score_name} score should be between 0 and 1, got {score_value}"
        
        print("✅ Decision scoring validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Decision scoring validation failed: {e}")
        traceback.print_exc()
        return False


def validate_lane_change_evaluation():
    """Validate complete lane change evaluation"""
    print(f"\n{'='*50}")
    print(f"VALIDATING LANE CHANGE EVALUATION")
    print(f"{'='*50}")
    
    try:
        engine = LaneChangeDecisionEngine()
        
        # Create test scenario
        current_lane = create_test_lane_state("current", is_current=True)
        adjacent_lanes = {
            "left": create_test_lane_state("left"),
            "right": create_test_lane_state("right")
        }
        vehicle_state = create_test_vehicle_state()
        obstacles = []
        
        # Evaluate lane change opportunity
        decision, reason, confidence = engine.evaluate_lane_change_opportunity(
            current_lane, adjacent_lanes, vehicle_state, obstacles
        )
        
        # Validate results
        assert isinstance(decision, LaneChangeDecision), "Invalid decision type"
        assert isinstance(reason, LaneChangeReason), "Invalid reason type"
        assert isinstance(confidence, float), "Confidence should be float"
        assert 0 <= confidence <= 1, f"Confidence should be between 0 and 1, got {confidence}"
        
        print(f"✅ Lane change evaluation results:")
        print(f"   Decision: {decision.value}")
        print(f"   Reason: {reason.value}")
        print(f"   Confidence: {confidence:.3f}")
        
        # Test with blocking obstacle
        blocking_obstacles = [create_test_obstacle("duckiebot", 1.5, 0.9)]
        current_lane_blocked = create_test_lane_state("current", is_current=True, obstacles=blocking_obstacles)
        
        decision_blocked, reason_blocked, confidence_blocked = engine.evaluate_lane_change_opportunity(
            current_lane_blocked, adjacent_lanes, vehicle_state, blocking_obstacles
        )
        
        print(f"✅ Lane change evaluation with obstacle:")
        print(f"   Decision: {decision_blocked.value}")
        print(f"   Reason: {reason_blocked.value}")
        print(f"   Confidence: {confidence_blocked:.3f}")
        
        print("✅ Lane change evaluation validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Lane change evaluation validation failed: {e}")
        traceback.print_exc()
        return False


def validate_performance_metrics():
    """Validate performance metrics tracking"""
    print(f"\n{'='*50}")
    print(f"VALIDATING PERFORMANCE METRICS")
    print(f"{'='*50}")
    
    try:
        engine = LaneChangeDecisionEngine()
        
        # Reset metrics
        engine.reset_metrics()
        initial_metrics = engine.get_performance_metrics()
        
        # Validate initial metrics structure
        required_metrics = [
            'total_evaluations', 'successful_decisions', 'success_rate',
            'average_evaluation_time_ms', 'gap_analyses_performed',
            'safety_violations_detected', 'emergency_decisions',
            'last_update_time', 'decision_history_length'
        ]
        
        for metric in required_metrics:
            assert metric in initial_metrics, f"Missing metric: {metric}"
        
        print(f"✅ Initial metrics:")
        for metric, value in initial_metrics.items():
            print(f"   {metric}: {value}")
        
        # Perform some evaluations
        current_lane = create_test_lane_state("current", is_current=True)
        adjacent_lanes = {"left": create_test_lane_state("left")}
        vehicle_state = create_test_vehicle_state()
        
        for i in range(3):
            engine.evaluate_lane_change_opportunity(
                current_lane, adjacent_lanes, vehicle_state, []
            )
        
        # Check updated metrics
        final_metrics = engine.get_performance_metrics()
        
        assert final_metrics['total_evaluations'] > initial_metrics['total_evaluations'], "Evaluations not tracked"
        assert final_metrics['gap_analyses_performed'] > initial_metrics['gap_analyses_performed'], "Gap analyses not tracked"
        
        print(f"✅ Final metrics after 3 evaluations:")
        for metric, value in final_metrics.items():
            print(f"   {metric}: {value}")
        
        # Test decision history
        history = engine.get_decision_history()
        assert isinstance(history, list), "History should be a list"
        assert len(history) > 0, "History should not be empty"
        
        print(f"✅ Decision history ({len(history)} entries):")
        for i, (timestamp, decision, confidence) in enumerate(history[-3:]):  # Show last 3
            print(f"   {i+1}: {decision} (confidence: {confidence:.3f}) at {timestamp:.3f}")
        
        print("✅ Performance metrics validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Performance metrics validation failed: {e}")
        traceback.print_exc()
        return False


def validate_utility_functions():
    """Validate utility functions"""
    print(f"\n{'='*50}")
    print(f"VALIDATING UTILITY FUNCTIONS")
    print(f"{'='*50}")
    
    try:
        engine = LaneChangeDecisionEngine()
        
        # Test lateral acceleration estimation
        test_cases = [
            (1.0, 8.0),   # Normal case
            (2.0, 6.0),   # Higher speed
            (0.5, 10.0),  # Lower speed
        ]
        
        print(f"✅ Lateral acceleration estimation:")
        for velocity, gap_length in test_cases:
            accel = engine.estimate_lateral_acceleration(velocity, gap_length)
            assert accel > 0, f"Acceleration should be positive, got {accel}"
            print(f"   v={velocity:.1f}m/s, gap={gap_length:.1f}m -> accel={accel:.2f}m/s²")
        
        # Test edge cases
        inf_accel = engine.estimate_lateral_acceleration(0.0, 5.0)
        assert inf_accel == float('inf'), "Should return infinity for zero velocity"
        
        inf_accel2 = engine.estimate_lateral_acceleration(1.0, 0.0)
        assert inf_accel2 == float('inf'), "Should return infinity for zero gap"
        
        # Test maneuver time estimation
        print(f"✅ Maneuver time estimation:")
        for velocity, gap_length in test_cases:
            maneuver_time = engine.estimate_maneuver_time(velocity, gap_length)
            assert maneuver_time > 0, f"Maneuver time should be positive, got {maneuver_time}"
            print(f"   v={velocity:.1f}m/s, gap={gap_length:.1f}m -> time={maneuver_time:.2f}s")
        
        # Test deceleration distance calculation
        print(f"✅ Deceleration distance calculation:")
        for velocity in [0.5, 1.0, 2.0]:
            decel_dist = engine.calculate_deceleration_distance(velocity)
            assert decel_dist >= 0, f"Deceleration distance should be non-negative, got {decel_dist}"
            print(f"   v={velocity:.1f}m/s -> decel_dist={decel_dist:.2f}m")
        
        # Test gap confidence calculation
        print(f"✅ Gap confidence calculation:")
        confidence = engine.calculate_gap_confidence(8.0, 2.0, 1.5)
        assert 0 <= confidence <= 1, f"Confidence should be between 0 and 1, got {confidence}"
        print(f"   gap=8.0m, front_margin=2.0m, rear_margin=1.5m -> confidence={confidence:.3f}")
        
        print("✅ Utility functions validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Utility functions validation failed: {e}")
        traceback.print_exc()
        return False


def main():
    """Run all validation tests"""
    print(f"\n{'='*60}")
    print(f"LANE CHANGE DECISION ENGINE VALIDATION")
    print(f"{'='*60}")
    
    validation_functions = [
        validate_initialization,
        validate_gap_analysis,
        validate_decision_scoring,
        validate_lane_change_evaluation,
        validate_performance_metrics,
        validate_utility_functions
    ]
    
    results = []
    for validation_func in validation_functions:
        try:
            result = validation_func()
            results.append(result)
        except Exception as e:
            print(f"❌ Validation function {validation_func.__name__} crashed: {e}")
            traceback.print_exc()
            results.append(False)
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"VALIDATION SUMMARY")
    print(f"{'='*60}")
    
    passed = sum(results)
    total = len(results)
    
    print(f"Validations passed: {passed}/{total}")
    print(f"Success rate: {(passed/total*100):.1f}%")
    
    if passed == total:
        print(f"\n✅ All validations passed! Lane change decision engine implementation is working correctly.")
        return True
    else:
        print(f"\n❌ Some validations failed. Please check the implementation.")
        return False


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
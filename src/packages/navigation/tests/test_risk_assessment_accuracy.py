#!/usr/bin/env python3

import unittest
import math
import time
from unittest.mock import Mock, patch

import rospy
from geometry_msgs.msg import Point, Vector3, Point32
from duckietown_enhanced_msgs.msg import ObjectDetection
from std_msgs.msg import Header

# Import the module under test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from risk_assessment_engine import RiskAssessmentEngine, RiskLevel, VehicleState, RiskFactors


class TestRiskAssessmentAccuracy(unittest.TestCase):
    """
    Comprehensive unit tests for RiskAssessmentEngine accuracy and performance.
    
    Tests cover:
    - Distance-based risk calculation accuracy
    - Velocity risk assessment
    - Object type risk weighting
    - Trajectory intersection prediction
    - Time-to-collision calculations
    - Overall risk level determination
    - Edge cases and boundary conditions
    - Performance requirements
    """
    
    def setUp(self):
        """Set up test fixtures and mock ROS environment."""
        # Mock rospy to avoid ROS dependencies in unit tests
        self.rospy_patcher = patch('risk_assessment_engine.rospy')
        self.mock_rospy = self.rospy_patcher.start()
        
        # Initialize the risk assessment engine
        self.engine = RiskAssessmentEngine()
        
        # Create test vehicle state
        self.vehicle_state = VehicleState(
            position=Point(x=0.0, y=0.0, z=0.0),
            velocity=Vector3(x=1.0, y=0.0, z=0.0),  # Moving forward at 1 m/s
            heading=0.0,
            timestamp=time.time()
        )
        
        # Test detection templates
        self.base_detection = ObjectDetection()
        self.base_detection.header = Header()
        self.base_detection.header.stamp = rospy.Time.now()
        self.base_detection.class_name = "duckiebot"
        self.base_detection.confidence = 0.8
        self.base_detection.top_left = Point32(x=100, y=100, z=0)
        self.base_detection.bottom_right = Point32(x=200, y=200, z=0)
        self.base_detection.distance = 2.0
        self.base_detection.relative_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.base_detection.risk_level = 0
        self.base_detection.processing_time = 0.01
        self.base_detection.is_tracked = True
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.rospy_patcher.stop()
    
    def create_test_detection(self, distance=2.0, velocity_x=0.0, velocity_y=0.0, 
                            class_name="duckiebot", confidence=0.8):
        """
        Create a test detection with specified parameters.
        
        Args:
            distance: Distance to object in meters
            velocity_x: X component of relative velocity
            velocity_y: Y component of relative velocity
            class_name: Object class name
            confidence: Detection confidence
            
        Returns:
            ObjectDetection message
        """
        detection = ObjectDetection()
        detection.header = Header()
        detection.header.stamp = rospy.Time.now()
        detection.class_name = class_name
        detection.confidence = confidence
        detection.top_left = Point32(x=100, y=100, z=0)
        detection.bottom_right = Point32(x=200, y=200, z=0)
        detection.distance = distance
        detection.relative_velocity = Vector3(x=velocity_x, y=velocity_y, z=0.0)
        detection.risk_level = 0
        detection.processing_time = 0.01
        detection.is_tracked = True
        return detection
    
    def test_distance_risk_calculation_accuracy(self):
        """Test accuracy of distance-based risk calculations."""
        print("\n=== Testing Distance Risk Calculation Accuracy ===")
        
        test_cases = [
            # (distance, expected_risk_range, description)
            (0.2, (0.9, 1.0), "Critical distance - very high risk"),
            (0.3, (0.9, 1.0), "At critical threshold - high risk"),
            (0.5, (0.4, 0.6), "Safe distance threshold - medium risk"),
            (1.0, (0.1, 0.3), "Warning distance threshold - low risk"),
            (2.0, (0.0, 0.1), "Far distance - very low risk"),
            (5.0, (0.0, 0.05), "Very far distance - minimal risk")
        ]
        
        for distance, expected_range, description in test_cases:
            with self.subTest(distance=distance):
                risk = self.engine._calculate_distance_risk(distance)
                
                print(f"Distance: {distance}m -> Risk: {risk:.3f} ({description})")
                
                # Verify risk is within expected range
                self.assertGreaterEqual(risk, expected_range[0], 
                                      f"Risk too low for distance {distance}m")
                self.assertLessEqual(risk, expected_range[1], 
                                   f"Risk too high for distance {distance}m")
                
                # Verify risk is properly bounded
                self.assertGreaterEqual(risk, 0.0, "Risk cannot be negative")
                self.assertLessEqual(risk, 1.0, "Risk cannot exceed 1.0")
        
        # Test monotonicity - risk should decrease with increasing distance
        distances = [0.2, 0.5, 1.0, 2.0, 5.0]
        risks = [self.engine._calculate_distance_risk(d) for d in distances]
        
        for i in range(len(risks) - 1):
            self.assertGreaterEqual(risks[i], risks[i + 1], 
                                  f"Risk should decrease with distance: {distances[i]}m vs {distances[i+1]}m")
    
    def test_velocity_risk_calculation_accuracy(self):
        """Test accuracy of velocity-based risk calculations."""
        print("\n=== Testing Velocity Risk Calculation Accuracy ===")
        
        test_cases = [
            # (velocity, distance, expected_behavior, description)
            (0.0, 1.0, "low", "Stationary object - low velocity risk"),
            (0.3, 1.0, "low", "Slow approach - low velocity risk"),
            (0.8, 1.0, "medium", "Moderate approach - medium velocity risk"),
            (1.5, 1.0, "high", "Fast approach - high velocity risk"),
            (2.5, 1.0, "high", "Very fast approach - high velocity risk"),
            (1.0, 0.5, "high", "Close fast object - high risk"),
            (1.0, 3.0, "low", "Distant fast object - lower risk")
        ]
        
        for velocity, distance, expected_behavior, description in test_cases:
            with self.subTest(velocity=velocity, distance=distance):
                risk = self.engine._calculate_velocity_risk(velocity, distance)
                
                print(f"Velocity: {velocity}m/s, Distance: {distance}m -> Risk: {risk:.3f} ({description})")
                
                # Verify risk bounds
                self.assertGreaterEqual(risk, 0.0, "Velocity risk cannot be negative")
                self.assertLessEqual(risk, 1.0, "Velocity risk cannot exceed 1.0")
                
                # Verify expected behavior
                if expected_behavior == "low":
                    self.assertLess(risk, 0.4, f"Expected low risk for {description}")
                elif expected_behavior == "medium":
                    self.assertGreaterEqual(risk, 0.3, f"Expected medium risk for {description}")
                    self.assertLess(risk, 0.7, f"Expected medium risk for {description}")
                elif expected_behavior == "high":
                    self.assertGreaterEqual(risk, 0.5, f"Expected high risk for {description}")
    
    def test_object_type_risk_weighting(self):
        """Test object type risk weight assignments."""
        print("\n=== Testing Object Type Risk Weighting ===")
        
        test_cases = [
            ("person", 1.0, "Person - highest risk"),
            ("barrier", 0.9, "Barrier - very high risk"),
            ("vehicle", 0.9, "Vehicle - very high risk"),
            ("duckiebot", 0.8, "Duckiebot - high risk"),
            ("cone", 0.6, "Cone - medium risk"),
            ("unknown", 0.5, "Unknown object - default risk"),
            ("duckie", 0.3, "Duckie - low risk"),
            ("nonexistent", 0.5, "Non-existent class - default risk")
        ]
        
        for object_class, expected_weight, description in test_cases:
            with self.subTest(object_class=object_class):
                risk = self.engine._calculate_object_type_risk(object_class)
                
                print(f"Object: {object_class} -> Risk Weight: {risk:.1f} ({description})")
                
                self.assertEqual(risk, expected_weight, 
                               f"Incorrect risk weight for {object_class}")
                
                # Verify risk weight bounds
                self.assertGreaterEqual(risk, 0.0, "Risk weight cannot be negative")
                self.assertLessEqual(risk, 1.0, "Risk weight cannot exceed 1.0")
    
    def test_time_to_collision_calculation(self):
        """Test time-to-collision calculation accuracy."""
        print("\n=== Testing Time-to-Collision Calculation ===")
        
        test_cases = [
            # (distance, velocity, expected_ttc_range, description)
            (2.0, 1.0, (1.4, 1.8), "2m at 1m/s - should be ~1.6s with safety factor"),
            (1.0, 0.5, (1.4, 1.8), "1m at 0.5m/s - should be ~1.6s with safety factor"),
            (3.0, 2.0, (1.0, 1.4), "3m at 2m/s - should be ~1.2s with safety factor"),
            (1.0, 0.0, (float('inf'), float('inf')), "No relative velocity - infinite TTC"),
            (2.0, 0.005, (float('inf'), float('inf')), "Very slow approach - infinite TTC")
        ]
        
        for distance, velocity, expected_range, description in test_cases:
            with self.subTest(distance=distance, velocity=velocity):
                detection = self.create_test_detection(
                    distance=distance, 
                    velocity_x=-velocity  # Negative for approaching
                )
                
                ttc = self.engine._calculate_time_to_collision(detection, self.vehicle_state)
                
                print(f"Distance: {distance}m, Velocity: {velocity}m/s -> TTC: {ttc:.2f}s ({description})")
                
                if math.isinf(expected_range[0]):
                    self.assertTrue(math.isinf(ttc), f"Expected infinite TTC for {description}")
                else:
                    self.assertGreaterEqual(ttc, expected_range[0], 
                                          f"TTC too low for {description}")
                    self.assertLessEqual(ttc, expected_range[1], 
                                       f"TTC too high for {description}")
                
                # TTC should never be negative
                self.assertGreaterEqual(ttc, 0.0, "TTC cannot be negative")
    
    def test_risk_level_determination_accuracy(self):
        """Test overall risk level determination accuracy."""
        print("\n=== Testing Risk Level Determination Accuracy ===")
        
        test_scenarios = [
            # (risk_factors, expected_level, description)
            (RiskFactors(1.0, 0.8, 0.9, 0.9, 0.5, 0.2), RiskLevel.CRITICAL, "All high risks - critical"),
            (RiskFactors(0.6, 0.5, 0.8, 0.7, 2.0, 0.3), RiskLevel.HIGH, "Multiple medium-high risks - high"),
            (RiskFactors(0.4, 0.3, 0.6, 0.5, 4.0, 0.4), RiskLevel.MEDIUM, "Moderate risks - medium"),
            (RiskFactors(0.2, 0.1, 0.3, 0.2, 10.0, 0.6), RiskLevel.LOW, "Low risks - low"),
            (RiskFactors(0.3, 0.2, 0.5, 0.4, 0.8, 0.3), RiskLevel.CRITICAL, "Low risks but critical TTC - critical"),
            (RiskFactors(0.2, 0.1, 0.3, 0.2, 2.5, 0.3), RiskLevel.MEDIUM, "Low risks but warning TTC - medium")
        ]
        
        for risk_factors, expected_level, description in test_scenarios:
            with self.subTest(description=description):
                risk_level = self.engine._determine_risk_level(risk_factors)
                
                print(f"Risk Factors: D:{risk_factors.distance_risk:.1f} V:{risk_factors.velocity_risk:.1f} "
                      f"T:{risk_factors.object_type_risk:.1f} Tr:{risk_factors.trajectory_risk:.1f} "
                      f"TTC:{risk_factors.time_to_collision:.1f}s -> Level: {risk_level.name} ({description})")
                
                self.assertEqual(risk_level, expected_level, 
                               f"Incorrect risk level for {description}")
    
    def test_comprehensive_risk_assessment_integration(self):
        """Test complete risk assessment integration."""
        print("\n=== Testing Comprehensive Risk Assessment Integration ===")
        
        # Create test scenarios with multiple objects
        detections = [
            self.create_test_detection(distance=0.3, velocity_x=-1.0, class_name="barrier"),  # Critical
            self.create_test_detection(distance=1.5, velocity_x=-0.5, class_name="duckiebot"),  # Medium
            self.create_test_detection(distance=3.0, velocity_x=0.0, class_name="duckie"),  # Low
            self.create_test_detection(distance=0.8, velocity_x=-2.0, class_name="person")  # High
        ]
        
        # Perform risk assessment
        start_time = time.time()
        risk_assessments = self.engine.assess_collision_risk(detections, self.vehicle_state)
        assessment_time = (time.time() - start_time) * 1000
        
        print(f"Assessed {len(detections)} objects in {assessment_time:.2f}ms")
        
        # Verify assessment results
        self.assertEqual(len(risk_assessments), len(detections), 
                        "Should have assessment for each detection")
        
        # Verify performance requirement (100ms for risk assessment)
        self.assertLess(assessment_time, 100.0, 
                       "Risk assessment should complete within 100ms")
        
        # Check individual assessments
        expected_levels = [RiskLevel.CRITICAL, RiskLevel.MEDIUM, RiskLevel.LOW, RiskLevel.HIGH]
        
        for i, (detection, risk_factors, risk_level) in enumerate(risk_assessments):
            print(f"Object {i}: {detection.class_name} at {detection.distance:.1f}m -> {risk_level.name}")
            
            # Verify risk factors are properly calculated
            self.assertIsInstance(risk_factors, RiskFactors, "Should return RiskFactors object")
            self.assertGreaterEqual(risk_factors.distance_risk, 0.0, "Distance risk should be non-negative")
            self.assertLessEqual(risk_factors.distance_risk, 1.0, "Distance risk should not exceed 1.0")
            
            # Verify time to collision is reasonable
            if not math.isinf(risk_factors.time_to_collision):
                self.assertGreaterEqual(risk_factors.time_to_collision, 0.0, "TTC should be non-negative")
        
        # Test risk statistics generation
        stats = self.engine.get_risk_statistics(risk_assessments)
        
        self.assertEqual(stats['total_objects'], len(detections), "Incorrect object count in statistics")
        self.assertIn('risk_distribution', stats, "Statistics should include risk distribution")
        self.assertIn('average_distance', stats, "Statistics should include average distance")
        self.assertIn('minimum_ttc', stats, "Statistics should include minimum TTC")
        
        print(f"Risk Statistics: {stats}")
    
    def test_edge_cases_and_boundary_conditions(self):
        """Test edge cases and boundary conditions."""
        print("\n=== Testing Edge Cases and Boundary Conditions ===")
        
        # Test empty detection list
        empty_assessments = self.engine.assess_collision_risk([], self.vehicle_state)
        self.assertEqual(len(empty_assessments), 0, "Empty detection list should return empty assessments")
        
        # Test zero distance (should not crash)
        zero_distance_detection = self.create_test_detection(distance=0.0)
        try:
            assessments = self.engine.assess_collision_risk([zero_distance_detection], self.vehicle_state)
            self.assertEqual(len(assessments), 1, "Should handle zero distance gracefully")
            print("Zero distance handled gracefully")
        except Exception as e:
            self.fail(f"Zero distance should not cause exception: {e}")
        
        # Test very large distance
        large_distance_detection = self.create_test_detection(distance=1000.0)
        assessments = self.engine.assess_collision_risk([large_distance_detection], self.vehicle_state)
        _, risk_factors, risk_level = assessments[0]
        self.assertEqual(risk_level, RiskLevel.LOW, "Very distant objects should have low risk")
        print(f"Large distance (1000m) -> Risk Level: {risk_level.name}")
        
        # Test very high velocity
        high_velocity_detection = self.create_test_detection(velocity_x=-10.0)
        assessments = self.engine.assess_collision_risk([high_velocity_detection], self.vehicle_state)
        _, risk_factors, risk_level = assessments[0]
        self.assertGreaterEqual(risk_factors.velocity_risk, 0.5, "High velocity should increase risk")
        print(f"High velocity (10m/s) -> Velocity Risk: {risk_factors.velocity_risk:.3f}")
        
        # Test negative velocity (receding object)
        receding_detection = self.create_test_detection(velocity_x=2.0)  # Positive = receding
        assessments = self.engine.assess_collision_risk([receding_detection], self.vehicle_state)
        _, risk_factors, risk_level = assessments[0]
        print(f"Receding object -> Risk Level: {risk_level.name}")
    
    def test_performance_requirements(self):
        """Test performance requirements compliance."""
        print("\n=== Testing Performance Requirements ===")
        
        # Create multiple detections to test performance
        num_objects = 10
        detections = []
        for i in range(num_objects):
            detection = self.create_test_detection(
                distance=1.0 + i * 0.5,
                velocity_x=-0.5 - i * 0.1,
                class_name=["duckiebot", "duckie", "cone", "barrier"][i % 4]
            )
            detections.append(detection)
        
        # Measure assessment time
        start_time = time.time()
        risk_assessments = self.engine.assess_collision_risk(detections, self.vehicle_state)
        total_time = (time.time() - start_time) * 1000
        
        print(f"Assessed {num_objects} objects in {total_time:.2f}ms")
        print(f"Average time per object: {total_time/num_objects:.2f}ms")
        
        # Verify performance requirements
        # Requirement 3.2: assess collision risk within 100ms
        self.assertLess(total_time, 100.0, 
                       "Risk assessment should complete within 100ms")
        
        # Verify all objects were assessed
        self.assertEqual(len(risk_assessments), num_objects, 
                        "All objects should be assessed")
        
        # Test repeated assessments for consistency
        times = []
        for _ in range(5):
            start = time.time()
            self.engine.assess_collision_risk(detections, self.vehicle_state)
            times.append((time.time() - start) * 1000)
        
        avg_time = sum(times) / len(times)
        max_time = max(times)
        
        print(f"Repeated assessments - Average: {avg_time:.2f}ms, Max: {max_time:.2f}ms")
        
        # Performance should be consistent
        self.assertLess(max_time, 100.0, "Maximum assessment time should be under 100ms")
        self.assertLess(max_time - avg_time, 20.0, "Performance should be consistent")


if __name__ == '__main__':
    # Configure test output
    import sys
    
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestRiskAssessmentAccuracy)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout, buffer=False)
    result = runner.run(suite)
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"RISK ASSESSMENT ENGINE TEST SUMMARY")
    print(f"{'='*60}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\nFAILURES:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print(f"\nERRORS:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)
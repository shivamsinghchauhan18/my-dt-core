#!/usr/bin/env python3

"""
Validation script for YOLO detection implementation.
This script validates the implementation without requiring ROS environment.
"""

import sys
import os
import importlib.util
import numpy as np
from unittest.mock import Mock, patch

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

def validate_optimized_yolo_detector():
    """Validate OptimizedYOLODetector class structure and methods."""
    print("🔍 Validating OptimizedYOLODetector implementation...")
    
    try:
        # Check if the module can be imported (with mocked dependencies)
        with patch.dict('sys.modules', {
            'rospy': Mock(),
            'cv_bridge': Mock(),
            'duckietown.dtros': Mock(),
            'duckietown_msgs.msg': Mock(),
            'geometry_msgs.msg': Mock(),
            'sensor_msgs.msg': Mock(),
            'std_msgs.msg': Mock(),
            'torch': Mock(),
            'ultralytics': Mock(),
            'psutil': Mock()
        }):
            spec = importlib.util.spec_from_file_location(
                "optimized_yolo_detector", 
                os.path.join(os.path.dirname(__file__), '..', 'src', 'optimized_yolo_detector.py')
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            
            # Check class exists
            assert hasattr(module, 'OptimizedYOLODetector'), "OptimizedYOLODetector class not found"
            assert hasattr(module, 'PerformanceMetrics'), "PerformanceMetrics class not found"
            
            detector_class = module.OptimizedYOLODetector
            
            # Check required methods exist
            required_methods = [
                '__init__',
                'detect_objects',
                'get_performance_metrics',
                'shutdown',
                '_setup_device',
                '_load_model',
                '_preprocess_image',
                '_postprocess_results',
                '_estimate_distance',
                '_assess_risk_level'
            ]
            
            for method in required_methods:
                assert hasattr(detector_class, method), f"Method {method} not found"
            
            # Check class attributes
            assert hasattr(detector_class, 'DUCKIETOWN_CLASSES'), "DUCKIETOWN_CLASSES not found"
            
            # Validate Duckietown classes
            expected_classes = {
                0: 'duckie',
                1: 'duckiebot', 
                2: 'traffic_cone',
                3: 'stop_sign',
                4: 'intersection_sign',
                5: 'pedestrian',
                6: 'vehicle',
                7: 'barrier'
            }
            
            assert detector_class.DUCKIETOWN_CLASSES == expected_classes, "DUCKIETOWN_CLASSES mismatch"
            
            print("✅ OptimizedYOLODetector validation passed")
            return True
            
    except Exception as e:
        print(f"❌ OptimizedYOLODetector validation failed: {e}")
        return False

def validate_enhanced_vehicle_detection_node():
    """Validate EnhancedVehicleDetectionNode class structure."""
    print("🔍 Validating EnhancedVehicleDetectionNode implementation...")
    
    try:
        # Read the file and check for required methods
        node_path = os.path.join(os.path.dirname(__file__), '..', 'src', 'enhanced_vehicle_detection_node.py')
        
        with open(node_path, 'r') as f:
            content = f.read()
        
        # Check for class definition
        assert 'class EnhancedVehicleDetectionNode' in content, "EnhancedVehicleDetectionNode class not found"
        
        # Check for required methods
        required_methods = [
            'def __init__',
            'def cb_image',
            'def cb_performance_monitor',
            'def on_shutdown',
            'def _create_debug_image'
        ]
        
        for method in required_methods:
            assert method in content, f"Method {method} not found in source code"
        
        # Check for ROS integration
        assert 'rospy.Subscriber' in content, "ROS subscriber not found"
        assert 'rospy.Publisher' in content, "ROS publisher not found"
        assert 'ObjectDetectionArray' in content, "ObjectDetectionArray message not used"
        
        print("✅ EnhancedVehicleDetectionNode validation passed")
        return True
        
    except Exception as e:
        print(f"❌ EnhancedVehicleDetectionNode validation failed: {e}")
        return False

def validate_file_structure():
    """Validate that all required files are present."""
    print("🔍 Validating file structure...")
    
    base_path = os.path.join(os.path.dirname(__file__), '..')
    
    required_files = [
        'src/optimized_yolo_detector.py',
        'src/enhanced_vehicle_detection_node.py',
        'tests/test_yolo_detection_accuracy.py',
        'tests/validate_implementation.py',
        'config/enhanced_vehicle_detection_node/default.yaml',
        'launch/enhanced_vehicle_detection_node.launch',
        'package.xml',
        'setup.py'
    ]
    
    missing_files = []
    for file_path in required_files:
        full_path = os.path.join(base_path, file_path)
        if not os.path.exists(full_path):
            missing_files.append(file_path)
    
    if missing_files:
        print(f"❌ Missing files: {missing_files}")
        return False
    else:
        print("✅ All required files present")
        return True

def validate_configuration():
    """Validate configuration file structure."""
    print("🔍 Validating configuration file...")
    
    try:
        config_path = os.path.join(
            os.path.dirname(__file__), '..', 
            'config', 'enhanced_vehicle_detection_node', 'default.yaml'
        )
        
        with open(config_path, 'r') as f:
            config_content = f.read()
        
        # Check required configuration sections exist in file
        required_sections = [
            'model_path:',
            'confidence_threshold:',
            'nms_threshold:',
            'device:',
            'object_classes:',
            'risk_assessment:',
            'distance_estimation:',
            'performance_requirements:'
        ]
        
        for section in required_sections:
            assert section in config_content, f"Configuration section {section} not found"
        
        # Check for object classes
        assert 'duckie' in config_content, "duckie class not found in config"
        assert 'duckiebot' in config_content, "duckiebot class not found in config"
        
        print("✅ Configuration validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Configuration validation failed: {e}")
        return False

def validate_dependencies():
    """Validate that required dependencies are listed."""
    print("🔍 Validating dependencies...")
    
    try:
        # Check dependencies-py3.txt
        deps_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'dependencies-py3.txt')
        
        with open(deps_path, 'r') as f:
            deps_content = f.read()
        
        required_deps = ['torch', 'ultralytics', 'scipy']
        
        for dep in required_deps:
            assert dep in deps_content, f"Dependency {dep} not found in dependencies-py3.txt"
        
        print("✅ Dependencies validation passed")
        return True
        
    except Exception as e:
        print(f"❌ Dependencies validation failed: {e}")
        return False

def main():
    """Run all validation tests."""
    print("🚀 Starting YOLO Detection Implementation Validation")
    print("=" * 60)
    
    validations = [
        validate_file_structure,
        validate_optimized_yolo_detector,
        validate_enhanced_vehicle_detection_node,
        validate_configuration,
        validate_dependencies
    ]
    
    results = []
    for validation in validations:
        try:
            result = validation()
            results.append(result)
        except Exception as e:
            print(f"❌ Validation error: {e}")
            results.append(False)
        print()
    
    # Summary
    print("=" * 60)
    print("📊 VALIDATION SUMMARY")
    print("=" * 60)
    
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print(f"🎉 ALL VALIDATIONS PASSED ({passed}/{total})")
        print("\n✅ YOLO Detection Implementation is ready for testing!")
        print("\nNext steps:")
        print("1. Build the ROS package: catkin build vehicle_detection")
        print("2. Source the workspace: source devel/setup.bash")
        print("3. Launch the node: roslaunch vehicle_detection enhanced_vehicle_detection_node.launch")
        return True
    else:
        print(f"⚠️  SOME VALIDATIONS FAILED ({passed}/{total})")
        print("\n❌ Please fix the issues above before proceeding.")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
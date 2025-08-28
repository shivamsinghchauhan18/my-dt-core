"""
Enhanced message namespace for custom Duckietown messages.
These messages extend the standard dt-core functionality without conflicts.
"""

# Import enhanced message types that are safe to deploy
try:
    from .LaneCurve import LaneCurve
    from .ObjectDetection import ObjectDetection
    from .SafetyStatus import SafetyStatus
    from .ObjectDetectionArray import ObjectDetectionArray
    from .LaneCurves import LaneCurves
    from .AdvancedLanePose import AdvancedLanePose
except ImportError as e:
    # Fallback for development/testing
    import warnings
    warnings.warn(f"Enhanced messages not available: {e}")
    
    # Create stub classes for testing
    class LaneCurve:
        pass
    class ObjectDetection:
        pass
    class SafetyStatus:
        pass
    class ObjectDetectionArray:
        pass
    class LaneCurves:
        pass
    class AdvancedLanePose:
        pass

__all__ = [
    'LaneCurve',
    'ObjectDetection', 
    'SafetyStatus',
    'ObjectDetectionArray',
    'LaneCurves',
    'AdvancedLanePose'
]

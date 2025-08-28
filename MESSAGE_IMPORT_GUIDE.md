# Message Import Guide

## Safe Import Patterns for Duckietown Messages

### Official dt-core Messages (Use Standard Imports)
These messages exist in the robot's dt-core and should be imported normally:

```python
# Official messages - use standard imports
from duckietown_msgs.msg import (
    SignalsDetection,
    CoordinationSignal,
    MaintenanceState,
    AprilTagsWithInfos,
    StopLineReading,
    WheelsCmdStamped,
    Segment,
    SegmentList,
    CoordinationClearance,
    BoolStamped,
    Twist2DStamped,
    FSMState,
    LanePose,
    VehicleCorners
)
```

### Enhanced Custom Messages (Use Enhanced Namespace)
These messages are custom/enhanced and isolated to prevent conflicts:

```python
# Enhanced messages - use enhanced namespace imports
from duckietown_msgs.msg.enhanced import (
    LaneCurve,
    ObjectDetection,
    SafetyStatus,
    ObjectDetectionArray,
    LaneCurves,
    AdvancedLanePose
)
```

### Mixed Imports Example
```python
# Standard dt-core messages
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped

# Enhanced custom messages  
from duckietown_msgs.msg.enhanced import SafetyStatus, ObjectDetection

class MyNode:
    def __init__(self):
        # Use official messages normally
        self.fsm_pub = rospy.Publisher('/fsm_state', FSMState, queue_size=1)
        
        # Use enhanced messages for custom functionality
        self.safety_pub = rospy.Publisher('/safety_status', SafetyStatus, queue_size=1)
```

### Development Guidelines

#### ✅ DO:
- Use standard imports for official dt-core messages
- Use enhanced namespace imports for custom messages
- Keep enhanced messages isolated and well-documented
- Test imports before deployment

#### ❌ DON'T:
- Mix official and enhanced versions of the same message
- Import enhanced messages using standard namespace
- Deploy official messages (let robot use its versions)
- Modify official message definitions

### Deployment Safety

The deployment system will:
1. ✅ Deploy only enhanced messages (no conflicts)
2. ✅ Use robot's official dt-core messages
3. ✅ Validate import compatibility
4. ✅ Provide rollback if issues occur

This ensures 100% compatibility with the robot's dt-core while enabling enhanced functionality.

# Message Exclusion Manifest

## Official dt-core Messages (Excluded from Deployment)

These messages exist in the official dt-core and will cause conflicts.
They have been backed up and excluded from our deployment.
The robot will use its official versions.

### Excluded Messages:
- SignalsDetection.msg
- CoordinationSignal.msg
- MaintenanceState.msg
- AprilTagsWithInfos.msg
- StopLineReading.msg
- WheelsCmdStamped.msg
- Segment.msg
- SegmentList.msg
- CoordinationClearance.msg
- BoolStamped.msg
- Twist2DStamped.msg
- FSMState.msg
- LanePose.msg
- VehicleCorners.msg

### Custom Enhanced Messages (Deployed in Enhanced Namespace):
- LaneCurve.msg
- ObjectDetection.msg
- SafetyStatus.msg
- ObjectDetectionArray.msg
- LaneCurves.msg
- AdvancedLanePose.msg

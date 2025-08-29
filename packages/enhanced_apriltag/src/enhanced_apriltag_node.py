#!/usr/bin/env python3
"""
Enhanced AprilTag Detection Node

Adds simple distance estimation and repackages results into enhanced messages
for downstream modules, while also republishing the original detections for
backward compatibility.
"""

import math
import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_enhanced_msgs.msg import ObjectDetection, ObjectDetectionArray


class EnhancedAprilTagNode:
    def __init__(self):
        rospy.init_node('enhanced_apriltag_node', anonymous=False)

        # Parameters for distance estimation (very rough pinhole model)
        self.tag_size_m = float(rospy.get_param('~tag_size_m', 0.065))  # 6.5 cm default
        # Camera intrinsics (pixels)
        self.fx = float(rospy.get_param('~fx', rospy.get_param('~focal_px', 525.0)))
        self.fy = float(rospy.get_param('~fy', self.fx))

        # Sub/Publishes
        self.sub_apriltag = rospy.Subscriber(
            rospy.get_param('~input', 'apriltag_detector_node/detections'),
            AprilTagDetectionArray,
            self.apriltag_callback,
            queue_size=1
        )

        self.pub_enhanced = rospy.Publisher(
            '~objects',
            ObjectDetectionArray,
            queue_size=1
        )

        # Back-compat republish
        self.pub_passthrough = rospy.Publisher(
            '~detections',
            AprilTagDetectionArray,
            queue_size=1
        )

        rospy.loginfo("Enhanced AprilTag node initialized")

    def _estimate_distance(self, detection) -> float:
        """Estimate distance using tag size and apparent size.

        This is a placeholder calculation. If the detection exposes the tag's
        corners in image pixels, use that to estimate pixel side length.
        """
        try:
            # Some dt-core AprilTag messages contain pose or size; fall back to constant
            if hasattr(detection, 'size') and detection.size > 0:
                pixel_size = detection.size
            elif hasattr(detection, 'corners') and len(detection.corners) >= 2:
                # Distance between first two corners as a proxy
                c0, c1 = detection.corners[0], detection.corners[1]
                dx = c0.x - c1.x
                dy = c0.y - c1.y
                pixel_size = math.hypot(dx, dy)
            else:
                pixel_size = None

            if pixel_size and pixel_size > 1e-3:
                # Use fx for horizontal size conversion
                return (self.tag_size_m * self.fx) / pixel_size
        except Exception:
            pass

        # Fallback nominal distance if we cannot estimate
        return 1.0

    def apriltag_callback(self, msg: AprilTagDetectionArray):
        # Backward-compatible republish
        self.pub_passthrough.publish(msg)

        # Build enhanced objects array
        out = ObjectDetectionArray()
        out.header = getattr(msg, 'header', None) or out.header
        out.total_objects = len(getattr(msg, 'detections', []))
        out.processing_time = 0.0
        out.inference_fps = 0.0

        avg_conf = 0.0
        for det in getattr(msg, 'detections', []):
            od = ObjectDetection()
            od.header = out.header
            tag_id = getattr(det, 'tag_id', -1)
            od.class_name = f"apriltag_{tag_id}"
            od.confidence = float(getattr(det, 'decision_margin', 0.5))

            # Bounding box if center/size provided
            if hasattr(det, 'center') and hasattr(det, 'size'):
                cx = getattr(det.center, 'x', 0.0)
                cy = getattr(det.center, 'y', 0.0)
                s = getattr(det, 'size', 0.0)
                half = s / 2.0
                # Populate minimal bbox points
                from geometry_msgs.msg import Point32
                od.top_left = Point32(x=cx - half, y=cy - half, z=0.0)
                od.bottom_right = Point32(x=cx + half, y=cy + half, z=0.0)

            od.distance = float(self._estimate_distance(det))
            od.relative_velocity.x = 0.0
            od.relative_velocity.y = 0.0
            od.relative_velocity.z = 0.0
            od.risk_level = od.RISK_LOW
            od.processing_time = 0.0
            od.is_tracked = False

            out.detections.append(od)
            avg_conf += od.confidence

        if out.total_objects > 0:
            out.average_confidence = avg_conf / out.total_objects
        else:
            out.average_confidence = 0.0
        out.high_risk_objects_count = 0

        self.pub_enhanced.publish(out)


if __name__ == '__main__':
    try:
        node = EnhancedAprilTagNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

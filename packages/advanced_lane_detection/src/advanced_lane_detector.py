#!/usr/bin/env python3
"""
Advanced Lane Detector Node

Performs lane detection via HSV color thresholding for white/yellow, ROI masking,
and quadratic polynomial fitting to estimate lateral offset (d), heading error (phi),
and curvature. Publishes duckietown_enhanced_msgs/AdvancedLanePose.

Assumptions:
- Input image topic: ~image (sensor_msgs/Image)
- Output topic: ~advanced_lane_pose (AdvancedLanePose)
"""

import time
import math
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from duckietown_enhanced_msgs.msg import AdvancedLanePose


class AdvancedLaneDetector:
	def __init__(self):
		self.node_name = rospy.get_name()
		self.bridge = CvBridge()

		# Params
		self.prediction_horizon = rospy.get_param("~prediction_horizon", 1.0)
		self.roi_y_start = int(rospy.get_param("~roi_y_start", 0.5 * 240))  # default for 320x240
		# meters per pixel scaling (approximate)
		self.m_per_px_x = float(rospy.get_param("~meters_per_pixel_x", 0.003))
		self.m_per_px_y = float(rospy.get_param("~meters_per_pixel_y", 0.003))

		# HSV thresholds (tuneable)
		self.yellow_lower = np.array(rospy.get_param("~yellow_lower", [15, 80, 80]), dtype=np.uint8)
		self.yellow_upper = np.array(rospy.get_param("~yellow_upper", [40, 255, 255]), dtype=np.uint8)
		self.white_lower = np.array(rospy.get_param("~white_lower", [0, 0, 200]), dtype=np.uint8)
		self.white_upper = np.array(rospy.get_param("~white_upper", [180, 40, 255]), dtype=np.uint8)

		# I/O
		self.sub_image = rospy.Subscriber("~image", Image, self.cb_image, queue_size=1)
		self.pub_pose = rospy.Publisher("~advanced_lane_pose", AdvancedLanePose, queue_size=1)

	def cb_image(self, msg: Image):
		t0 = time.time()
		try:
			cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		except Exception as e:
			rospy.logwarn_throttle(5.0, f"cv_bridge failed: {e}")
			return

		h, w = cv_img.shape[:2]

		# ROI to bottom part of the image
		y0 = max(0, min(h - 1, self.roi_y_start))
		roi = cv_img[y0:h, :]

		# HSV thresholding for yellow and white
		hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		mask_y = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
		mask_w = cv2.inRange(hsv, self.white_lower, self.white_upper)
		mask = cv2.bitwise_or(mask_y, mask_w)

		# Morphology to clean noise
		kernel = np.ones((3, 3), np.uint8)
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

		# Compute centerline by column-wise mass center per row
		ys, xs = np.nonzero(mask)
		detection_confidence = float(xs.size) / float(mask.size + 1e-6)

		d_m = 0.0
		phi = 0.0
		curvature_m_inv = 0.0
		curve_confidence = min(1.0, 3.0 * detection_confidence)

		if xs.size > 100:  # require minimal points
			# Convert coordinates to full image frame
			ys_full = ys + y0

			# Fit a second-order polynomial x = a*y^2 + b*y + c (in pixels)
			y_vals = ys_full.astype(np.float32)
			x_vals = xs.astype(np.float32)
			try:
				coeffs = np.polyfit(y_vals, x_vals, 2)
				a, b, c = coeffs
				# Evaluate at bottom of image (vehicle location)
				y_eval = float(h - 1)
				x_eval = a * y_eval ** 2 + b * y_eval + c
				dx_dy = 2 * a * y_eval + b
				d2x_dy2 = 2 * a

				# Convert to meters
				x_eval_m = (x_eval - (w / 2.0)) * self.m_per_px_x
				# lateral offset is negative if lane center is to the right of image center
				d_m = -float(x_eval_m)

				# Heading from dx/dy: tan(theta) ~ dx/dy; small-angle approx
				phi = float(math.atan(dx_dy * (self.m_per_px_x / max(self.m_per_px_y, 1e-6))))

				# Curvature of x(y): k = |x''| / (1 + x'^2)^(3/2); convert px to meters scaling
				dx_dy_m = dx_dy * (self.m_per_px_x / max(self.m_per_px_y, 1e-6))
				d2x_dy2_m = d2x_dy2 * (self.m_per_px_x / max(self.m_per_px_y ** 2, 1e-9))
				denom = (1.0 + dx_dy_m ** 2) ** 1.5
				curvature_m_inv = float(abs(d2x_dy2_m) / max(denom, 1e-6))
			except Exception as e:
				rospy.logwarn_throttle(5.0, f"polyfit failed: {e}")

		curve_radius = (1.0 / curvature_m_inv) if curvature_m_inv > 1e-6 else float('inf')

		processing_time = time.time() - t0

		out = AdvancedLanePose()
		out.header = msg.header
		out.d = float(d_m)
		out.phi = float(phi)
		out.curvature = float(curvature_m_inv)
		out.curve_radius = float(curve_radius if math.isfinite(curve_radius) else 0.0)
		out.curve_confidence = float(curve_confidence)
		out.predicted_path = []
		out.prediction_horizon = float(self.prediction_horizon)
		out.detection_confidence = float(detection_confidence)
		out.left_lane_confidence = 0.0
		out.right_lane_confidence = 0.0
		out.frames_tracked = 0
		out.temporal_consistency = 0.0
		out.processing_time = float(processing_time)
		out.detection_method = "hsv_polyfit"

		self.pub_pose.publish(out)


def main():
	rospy.init_node("advanced_lane_detector_node")
	node = AdvancedLaneDetector()
	rospy.loginfo("advanced_lane_detector_node is up")
	rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

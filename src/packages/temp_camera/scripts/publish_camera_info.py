#!/usr/bin/env python3
"""
Publish CameraInfo for a vehicle camera from a YAML calibration file.
Usage:
    # using an absolute path to a calibration file
    rosrun temp_camera publish_camera_info.py _veh:=blueduckie _calib_file:=/data/config/calibrations/camera_intrinsic/blueduckie.yaml
    # using the calibration shipped in this package
    rosrun temp_camera publish_camera_info.py _veh:=blueduckie _calib_file:=$(rospack find temp_camera)/config/blueduckie.yaml
"""
import rospy
import yaml
import os
from sensor_msgs.msg import CameraInfo


def load_calibration(yaml_file):
    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f'Calibration file not found: {yaml_file}')
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    # Expect keys similar to camera_info_manager YAML: image_width/height, camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix
    ci = CameraInfo()
    ci.width = int(data.get('image_width', 0))
    ci.height = int(data.get('image_height', 0))
    K = data.get('camera_matrix', {}).get('data') or data.get('camera_matrix', {}).get('K')
    if K:
        ci.K = list(map(float, K))
    D = data.get('distortion_coefficients', {}).get('data') or data.get('distortion_coefficients', {}).get('D')
    if D:
        ci.D = list(map(float, D))
    R = data.get('rectification_matrix', {}).get('data') or data.get('rectification_matrix', {}).get('R')
    if R:
        ci.R = list(map(float, R))
    P = data.get('projection_matrix', {}).get('data') or data.get('projection_matrix', {}).get('P')
    if P:
        ci.P = list(map(float, P))
    return ci


def main():
    rospy.init_node('camera_info_publisher', anonymous=False)
    veh = rospy.get_param('~veh', 'blueduckie')
    yaml_file = rospy.get_param('~calib_file', '/data/config/calibrations/camera_intrinsic/blueduckie.yaml')
    topic_ns = f'/{veh}/camera_node'
    pub_topic = topic_ns + '/camera_info'

    rospy.loginfo(f'[camera_info_publisher] veh={veh} calib_file={yaml_file} publishing to {pub_topic}')

    try:
        cam_info = load_calibration(yaml_file)
        rospy.loginfo('[camera_info_publisher] Calibration loaded')
    except Exception as e:
        rospy.logwarn(f'[camera_info_publisher] Failed to load calibration: {e} -- publishing minimal CameraInfo')
        cam_info = CameraInfo()
        cam_info.width = rospy.get_param('~width', 640)
        cam_info.height = rospy.get_param('~height', 480)
        # fill K with focal approximations if not present
        fx = rospy.get_param('~fx', cam_info.width)
        fy = rospy.get_param('~fy', cam_info.height)
        cx = rospy.get_param('~cx', cam_info.width / 2)
        cy = rospy.get_param('~cy', cam_info.height / 2)
        cam_info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        cam_info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    pub = rospy.Publisher(pub_topic, CameraInfo, queue_size=1, latch=True)
    rate_hz = rospy.get_param('~rate', 5)
    rate = rospy.Rate(rate_hz)

    # publish repeatedly (latch True will keep last message for new subscribers)
    while not rospy.is_shutdown():
        cam_info.header.stamp = rospy.Time.now()
        cam_info.header.frame_id = topic_ns + '/camera_frame'
        pub.publish(cam_info)
        rate.sleep()


if __name__ == '__main__':
    main()

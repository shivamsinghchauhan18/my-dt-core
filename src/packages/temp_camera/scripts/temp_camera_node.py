#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import threading


def publisher(vehicle_name, device='/dev/video0', rate=10):
    topic = f'/{vehicle_name}/camera_node/image/compressed'
    pub = rospy.Publisher(topic, CompressedImage, queue_size=1)
    rospy.loginfo(f'[temp_camera] Opening video device: {device}')
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        rospy.logerr(f'[temp_camera] Failed to open video device: {device}')
    else:
        rospy.loginfo(f'[temp_camera] Video device opened successfully')

    # try MJPEG to reduce CPU
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    except Exception as e:
        rospy.logwarn(f'[temp_camera] Warning configuring capture properties: {e}')

    rate_hz = rospy.Rate(rate)
    published_first = False
    while not rospy.is_shutdown():
        if not cap.isOpened():
            # attempt to reopen periodically
            rospy.logwarn('[temp_camera] Capture not open, retrying in 1s')
            cap.open(device)
            rospy.sleep(1.0)
            continue

        try:
            ret, frame = cap.read()
        except Exception as e:
            rospy.logerr(f'[temp_camera] Exception reading frame: {e}')
            rospy.sleep(0.1)
            continue

        if not ret or frame is None:
            # no frame read
            rospy.logdebug('[temp_camera] No frame read, sleeping briefly')
            rospy.sleep(0.1)
            continue

        try:
            _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        except Exception as e:
            rospy.logerr(f'[temp_camera] Error encoding frame to JPEG: {e}')
            rospy.sleep(0.1)
            continue

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        pub.publish(msg)
        if not published_first:
            rospy.loginfo(f'[temp_camera] Published first frame to topic: {topic}')
            published_first = True
        rate_hz.sleep()


if __name__ == '__main__':
    rospy.init_node('temp_camera_node', anonymous=False, log_level=rospy.INFO)
    veh = rospy.get_param('~veh', 'blueduckie')
    rospy.loginfo(f'[temp_camera] Node started, veh={veh}')
    try:
        publisher(veh)
    except rospy.ROSInterruptException:
        pass

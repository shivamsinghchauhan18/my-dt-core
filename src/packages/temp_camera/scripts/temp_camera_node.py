#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import threading


def publisher(vehicle_name, device='/dev/video0', rate=10):
    topic = f'/{vehicle_name}/camera_node/image/compressed'
    pub = rospy.Publisher(topic, CompressedImage, queue_size=1)
    cap = cv2.VideoCapture(device)
    # try MJPEG to reduce CPU
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    rate_hz = rospy.Rate(rate)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.sleep(0.1)
            continue
        _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        pub.publish(msg)
        rate_hz.sleep()


if __name__ == '__main__':
    rospy.init_node('temp_camera_node')
    veh = rospy.get_param('~veh', 'blueduckie')
    try:
        publisher(veh)
    except rospy.ROSInterruptException:
        pass

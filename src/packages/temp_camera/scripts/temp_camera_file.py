#!/usr/bin/env python3
"""
Fake camera publisher that reads a file (video or single image) and publishes compressed images to the camera topic.
Usage: run inside ROS environment and set `~veh` param.
"""
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import os


def publisher_from_file(vehicle_name, path, loop=True, rate=10):
    topic = f'/{vehicle_name}/camera_node/image/compressed'
    pub = rospy.Publisher(topic, CompressedImage, queue_size=1)
    rospy.loginfo(f'[temp_camera_file] Publishing frames from {path} to {topic}')

    if os.path.isdir(path):
        # publish images alphabetically
        files = sorted([os.path.join(path, f) for f in os.listdir(path) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
        frames = []
        for f in files:
            img = cv2.imread(f)
            if img is not None:
                frames.append(img)
        if not frames:
            rospy.logerr('[temp_camera_file] No images found in directory')
            return
    else:
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            # try load as single image
            img = cv2.imread(path)
            if img is None:
                rospy.logerr(f'[temp_camera_file] Cannot open {path} as video or image')
                return
            frames = [img]
            cap = None
        else:
            frames = None

    rate_hz = rospy.Rate(rate)
    idx = 0
    while not rospy.is_shutdown():
        if frames is not None:
            img = frames[idx % len(frames)]
            idx += 1
        else:
            ret, frame = cap.read()
            if not ret:
                if loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                else:
                    break
            img = frame

        try:
            _, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        except Exception as e:
            rospy.logerr(f'[temp_camera_file] Error encoding frame: {e}')
            rate_hz.sleep()
            continue

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        pub.publish(msg)
        rate_hz.sleep()


if __name__ == '__main__':
    rospy.init_node('temp_camera_file', anonymous=False)
    veh = rospy.get_param('~veh', 'blueduckie')
    path = rospy.get_param('~file', '/data/sample_video.avi')
    rate = rospy.get_param('~rate', 10)
    loop = rospy.get_param('~loop', True)
    try:
        publisher_from_file(veh, path, loop=loop, rate=rate)
    except rospy.ROSInterruptException:
        pass

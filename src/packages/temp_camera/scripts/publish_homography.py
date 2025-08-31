#!/usr/bin/env python3
"""
Publish a 3x3 homography matrix for the vehicle camera.
Reads private param '~homography' (list of 9 numbers) or '~homography_file' (YAML) and publishes
to '/<veh>/camera_node/homography' as std_msgs/Float64MultiArray (row-major, length=9).

Usage examples:
  rosrun temp_camera publish_homography.py _veh:=blueduckie _homography:="[a,b,c,d,e,f,g,h,i]"
  rosrun temp_camera publish_homography.py _veh:=blueduckie _homography_file:=/path/to/homography.yaml
"""
import rospy
import yaml
import os
from std_msgs.msg import Float64MultiArray


def load_from_file(path):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    # expect list under key 'homography' or top-level list
    if isinstance(data, dict) and 'homography' in data:
        return data['homography']
    if isinstance(data, list):
        return data
    raise ValueError('YAML must contain a top-level list or a key "homography"')


def main():
    rospy.init_node('publish_homography', anonymous=False)
    veh = rospy.get_param('~veh', 'blueduckie')
    topic = f'/{veh}/camera_node/homography'
    homography = rospy.get_param('~homography', None)
    homography_file = rospy.get_param('~homography_file', None)

    if homography_file:
        try:
            homography = load_from_file(homography_file)
        except Exception as e:
            rospy.logerr(f'Failed to load homography file: {e}')
            return

    if homography is None:
        rospy.logerr('No homography provided. Set private param ~homography or ~homography_file')
        return

    if not isinstance(homography, (list, tuple)) or len(homography) != 9:
        rospy.logerr('Homography must be a list of 9 numbers (3x3 row-major)')
        return

    arr = Float64MultiArray()
    arr.data = [float(x) for x in homography]

    pub = rospy.Publisher(topic, Float64MultiArray, queue_size=1, latch=True)
    rospy.loginfo(f'Publishing homography to {topic}: {arr.data}')
    # publish once latched and exit (or keep publishing if desired)
    pub.publish(arr)
    rospy.loginfo('Homography published (latched).')
    rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rospy
from performance_optimizer import PerformanceOptimizer


def main():
    rospy.init_node('performance_optimizer')
    optimizer = PerformanceOptimizer('performance_optimizer')
    rospy.loginfo('[performance_optimizer_node] Node started')
    rospy.spin()


if __name__ == '__main__':
    main()

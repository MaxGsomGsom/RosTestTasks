#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


def pose_callback(msg):
    rospy.loginfo("Runner pose: x=%f, y=%f, z=%f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


def main():
    rospy.init_node('observer_node', anonymous=True)
    rate = rospy.Rate(10)
    pose_sub = rospy.Subscriber('pose', PoseStamped, pose_callback)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()

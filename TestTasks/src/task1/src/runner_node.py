#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('runner_node', anonymous=True)
    rate = rospy.Rate(10)
    pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)

    arg = 0  # main variable for calculation of coordinates
    speed = 1.5
    prev_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # get time delta
        cur_time = rospy.Time.now()
        delta_time = (cur_time - prev_time).nsecs / float(pow(10, 9))
        arg += delta_time * speed

        # calculate x,y,z coordinates from arg
        msg = PoseStamped()
        msg.header.stamp = cur_time
        msg.header.frame_id = "map"
        msg.pose.position.x = arg
        msg.pose.position.y = math.sin(arg)
        msg.pose.position.z = math.cos(arg)

        pose_pub.publish(msg)

        prev_time = cur_time
        rate.sleep()


if __name__ == '__main__':
    main()

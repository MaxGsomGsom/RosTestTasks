#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

interval = 0.1
poses = []
pose_array_pub = None
prev_time = None

def pose_callback(msg):
    global prev_time, poses, pose_array_pub
    # save one pose per interval
    cur_time = rospy.Time.now()
    delta_time = (cur_time - prev_time).nsecs / float(pow(10, 9))
    if delta_time < interval:
        return
    prev_time = cur_time

    # save pose to array
    poses.append(msg.pose)

    # create pose array message
    new_msg = PoseArray()
    new_msg.poses = poses
    new_msg.header.stamp = cur_time
    new_msg.header.frame_id = "map"
    pose_array_pub.publish(new_msg)

    rospy.loginfo("Runner pose: x=%f, y=%f, z=%f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


def main():
    global prev_time, poses, pose_array_pub
    rospy.init_node('observer_node', anonymous=True)
    rate = rospy.Rate(10)
    prev_time = rospy.Time.now()
    pose_sub = rospy.Subscriber('pose', PoseStamped, pose_callback)
    pose_array_pub = rospy.Publisher("pose_array", PoseArray, queue_size=10)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()

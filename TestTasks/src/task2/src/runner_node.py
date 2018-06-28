#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf import transformations
from geometry_msgs.msg import Quaternion


def main():
    rospy.init_node('runner_node', anonymous=True)
    rate = rospy.Rate(100)
    pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)

    # set targets chain
    targets_num = 5
    cur_target = 1
    traj = [[8, 7, 0], [-5, 8, 0], [-9, -4, 0], [6, -9, 0], [11, -1, 0]]

    # calculate yaw of targets
    traj[0][2] = math.atan2(traj[targets_num - 1][1] - traj[0][1], traj[targets_num - 1][0] - traj[0][0]) + math.pi;
    for i in range(1, targets_num):
        traj[i][2] = math.atan2(traj[i - 1][1] - traj[i][1], traj[i - 1][0] - traj[i][0]) + math.pi;

    # set initial position
    x = traj[0][0]
    y = traj[0][1]
    yaw = traj[0][2]

    # set speed
    speed_gain = 5
    speed_angular = math.pi / 2

    # indicates
    planar_dev = 0.1
    angular_dev = math.pi / 32

    prev_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # get time delta
        cur_time = rospy.Time.now()
        delta_time = (cur_time - prev_time).nsecs / float(pow(10, 9))

        # calculate speed
        dist_x = math.fabs(x - traj[cur_target][0])
        dist_y = math.fabs(y - traj[cur_target][1])
        if dist_x < dist_y:
            speed_y = speed_gain
            speed_x = dist_x / dist_y * speed_gain
        else:
            speed_x = speed_gain
            speed_y = dist_y / dist_x * speed_gain

        # calculate deltas
        sign_x = 1 if x < traj[cur_target][0] else -1
        delta_x = delta_time * speed_x * sign_x

        sign_y = 1 if y < traj[cur_target][1] else -1
        delta_y = delta_time * speed_y * sign_y

        # calculate angular delta
        dist_yaw = math.fabs(yaw - traj[cur_target][2])
        sign_yaw = 1 if yaw < traj[cur_target][2] else -1

        # if angular distance bigger than Pi
        if dist_yaw > math.pi:
            dist_yaw -= math.pi
            sign_yaw *= -1
        delta_yaw = delta_time * speed_angular * sign_yaw

        # rotate robot first
        if dist_yaw > angular_dev:
            yaw += delta_yaw
            if yaw > 2 * math.pi:
                yaw -= 2 * math.pi
            if yaw < -2 * math.pi:
                yaw += 2 * math.pi
        # add deltas to coordinates
        else:
            x += delta_x
            y += delta_y

        # check if target reached and go to new
        if dist_x < planar_dev and dist_y < planar_dev:
            cur_target += 1
        if cur_target == targets_num:
            cur_target -= targets_num

        # create message
        msg = PoseStamped()
        msg.header.stamp = cur_time
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0
        quat = transformations.quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = Quaternion(*quat)
        pose_pub.publish(msg)

        prev_time = cur_time
        rate.sleep()


if __name__ == '__main__':
    main()

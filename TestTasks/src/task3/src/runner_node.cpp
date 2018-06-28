#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <math.h>

using namespace ros;

int main(int argc, char **argv)
{
  init(argc, argv, "runner_node");
  NodeHandle nh;
  Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);
  Rate loop_rate(100);

  //error gain for x and y coordinates
  double error_gain = 0.05;

  //set targets chain
  int targets_num = 5;
  int cur_target = 1;
  double traj[targets_num][3] = {{8, 7, 0}, {-5, 8, 0}, {-9, -4, 0}, {6, -9, 0}, {11, -1, 0}}; //pentagon
  //calculate yaw of targets
  traj[0][2] = std::atan2(traj[targets_num - 1][1] - traj[0][1], traj[targets_num - 1][0] - traj[0][0]) + M_PI;
  for (int i = 1; i < targets_num; i++) {
    traj[i][2] = std::atan2(traj[i - 1][1] - traj[i][1], traj[i - 1][0] - traj[i][0]) + M_PI;
  }

  //set initial position
  double x = traj[0][0], y = traj[0][1], yaw = traj[0][2];

  //set speed
  double speed_gain = 5;
  double speed_angular = M_PI / 2;

  //indicates that target reached
  double planar_dev = 0.1;
  double angular_dev = M_PI / 32;

  Time prev_time = Time::now();
  while (ok()) {
    //get time delta
    Time cur_time = Time::now();
    double delta_time = (cur_time - prev_time).nsec / pow(10, 9);

    //calculate speed
    double dist_x = fabs(x - traj[cur_target][0]);
    double dist_y = fabs(y - traj[cur_target][1]);
    double speed_x, speed_y;
    if (dist_x < dist_y) {
      speed_y = speed_gain;
      speed_x = dist_x / dist_y * speed_gain;
    } else {
      speed_x = speed_gain;
      speed_y = dist_y / dist_x * speed_gain;
    }

    //calculate deltas
    double sign_x = x < traj[cur_target][0] ? 1 : -1;
    double delta_x = delta_time * speed_x * sign_x;

    double sign_y = y < traj[cur_target][1] ? 1 : -1;
    double delta_y = delta_time * speed_y * sign_y;

    //calculate angular delta
    double dist_yaw = fabs(yaw - traj[cur_target][2]);
    double sign_yaw = yaw < traj[cur_target][2] ? 1 : -1;
    //if angular distance bigger than Pi
    if (dist_yaw > M_PI) {
      dist_yaw -= M_PI;
      sign_yaw *= -1;
    }
    double delta_yaw = delta_time * speed_angular * sign_yaw;

    //rotate robot first
    if (dist_yaw > angular_dev) {
      yaw += delta_yaw;
      if (yaw > 2 * M_PI)
        yaw -= 2 * M_PI;
      if (yaw < -2 * M_PI)
        yaw += 2 * M_PI;
    } else {
      //add deltas to coordinates
      x += delta_x;
      y += delta_y;
    }

    //check if target reached and go to new
    if (dist_x < planar_dev && dist_y < planar_dev)
      cur_target++;
    if (cur_target == targets_num)
      cur_target -= targets_num;

    //create message
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = cur_time;
    msg.header.frame_id = "map";
    msg.pose.position.x = x + std::fmod(rand(), x * error_gain); //add error
    msg.pose.position.y = y + std::fmod(rand(), y * error_gain);
    msg.pose.position.z = 0;
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    pose_pub.publish(msg);
    prev_time = cur_time;

    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

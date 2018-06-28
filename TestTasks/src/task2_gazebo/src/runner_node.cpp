#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <vector>

using namespace ros;

double x = 0, y = 0, yaw = 0;

//read position of model from gazebo
void statesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "runner") {
      x = msg->pose[i].position.x;
      y = msg->pose[i].position.y;
      yaw = tf::getYaw(msg->pose[i].orientation);
      return;
    }
  }
}

int main(int argc, char **argv)
{
  init(argc, argv, "runner_node");
  NodeHandle nh;
  Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);
  Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  Subscriber states_sub = nh.subscribe("/gazebo/model_states", 1000, statesCallback);
  Rate loop_rate(100);

  //set targets chain
  int targets_num = 5;
  int cur_target = 0;
  double traj[targets_num][2] = { {8, 7}, {-5, 8}, {-9, -4}, {6, -9}, {11, -1}};

  //set speed per second
  double speed_linear = 3;
  double speed_angular = M_PI / 8;

  //indicate that target reached
  double planar_dev = 0.5;
  double angular_dev = M_PI / 16;

  //give time for model spawning
  sleep(5);

  while (ok()) {
    //calculate distance to current target
    double dist_linear = sqrt(pow(traj[cur_target][0] - x, 2) + pow(traj[cur_target][1] - y, 2));

    //calculate angular distance
    double dist_yaw_signed = std::atan2(traj[cur_target][1] - y, traj[cur_target][0] - x) - yaw;
    //if angular distance bigger than Pi, rotate to another direction
    if (dist_yaw_signed > M_PI) {
      dist_yaw_signed -= M_PI;
      dist_yaw_signed *= -1;
    } else if (dist_yaw_signed < -M_PI) {
      dist_yaw_signed += M_PI;
      dist_yaw_signed *= -1;
    }
    //get sign from angular distance
    double sign_yaw = dist_yaw_signed > 0 ? 1 : -1;

    //create twist message
    geometry_msgs::Twist twist_msg;

    //rotate robot first
    if (fabs(dist_yaw_signed) > angular_dev)
      twist_msg.angular.z = speed_angular * sign_yaw;
    //then go to target
    else
      twist_msg.linear.x = speed_linear;

    //publish twist message
    twist_pub.publish(twist_msg);

    //publish pose for observer
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0;
    msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pose_pub.publish(msg);

    //check if target reached and go to new
    if (dist_linear < planar_dev)
      cur_target++;
    if (cur_target == targets_num)
      cur_target -= targets_num;

    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

using namespace ros;

Publisher pose_array_pub;
std::vector<geometry_msgs::Pose> poses;
Time prev_time;
double interval = 0.1;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //save one pose per interval
  Time cur_time = Time::now();
  double delta_time = (cur_time - prev_time).nsec / pow(10, 9);
  if (delta_time<interval) return;
  prev_time = cur_time;

  //save pose to array
  geometry_msgs::PoseArray new_msg;
  poses.push_back(msg->pose);
  new_msg.poses = poses;
  new_msg.header.stamp = Time::now();
  new_msg.header.frame_id = "map";
  pose_array_pub.publish(new_msg);
}

int main(int argc, char **argv)
{
  init(argc, argv, "observer_node");
  NodeHandle nh;
  Subscriber pose_sub = nh.subscribe("pose", 1000, poseCallback);
  pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_array", 1000);

  Rate loop_rate(10);
  prev_time = Time::now();

  while (ok())
  {
    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

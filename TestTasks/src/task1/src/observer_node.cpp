#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

using namespace ros;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("Runner pose: x=%f, y=%f, z=%f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

int main(int argc, char **argv)
{
  init(argc, argv, "observer_node");
  NodeHandle nh;
  Subscriber pose_sub = nh.subscribe("pose", 1000, poseCallback);

  Rate loop_rate(10);

  while (ok())
  {
    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

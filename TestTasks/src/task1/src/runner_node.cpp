#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

using namespace ros;

int main(int argc, char **argv)
{
  init(argc, argv, "runner_node");
  NodeHandle nh;
  Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);
  Rate loop_rate(10);

  double x = 0;
  double speed = 1;
  Time prev_time = Time::now();

  while (ok()) {
    //get movement distanse from time delta
    Time cur_time = Time::now();
    double delta_time = (cur_time - prev_time).nsec / pow(10, 9);
    x += delta_time * speed;

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = cur_time;
    msg.header.frame_id = "map";

    //calculate y and z coordinates from x
    msg.pose.position.x = x;
    msg.pose.position.y = sin(x);
    msg.pose.position.z = cos(x);

    pose_pub.publish(msg);
    prev_time = cur_time;

    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

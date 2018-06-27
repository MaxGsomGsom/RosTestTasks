#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

using namespace ros;

int main(int argc, char **argv)
{
  init(argc, argv, "runner_node");
  NodeHandle nh;
  Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);
  Rate loop_rate(10);

  double arg = 0; //main variable for calculation of coordinates
  double speed = 1.5;
  Time prev_time = Time::now();

  while (ok()) {
    //get time delta
    Time cur_time = Time::now();
    double delta_time = (cur_time - prev_time).nsec / pow(10, 9);
    arg += delta_time * speed;

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = cur_time;
    msg.header.frame_id = "map";

    //calculate x,y,z coordinates from arg
    msg.pose.position.x = arg;
    msg.pose.position.y = sin(arg);
    msg.pose.position.z = cos(arg);

    pose_pub.publish(msg);
    prev_time = cur_time;

    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <chrono>

using namespace ros;

double rnd(double max)
{
  return std::fmod((rand() / 100.0), max) * pow(-1, rand());
}

double enbl()
{
  return rand() % 3 - 1;
}


int main(int argc, char **argv)
{
  init(argc, argv, "runner_node");
  NodeHandle nh("~");
  Rate loop_rate(10);
  srand(std::chrono::system_clock::now().time_since_epoch().count());

  //std::string robot_name = nh.param("name", "robot_" + std::to_string(rand()));
  Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);

  double max_pos = 10, max_gain = 2;

  double arg = 0;  //main variable for calculation of coordinates

  //initial position
  double x = rnd(max_pos),
         y = rnd(max_pos),
         z = rnd(max_pos);

  //speed
  double gain_x = rnd(max_gain),
         gain_y = rnd(max_gain),
         gain_z = rnd(max_gain);

  //enable or disable members of equation for coordinates
  double enbl1 = enbl(), enbl2 = enbl(),
         enbl3 = enbl(), enbl4 = enbl(),
         enbl5 = enbl(), enbl6 = enbl();

  Time prev_time = Time::now();

  while (ok()) {
    //get time delta
    Time cur_time = Time::now();
    double delta_time = (cur_time - prev_time).nsec / pow(10, 9);
    arg += delta_time;

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = cur_time;
    msg.header.frame_id = "map";

    //calculate x,y,z coordinates
    msg.pose.position.x = (sin(arg) * enbl1 + arg * enbl2) * gain_x;
    msg.pose.position.y = (sin(arg) * enbl3 + arg * enbl4) * gain_y;
    msg.pose.position.z = (sin(arg) * enbl5 + arg * enbl6) * gain_z;

    pose_pub.publish(msg);
    prev_time = cur_time;

    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

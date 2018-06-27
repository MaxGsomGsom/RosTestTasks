#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

using namespace ros;

//use neighbour poses to smooth current pose
std::vector<geometry_msgs::Pose> meanFilter(std::vector<geometry_msgs::Pose> data, int halfWindowSize = 2)
{
  std::vector<geometry_msgs::Pose> output(data);
  for (int i = halfWindowSize; i < data.size() - halfWindowSize; i++) {
    double sumX = data[i].position.x;
    double sumY = data[i].position.y;
    for (int k = 1; k <= halfWindowSize; k++) {
      sumX += data[i - k].position.x;
      sumX += data[i + k].position.x;
      sumY += data[i - k].position.y;
      sumY += data[i + k].position.y;
    }
    output[i].position.x = sumX / (halfWindowSize * 2 + 1);
    output[i].position.y = sumY / (halfWindowSize * 2 + 1);
  }
  return output;
}

Publisher pose_array_pub;
std::vector<geometry_msgs::Pose> poses;
Time prev_time;
double interval = 0.1;

//indicates that loop closed
double planar_dev = 0.25;
double loop_min_poses = 10;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  //save one pose per interval
  Time cur_time = Time::now();
  double delta_time = (cur_time - prev_time).nsec / pow(10, 9);
  if (delta_time < interval)
    return;
  prev_time = cur_time;

  //save pose to array
  poses.push_back(msg->pose);

  //check if loop finished, smooth it and publish
  if (poses.size() >= loop_min_poses &&
      std::fabs(poses[0].position.x - msg->pose.position.x) < planar_dev &&
      std::fabs(poses[0].position.y - msg->pose.position.y) < planar_dev) {

    //smooth trajectory
    auto smoothed = meanFilter(poses);
    poses.clear();

    //send message
    geometry_msgs::PoseArray new_msg;
    new_msg.poses = smoothed;
    new_msg.header.stamp = Time::now();
    new_msg.header.frame_id = "map";
    pose_array_pub.publish(new_msg);
  }
}

int main(int argc, char **argv)
{
  init(argc, argv, "observer_node");
  NodeHandle nh;
  Subscriber pose_sub = nh.subscribe("pose", 1000, poseCallback);
  pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_array", 1000);

  Rate loop_rate(10);
  prev_time = Time::now();

  while (ok()) {
    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <algorithm>

using namespace ros;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, std::string topic_name)
{
  ROS_INFO("%s: x=%f, y=%f, z=%f", topic_name.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

int main(int argc, char **argv)
{
  init(argc, argv, "observer_node");
  NodeHandle nh;
  //for saving all publishers and associated topics
  std::vector<Subscriber> pose_subs;
  std::vector<master::TopicInfo> known_topics;

  Rate loop_rate(1);

  while (ok()) {
    //get all topics
    master::V_TopicInfo topics;
    master::getTopics(topics);

    //find topics that publishes pose
    for (master::TopicInfo topic : topics) {
      if (topic.datatype == "geometry_msgs/PoseStamped") {
        //add subscriber for all new found topics
        if (std::find_if(known_topics.begin(), known_topics.end(),
        [&](const master::TopicInfo & t) { return t.name == topic.name; }) == known_topics.end()) {

          pose_subs.push_back(nh.subscribe<geometry_msgs::PoseStamped>(topic.name, 1, boost::bind(poseCallback, _1, topic.name)));
          known_topics.push_back(topic);
        }
      }
    }

    spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

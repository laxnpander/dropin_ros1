#pragma once

#include <shared_mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <opencv2/core.hpp>

namespace ai_ros
{

enum class TopicType
{
  IMAGE
};

struct Topic
{
  std::string name;
  TopicType   type;
};

class RosNode
{
public:
  explicit RosNode(
      const std::vector<Topic> &topics,
      double rate);

  RosNode(const RosNode &) = default;
  ~RosNode() = default;
  RosNode &operator=(const RosNode &) = default;

  bool isOkay() const;

protected:

  double rate_;

  ros::NodeHandle nh_;

  std::unordered_map<std::string, ros::Subscriber> subs_;

  void subImage(
      const sensor_msgs::Image &msg);

  void subImu(
      const sensor_msgs::Imu &msg);

  void subGnss(
      const sensor_msgs::NavSatFix &msg);

  static cv_bridge::CvImage toRos(
      const std_msgs::Header &header,
      const cv::Mat &cv_img);

  static cv::Mat fromRos(
      const sensor_msgs::Image &msg);
};

}
#include <string>

#include <ai_ros/ros_node.h>

#include <ros/package.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <opencv2/imgcodecs.hpp>

using namespace ai_ros;

RosNode::RosNode(
    const std::vector<Topic> &topics,
    double rate)
 : rate_(rate)
{
  for (const Topic& topic : topics)
  {
    switch(topic.type)
    {
      case TopicType::IMAGE:
      {
        break;
      }
      default:
        throw std::invalid_argument("Error loading topic: Topic type unknown.");
    }

    ROS_INFO_STREAM("Successfully created topic " << topic.name);
  }
}

bool RosNode::isOkay() const
{
  return nh_.ok();
}

cv_bridge::CvImage RosNode::toRos(const std_msgs::Header &header, const cv::Mat &cv_img)
{
  std::string encoding;
  switch (cv_img.type())
  {
    case CV_32FC1:
      encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      break;
    case CV_32FC3:
      encoding = sensor_msgs::image_encodings::TYPE_32FC3;
      break;
    case CV_64F:
      encoding = sensor_msgs::image_encodings::TYPE_64FC1;
      break;
    case CV_16UC1:
      encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      break;
    case CV_8UC1:
      encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      break;
    case CV_8UC3:
      encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      break;
    case CV_8UC4:
      encoding = sensor_msgs::image_encodings::TYPE_8UC4;
      break;
    default:
      throw std::out_of_range("Error convertig OpenCV Mat: Unknown data type.");
  }
  return {header, encoding, cv_img};
}

cv::Mat RosNode::fromRos(const sensor_msgs::Image &msg)
{
  cv_bridge::CvImagePtr img_ptr;

  try
  {
    img_ptr = cv_bridge::toCvCopy(msg);
  }
  catch(...)
  {
    throw(cv_bridge::Exception("Error converting compressed image!"));
  }

  return img_ptr->image;
}
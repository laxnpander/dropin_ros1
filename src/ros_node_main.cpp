#include <ai_ros/ros_node.h>

using namespace ai_ros;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spear_ros");

  std::vector<Topic> topics {
      { "/camera/image_raw", TopicType::IMAGE }
  };

  RosNode node(topics, 30.0);

  while (node.isOkay())
  {
    ros::spinOnce();
  }

  ros::shutdown();

  return 0;
}

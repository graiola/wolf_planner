#include <ros/init.h>
#include <ros/package.h>

#include "ocs2_legged_robot_ros/gait/GaitKeyboardPublisher.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char* argv[])
{
  // Initialize ros node
  ros::init(argc, argv, "wolf_gait_node");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string gaitCommandFile;
  const std::string topicPrefix = "wolf_planner";
  nodeHandle.getParam(topicPrefix+"/gaitCommandFile", gaitCommandFile);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

  GaitKeyboardPublisher gaitCommand(nodeHandle, gaitCommandFile, topicPrefix, true);

  while (ros::ok() && ros::master::check()) {
    gaitCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}

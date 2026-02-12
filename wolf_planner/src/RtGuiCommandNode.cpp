/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#include "wolf_planner/RtGuiGaitPublisher.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "wolf_planner_rt_gui_node");
  ros::NodeHandle nh;

  // Get node parameters
  std::string gaitCommandFile;
  const std::string topicPrefix = "wolf_planner";
  nh.getParam(topicPrefix+"/gaitCommandFile", gaitCommandFile);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

  try {
    wolf::RtGuiGaitPublisher guiNode(nh, gaitCommandFile, topicPrefix);
    guiNode.run();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Exception caught: " << e.what());
    return 1;
  }

  return 0;
}

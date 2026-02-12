/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <ros/ros.h>
#include <ocs2_msgs/mode_schedule.h>

#include <string>
#include <vector>
#include <map>

#include <ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h>

namespace wolf {

class RtGuiGaitPublisher {
public:
  /**
   * Constructor
   * @param nh ROS node handle.
   * @param gaitFile Path to the gait configuration file.
   * @param robotName Robot namespace for ROS topics.
   * @param rateHz Loop frequency for checking GUI updates.
   */
  RtGuiGaitPublisher(ros::NodeHandle& nh, const std::string& gaitFile, const std::string& robotName, double rateHz = 10.0);

  /**
   * Main loop that synchronizes with RtGui and publishes gait commands.
   */
  void run();

private:
  // ROS
  ros::Publisher modeSequenceTemplatePublisher_;

  // Gait configuration
  std::vector<std::string> gaitList_;
  std::map<std::string, ocs2::legged_robot::ModeSequenceTemplate> gaitMap_;

  // GUI control
  std::string selectedGait_;
  std::string lastGait_;

  // Runtime control
  double loopRateHz_;
};

}  // namespace wolf

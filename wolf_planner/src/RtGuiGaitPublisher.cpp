/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#include "wolf_planner/RtGuiGaitPublisher.h"

#include <rt_gui_ros/rt_gui_client.h>
#include <ocs2_core/misc/LoadData.h>

#include <algorithm>

namespace wolf {

RtGuiGaitPublisher::RtGuiGaitPublisher(ros::NodeHandle& nh, const std::string& gaitFile, const std::string& robotName, double rateHz)
    : loopRateHz_(rateHz) {
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");

  // Load gait list from file
  bool verbose = true;
  ocs2::loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  if (gaitList_.empty()) {
    ROS_ERROR("No gaits found in configuration file.");
    throw std::runtime_error("Gait list is empty.");
  }

  // Load each gait into the map
  for (const auto& gaitName : gaitList_) {
    gaitMap_.emplace(gaitName, ocs2::legged_robot::loadModeSequenceTemplate(gaitFile, gaitName, verbose));
  }

  selectedGait_ = gaitList_.front();
  lastGait_ = selectedGait_;

  // Setup publisher
  modeSequenceTemplatePublisher_ = nh.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

  // Setup GUI
  rt_gui::RtGuiClient::getIstance().init("/wolf_rviz","wolf_planner_gui");
  rt_gui::RtGuiClient::getIstance().addList("gait", "select", gaitList_, &selectedGait_);

  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule GUI node is ready.");
}

void RtGuiGaitPublisher::run() {
  ros::Rate rate(loopRateHz_);

  while (ros::ok()) {
    rt_gui::RtGuiClient::getIstance().sync();

    if (selectedGait_ != lastGait_) {
      std::string gaitKey = selectedGait_;
      std::transform(gaitKey.begin(), gaitKey.end(), gaitKey.begin(), ::tolower);

      try {
        const auto& modeSequence = gaitMap_.at(gaitKey);
        modeSequenceTemplatePublisher_.publish(ocs2::legged_robot::createModeSequenceTemplateMsg(modeSequence));
        ROS_INFO_STREAM("Published gait: " << gaitKey);
        lastGait_ = selectedGait_;
      } catch (const std::out_of_range&) {
        ROS_WARN_STREAM("Gait \"" << gaitKey << "\" not found in gait map.");
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}

}  // namespace wolf

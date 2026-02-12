/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <wolf_planner_interface/PlannerInterface.h>

#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

#include "wolf_planner_interface/visualization/LeggedSelfCollisionVisualization.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class AdaptivePlanner : public PlannerInterface {

public:

  AdaptivePlanner() {};

  virtual ~AdaptivePlanner();

  virtual void updateVisualization(const SystemObservation& observation) override;

protected:

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose) override;

  virtual void setupSynchronizedModules() override;

  virtual void setupVisualization() override;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;

};

}  // namespace wolf_planner

PLUGINLIB_EXPORT_CLASS(wolf_planner::AdaptivePlanner, wolf_planner::PlannerInterface)

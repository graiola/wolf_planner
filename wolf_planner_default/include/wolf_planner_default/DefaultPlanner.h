/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

// OCS2
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

#include "wolf_planner_interface/PlannerInterface.h"
#include "wolf_planner_interface/visualization/LeggedSelfCollisionVisualization.h"
#include "wolf_planner_interface/LeggedReferenceManager.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class DefaultPlanner : public PlannerInterface {

public:

  DefaultPlanner() {};

  virtual ~DefaultPlanner();

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

PLUGINLIB_EXPORT_CLASS(wolf_planner::DefaultPlanner, wolf_planner::PlannerInterface)


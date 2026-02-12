/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <wolf_msgs/TerrainEstimation.h>

#include "wolf_planner_adaptive/TerrainEstimator.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class TerrainEstimationReceiver : public SolverSynchronizedModule {
 public:
  TerrainEstimationReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<TerrainEstimator> TerrainEstimatorPtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void terrainEstimationCallback(const wolf_msgs::TerrainEstimation::ConstPtr& msg);

  std::shared_ptr<TerrainEstimator> ptr_;

  ros::Subscriber subscriber_;

  std::mutex mtx_;
  std::atomic_bool updated_;

  Eigen::Vector3d terrainNormal_;
  Eigen::Vector3d terrainCenter_;
};

}  // namespace wolf_planner

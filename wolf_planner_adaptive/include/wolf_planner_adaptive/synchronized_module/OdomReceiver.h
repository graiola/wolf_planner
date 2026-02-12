/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <nav_msgs/Odometry.h>

#include "wolf_planner_adaptive/OdomEstimator.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class OdomReceiver : public SolverSynchronizedModule {
 public:
  OdomReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<OdomEstimator> OdomEstimatorPtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  std::shared_ptr<OdomEstimator> ptr_;

  ros::Subscriber subscriber_;

  std::mutex mtx_;
  std::atomic_bool updated_;

  vector3_t base_linear_vel_;
  vector3_t base_angular_vel_;
};

}  // namespace wolf_planner

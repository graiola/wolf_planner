/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <wolf_msgs/ContactForces.h>

#include "wolf_planner_adaptive/ContactForcesEstimator.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class ContactForcesReceiver : public SolverSynchronizedModule {
 public:
  ContactForcesReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<ContactForcesEstimator> ContactForcesEstimatorPtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void contactForcesCallback(const wolf_msgs::ContactForces::ConstPtr& msg);

  std::shared_ptr<ContactForcesEstimator> ptr_;

  ros::Subscriber subscriber_;

  std::mutex mtx_;
  std::atomic_bool updated_;

  std::vector<vector3_t> contact_forces_;
  std::vector<bool> contact_states_;
  std::vector<std::string> contact_names_;
};

}  // namespace wolf_planner

/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

// ROS
#include <ros/ros.h>

// OCS2
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

#include "wolf_planner_interface/SafetyChecker.h"
#include "wolf_planner_interface/LeggedInterface.h"

#define WORLD_FRAME_NAME "odom"
#define RBDL_CONTROL_FRAME "world"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class PlannerInterface {

public:

  PlannerInterface() {};

  virtual ~PlannerInterface();

  bool setup(ros::NodeHandle& nodeHandle, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose = true);

  void setupMrt();

  void starting(SystemObservation& observation);

  void stopping();

  bool isRunning() { return plannerRunning_; };

  bool updatePolicy(SystemObservation& observation);

  virtual void updateVisualization(const SystemObservation& observation) = 0;

  std::shared_ptr<LeggedInterface> getLeggedInterface() const { return leggedInterface_; }

  std::shared_ptr<MPC_BASE> getMpc() const { return mpc_; }

  const std::vector<vector3_t>& getDesiredContactForces() const { return mpcDesContactForces_; }

  const std::vector<vector3_t>& getDesiredFootPositions() const { return mpcDesFootPositions_; }

  const std::vector<vector3_t>& getDesiredFootVelocities() const { return mpcDesFootVelocities_; }

  const Eigen::Quaterniond& getDesiredBaseQuaternion() const { return mpcDesBaseQuat_; }

  const vector3_t& getDesiredBasePosition() const { return mpcDesBasePosition_; }

  const vector3_t& getDesiredBaseLinearVelocity() const { return mpcDesBaseLinearVelocity_; }

  const vector3_t& getDesiredBaseAngularVelocity() const { return mpcDesBaseAngularVelocity_; }

  const vector_t& getDesiredJointVelocities() const { return mpcDesJointVelocities_; }

  const vector_t& getDesiredJointPositions() const { return mpcDesJointPositions_; }

  const std::vector<std::string>& getJointNames() const { return jointNames_; }

  void setTopicPrefix(const std::string& topicPrefix) { topicPrefix_ = topicPrefix; }

  void setFramePrefix(const std::string& framePrefix) { framePrefix_ = framePrefix; }

  void setRobotName(const std::string& robotName) { robotName_ = robotName; }

  void setRobotBaseName(const std::string& robotBaseName) { robotBaseName_ = robotBaseName; }

  benchmark::RepeatedTimer mpcTimer_;

  std::atomic_bool threadRunning_{false};

  std::thread mpcThread_;

  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

protected:

  void setupPinocchioKinematics();

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose) = 0;

  virtual void setupSynchronizedModules() = 0;

  virtual void setupVisualization() = 0;

  ros::NodeHandle nodeHandle_;

  std::string topicPrefix_;

  std::string framePrefix_;

  std::string robotName_;

  std::string robotBaseName_;

  // Pinocchio
  std::shared_ptr<CentroidalModelPinocchioMapping> pinocchioMapping_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  // Safety checker
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // MPC BASE
  std::shared_ptr<MPC_BASE> mpc_;

  // Legged interface
  std::shared_ptr<LeggedInterface> leggedInterface_;

  // Desired Contact Forces
  std::vector<vector3_t> mpcDesContactForces_;

  // Desired Foot positions and velocities
  std::vector<vector3_t> mpcDesFootPositions_;
  std::vector<vector3_t> mpcDesFootVelocities_;

  // Desired joint positions
  vector_t mpcDesJointPositions_;

  // Desired joint velocities
  vector_t mpcDesJointVelocities_;

  // Desired base quaternion
  Eigen::Quaterniond mpcDesBaseQuat_;

  // Desired base position
  vector3_t mpcDesBasePosition_;

  // Desired base linear velocity
  vector3_t mpcDesBaseLinearVelocity_;

  // Desired base angular velocity
  vector3_t mpcDesBaseAngularVelocity_;

  // Joint names
  std::vector<std::string> jointNames_;

  // Running planner flag
  std::atomic_bool plannerRunning_{false};

  // Observation time offset
  double timeOffset_;
};

}  // namespace wolf_planner

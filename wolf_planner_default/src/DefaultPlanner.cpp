/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_sqp/SqpMpc.h>

#include "wolf_planner_default/DefaultPlanner.h"

#include <ocs2_sqp/SqpMpc.h>

namespace wolf_planner
{

DefaultPlanner::~DefaultPlanner()
{

}

void DefaultPlanner::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  // Legged interface
  leggedInterface_ = std::make_shared<LeggedInterface>(robotName_, taskFile, urdfFile, referenceFile, verbose);

  // Optimal control problem
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  // MPC
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());

}

void DefaultPlanner::setupSynchronizedModules()
{

  auto gaitReceiver = std::make_shared<GaitReceiver>(nodeHandle_, leggedInterface_->getLeggedReferenceManagerPtr()->getGaitSchedule(), topicPrefix_);

  // ROS ReferenceManager
  auto rosReferenceManager = std::make_shared<RosReferenceManager>(topicPrefix_, leggedInterface_->getReferenceManagerPtr());

  rosReferenceManager->subscribe(nodeHandle_);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiver);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManager);
}

void DefaultPlanner::setupVisualization()
{
  const auto visualizationFramePrefix = framePrefix_.empty() ? topicPrefix_ : framePrefix_;
  ros::NodeHandle visualizationNodeHandle(topicPrefix_);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                      leggedInterface_->getCentroidalModelInfo(), *eeKinematics_, visualizationNodeHandle, visualizationFramePrefix);

  robotVisualizer_->frameId_ =  visualizationFramePrefix + "/" + WORLD_FRAME_NAME;
  robotVisualizer_->baseName_ = robotBaseName_;

  // Self collision visualizer
  selfCollisionVisualization_ = std::make_shared<LeggedSelfCollisionVisualization>(leggedInterface_->getPinocchioInterface(),
                                                                                   leggedInterface_->getGeometryInterface(), *pinocchioMapping_, visualizationNodeHandle, visualizationFramePrefix);
}

void DefaultPlanner::updateVisualization(const SystemObservation &observation)
{
  // Visualization
  if(robotVisualizer_ != nullptr)
    robotVisualizer_->update(observation, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  if(selfCollisionVisualization_ != nullptr)
    selfCollisionVisualization_->update(observation);
}

} // namespace wolf_planner

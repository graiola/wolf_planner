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

#include "wolf_planner_perceptive/PerceptivePlanner.h"
#include "wolf_planner_perceptive/synchronized_module/PlanarTerrainReceiver.h"
#include "wolf_planner_perceptive/PerceptivePlannerRobotInterface.h"
#include "wolf_planner_perceptive/PerceptivePlannerReferenceManager.h"

#include <ocs2_sqp/SqpMpc.h>

namespace wolf_planner
{

PerceptivePlanner::~PerceptivePlanner()
{

}

void PerceptivePlanner::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  // Legged interface
  leggedInterface_ = std::make_shared<PerceptivePlannerRobotInterface>(taskFile, urdfFile, referenceFile, verbose);

  // Optimal control problem
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  // MPC
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
}

void PerceptivePlanner::setupSynchronizedModules()
{

  auto gaitReceiver = std::make_shared<GaitReceiver>(nodeHandle_, leggedInterface_->getLeggedReferenceManagerPtr()->getGaitSchedule(), topicPrefix_);

  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nodeHandle_,
                                              dynamic_cast<PerceptivePlannerRobotInterface&>(*leggedInterface_).getPlanarTerrainPtr(),
                                              dynamic_cast<PerceptivePlannerRobotInterface&>(*leggedInterface_).getSignedDistanceFieldPtr(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation");

  // ROS ReferenceManager
  auto rosReferenceManager = std::make_shared<RosReferenceManager>(topicPrefix_, leggedInterface_->getReferenceManagerPtr());

  rosReferenceManager->subscribe(nodeHandle_);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiver);
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManager);
}

void PerceptivePlanner::setupVisualization()
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
  // Foot placement visualizer
  footPlacementVisualization_ = std::make_shared<FootPlacementVisualization>(
       *dynamic_cast<PerceptivePlannerReferenceManager&>(*leggedInterface_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
       leggedInterface_->getCentroidalModelInfo().numThreeDofContacts, visualizationNodeHandle,
       visualizationFramePrefix + "/" + WORLD_FRAME_NAME);

   // Sphere visualizer
   sphereVisualization_ = std::make_shared<SphereVisualization>(
       leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
       *dynamic_cast<PerceptivePlannerRobotInterface&>(*leggedInterface_).getPinocchioSphereInterfacePtr(), visualizationNodeHandle,
       visualizationFramePrefix + "/" + WORLD_FRAME_NAME);
}

void PerceptivePlanner::updateVisualization(const SystemObservation& observation)
{
  // Visualization
  if(robotVisualizer_ != nullptr)
    robotVisualizer_->update(observation, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  if(selfCollisionVisualization_ != nullptr)
    selfCollisionVisualization_->update(observation);
  if(footPlacementVisualization_ != nullptr)
    footPlacementVisualization_->update(observation);
  if(sphereVisualization_ != nullptr)
    sphereVisualization_->update(observation);
}

} // namespace wolf_planner

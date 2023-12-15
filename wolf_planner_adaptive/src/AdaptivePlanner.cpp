#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_sqp/SqpMpc.h>

#include "wolf_planner_adaptive/AdaptivePlanner.h"
#include "wolf_planner_adaptive/AdaptivePlannerRobotInterface.h"
#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"
#include "wolf_planner_adaptive/synchronizer/TerrainEstimationReceiver.h"

namespace wolf_planner
{

AdaptivePlanner::~AdaptivePlanner()
{

}

void AdaptivePlanner::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  // Legged interface
  leggedInterface_ = std::make_shared<AdaptivePlannerRobotInterface>(taskFile, urdfFile, referenceFile, verbose);

  // Optimal control problem
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  // MPC
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
}

void AdaptivePlanner::setupSynchronizedModules()
{

  auto gaitReceiver = std::make_shared<GaitReceiver>(nodeHandle_, leggedInterface_->getLeggedReferenceManagerPtr()->getGaitSchedule(), topicPrefix_);

  // Terrain estimation receiver
  auto terrainEstimationReceiver = std::make_shared<TerrainEstimationReceiver>(nodeHandle_,
                                                                               std::dynamic_pointer_cast<AdaptivePlannerReferenceManager>(leggedInterface_->getLeggedReferenceManagerPtr())->getTerrainEstimator(),
                                                                               robotName_);

  // ROS ReferenceManager
  auto rosReferenceManager = std::make_shared<RosReferenceManager>(topicPrefix_, leggedInterface_->getReferenceManagerPtr());

  rosReferenceManager->subscribe(nodeHandle_);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiver);
  mpc_->getSolverPtr()->addSynchronizedModule(terrainEstimationReceiver);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManager);
}

void AdaptivePlanner::setupVisualization()
{
  ros::NodeHandle mpcNodeHandle(topicPrefix_);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                      leggedInterface_->getCentroidalModelInfo(), *eeKinematics_, mpcNodeHandle, topicPrefix_);

  robotVisualizer_->frameId_ =  topicPrefix_+"/"+WORLD_FRAME_NAME;
  robotVisualizer_->baseName_ = robotBaseName_;

  // Self collision visualizer
  selfCollisionVisualization_ = std::make_shared<LeggedSelfCollisionVisualization>(leggedInterface_->getPinocchioInterface(),
                                                                                   leggedInterface_->getGeometryInterface(), *pinocchioMapping_, mpcNodeHandle, topicPrefix_);
}

void AdaptivePlanner::updateVisualization(const SystemObservation &observation)
{
  // Visualization
  if(robotVisualizer_ != nullptr)
    robotVisualizer_->update(observation, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  if(selfCollisionVisualization_ != nullptr)
    selfCollisionVisualization_->update(observation);
}

} // namespace wolf_planner

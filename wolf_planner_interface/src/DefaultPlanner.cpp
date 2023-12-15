#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "wolf_planner_interface/DefaultPlanner.h"

#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <angles/angles.h>

#include <wolf_controller_utils/geometry.h>

namespace wolf_planner
{

DefaultPlanner::~DefaultPlanner()
{

}

void DefaultPlanner::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  // Legged interface
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile, verbose);

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
  ros::NodeHandle mpcNodeHandle(topicPrefix_);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                      leggedInterface_->getCentroidalModelInfo(), *eeKinematics_, mpcNodeHandle, topicPrefix_);

  robotVisualizer_->frameId_ =  topicPrefix_+"/"+WORLD_FRAME_NAME;
  robotVisualizer_->baseName_ = robotBaseName_;

  // Self collision visualizer
  selfCollisionVisualization_ = std::make_shared<LeggedSelfCollisionVisualization>(leggedInterface_->getPinocchioInterface(),
                                                                                   leggedInterface_->getGeometryInterface(), *pinocchioMapping_, mpcNodeHandle, topicPrefix_);
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

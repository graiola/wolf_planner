#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "wolf_planner_adaptive/AdaptivePlanner.h"
#include "wolf_planner_adaptive/AdaptivePlannerRobotInterface.h"
#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"
#include "wolf_planner_adaptive/synchronizer/TerrainEstimationReceiver.h"

namespace wolf_planner
{

AdaptivePlanner::AdaptivePlanner(ros::NodeHandle& nodeHandle, const std::string &topicPrefix, const std::string& robotName, const std::string &robotBaseName)
  :DefaultPlanner(nodeHandle,topicPrefix,robotName,robotBaseName)
{

}

void AdaptivePlanner::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  // Legged interface
  leggedInterface_ = std::make_shared<AdaptivePlannerRobotInterface>(taskFile, urdfFile, referenceFile, verbose);

  // Optimal control problem
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void AdaptivePlanner::setupSynchronizedModules(ros::NodeHandle &nodeHandle, const std::string topicPrefix)
{

  auto gaitReceiver = std::make_shared<GaitReceiver>(nodeHandle, leggedInterface_->getLeggedReferenceManagerPtr()->getGaitSchedule(), topicPrefix);

  // Terrain estimation receiver
  auto terrainEstimationReceiver = std::make_shared<TerrainEstimationReceiver>(nodeHandle,
                                                                               std::dynamic_pointer_cast<AdaptivePlannerReferenceManager>(leggedInterface_->getLeggedReferenceManagerPtr())->getTerrainEstimator(),
                                                                               robotName_);

  // ROS ReferenceManager
  auto rosReferenceManager = std::make_shared<RosReferenceManager>(topicPrefix, leggedInterface_->getReferenceManagerPtr());

  rosReferenceManager->subscribe(nodeHandle);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiver);
  mpc_->getSolverPtr()->addSynchronizedModule(terrainEstimationReceiver);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManager);
}

} // namespace wolf_planner

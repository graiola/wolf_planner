#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "wolf_planner_adaptive/AdaptivePlanner.h"

namespace wolf_planner
{

AdaptivePlanner::AdaptivePlanner(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose)
  :PlannerInterface(taskFile,urdfFile,referenceFile,verbose)
{

}

void AdaptivePlanner::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  // Legged interface
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile, verbose);

  // Optimal control problem
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void AdaptivePlanner::setupSynchronizedModules(ros::NodeHandle &nodeHandle, const std::string topicPrefix)
{

  auto gaitReceiver = std::make_shared<GaitReceiver>(nodeHandle, leggedInterface_->getLeggedReferenceManagerPtr()->getGaitSchedule(), topicPrefix);

  // Terrain estimation receiver
  //auto terrainEstimationReceiverPtr = std::make_shared<TerrainEstimationReceiver>(nodeHandle, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getTerrainEstimator(), robotName);

  // ROS ReferenceManager
  auto rosReferenceManager = std::make_shared<RosReferenceManager>(topicPrefix, leggedInterface_->getReferenceManagerPtr());

  rosReferenceManager->subscribe(nodeHandle);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiver);
  //mpc_->getSolverPtr()->addSynchronizedModule(terrainEstimationReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManager);
}

void AdaptivePlanner::setupVisualization(ros::NodeHandle &nodeHandle, const std::string robotBaseName, const std::string& topicPrefix)
{
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematics_, nodeHandle, topicPrefix);
  robotVisualizer_->frameId_ =  topicPrefix+"/"+WORLD_FRAME_NAME;
  robotVisualizer_->baseName_ = robotBaseName;

  // Self collision visualizer
  selfCollisionVisualization_ = std::make_shared<LeggedSelfCollisionVisualization>(leggedInterface_->getPinocchioInterface(),
                                                                                   leggedInterface_->getGeometryInterface(), *pinocchioMapping_, nodeHandle, topicPrefix);
}

void AdaptivePlanner::starting(SystemObservation &observation)
{

  timeOffset_ = observation.time;

  TargetTrajectories targetTrajectories({observation.time}, {observation.state}, {observation.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(observation);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(targetTrajectories);
  ROS_INFO_STREAM("[AdaptivePlanner] Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("[AdaptivePlanner] Initial policy has been received.");

  ROS_INFO_STREAM("[AdaptivePlanner] Starting the planner");

  plannerRunning_ = true;
}

void AdaptivePlanner::stopping()
{
  ROS_INFO_STREAM("[AdaptivePlanner] Stopping the planner");

  plannerRunning_ = false;
}

bool AdaptivePlanner::isRunning()
{
  return plannerRunning_;
}

bool AdaptivePlanner::updatePolicy(SystemObservation &observation)
{
  // Remove time offset
  observation.time = observation.time - timeOffset_;

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(observation);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at (NOTE: this is modified by the planner)
  mpcMrtInterface_->evaluatePolicy(observation.time, observation.state, optimizedState, optimizedInput, plannedMode);

  // This is the input for the WBC (NOTE: we don't use it right now, instead we publish with specific topics for WoLF)
  observation.input = optimizedInput;

  // Safety check, if failed, stop the planner
  if (!safetyChecker_->check(observation, optimizedState, optimizedInput))
  {
    ROS_ERROR_STREAM("[AdaptivePlanner] Safety check failed, stopping the planner.");
    threadRunning_ = false;
    return false;
  }

  // Update pinocchio
  const auto& mpcModel = leggedInterface_->getPinocchioInterface().getModel();
  auto& mpcData = leggedInterface_->getPinocchioInterface().getData();
  pinocchio::forwardKinematics(mpcModel, mpcData, centroidal_model::getGeneralizedCoordinates(optimizedState, leggedInterface_->getCentroidalModelInfo()));
  pinocchio::computeJointJacobians(mpcModel, mpcData);
  pinocchio::updateFramePlacements(mpcModel, mpcData);

  // Update desired values
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  pinocchioMapping.setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  mpcDesJointPositions_ = pinocchioMapping.getPinocchioJointPosition(optimizedState);
  pinocchio::forwardKinematics(mpcModel, mpcData, mpcDesJointPositions_);
  pinocchio::computeJointJacobians(mpcModel, mpcData, mpcDesJointPositions_);
  pinocchio::updateFramePlacements(mpcModel, mpcData);
  ocs2::updateCentroidalDynamics(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), mpcDesJointPositions_);
  mpcDesJointVelocities_ = pinocchioMapping.getPinocchioJointVelocity(optimizedState, optimizedInput);
  pinocchio::forwardKinematics(mpcModel, mpcData, mpcDesJointPositions_, mpcDesJointVelocities_);

  // Retrieve MPC optimized output
  //const auto& mpcPosDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  //const auto& mpcVelDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
  //const auto& mpcBasePosDesEul = centroidal_model::getBasePose(optimizedState, leggedInterface_->getCentroidalModelInfo());

  mpcDesBasePosition_ << mpcDesJointPositions_(0), mpcDesJointPositions_(1), mpcDesJointPositions_(2);

  mpcDesBaseVelocity_ << mpcDesJointVelocities_(0), mpcDesJointVelocities_(1), mpcDesJointVelocities_(2);

  // ZYX conversion to quat
  wolf_controller_utils::rpyToQuat(mpcDesJointPositions_(5),mpcDesJointPositions_(4),mpcDesJointPositions_(3),mpcDesBaseQuat_);
  mpcDesBaseQuat_.normalize();

  // std::vector<size_t> contactIds = leggedInterface_->getCentroidalModelInfo().endEffectorFrameIndices;
  // Absolute ids not required. Ids are referred to leggedInterface_->getCentroidalModelInfo().numThreeDofContacts
  mpcDesContactForces_[0] = centroidal_model::getContactForces(optimizedInput, 0, leggedInterface_->getCentroidalModelInfo()); // LF
  mpcDesContactForces_[1] = centroidal_model::getContactForces(optimizedInput, 1, leggedInterface_->getCentroidalModelInfo()); // LH
  mpcDesContactForces_[2] = centroidal_model::getContactForces(optimizedInput, 2, leggedInterface_->getCentroidalModelInfo()); // RF
  mpcDesContactForces_[3] = centroidal_model::getContactForces(optimizedInput, 3, leggedInterface_->getCentroidalModelInfo()); // RH

  eeKinematics_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  mpcDesFootPositions_ = eeKinematics_->getPosition(optimizedState);
  mpcDesFootVelocities_ = eeKinematics_->getVelocity(optimizedState, optimizedInput);

  return true;
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

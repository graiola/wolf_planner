#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "wolf_planner/WolfMpc.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <angles/angles.h>

// Uncomment this macro to run the planner openloop i.e. by integrating its own solution over time
//#define OPENLOOP

namespace wolf_planner
{

WolfMpc::~WolfMpc()
{
  plannerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
}

bool WolfMpc::init()
{

  ros::NodeHandle nodeHandle;
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  std::string robotName;
  std::string robotModel;
  std::string topicPrefix = "wolf_planner";
  std::vector<std::string> robotFootNames;
  std::string robotBaseName;
  nodeHandle.getParam(topicPrefix+"/robotName",  robotName);
  nodeHandle.getParam(topicPrefix+"/robotModel", robotModel);
  nodeHandle.getParam(topicPrefix+"/urdfFile", urdfFile);
  nodeHandle.getParam(topicPrefix+"/taskFile", taskFile);
  nodeHandle.getParam(topicPrefix+"/referenceFile", referenceFile);
  bool verbose = true;
  loadData::loadCppDataType(taskFile, "wolf_planner_interface.verbose", verbose);

  // Wait for the controller to start
  ROS_INFO("[WoLFMpc] waiting for WoLF controller to start...");
  while (!nodeHandle.hasParam(robotName+"/wolf_controller/robot_foot_names") && ros::ok())
    ros::Rate(1).sleep();
  nodeHandle.getParam(robotName+"/wolf_controller/robot_foot_names", robotFootNames);
  if(robotFootNames.empty())
  {
    ROS_ERROR("[WolfMpc] robot foot names is empty!");
    return false;
  }
  while (!nodeHandle.hasParam(robotName+"/wolf_controller/robot_base_name") && ros::ok())
    ros::Rate(1).sleep();
  nodeHandle.getParam(robotName+"/wolf_controller/robot_base_name", robotBaseName);
  if(robotBaseName.empty())
  {
    ROS_ERROR("[WolfMpc] robot base name is empty!");
    return false;
  }

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);

  // Initialize the observation data structure
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.time = 0.0;
  currentObservation_.mode = ModeNumber::STANCE;
  callbackObservation_ = currentObservation_;

  // MPC
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  // Gait receiver
  auto gaitReceiverPtr = std::make_shared<GaitReceiver>(nodeHandle, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), topicPrefix);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(topicPrefix, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Setup the MPC thread loop
  setupMrt();

  // Pinocchio EE Kinematics
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

  // Robot visualizer
  ros::NodeHandle mpcNodeHandle(topicPrefix);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, mpcNodeHandle, topicPrefix);
  robotVisualizer_->frameId_ =  topicPrefix+"/world";
  robotVisualizer_->baseName_ = robotBaseName;

  // Self collision visualizer
  selfCollisionVisualization_ = std::make_shared<LeggedSelfCollisionVisualization>(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, mpcNodeHandle, topicPrefix);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  ROS_INFO_STREAM("[WolfMpc] Robot model is: "<< robotModel);
  ROS_INFO_STREAM("[WolfMpc] Robot name is: "<< robotName);
  ROS_INFO_STREAM("[WolfMpc] Robot base name is: "<< robotBaseName);
  auto jointNames = leggedInterface_->getPinocchioInterface().getModel().names;
  for(unsigned int i=0;i<jointNames.size();i++)
    ROS_INFO_STREAM("[WolfMpc] Loading joint["<<i<<"]: "<<jointNames[i]);
  ROS_INFO_STREAM("[WolfMpc] WoLF planner period is: "<< 1.0/leggedInterface_->mpcSettings().mpcDesiredFrequency_);

  // Observation used by the target node
  observationPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_observation>(topicPrefix + "/mpc_observation", 1);

  // MPC publishers (FIXME hardcoded)
  mpcWrenchPublisher_lf_  = nodeHandle.advertise<wolf_msgs::Wrench>   (robotName+"/wolf_controller/reference/lf_foot_wrench", 1);
  mpcFootPublisher_lf_    = nodeHandle.advertise<wolf_msgs::Cartesian>(robotName+"/wolf_controller/reference/lf_foot",   1);

  mpcWrenchPublisher_lh_  = nodeHandle.advertise<wolf_msgs::Wrench>   (robotName+"/wolf_controller/reference/lh_foot_wrench", 1);
  mpcFootPublisher_lh_    = nodeHandle.advertise<wolf_msgs::Cartesian>(robotName+"/wolf_controller/reference/lh_foot",   1);

  mpcWrenchPublisher_rf_  = nodeHandle.advertise<wolf_msgs::Wrench>   (robotName+"/wolf_controller/reference/rf_foot_wrench", 1);
  mpcFootPublisher_rf_    = nodeHandle.advertise<wolf_msgs::Cartesian>(robotName+"/wolf_controller/reference/rf_foot",   1);

  mpcWrenchPublisher_rh_  = nodeHandle.advertise<wolf_msgs::Wrench>   (robotName+"/wolf_controller/reference/rh_foot_wrench", 1);
  mpcFootPublisher_rh_    = nodeHandle.advertise<wolf_msgs::Cartesian>(robotName+"/wolf_controller/reference/rh_foot",   1);

  mpcBasePublisher_       = nodeHandle.advertise<wolf_msgs::Cartesian>(robotName+"/wolf_controller/reference/waist",     1);

  mpcPosturalPublisher_   = nodeHandle.advertise<wolf_msgs::Postural> (robotName+"/wolf_controller/reference/postural",  1);

  // MPC subscribers (FIXME hardcoded)
  mpcObservation_         = nodeHandle.subscribe(robotName+"/wolf_controller/mpc_observation",  1, &WolfMpc::observationCallback, this);
  controllerState_        = nodeHandle.subscribe(robotName+"/wolf_controller/controller_state", 1, &WolfMpc::controllerStateCallback, this);

  return true;
}

void WolfMpc::starting()
{

  currentObservation_ = callbackObservation_;

  TargetTrajectories target_trajectories({callbackObservation_.time}, {callbackObservation_.state}, {callbackObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(callbackObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("[WolfMpc] Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("[WolfMpc] Initial policy has been received.");

  ROS_INFO_STREAM("[WolfMpc] Starting the planner");

  mpcRunning_ = true;
}

void WolfMpc::stopping()
{
  ROS_INFO_STREAM("[WolfMpc] Stopping the planner");

  mpcRunning_ = false;
}

void WolfMpc::setupMrt()
{
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  plannerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (plannerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        plannerRunning_ = false;
        ROS_ERROR_STREAM("[WolfMpc] MPC error: " << e.what());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void WolfMpc::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose)
{
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void WolfMpc::updatePolicyAndPublish(SystemObservation& observation)
{

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
#ifdef OPENLOOP
  observation.state = optimizedState;
  observation.time += 0.001;
#endif

  // Safety check, if failed, stop the planner
  if (!safetyChecker_->check(observation, optimizedState, optimizedInput))
  {
    ROS_ERROR_STREAM("[WolfMpc] Safety check failed, stopping the planner.");
    plannerRunning_ = false;
    return;
  }

  // Update pinocchio
  const auto& mpc_model = leggedInterface_->getPinocchioInterface().getModel();
  auto& mpc_data = leggedInterface_->getPinocchioInterface().getData();
  pinocchio::forwardKinematics(mpc_model, mpc_data, centroidal_model::getGeneralizedCoordinates(optimizedState, leggedInterface_->getCentroidalModelInfo()));
  pinocchio::computeJointJacobians(mpc_model, mpc_data);
  pinocchio::updateFramePlacements(mpc_model, mpc_data);

  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  pinocchioMapping.setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  const auto& qDesired = pinocchioMapping.getPinocchioJointPosition(optimizedState);
  ocs2::updateCentroidalDynamics(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), qDesired);
  const auto& vDesired = pinocchioMapping.getPinocchioJointVelocity(optimizedState, optimizedInput);

  // Retrieve MPC optimized output
  vector_t mpc_posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_basePosDes_eul = centroidal_model::getBasePose(optimizedState, leggedInterface_->getCentroidalModelInfo());

  Eigen::Quaterniond mpc_base_quat;
  mpc_base_quat = Eigen::AngleAxisd(mpc_basePosDes_eul(3), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(mpc_basePosDes_eul(4), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(mpc_basePosDes_eul(5), Eigen::Vector3d::UnitX());

  // std::vector<size_t> contactIds = leggedInterface_->getCentroidalModelInfo().endEffectorFrameIndices;
  // Absolute ids not required. Ids are referred to leggedInterface_->getCentroidalModelInfo().numThreeDofContacts
  vector_t mpc_contactDes_lf = centroidal_model::getContactForces(optimizedInput, 0, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes_lh = centroidal_model::getContactForces(optimizedInput, 1, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes_rf = centroidal_model::getContactForces(optimizedInput, 2, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes_rh = centroidal_model::getContactForces(optimizedInput, 3, leggedInterface_->getCentroidalModelInfo());

  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  std::vector<vector3_t> mpc_foot_pos = eeKinematicsPtr_->getPosition(optimizedState);
  std::vector<vector3_t> mpc_foot_vel = eeKinematicsPtr_->getVelocity(optimizedState, optimizedInput);

  //std::cout << "**********************" << std::endl;
  //for(unsigned int i=0; i<qDesired.size(); i++)
  //  std::cout << qDesired(i) << std::endl;

  // Pack messages
  wolf_msgs::Wrench force_msg_lf, force_msg_lh, force_msg_rf, force_msg_rh;
  force_msg_lf.wrench.force.x = mpc_contactDes_lf(0); // LF
  force_msg_lf.wrench.force.y = mpc_contactDes_lf(1); // LF
  force_msg_lf.wrench.force.z = mpc_contactDes_lf(2); // LF
  force_msg_lh.wrench.force.x = mpc_contactDes_lh(0); // LH
  force_msg_lh.wrench.force.y = mpc_contactDes_lh(1); // LH
  force_msg_lh.wrench.force.z = mpc_contactDes_lh(2); // LH
  force_msg_rf.wrench.force.x = mpc_contactDes_rf(0); // RF
  force_msg_rf.wrench.force.y = mpc_contactDes_rf(1); // RF
  force_msg_rf.wrench.force.z = mpc_contactDes_rf(2); // RF
  force_msg_rh.wrench.force.x = mpc_contactDes_rh(0); // RH
  force_msg_rh.wrench.force.y = mpc_contactDes_rh(1); // RH
  force_msg_rh.wrench.force.z = mpc_contactDes_rh(2); // RH

  wolf_msgs::Cartesian foot_msg_lf, foot_msg_lh, foot_msg_rf, foot_msg_rh;
  foot_msg_lf.pose.position.x = mpc_foot_pos[0](0);
  foot_msg_lf.pose.position.y = mpc_foot_pos[0](1);
  foot_msg_lf.pose.position.z = mpc_foot_pos[0](2);
  foot_msg_lf.twist.linear.x = mpc_foot_vel[0](0);
  foot_msg_lf.twist.linear.y = mpc_foot_vel[0](1);
  foot_msg_lf.twist.linear.z = mpc_foot_vel[0](2);
  foot_msg_lh.pose.position.x = mpc_foot_pos[1](0);
  foot_msg_lh.pose.position.y = mpc_foot_pos[1](1);
  foot_msg_lh.pose.position.z = mpc_foot_pos[1](2);
  foot_msg_lh.twist.linear.x = mpc_foot_vel[1](0);
  foot_msg_lh.twist.linear.y = mpc_foot_vel[1](1);
  foot_msg_lh.twist.linear.z = mpc_foot_vel[1](2);
  foot_msg_rf.pose.position.x = mpc_foot_pos[2](0);
  foot_msg_rf.pose.position.y = mpc_foot_pos[2](1);
  foot_msg_rf.pose.position.z = mpc_foot_pos[2](2);
  foot_msg_rf.twist.linear.x = mpc_foot_vel[2](0);
  foot_msg_rf.twist.linear.y = mpc_foot_vel[2](1);
  foot_msg_rf.twist.linear.z = mpc_foot_vel[2](2);
  foot_msg_rh.pose.position.x = mpc_foot_pos[3](0);
  foot_msg_rh.pose.position.y = mpc_foot_pos[3](1);
  foot_msg_rh.pose.position.z = mpc_foot_pos[3](2);
  foot_msg_rh.twist.linear.x = mpc_foot_vel[3](0);
  foot_msg_rh.twist.linear.y = mpc_foot_vel[3](1);
  foot_msg_rh.twist.linear.z = mpc_foot_vel[3](2);

  wolf_msgs::Cartesian base_msg;
  base_msg.pose.position.x = mpc_basePosDes_eul(0);
  base_msg.pose.position.y = mpc_basePosDes_eul(1);
  base_msg.pose.position.z = mpc_basePosDes_eul(2);
  base_msg.pose.orientation.w = mpc_base_quat.w();
  base_msg.pose.orientation.x = mpc_base_quat.x();
  base_msg.pose.orientation.y = mpc_base_quat.y();
  base_msg.pose.orientation.z = mpc_base_quat.z();

  base_msg.twist.linear.x =  vDesired(0);
  base_msg.twist.linear.y =  vDesired(1);
  base_msg.twist.linear.z =  vDesired(2);
  base_msg.twist.angular.z = vDesired(3);
  base_msg.twist.angular.y = vDesired(4);
  base_msg.twist.angular.x = vDesired(5);

  wolf_msgs::Postural postural_msg;
  for (size_t i = 0; i < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++i)
  {
    postural_msg.positions.push_back(mpc_posDes(i));
    postural_msg.velocities.push_back(mpc_velDes(i));
  }

  // Publish the MPC output
  mpcWrenchPublisher_lf_.publish(force_msg_lf);
  mpcFootPublisher_lf_.publish(foot_msg_lf);
  mpcWrenchPublisher_lh_.publish(force_msg_lh);
  mpcFootPublisher_lh_.publish(foot_msg_lh);
  mpcWrenchPublisher_rf_.publish(force_msg_rf);
  mpcFootPublisher_rf_.publish(foot_msg_rf);
  mpcWrenchPublisher_rh_.publish(force_msg_rh);
  mpcFootPublisher_rh_.publish(foot_msg_rh);
  mpcBasePublisher_.publish(base_msg);
  mpcPosturalPublisher_.publish(postural_msg);

  // Visualization
  if(robotVisualizer_ != nullptr)
    robotVisualizer_->update(observation, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  if(selfCollisionVisualization_ != nullptr)
    selfCollisionVisualization_->update(observation);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(observation));
}

void WolfMpc::observationCallback(const ocs2_msgs::mpc_observationConstPtr& msg)
{

   callbackObservation_.time = msg->time;
   callbackObservation_.mode = msg->mode;

   for (size_t i = 0; i < leggedInterface_->getCentroidalModelInfo().stateDim; ++i)
     callbackObservation_.state(i) = msg->state.value[i];

  // Update the current state of the system
  if(mpcRunning_)
  {
#ifdef OPENLOOP
    updatePolicyAndPublish(currentObservation_);
#else
    updatePolicyAndPublish(callbackObservation_);
#endif
  }
}

void WolfMpc::controllerStateCallback(const wolf_msgs::ControllerStateConstPtr& msg)
{
  if(msg->current_state == "ACTIVE")
  {
    if(!mpcRunning_) starting();
  }
  else
  {
    if(mpcRunning_) stopping();
  }
}

} // namespace wolf_planner

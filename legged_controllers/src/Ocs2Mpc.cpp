#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/Ocs2Mpc.h"

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

namespace legged {

bool MpcClass::init(ros::NodeHandle& mpc_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  std::string robot_name;
  mpc_nh.getParam("/robot_name", robot_name);
  mpc_nh.getParam("/urdfFile", urdfFile);
  mpc_nh.getParam("/taskFile", taskFile);
  mpc_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();

  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  robotVisualizer_->frameId_ = "world";

  // MPC publishers (FIXME hardcoded)
  mpcWrenchPublisher_lf_  = nh.advertise<wolf_msgs::Wrench>   (robot_name+"/wolf_controller/reference/lf_wrench", 1);
  mpcFootPublisher_lf_    = nh.advertise<wolf_msgs::Cartesian>(robot_name+"/wolf_controller/reference/lf_foot",   1);

  mpcWrenchPublisher_lh_  = nh.advertise<wolf_msgs::Wrench>   (robot_name+"/wolf_controller/reference/lh_wrench", 1);
  mpcFootPublisher_lh_    = nh.advertise<wolf_msgs::Cartesian>(robot_name+"/wolf_controller/reference/lh_foot",   1);

  mpcWrenchPublisher_rf_  = nh.advertise<wolf_msgs::Wrench>   (robot_name+"/wolf_controller/reference/rf_wrench", 1);
  mpcFootPublisher_rf_    = nh.advertise<wolf_msgs::Cartesian>(robot_name+"/wolf_controller/reference/rf_foot",   1);

  mpcWrenchPublisher_rh_  = nh.advertise<wolf_msgs::Wrench>   (robot_name+"/wolf_controller/reference/rh_wrench", 1);
  mpcFootPublisher_rh_    = nh.advertise<wolf_msgs::Cartesian>(robot_name+"/wolf_controller/reference/rh_foot",   1);

  mpcBasePublisher_       = nh.advertise<wolf_msgs::Cartesian>(robot_name+"/wolf_controller/reference/waist",     1);

  mpcPosturalPublisher_   = nh.advertise<wolf_msgs::Postural> (robot_name+"/wolf_controller/reference/postural",  1);

  // MPC subscriber (FIXME hardcoded)
  mpcObservation_         = nh.subscribe(robot_name+"/wolf_controller/mpc_observation", 1, &MpcClass::observationCallback, this);

  return true;
}

void MpcClass::starting() {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  currentObservation_.time += ros::Duration(0.002).toSec();
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void MpcClass::update() {

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
    ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
  }
}

void MpcClass::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void MpcClass::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());

  const std::string robotName = "legged_robot";
  // FIXME (hardcoded names)
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void MpcClass::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();
}

void MpcClass::retrieveAndPublish(){

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Retrieve MPC optimized output
  vector_t mpc_posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_basePosDes_eul = centroidal_model::getBasePose(optimizedState, leggedInterface_->getCentroidalModelInfo());

  // Conversion from ZYX euler to quaternion
  // Abbreviations for the various angular functions
  double cr = cos(mpc_basePosDes_eul(5) * 0.5);
  double sr = sin(mpc_basePosDes_eul(5) * 0.5);
  double cp = cos(mpc_basePosDes_eul(4) * 0.5);
  double sp = sin(mpc_basePosDes_eul(4) * 0.5);
  double cy = cos(mpc_basePosDes_eul(3) * 0.5);
  double sy = sin(mpc_basePosDes_eul(3) * 0.5);

  Eigen::Quaterniond mpc_base_quat;
  mpc_base_quat.w() = cr * cp * cy + sr * sp * sy;
  mpc_base_quat.x() = sr * cp * cy - cr * sp * sy;
  mpc_base_quat.y() = cr * sp * cy + sr * cp * sy;
  mpc_base_quat.z() = cr * cp * sy - sr * sp * cy;

//  std::vector<size_t> contactIds = leggedInterface_->getCentroidalModelInfo().endEffectorFrameIndices;
  // Absolute ids not required. Ids are referred to leggedInterface_->getCentroidalModelInfo().numThreeDofContacts
  vector_t mpc_contactDes_lf = centroidal_model::getContactForces(optimizedInput, 0, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes_lh = centroidal_model::getContactForces(optimizedInput, 1, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes_rf = centroidal_model::getContactForces(optimizedInput, 2, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes_rh = centroidal_model::getContactForces(optimizedInput, 3, leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  const auto& mpc_model = leggedInterface_->getPinocchioInterface().getModel();
  auto& mpc_data = leggedInterface_->getPinocchioInterface().getData();
  pinocchio::forwardKinematics(mpc_model, mpc_data, centroidal_model::getGeneralizedCoordinates(optimizedState, leggedInterface_->getCentroidalModelInfo()));
  pinocchio::updateFramePlacements(mpc_model, mpc_data);
  std::vector<vector3_t> mpc_foot_pos = eeKinematicsPtr_->getPosition(optimizedState);
  std::vector<vector3_t> mpc_foot_vel = eeKinematicsPtr_->getVelocity(optimizedState, optimizedInput);

  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  pinocchioMapping.setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  const auto qDesired = pinocchioMapping.getPinocchioJointPosition(optimizedState);
  ocs2::updateCentroidalDynamics(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), qDesired);
  const vector_t vDesired = pinocchioMapping.getPinocchioJointVelocity(optimizedState, optimizedInput);

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
  foot_msg_lf.twist.linear.x = mpc_foot_vel[0](1);
  foot_msg_lf.twist.linear.y = mpc_foot_vel[0](2);
  foot_msg_lf.twist.linear.z = mpc_foot_vel[0](3);
  foot_msg_lh.pose.position.x = mpc_foot_pos[1](0);
  foot_msg_lh.pose.position.y = mpc_foot_pos[1](1);
  foot_msg_lh.pose.position.z = mpc_foot_pos[1](2);
  foot_msg_lh.twist.linear.x = mpc_foot_vel[1](1);
  foot_msg_lh.twist.linear.y = mpc_foot_vel[1](2);
  foot_msg_lh.twist.linear.z = mpc_foot_vel[1](3);
  foot_msg_rf.pose.position.x = mpc_foot_pos[2](0);
  foot_msg_rf.pose.position.y = mpc_foot_pos[2](1);
  foot_msg_rf.pose.position.z = mpc_foot_pos[2](2);
  foot_msg_rf.twist.linear.x = mpc_foot_vel[2](1);
  foot_msg_rf.twist.linear.y = mpc_foot_vel[2](2);
  foot_msg_rf.twist.linear.z = mpc_foot_vel[2](3);
  foot_msg_rh.pose.position.x = mpc_foot_pos[3](0);
  foot_msg_rh.pose.position.y = mpc_foot_pos[3](1);
  foot_msg_rh.pose.position.z = mpc_foot_pos[3](2);
  foot_msg_rh.twist.linear.x = mpc_foot_vel[3](1);
  foot_msg_rh.twist.linear.y = mpc_foot_vel[3](2);
  foot_msg_rh.twist.linear.z = mpc_foot_vel[3](3);

  wolf_msgs::Cartesian base_msg;
  base_msg.pose.position.x = mpc_basePosDes_eul(0);
  base_msg.pose.position.y = mpc_basePosDes_eul(1);
  base_msg.pose.position.z = mpc_basePosDes_eul(2);
  base_msg.pose.orientation.w = mpc_base_quat.w();
  base_msg.pose.orientation.x = mpc_base_quat.x();
  base_msg.pose.orientation.y = mpc_base_quat.y();
  base_msg.pose.orientation.z = mpc_base_quat.z();

  base_msg.twist.linear.x = vDesired(0);
  base_msg.twist.linear.y = vDesired(1);
  base_msg.twist.linear.z = vDesired(2);
  base_msg.twist.angular.z = vDesired(3);
  base_msg.twist.angular.y = vDesired(4);
  base_msg.twist.angular.x = vDesired(5);

  wolf_msgs::Postural postural_msg;
  for (size_t i = 0; i < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++i) {
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
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void MpcClass::observationCallback(const ocs2_msgs::mpc_observationConstPtr& msg){

  currentObservation_.time = msg->time;
  currentObservation_.state.setZero();
  currentObservation_.input.setZero();

  for (size_t i = 0; i < leggedInterface_->getCentroidalModelInfo().stateDim; ++i)
    currentObservation_.state(i) = msg->state.value[i];
  for (size_t i = 0; i < leggedInterface_->getCentroidalModelInfo().inputDim; ++i)
    currentObservation_.input(i) = msg->input.value[i];

  currentObservation_.mode = msg->mode;

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
}

} // namespace legged

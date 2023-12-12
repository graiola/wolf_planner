#include "wolf_planner/WolfMpc.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#ifdef PERCEPTIVE_INTERFACE
  #include <wolf_planner_perceptive_interface/PerceptivePlanner.h>
#endif

namespace wolf_planner
{

bool WolfMpc::init()
{
  ros::NodeHandle nodeHandle; // robotNamespace
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
  while (!nodeHandle.hasParam("/"+robotName+"/wolf_controller/robot_foot_names") && ros::ok())
    ros::Rate(1).sleep();
  nodeHandle.getParam("/"+robotName+"/wolf_controller/robot_foot_names", robotFootNames);
  if(robotFootNames.empty())
  {
    ROS_ERROR("[WolfMpc] robot foot names is empty!");
    return false;
  }
  while (!nodeHandle.hasParam("/"+robotName+"/wolf_controller/robot_base_name") && ros::ok())
    ros::Rate(1).sleep();
  nodeHandle.getParam("/"+robotName+"/wolf_controller/robot_base_name", robotBaseName);
  if(robotBaseName.empty())
  {
    ROS_ERROR("[WolfMpc] robot base name is empty!");
    return false;
  }

  // Initialize the planner
  // FIXME create a factory
  planner_ = std::make_shared<PlannerInterface>(taskFile,urdfFile,referenceFile,verbose);

  planner_->setupMrt();

  planner_->setupPinocchioKinematics();

  ROS_INFO_STREAM("[WolfMpc] Robot model is: "<< robotModel);
  ROS_INFO_STREAM("[WolfMpc] Robot name is: "<< robotName);
  ROS_INFO_STREAM("[WolfMpc] Robot base name is: "<< robotBaseName);
  auto jointNames = planner_->getJointNames();
  for(unsigned int i=0;i<jointNames.size();i++)
    ROS_INFO_STREAM("[WolfMpc] Loading joint["<<i<<"]: "<<jointNames[i]);
  ROS_INFO_STREAM("[WolfMpc] WoLF planner period is: "<< 1.0/planner_->getLeggedInterface()->mpcSettings().mpcDesiredFrequency_);

  ros::NodeHandle mpcNodeHandle(topicPrefix);
  planner_->setupVisualization(mpcNodeHandle,robotBaseName,topicPrefix);

  planner_->setupSynchronizedModules(nodeHandle,topicPrefix);

  // Initialize the observation data structure
  observation_.state.setZero(planner_->getLeggedInterface()->getCentroidalModelInfo().stateDim);
  observation_.input.setZero(planner_->getLeggedInterface()->getCentroidalModelInfo().inputDim);
  observation_.time = 0.0;
  observation_.mode = ModeNumber::STANCE;

  // Observation used by the target node
  observationPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_observation>(topicPrefix + "/mpc_observation", 1);

  // MPC publishers (FIXME hardcoded, export to a config file)
  mpcWrenchPublisher_lf_  = nodeHandle.advertise<wolf_msgs::Wrench>   ("/"+robotName+"/wolf_controller/reference/lf_foot_wrench", 1);
  mpcFootPublisher_lf_    = nodeHandle.advertise<wolf_msgs::Cartesian>("/"+robotName+"/wolf_controller/reference/lf_foot",   1);

  mpcWrenchPublisher_lh_  = nodeHandle.advertise<wolf_msgs::Wrench>   ("/"+robotName+"/wolf_controller/reference/lh_foot_wrench", 1);
  mpcFootPublisher_lh_    = nodeHandle.advertise<wolf_msgs::Cartesian>("/"+robotName+"/wolf_controller/reference/lh_foot",   1);

  mpcWrenchPublisher_rf_  = nodeHandle.advertise<wolf_msgs::Wrench>   ("/"+robotName+"/wolf_controller/reference/rf_foot_wrench", 1);
  mpcFootPublisher_rf_    = nodeHandle.advertise<wolf_msgs::Cartesian>("/"+robotName+"/wolf_controller/reference/rf_foot",   1);

  mpcWrenchPublisher_rh_  = nodeHandle.advertise<wolf_msgs::Wrench>   ("/"+robotName+"/wolf_controller/reference/rh_foot_wrench", 1);
  mpcFootPublisher_rh_    = nodeHandle.advertise<wolf_msgs::Cartesian>("/"+robotName+"/wolf_controller/reference/rh_foot",   1);

  mpcBasePublisher_       = nodeHandle.advertise<wolf_msgs::Cartesian>("/"+robotName+"/wolf_controller/reference/waist",     1);

  mpcPosturalPublisher_   = nodeHandle.advertise<wolf_msgs::Postural> ("/"+robotName+"/wolf_controller/reference/postural",  1);

  // MPC subscribers (FIXME hardcoded, export to a config file)
  mpcObservation_         = nodeHandle.subscribe("/"+robotName+"/wolf_controller/mpc_observation",   1, &WolfMpc::observationCallback, this);
  controllerState_        = nodeHandle.subscribe("/"+robotName+"/wolf_controller/controller_state",  1, &WolfMpc::controllerStateCallback, this);

  return true;
}

void WolfMpc::updatePolicyAndPublish()
{

  // Update the current state of the system
  if(!planner_->updatePolicy(observation_))
  {
    mpcObservation_.shutdown();
    controllerState_.shutdown();
    return;
  }

  const auto& desiredContactForces = planner_->getDesiredContactForces();
  const auto& desiredFootPositions = planner_->getDesiredFootPositions();
  const auto& desiredFootVelocities = planner_->getDesiredFootVelocities();
  const auto& desiredBaseQuaternion = planner_->getDesiredBaseQuaternion();
  const auto& desiredBasePosition = planner_->getDesiredBasePosition();
  const auto& desiredBaseVelocity = planner_->getDesiredBaseVelocity();
  const auto& desiredJointPositions = planner_->getDesiredJointPositions();
  const auto& desiredJointVelocities = planner_->getDesiredJointVelocities();

  // Pack messages
  wolf_msgs::Wrench force_msg_lf, force_msg_lh, force_msg_rf, force_msg_rh;
  force_msg_lf.header.frame_id = force_msg_lh.header.frame_id = force_msg_rf.header.frame_id = force_msg_rh.header.frame_id = WORLD_FRAME_NAME;
  force_msg_lf.wrench.force.x = desiredContactForces[0](0); // LF
  force_msg_lf.wrench.force.y = desiredContactForces[0](1); // LF
  force_msg_lf.wrench.force.z = desiredContactForces[0](2); // LF
  force_msg_lh.wrench.force.x = desiredContactForces[1](0); // LH
  force_msg_lh.wrench.force.y = desiredContactForces[1](1); // LH
  force_msg_lh.wrench.force.z = desiredContactForces[1](2); // LH
  force_msg_rf.wrench.force.x = desiredContactForces[2](0); // RF
  force_msg_rf.wrench.force.y = desiredContactForces[2](1); // RF
  force_msg_rf.wrench.force.z = desiredContactForces[2](2); // RF
  force_msg_rh.wrench.force.x = desiredContactForces[3](0); // RH
  force_msg_rh.wrench.force.y = desiredContactForces[3](1); // RH
  force_msg_rh.wrench.force.z = desiredContactForces[3](2); // RH

  wolf_msgs::Cartesian foot_msg_lf, foot_msg_lh, foot_msg_rf, foot_msg_rh;
  foot_msg_lf.header.frame_id = foot_msg_lh.header.frame_id = foot_msg_rf.header.frame_id = foot_msg_rh.header.frame_id = WORLD_FRAME_NAME;
  foot_msg_lf.pose.position.x = desiredFootPositions[0](0);
  foot_msg_lf.pose.position.y = desiredFootPositions[0](1);
  foot_msg_lf.pose.position.z = desiredFootPositions[0](2);
  foot_msg_lf.twist.linear.x = desiredFootVelocities[0](0);
  foot_msg_lf.twist.linear.y = desiredFootVelocities[0](1);
  foot_msg_lf.twist.linear.z = desiredFootVelocities[0](2);
  foot_msg_lh.pose.position.x = desiredFootPositions[1](0);
  foot_msg_lh.pose.position.y = desiredFootPositions[1](1);
  foot_msg_lh.pose.position.z = desiredFootPositions[1](2);
  foot_msg_lh.twist.linear.x = desiredFootVelocities[1](0);
  foot_msg_lh.twist.linear.y = desiredFootVelocities[1](1);
  foot_msg_lh.twist.linear.z = desiredFootVelocities[1](2);
  foot_msg_rf.pose.position.x = desiredFootPositions[2](0);
  foot_msg_rf.pose.position.y = desiredFootPositions[2](1);
  foot_msg_rf.pose.position.z = desiredFootPositions[2](2);
  foot_msg_rf.twist.linear.x = desiredFootVelocities[2](0);
  foot_msg_rf.twist.linear.y = desiredFootVelocities[2](1);
  foot_msg_rf.twist.linear.z = desiredFootVelocities[2](2);
  foot_msg_rh.pose.position.x = desiredFootPositions[3](0);
  foot_msg_rh.pose.position.y = desiredFootPositions[3](1);
  foot_msg_rh.pose.position.z = desiredFootPositions[3](2);
  foot_msg_rh.twist.linear.x = desiredFootVelocities[3](0);
  foot_msg_rh.twist.linear.y = desiredFootVelocities[3](1);
  foot_msg_rh.twist.linear.z = desiredFootVelocities[3](2);

  wolf_msgs::Cartesian base_msg;
  base_msg.header.frame_id = WORLD_FRAME_NAME;
  base_msg.pose.position.x = desiredBasePosition(0);
  base_msg.pose.position.y = desiredBasePosition(1);
  base_msg.pose.position.z = desiredBasePosition(2);
  base_msg.pose.orientation.w = desiredBaseQuaternion.w();
  base_msg.pose.orientation.x = desiredBaseQuaternion.x();
  base_msg.pose.orientation.y = desiredBaseQuaternion.y();
  base_msg.pose.orientation.z = desiredBaseQuaternion.z();
  base_msg.twist.linear.x =  desiredBaseVelocity(0);
  base_msg.twist.linear.y =  desiredBaseVelocity(1);
  base_msg.twist.linear.z =  desiredBaseVelocity(2);
  // FIXME the angular velocities are coupled once the robot rotates more than 180
  //base_msg.twist.angular.z = vDesired(3);
  //base_msg.twist.angular.y = vDesired(4);
  //base_msg.twist.angular.x = vDesired(5);

  wolf_msgs::Postural postural_msg;
  for (size_t i = 0; i < planner_->getLeggedInterface()->getCentroidalModelInfo().actuatedDofNum; ++i)
  {
    postural_msg.positions.push_back(desiredJointPositions(i));
    postural_msg.velocities.push_back(desiredJointVelocities(i));
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

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(observation_));

  // Visualization
  planner_->updateVisualization(observation_);
}

void WolfMpc::observationCallback(const ocs2_msgs::mpc_observationConstPtr& msg)
{
   // Create the observation from the ROS message
   observation_.time = msg->time;
   observation_.mode = msg->mode;
   for (size_t i = 0; i < planner_->getLeggedInterface()->getCentroidalModelInfo().stateDim; ++i)
     observation_.state(i) = msg->state.value[i];

  // Start/Stop the mpc
  if(controllerRunning_ && !planner_->isRunning())
  {
    planner_->starting(observation_);
  }
  else if (!controllerRunning_ && planner_->isRunning())
    planner_->stopping();

  // Update the current state of the system
  if(planner_->isRunning())
    updatePolicyAndPublish();
}

void WolfMpc::controllerStateCallback(const wolf_msgs::ControllerStateConstPtr& msg)
{
  if(msg->current_state == "ACTIVE")
    controllerRunning_ = true;
  else
    controllerRunning_ = false;
}

} // namespace wolf_planner

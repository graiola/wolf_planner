#include "wolf_planner/WolfPlannerRos.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace wolf_planner
{

bool WolfPlannerRos::init()
{
  ros::NodeHandle nodeHandle; // robotNamespace
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  std::string robotName;
  std::string robotModel;
  std::string topicPrefix = "wolf_planner";
  std::string plannerType;
  std::vector<std::string> robotFootNames;
  std::string robotBaseName;

  nodeHandle.getParam(topicPrefix+"/robotName",   robotName);
  nodeHandle.getParam(topicPrefix+"/robotModel",  robotModel);
  nodeHandle.getParam(topicPrefix+"/plannerType", plannerType);
  nodeHandle.getParam(topicPrefix+"/urdfFile", urdfFile);
  nodeHandle.getParam(topicPrefix+"/taskFile", taskFile);
  nodeHandle.getParam(topicPrefix+"/referenceFile", referenceFile);
  bool verbose = true;
  loadData::loadCppDataType(taskFile, "wolf_planner_interface.verbose", verbose);

  // Wait for the controller to start
  ROS_INFO("[WolfPlannerRos] waiting for WoLF controller to start...");
  while (!nodeHandle.hasParam("/"+robotName+"/wolf_controller/robot_foot_names") && ros::ok())
    ros::Rate(1).sleep();
  nodeHandle.getParam("/"+robotName+"/wolf_controller/robot_foot_names", robotFootNames);
  if(robotFootNames.empty())
  {
    ROS_ERROR("[WolfPlannerRos] robot foot names is empty!");
    return false;
  }
  while (!nodeHandle.hasParam("/"+robotName+"/wolf_controller/robot_base_name") && ros::ok())
    ros::Rate(1).sleep();
  nodeHandle.getParam("/"+robotName+"/wolf_controller/robot_base_name", robotBaseName);
  if(robotBaseName.empty())
  {
    ROS_ERROR("[WolfPlannerRos] robot base name is empty!");
    return false;
  }

  // Initialize the planner
  try
  {
    planner_loader_ = std::make_shared<pluginlib::ClassLoader<PlannerInterface>>("wolf_planner_interface", "wolf_planner::PlannerInterface");
    if(plannerType == "default")
    {
      planner_ = planner_loader_->createInstance("wolf_planner::DefaultPlanner");
    }
    else if (plannerType == "adaptive")
    {
      planner_ = planner_loader_->createInstance("wolf_planner::AdaptivePlanner");
    }
    else if (plannerType == "perceptive")
    {
      planner_ = planner_loader_->createInstance("wolf_planner::PerceptivePlanner");
    }
    else
    {
      ROS_ERROR("[WolfPlannerRos] please choose a correct planner type: [default|adaptive|perceptive]");
      return false;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[WolfPlannerRos] Can not load the plugin, reason: " << e.what());
    return false;
  }

  // Set some variables
  planner_->setRobotName(robotName);
  planner_->setTopicPrefix(topicPrefix);
  planner_->setRobotBaseName(robotBaseName);

  if(!planner_->setup(nodeHandle,taskFile,urdfFile,referenceFile,verbose))
  {
    ROS_ERROR("[WolfPlannerRos] Error in planner setup");
    return false;
  }

  planner_->setupMrt();

  ROS_INFO_STREAM("[WolfPlannerRos] Planner type is: "<< plannerType);
  ROS_INFO_STREAM("[WolfPlannerRos] Robot model is: "<< robotModel);
  ROS_INFO_STREAM("[WolfPlannerRos] Robot name is: "<< robotName);
  ROS_INFO_STREAM("[WolfPlannerRos] Robot base name is: "<< robotBaseName);
  auto jointNames = planner_->getJointNames();
  for(unsigned int i=0;i<jointNames.size();i++)
    ROS_INFO_STREAM("[WolfPlannerRos] Loading joint["<<i<<"]: "<<jointNames[i]);
  ROS_INFO_STREAM("[WolfPlannerRos] WoLF planner period is: "<< 1.0/planner_->getLeggedInterface()->mpcSettings().mpcDesiredFrequency_);

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
  mpcObservation_         = nodeHandle.subscribe("/"+robotName+"/wolf_controller/mpc_observation",   1, &WolfPlannerRos::observationCallback, this);
  controllerState_        = nodeHandle.subscribe("/"+robotName+"/wolf_controller/controller_state",  1, &WolfPlannerRos::controllerStateCallback, this);

  return true;
}

void WolfPlannerRos::updatePolicyAndPublish()
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
  forceMsg_lf_.header.frame_id = forceMsg_lh_.header.frame_id = forceMsg_rf_.header.frame_id = forceMsg_rh_.header.frame_id = RBDL_CONTROL_FRAME;
  forceMsg_lf_.wrench.force.x = desiredContactForces[0](0); // LF
  forceMsg_lf_.wrench.force.y = desiredContactForces[0](1); // LF
  forceMsg_lf_.wrench.force.z = desiredContactForces[0](2); // LF
  forceMsg_lh_.wrench.force.x = desiredContactForces[1](0); // LH
  forceMsg_lh_.wrench.force.y = desiredContactForces[1](1); // LH
  forceMsg_lh_.wrench.force.z = desiredContactForces[1](2); // LH
  forceMsg_rf_.wrench.force.x = desiredContactForces[2](0); // RF
  forceMsg_rf_.wrench.force.y = desiredContactForces[2](1); // RF
  forceMsg_rf_.wrench.force.z = desiredContactForces[2](2); // RF
  forceMsg_rh_.wrench.force.x = desiredContactForces[3](0); // RH
  forceMsg_rh_.wrench.force.y = desiredContactForces[3](1); // RH
  forceMsg_rh_.wrench.force.z = desiredContactForces[3](2); // RH

  footMsg_lf_.header.frame_id = footMsg_lh_.header.frame_id = footMsg_rf_.header.frame_id = footMsg_rh_.header.frame_id = RBDL_CONTROL_FRAME;
  footMsg_lf_.pose.position.x = desiredFootPositions[0](0);
  footMsg_lf_.pose.position.y = desiredFootPositions[0](1);
  footMsg_lf_.pose.position.z = desiredFootPositions[0](2);
  footMsg_lf_.twist.linear.x = desiredFootVelocities[0](0);
  footMsg_lf_.twist.linear.y = desiredFootVelocities[0](1);
  footMsg_lf_.twist.linear.z = desiredFootVelocities[0](2);
  footMsg_lh_.pose.position.x = desiredFootPositions[1](0);
  footMsg_lh_.pose.position.y = desiredFootPositions[1](1);
  footMsg_lh_.pose.position.z = desiredFootPositions[1](2);
  footMsg_lh_.twist.linear.x = desiredFootVelocities[1](0);
  footMsg_lh_.twist.linear.y = desiredFootVelocities[1](1);
  footMsg_lh_.twist.linear.z = desiredFootVelocities[1](2);
  footMsg_rf_.pose.position.x = desiredFootPositions[2](0);
  footMsg_rf_.pose.position.y = desiredFootPositions[2](1);
  footMsg_rf_.pose.position.z = desiredFootPositions[2](2);
  footMsg_rf_.twist.linear.x = desiredFootVelocities[2](0);
  footMsg_rf_.twist.linear.y = desiredFootVelocities[2](1);
  footMsg_rf_.twist.linear.z = desiredFootVelocities[2](2);
  footMsg_rh_.pose.position.x = desiredFootPositions[3](0);
  footMsg_rh_.pose.position.y = desiredFootPositions[3](1);
  footMsg_rh_.pose.position.z = desiredFootPositions[3](2);
  footMsg_rh_.twist.linear.x = desiredFootVelocities[3](0);
  footMsg_rh_.twist.linear.y = desiredFootVelocities[3](1);
  footMsg_rh_.twist.linear.z = desiredFootVelocities[3](2);

  baseMsg_.header.frame_id = RBDL_CONTROL_FRAME;
  baseMsg_.pose.position.x = desiredBasePosition(0);
  baseMsg_.pose.position.y = desiredBasePosition(1);
  baseMsg_.pose.position.z = desiredBasePosition(2);
  baseMsg_.pose.orientation.w = desiredBaseQuaternion.w();
  baseMsg_.pose.orientation.x = desiredBaseQuaternion.x();
  baseMsg_.pose.orientation.y = desiredBaseQuaternion.y();
  baseMsg_.pose.orientation.z = desiredBaseQuaternion.z();
  baseMsg_.twist.linear.x =  desiredBaseVelocity(0);
  baseMsg_.twist.linear.y =  desiredBaseVelocity(1);
  baseMsg_.twist.linear.z =  desiredBaseVelocity(2);
  // FIXME the angular velocities are coupled once the robot rotates more than 180
  //baseMsg_.twist.angular.z = vDesired(3);
  //baseMsg_.twist.angular.y = vDesired(4);
  //baseMsg_.twist.angular.x = vDesired(5);

  for (size_t i = 0; i < planner_->getLeggedInterface()->getCentroidalModelInfo().actuatedDofNum; ++i)
  {
    posturalMsg_.positions.push_back(desiredJointPositions(i));
    posturalMsg_.velocities.push_back(desiredJointVelocities(i));
  }

  // Publish the MPC output
  mpcWrenchPublisher_lf_.publish(forceMsg_lf_);
  mpcFootPublisher_lf_.publish(footMsg_lf_);
  mpcWrenchPublisher_lh_.publish(forceMsg_lh_);
  mpcFootPublisher_lh_.publish(footMsg_lh_);
  mpcWrenchPublisher_rf_.publish(forceMsg_rf_);
  mpcFootPublisher_rf_.publish(footMsg_rf_);
  mpcWrenchPublisher_rh_.publish(forceMsg_rh_);
  mpcFootPublisher_rh_.publish(footMsg_rh_);
  mpcBasePublisher_.publish(baseMsg_);
  mpcPosturalPublisher_.publish(posturalMsg_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(observation_));

  // Visualization
  planner_->updateVisualization(observation_);
}

void WolfPlannerRos::observationCallback(const ocs2_msgs::mpc_observationConstPtr& msg)
{
   // Create the observation from the ROS message
   observation_.time = msg->time;
   observation_.mode = msg->mode;
   for (size_t i = 0; i < planner_->getLeggedInterface()->getCentroidalModelInfo().stateDim; ++i)
     observation_.state(i) = msg->state.value[i];

  // Start/Stop the planner
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

void WolfPlannerRos::controllerStateCallback(const wolf_msgs::ControllerStateConstPtr& msg)
{
  if(msg->current_state == "ACTIVE")
    controllerRunning_ = true;
  else
    controllerRunning_ = false;
}

} // namespace wolf_planner

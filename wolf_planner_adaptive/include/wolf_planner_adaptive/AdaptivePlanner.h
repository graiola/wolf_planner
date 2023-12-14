#pragma once

// ROS
#include <ros/ros.h>

#include <wolf_planner_interface/PlannerInterface.h>

#include <pluginlib/class_list_macros.h>

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class AdaptivePlanner : public DefaultPlanner {

public:

  AdaptivePlanner() {}

  AdaptivePlanner(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose = true);

  virtual ~AdaptivePlanner() {};

  virtual void setupSynchronizedModules(ros::NodeHandle& nodeHandle, const std::string topicPrefix = "");

protected:

  virtual void setupRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

};

}  // namespace wolf_planner

PLUGINLIB_EXPORT_CLASS(wolf_planner::AdaptivePlanner, wolf_planner::PlannerInterface)

#pragma once

// ROS
#include <ros/ros.h>

#include <wolf_planner_interface/PlannerInterface.h>


namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class AdaptivePlanner : public PlannerInterface{

public:

  AdaptivePlanner(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose = true);

  virtual ~AdaptivePlanner();

  virtual void setupSynchronizedModules(ros::NodeHandle& nodeHandle, const std::string topicPrefix = "");

protected:

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

};

}  // namespace wolf_planner

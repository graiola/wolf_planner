#pragma once

// ROS
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <wolf_planner_interface/DefaultPlanner.h>

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class AdaptivePlanner : public DefaultPlanner {

public:

  AdaptivePlanner() {};

protected:

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose) override;

  virtual void setupSynchronizedModules() override;

};

}  // namespace wolf_planner

PLUGINLIB_EXPORT_CLASS(wolf_planner::AdaptivePlanner, wolf_planner::PlannerInterface)

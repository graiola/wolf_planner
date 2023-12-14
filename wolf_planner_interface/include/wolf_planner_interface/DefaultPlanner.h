#pragma once

// ROS
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

// OCS2
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

#include "wolf_planner_interface/PlannerInterface.h"
#include "wolf_planner_interface/SafetyChecker.h"
#include "wolf_planner_interface/visualization/LeggedSelfCollisionVisualization.h"
#include "wolf_planner_interface/LeggedReferenceManager.h"

#define WORLD_FRAME_NAME "world"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class DefaultPlanner : public PlannerInterface {

public:

  DefaultPlanner() {};

  DefaultPlanner(ros::NodeHandle& nodeHandle, const std::string& topicPrefix, const std::string& robotName, const std::string& robotBaseName);

  virtual ~DefaultPlanner();

  virtual bool setup(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose = true) override;

  virtual void starting(SystemObservation& observation) override;

  virtual void stopping() override;

  virtual bool updatePolicy(SystemObservation& observation) override;

  virtual void updateVisualization(const SystemObservation& observation) override;

protected:

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

  virtual void setupMrt();

  virtual void setupPinocchioKinematics();

  virtual void setupSynchronizedModules(ros::NodeHandle& nodeHandle, const std::string topicPrefix = "");

  virtual void setupVisualization(ros::NodeHandle& nodeHandle, const std::string robotBaseName = "base_link", const std::string& topicPrefix = "");

  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

private:

  // Pinocchio
  std::shared_ptr<CentroidalModelPinocchioMapping> pinocchioMapping_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;

  // Observation time offset
  double timeOffset_;

  std::thread mpcThread_;
  std::atomic_bool threadRunning_{false};
  benchmark::RepeatedTimer mpcTimer_;

  std::shared_ptr<SafetyChecker> safetyChecker_;
};

}  // namespace wolf_planner

PLUGINLIB_EXPORT_CLASS(wolf_planner::DefaultPlanner, wolf_planner::PlannerInterface)


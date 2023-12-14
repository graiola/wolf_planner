#pragma once

// ROS
#include <ros/ros.h>

// OCS2
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

#include "wolf_planner_interface/SafetyChecker.h"
#include "wolf_planner_interface/visualization/LeggedSelfCollisionVisualization.h"

#include "wolf_planner_interface/LeggedInterface.h"
#include "wolf_planner_interface/LeggedReferenceManager.h"

#define WORLD_FRAME_NAME "world"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class PlannerInterface {

public:

  PlannerInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose = true);

  virtual ~PlannerInterface();

  void setupMrt();

  virtual void setupPinocchioKinematics();

  virtual void setupSynchronizedModules(ros::NodeHandle& nodeHandle, const std::string topicPrefix = "");

  virtual void setupVisualization(ros::NodeHandle& nodeHandle, const std::string robotBaseName = "base_link", const std::string& topicPrefix = "");

  void starting(SystemObservation& observation);

  void stopping();

  bool isRunning();

  virtual bool updatePolicy(SystemObservation& observation);

  virtual void updateVisualization(const SystemObservation& observation);

  std::shared_ptr<LeggedInterface> getLeggedInterface() const { return leggedInterface_; }

  std::shared_ptr<MPC_BASE> getMpc() const { return mpc_; }

  const std::vector<vector3_t>& getDesiredContactForces() const { return mpcDesContactForces_; }

  const std::vector<vector3_t>& getDesiredFootPositions() const { return mpcDesFootPositions_; }

  const std::vector<vector3_t>& getDesiredFootVelocities() const { return mpcDesFootVelocities_; }

  const Eigen::Quaterniond& getDesiredBaseQuaternion() const { return mpcDesBaseQuat_; }

  const vector3_t& getDesiredBasePosition() const { return mpcDesBasePosition_; }

  const vector3_t& getDesiredBaseVelocity() const { return mpcDesBaseVelocity_; }

  const vector_t& getDesiredJointVelocities() const { return mpcDesJointVelocities_; }

  const vector_t& getDesiredJointPositions() const { return mpcDesJointPositions_; }

  const std::vector<std::string>& getJointNames() const { return jointNames_; }

protected:

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);

  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;
  std::shared_ptr<LeggedInterface> leggedInterface_;

private:

  // Pinocchio
  std::shared_ptr<CentroidalModelPinocchioMapping> pinocchioMapping_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;

  // Desired Contact Forces
  std::vector<vector3_t> mpcDesContactForces_;

  // Desired Foot positions and velocities
  std::vector<vector3_t> mpcDesFootPositions_;
  std::vector<vector3_t> mpcDesFootVelocities_;

  // Desired joint positions
  vector_t mpcDesJointPositions_;

  // Desired joint velocities
  vector_t mpcDesJointVelocities_;

  // Desired base quaternion
  Eigen::Quaterniond mpcDesBaseQuat_;

  // Desired base position
  vector3_t mpcDesBasePosition_;

  // Desired base velocity
  vector3_t mpcDesBaseVelocity_;

  // Pinocchio joint names
  std::vector<std::string> jointNames_;

  // Observation time offset
  double timeOffset_;

  std::thread mpcThread_;
  std::atomic_bool plannerRunning_{false}, threadRunning_{false};
  benchmark::RepeatedTimer mpcTimer_;

  std::shared_ptr<SafetyChecker> safetyChecker_;
};

}  // namespace wolf_planner

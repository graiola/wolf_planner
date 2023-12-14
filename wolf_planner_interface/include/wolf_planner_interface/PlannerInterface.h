#pragma once

// ROS
#include <ros/ros.h>

// OCS2
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>

#include "wolf_planner_interface/LeggedInterface.h"

#define WORLD_FRAME_NAME "world"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class PlannerInterface {

public:

  PlannerInterface(ros::NodeHandle& nodeHandle, const std::string& topicPrefix, const std::string& robotBaseName)
    :nodeHandle_(nodeHandle), topicPrefix_(topicPrefix), robotBaseName_(robotBaseName)
  {
  };

  virtual ~PlannerInterface() {};

  virtual bool setup(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose = true) = 0;

  virtual void starting(SystemObservation& observation) = 0;

  virtual void stopping() = 0;

  bool isRunning() { return plannerRunning_; };

  virtual bool updatePolicy(SystemObservation& observation) = 0;

  virtual void updateVisualization(const SystemObservation& observation) = 0;

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

  ros::NodeHandle nodeHandle_;

  std::string topicPrefix_;

  std::string robotBaseName_;

  // MPC BASE
  std::shared_ptr<MPC_BASE> mpc_;

  // Legged interface
  std::shared_ptr<LeggedInterface> leggedInterface_;

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

  // Joint names
  std::vector<std::string> jointNames_;

  // Running planner flag
  std::atomic_bool plannerRunning_{false};
};

}  // namespace wolf_planner

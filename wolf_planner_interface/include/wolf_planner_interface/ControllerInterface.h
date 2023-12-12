#pragma once

#include <ros/ros.h>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "wolf_planner_interface/LeggedInterface.h"
#include "wolf_planner_interface/LeggedReferenceManager.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class ControllerInterface {

public:

  ControllerInterface() {};

  virtual ~ControllerInterface() {};

  virtual void setup(ros::NodeHandle& nodeHandle, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                     bool verbose = false, bool visualization = false) = 0;

  virtual void updateVisualization(const SystemObservation& currentObservation) {};

  std::shared_ptr<LeggedInterface> getLeggedInterfacePtr() const { return leggedInterfacePtr_; }

  std::shared_ptr<MPC_BASE> getMpcPtr() const { return mpcPtr_; }

protected:

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose) = 0;

  virtual void setupSynchronizedModules(ros::NodeHandle& nodeHandle) {};

  virtual void setupVisualization(ros::NodeHandle& nodeHandle) {};

  std::shared_ptr<MPC_BASE> mpcPtr_;
  std::shared_ptr<LeggedInterface> leggedInterfacePtr_;

};

}  // namespace wolf_planner

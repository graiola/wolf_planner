#pragma once

#include <ros/ros.h>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "wolf_planner_interface/LeggedInterface.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class ControllerInterface {

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) = 0;

  virtual void setupSynchronizedModules(ros::NodeHandle& nh, std::shared_ptr<MPC_BASE> mpc) = 0;

  virtual void setupVisualization(ros::NodeHandle& nh) = 0;

  virtual void updateVisualization(const SystemObservation& currentObservation) = 0;

  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const { return referenceManagerPtr_; }

  std::shared_ptr<LeggedInterface> getLeggedInterfacePtr() const { return leggedInterfacePtr_; }

protected:

  std::shared_ptr<LeggedInterface> leggedInterfacePtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

};

}  // namespace wolf_planner

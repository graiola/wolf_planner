#pragma once

#include <pinocchio/fwd.hpp>

#include <wolf_planner_interface/ControllerInterface.h>

#include "wolf_planner_perceptive_interface/visualization/FootPlacementVisualization.h"
#include "wolf_planner_perceptive_interface/visualization/SphereVisualization.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class PerceptiveController : public ControllerInterface {
 protected:
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;

  virtual void setupSynchronizedModules(ros::NodeHandle &nh, std::shared_ptr<MPC_BASE> mpc) override;

  virtual void setupVisualization(ros::NodeHandle& nh) override;

  virtual void updateVisualization(const SystemObservation& currentObservation) override;

private:

  std::shared_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
  std::shared_ptr<SphereVisualization> sphereVisualizationPtr_;

};

}  // namespace wolf_planner

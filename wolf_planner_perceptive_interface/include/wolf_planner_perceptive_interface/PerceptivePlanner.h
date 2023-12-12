#pragma once

#include <pinocchio/fwd.hpp>

#include <wolf_planner_interface/PlannerInterface.h>

#include "wolf_planner_perceptive_interface/visualization/FootPlacementVisualization.h"
#include "wolf_planner_perceptive_interface/visualization/SphereVisualization.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class PerceptivePlanner : public PlannerInterface {

public:

  PerceptivePlanner() {};

  virtual ~PerceptivePlanner() {};

  virtual void setup(ros::NodeHandle& nodeHandle, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                     bool verbose = false, bool visualization = false) override;

  virtual void updateVisualization(const SystemObservation& currentObservation) override;

protected:

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;

  virtual void setupSynchronizedModules(ros::NodeHandle& nodeHandle) override;

  virtual void setupVisualization(ros::NodeHandle& nodeHandle) override;

private:

  std::shared_ptr<FootPlacementVisualization> footPlacementVisualization_;
  std::shared_ptr<SphereVisualization> sphereVisualization_;

};

}  // namespace wolf_planner

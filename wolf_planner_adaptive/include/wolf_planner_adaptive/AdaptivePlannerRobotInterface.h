#pragma once

#include "wolf_planner_interface/LeggedInterface.h"

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class AdaptivePlannerRobotInterface : public LeggedInterface {
 public:
  AdaptivePlannerRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                  bool useHardFrictionConeConstraint = true);

  ~AdaptivePlannerRobotInterface() override = default;

 protected:

  virtual void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                     bool verbose);
  virtual void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                   bool verbose);
};

}  // namespace wolf_planner

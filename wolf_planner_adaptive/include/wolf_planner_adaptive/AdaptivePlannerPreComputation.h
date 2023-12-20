#pragma once

#include <memory>
#include <string>

#include "wolf_planner_interface/LeggedRobotPreComputation.h"
#include "wolf_planner_interface/constraint/EndEffectorLinearConstraint.h"
#include "wolf_planner_interface/SwingTrajectoryPlanner.h"

#include "wolf_planner_adaptive/TerrainEstimator.h"

#include "wolf_planner_adaptive/constraint/FrictionConeConstraint.h"

namespace ocs2 {
namespace legged_robot {

/** Callback for caching and reference update */
class AdaptivePlannerPreComputation : public LeggedRobotPreComputation {
 public:
  AdaptivePlannerPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                            const TerrainEstimator& terrainEstimator,
                            ModelSettings settings);
  ~AdaptivePlannerPreComputation() override = default;

  AdaptivePlannerPreComputation* clone() const override { return new AdaptivePlannerPreComputation(*this); }

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  std::vector<FrictionConeConstraint::Config>& getFrictionConeConstraintConfigs() { return frictionConeConConfigs_; }
  const std::vector<FrictionConeConstraint::Config>& getFrictionConeConstraintConfigs() const { return frictionConeConConfigs_; }

  AdaptivePlannerPreComputation(const AdaptivePlannerPreComputation& other);

 protected:

  const TerrainEstimator* terrainEstimatorPtr_;

  std::vector<FrictionConeConstraint::Config> frictionConeConConfigs_;
};

}  // namespace legged_robot
}  // namespace ocs2

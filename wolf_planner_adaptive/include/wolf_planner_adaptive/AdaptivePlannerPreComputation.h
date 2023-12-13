#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/common/ModelSettings.h>

#include "wolf_planner_interface/constraint/EndEffectorLinearConstraint.h"
#include "wolf_planner_interface/SwingTrajectoryPlanner.h"

#include "wolf_planner_adaptive/constraint/FrictionConeConstraint.h"
#include "wolf_planner_adaptive/TerrainEstimator.h"

namespace ocs2 {
namespace legged_robot {

/** Callback for caching and reference update */
class AdaptivePlannerPreComputation : public PreComputation {
 public:
  AdaptivePlannerPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                            const TerrainEstimator& terrainEstimator,
                            ModelSettings settings);
  ~AdaptivePlannerPreComputation() override = default;

  AdaptivePlannerPreComputation* clone() const override { return new AdaptivePlannerPreComputation(*this); }

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() { return eeNormalVelConConfigs_; }
  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

  std::vector<FrictionConeConstraint::Config>& getFrictionConeConstraintConfigs() { return frictionConeConConfigs_; }
  const std::vector<FrictionConeConstraint::Config>& getFrictionConeConstraintConfigs() const { return frictionConeConConfigs_; }

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 protected:
  AdaptivePlannerPreComputation(const AdaptivePlannerPreComputation& other);

 private:
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  const TerrainEstimator* terrainEstimatorPtr_;
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
  const ModelSettings settings_;

  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
  std::vector<FrictionConeConstraint::Config> frictionConeConConfigs_;
};

}  // namespace legged_robot
}  // namespace ocs2

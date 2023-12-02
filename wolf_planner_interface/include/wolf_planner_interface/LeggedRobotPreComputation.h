/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include <ocs2_legged_robot/common/ModelSettings.h>

#include "wolf_planner_interface/constraint/EndEffectorLinearConstraint.h"
#include "wolf_planner_interface/constraint/FrictionConeConstraint.h"

#include "wolf_planner_interface/SwingTrajectoryPlanner.h"
#include "wolf_planner_interface/TerrainEstimator.h"

namespace ocs2 {
namespace legged_robot {

/** Callback for caching and reference update */
class LeggedRobotPreComputation : public PreComputation {
 public:
  LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                            const TerrainEstimator& terrainEstimator,
                            ModelSettings settings);
  ~LeggedRobotPreComputation() override = default;

  LeggedRobotPreComputation* clone() const override { return new LeggedRobotPreComputation(*this); }

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() { return eeNormalVelConConfigs_; }
  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

  std::vector<FrictionConeConstraint::Config>& getFrictionConeConstraintConfigs() { return frictionConeConConfigs_; }
  const std::vector<FrictionConeConstraint::Config>& getFrictionConeConstraintConfigs() const { return frictionConeConConfigs_; }

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 protected:
  LeggedRobotPreComputation(const LeggedRobotPreComputation& other);

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

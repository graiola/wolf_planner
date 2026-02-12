/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <wolf_planner_interface/LeggedReferenceManager.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace ocs2 {
namespace legged_robot {

class FootCollisionConstraint final : public StateConstraint {
 public:
  FootCollisionConstraint(const LeggedReferenceManager& referenceManager,
                          const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                          std::shared_ptr<grid_map::SignedDistanceField> sdfPtr, size_t contactPointIndex, scalar_t clearance);

  ~FootCollisionConstraint() override = default;
  FootCollisionConstraint* clone() const override { return new FootCollisionConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t /*time*/) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

 private:
  FootCollisionConstraint(const FootCollisionConstraint& rhs);

  const LeggedReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;

  const size_t contactPointIndex_;
  const scalar_t clearance_;
};

}  // namespace legged_robot
}  // namespace ocs2

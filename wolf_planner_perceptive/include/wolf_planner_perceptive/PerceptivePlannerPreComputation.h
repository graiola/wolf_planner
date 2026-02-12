/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <wolf_planner_interface/LeggedRobotPreComputation.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>

#include "wolf_planner_perceptive/ConvexRegionSelector.h"
#include "wolf_planner_perceptive/constraint/FootPlacementConstraint.h"

namespace ocs2 {
namespace legged_robot {

/** Callback for caching and reference update */
class PerceptivePlannerPreComputation : public LeggedRobotPreComputation {
 public:
  PerceptivePlannerPreComputation(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info,
                                 const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                 const ConvexRegionSelector& convexRegionSelector);
  ~PerceptivePlannerPreComputation() override = default;

  PerceptivePlannerPreComputation* clone() const override { return new PerceptivePlannerPreComputation(*this); }

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<FootPlacementConstraint::Parameter>& getFootPlacementConParameters() const { return footPlacementConParameters_; }

  PerceptivePlannerPreComputation(const PerceptivePlannerPreComputation& rhs);

 private:
  std::pair<matrix_t, vector_t> getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  const ConvexRegionSelector* convexRegionSelectorPtr_;

  std::vector<FootPlacementConstraint::Parameter> footPlacementConParameters_;
};

}  // namespace legged_robot
}  // namespace ocs2

#pragma once

#include "wolf_planner_interface/SwingTrajectoryPlanner.h"

namespace ocs2 {
namespace legged_robot {

/**
 * @brief Extended SwingTrajectoryPlanner that adds swing planning in X and Y directions.
 *
 * Inherits the default Z-direction spline planning from the base class,
 * and overrides the update() method to also support XY modifications.
 */
class SwingTrajectoryPlannerXY : public SwingTrajectoryPlanner {
 public:
  using SwingTrajectoryPlanner::SwingTrajectoryPlanner;

  /** 
   * Override base class method to support polymorphism.
   * This fallback implementation injects 0 values for XY if none are provided.
   */
  void update(const ModeSchedule& modeSchedule,
              const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence,
              const feet_array_t<scalar_array_t>& maxHeightSequence) override;

  /**
   * Full 3D swing trajectory planner (X, Y, Z) using cubic splines.
   * Allows retraction, overshoot, and lateral motion.
   */
  void updateXY(const ModeSchedule& modeSchedule,
                const feet_array_t<scalar_array_t>& liftOffHeightSequence,
                const feet_array_t<scalar_array_t>& touchDownHeightSequence,
                const feet_array_t<scalar_array_t>& maxHeightSequence,
                const feet_array_t<scalar_array_t>& liftOffXSequence,
                const feet_array_t<scalar_array_t>& touchDownXSequence,
                const feet_array_t<scalar_array_t>& liftOffYSequence,
                const feet_array_t<scalar_array_t>& touchDownYSequence);

  /**
   * Evaluate the X-position constraint at a specific time.
   */
  scalar_t getXpositionConstraint(size_t leg, scalar_t time) const;

  /**
   * Evaluate the Y-position constraint at a specific time.
   */
  scalar_t getYpositionConstraint(size_t leg, scalar_t time) const;

 private:
  // Each leg's X/Y swing trajectory is split into 2 segments per swing phase (lift-off to mid, mid to touch-down)
  feet_array_t<std::vector<CubicSpline>> feetXTrajectories_;
  feet_array_t<std::vector<CubicSpline>> feetYTrajectories_;
};

}  // namespace legged_robot
}  // namespace ocs2

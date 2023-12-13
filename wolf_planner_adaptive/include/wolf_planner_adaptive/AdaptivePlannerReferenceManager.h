#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include <wolf_planner_interface/SwingTrajectoryPlanner.h>
#include <wolf_planner_interface/LeggedReferenceManager.h>

#include "wolf_planner_adaptive/TerrainEstimator.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class AdaptivePlannerReferenceManager : public LeggedReferenceManager {
 public:
  AdaptivePlannerReferenceManager(CentroidalModelInfo info,
                                std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                std::shared_ptr<TerrainEstimator> terrainEstimator,
                                scalar_t comHeight);

  ~AdaptivePlannerReferenceManager() override = default;

  const std::shared_ptr<TerrainEstimator>& getTerrainEstimator() { return terrainEstimatorPtr_; }

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;


  const CentroidalModelInfo info_;
  std::shared_ptr<TerrainEstimator> terrainEstimatorPtr_;
  scalar_t comHeight_;
};

}  // namespace legged_robot
}  // namespace ocs2

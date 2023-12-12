#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include "wolf_planner_interface/TerrainEstimator.h"
#include "wolf_planner_interface/SwingTrajectoryPlanner.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class LeggedReferenceManager : public ReferenceManager {
 public:
  LeggedReferenceManager(CentroidalModelInfo info,
                                std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~LeggedReferenceManager() override = default;

  void setModeSchedule(const ModeSchedule& modeSchedule) override;

  contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;


  const CentroidalModelInfo info_;
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2

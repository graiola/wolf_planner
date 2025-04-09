#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <wolf_planner_interface/SwingTrajectoryPlanner.h>
#include <wolf_planner_interface/LeggedReferenceManager.h>

#include "wolf_planner_adaptive/TerrainEstimator.h"
#include "wolf_planner_adaptive/ContactForcesEstimator.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class AdaptivePlannerReferenceManager : public LeggedReferenceManager {
 public:
  AdaptivePlannerReferenceManager(PinocchioInterface pinocchioInterface,
                                  CentroidalModelInfo info,
                                  std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                  std::shared_ptr<TerrainEstimator> terrainEstimator,
                                  std::shared_ptr<ContactForcesEstimator> contactForcesEstimator,
                                  const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                  scalar_t comHeight,
                                  scalar_t stepReflexHeight,
                                  const std::vector<std::string>& contactFrameNames);

  ~AdaptivePlannerReferenceManager() override = default;

  const std::shared_ptr<TerrainEstimator>& getTerrainEstimator() { return terrainEstimatorPtr_; }

  const std::shared_ptr<ContactForcesEstimator>& getContactForcesEstimator() { return contactForcesEstimatorPtr_; }

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) override;

  void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                    ModeSchedule& modeSchedule);

  void triggerStepReflex(size_t leg, scalar_t time);

  void resetStepReflex(size_t leg);

  // Members
  PinocchioInterface pinocchioInterface_;
  std::shared_ptr<TerrainEstimator> terrainEstimatorPtr_;
  std::shared_ptr<ContactForcesEstimator> contactForcesEstimatorPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  scalar_t comHeight_;

  std::vector<std::string> contactFrameNames_; 

  double stepReflexHeight_;                 // fixed extra height to add (e.g. 0.05 m for 5 cm)
  std::array<bool, 4> stepReflexTriggered_; // whether reflex triggered in current swing for each foot
  std::array<int, 4> stepReflexCount_;      // how many times reflex triggered in the current swing
  std::array<double, 4> reflexTriggerTime_; // time when reflex was triggered (for shaping the offset)
};

}  // namespace legged_robot
}  // namespace ocs2

/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

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
#include "wolf_planner_adaptive/OdomEstimator.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 * Implements reflex-based step height adaptation with smoothing.
 */
class AdaptivePlannerReferenceManager : public LeggedReferenceManager {
 public:
  AdaptivePlannerReferenceManager(PinocchioInterface pinocchioInterface,
                                  CentroidalModelInfo info,
                                  std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                  std::shared_ptr<TerrainEstimator> terrainEstimator,
                                  std::shared_ptr<ContactForcesEstimator> contactForcesEstimator,
                                  std::shared_ptr<OdomEstimator> odomEstimatorPtr,
                                  const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                  scalar_t comHeight,
                                  scalar_t stepReflexHeight,
                                  scalar_t forceThreshold);

  ~AdaptivePlannerReferenceManager() override = default;

  const std::shared_ptr<TerrainEstimator>& getTerrainEstimator() { return terrainEstimatorPtr_; }
  const std::shared_ptr<ContactForcesEstimator>& getContactForcesEstimator() { return contactForcesEstimatorPtr_; }
  const std::shared_ptr<OdomEstimator>& getOdomEstimator() { return odomEstimatorPtr_; }

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) override;

  void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                    ModeSchedule& modeSchedule);

  void triggerStepReflex(size_t leg, scalar_t time);
  void resetStepReflex(size_t leg);

  /** Adjust base pose height and orientation to match terrain */
  void adjustBasePoseToTerrain(vector_t& state, scalar_t terrainHeight) const;

  /** Detects step reflex triggers and populates reflexEvents */
  void detectReflexes(scalar_t time, std::vector<std::pair<scalar_t, size_t>>& reflexEvents);

  // Members
  PinocchioInterface pinocchioInterface_;
  std::shared_ptr<TerrainEstimator> terrainEstimatorPtr_;
  std::shared_ptr<ContactForcesEstimator> contactForcesEstimatorPtr_;
  std::shared_ptr<OdomEstimator> odomEstimatorPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

  scalar_t comHeight_;        // Nominal center of mass height
  scalar_t forceThreshold_;   // Force magnitude threshold for triggering reflex
  scalar_t stepReflexHeight_; // Height increment per reflex trigger

  std::array<bool, 4> stepReflexTriggered_;  // Whether reflex is currently active per leg
  std::array<int, 4> stepReflexCount_;       // Reflex height multiplier (with decay) per leg
  std::array<double, 4> reflexTriggerTime_;  // Time of last reflex trigger per leg
};

}  // namespace legged_robot
}  // namespace ocs2

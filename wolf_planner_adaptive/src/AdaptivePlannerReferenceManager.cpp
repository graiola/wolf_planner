#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace ocs2
{
  namespace legged_robot
  {

    AdaptivePlannerReferenceManager::AdaptivePlannerReferenceManager(PinocchioInterface pinocchioInterface,
                                                                     CentroidalModelInfo info,
                                                                     std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                     std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                                     std::shared_ptr<TerrainEstimator> terrainEstimatorPtr,
                                                                     std::shared_ptr<ContactForcesEstimator> contactForcesEstimatorPtr,
                                                                     const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
                                                                     scalar_t comHeight,
                                                                     scalar_t stepReflexHeight,
                                                                     const std::vector<std::string> &contactFrameNames)
        : LeggedReferenceManager(info, gaitSchedulePtr, swingTrajectoryPtr),
          pinocchioInterface_(std::move(pinocchioInterface)),
          terrainEstimatorPtr_(std::move(terrainEstimatorPtr)),
          contactForcesEstimatorPtr_(std::move(contactForcesEstimatorPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          comHeight_(comHeight),
          stepReflexHeight_(stepReflexHeight),
          contactFrameNames_(contactFrameNames)
    {
      stepReflexTriggered_.fill(false);
      stepReflexCount_.fill(0);
      reflexTriggerTime_.fill(0.0);
    }

void AdaptivePlannerReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                        TargetTrajectories &targetTrajectories, ModeSchedule &modeSchedule)
{
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  terrainEstimatorPtr_->update();
  contactForcesEstimatorPtr_->update();
  const scalar_t terrainHeight = terrainEstimatorPtr_->getTerrainCenter().z();

  TargetTrajectories newTargetTrajectories;
  size_t nodeNum = targetTrajectories.timeTrajectory.size();

  for (size_t i = 0; i < nodeNum; ++i)
  {
    scalar_t time = targetTrajectories.timeTrajectory[i];
    vector_t state = targetTrajectories.getDesiredState(time);
    vector_t input = targetTrajectories.getDesiredInput(time);

    // Base pose orientation adjustment
    vector_t pos = centroidal_model::getBasePose(state, info_).head(3);
    auto normalVector = terrainEstimatorPtr_->getTerrainNormal();
    normalVector.normalize();
    matrix3_t R;
    scalar_t z = centroidal_model::getBasePose(state, info_)(3);
    R << cos(z), -sin(z), 0,
        sin(z), cos(z), 0,
        0, 0, 1;
    vector_t v = R.transpose() * normalVector;
    centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());
    centroidal_model::getBasePose(state, info_)(2) =
        terrainHeight + comHeight_ / cos(centroidal_model::getBasePose(state, info_)(4));

    // Update kinematics
    pinocchio::forwardKinematics(pinocchioInterface_.getModel(), pinocchioInterface_.getData(),
                                  centroidal_model::getGeneralizedCoordinates(state, info_));
    pinocchio::updateFramePlacements(pinocchioInterface_.getModel(), pinocchioInterface_.getData());

    newTargetTrajectories.timeTrajectory.push_back(time);
    newTargetTrajectories.stateTrajectory.push_back(state);
    newTargetTrajectories.inputTrajectory.push_back(input);

    for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
    {
      if (!getContactFlags(time)[leg])
      {
        // Read foot contact force
        Eigen::Vector3d footForce;
        footForce << contactForcesEstimatorPtr_->getContactForces()[leg][0],
            contactForcesEstimatorPtr_->getContactForces()[leg][1],
            contactForcesEstimatorPtr_->getContactForces()[leg][2];

        // Transform force into swing frame
        const auto &footRotation = pinocchioInterface_.getData().oMf[pinocchioInterface_.getModel().getFrameId(contactFrameNames_[leg])].rotation();
        const Eigen::Matrix3d swingFrameRotation = footRotation.transpose();
        const Eigen::Vector3d contactForceSwingFrame = swingFrameRotation * footForce;

        const double forceNormXZ = std::hypot(contactForceSwingFrame.x(), contactForceSwingFrame.z());

        constexpr double backwardXThreshold = -5.0; // negative = pushing backward
        constexpr double forceThreshold = 2.5;

        const bool rearOrFrontalImpact = (contactForceSwingFrame.x() < backwardXThreshold) && forceNormXZ > forceThreshold;

        if (rearOrFrontalImpact)
        {
          triggerStepReflex(leg, time);
        }
      }
      else
      {
        if (getContactFlags(time)[leg]) {
          if (reflexTriggerTime_[leg] + 0.5 < time) {
              resetStepReflex(leg);
          }
      }
      }
    }
  }

  targetTrajectories = newTargetTrajectories;
  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
}

void AdaptivePlannerReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                   ModeSchedule& modeSchedule) {
  feet_array_t<std::vector<bool>> contactFlagStocks;
  contactFlagStocks.fill(std::vector<bool>(modeSchedule.modeSequence.size()));

  for (size_t i = 0; i < modeSchedule.modeSequence.size(); ++i) {
    const auto flags = getContactFlags(modeSchedule.eventTimes[i]);
    for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
      contactFlagStocks[leg][i] = flags[leg];
    }
  }

  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence;

  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    scalar_array_t liftOffHeights, touchDownHeights, maxHeights;
    liftOffHeights.resize(modeSchedule.modeSequence.size());
    touchDownHeights.resize(modeSchedule.modeSequence.size());
    maxHeights.resize(modeSchedule.modeSequence.size());

    const auto footPos = endEffectorKinematicsPtr_->getPosition(initState)[leg];
    const scalar_t terrainZ = terrainEstimatorPtr_->getTerrainHeightAt(footPos.x(), footPos.y());

    for (size_t i = 0; i < modeSchedule.modeSequence.size(); ++i) {
      const bool inSwing = !contactFlagStocks[leg][i];

      liftOffHeights[i] = inSwing ? terrainZ : 0.0;
      touchDownHeights[i] = inSwing ? terrainZ : 0.0;
      maxHeights[i] = std::max(liftOffHeights[i], touchDownHeights[i]);

      if (stepReflexTriggered_[leg]) {
        maxHeights[i] += stepReflexCount_[leg] * stepReflexHeight_;
      }
    }

    liftOffHeightSequence[leg] = liftOffHeights;
    touchDownHeightSequence[leg] = touchDownHeights;
    maxHeightSequence[leg] = maxHeights;
  }

  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);
}

void AdaptivePlannerReferenceManager::triggerStepReflex(size_t leg, scalar_t time) {
  stepReflexTriggered_[leg] = true;
  stepReflexCount_[leg] = std::min(stepReflexCount_[leg] + 1, 10);  // clamp to 10
  reflexTriggerTime_[leg] = time;
}

void AdaptivePlannerReferenceManager::resetStepReflex(size_t leg) {
  stepReflexTriggered_[leg] = false;
  stepReflexCount_[leg] = 0;
  reflexTriggerTime_[leg] = 0.0;
}

} // namespace legged_robot
} // namespace ocs2

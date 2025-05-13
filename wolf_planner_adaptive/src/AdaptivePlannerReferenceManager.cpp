#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace ocs2 {
namespace legged_robot {

namespace {
constexpr scalar_t kAngleThresholdLower = -120.0 * M_PI / 180.0;
constexpr scalar_t kAngleThresholdUpper = 140.0 * M_PI / 180.0;
constexpr size_t kMaxReflexCount = 5;
constexpr size_t kResetHysteresys = 0.1;
constexpr scalar_t kReflexDecayFactor = 0.8;

uint32_t setLegSwing(uint32_t mode, size_t leg) {
  return mode & ~(1UL << leg);
}

scalar_t computeMaxStepHeight(int reflexCount, scalar_t baseHeight, scalar_t stepReflexHeight, scalar_t comHeight) {
  if (reflexCount > 0) {
    scalar_t adjusted = baseHeight + reflexCount * stepReflexHeight;
    return std::min(adjusted, comHeight / 2.5);
  }
  return baseHeight;
}
}  // anonymous namespace

AdaptivePlannerReferenceManager::AdaptivePlannerReferenceManager(PinocchioInterface pinocchioInterface,
                                                                 CentroidalModelInfo info,
                                                                 std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                 std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                                 std::shared_ptr<TerrainEstimator> terrainEstimatorPtr,
                                                                 std::shared_ptr<ContactForcesEstimator> contactForcesEstimatorPtr,
                                                                 std::shared_ptr<OdomEstimator> odomEstimatorPtr,
                                                                 const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                 scalar_t comHeight,
                                                                 scalar_t stepReflexHeight,
                                                                 scalar_t forceThreshold)
    : LeggedReferenceManager(info, gaitSchedulePtr, swingTrajectoryPtr),
      pinocchioInterface_(std::move(pinocchioInterface)),
      terrainEstimatorPtr_(std::move(terrainEstimatorPtr)),
      contactForcesEstimatorPtr_(std::move(contactForcesEstimatorPtr)),
      odomEstimatorPtr_(std::move(odomEstimatorPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      comHeight_(comHeight),
      stepReflexHeight_(stepReflexHeight),
      forceThreshold_(forceThreshold) {
  stepReflexTriggered_.fill(false);
  stepReflexCount_.fill(0);
  reflexTriggerTime_.fill(0.0);
}

void AdaptivePlannerReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime,
                                                       const vector_t& initState,
                                                       TargetTrajectories& targetTrajectories,
                                                       ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  terrainEstimatorPtr_->update();
  contactForcesEstimatorPtr_->update();
  odomEstimatorPtr_->update();

  const scalar_t terrainHeight = terrainEstimatorPtr_->getTerrainCenter().z();

  TargetTrajectories newTargetTrajectories;
  size_t nodeNum = targetTrajectories.timeTrajectory.size();
  std::vector<std::pair<scalar_t, size_t>> reflexEvents;

  for (size_t i = 0; i < nodeNum; ++i) {
    scalar_t time = targetTrajectories.timeTrajectory[i];
    vector_t state = targetTrajectories.getDesiredState(time);
    vector_t input = targetTrajectories.getDesiredInput(time);

    adjustBasePoseToTerrain(state, terrainHeight);

    pinocchio::forwardKinematics(pinocchioInterface_.getModel(), pinocchioInterface_.getData(),
                                 centroidal_model::getGeneralizedCoordinates(state, info_));
    pinocchio::updateFramePlacements(pinocchioInterface_.getModel(), pinocchioInterface_.getData());

    newTargetTrajectories.timeTrajectory.push_back(time);
    newTargetTrajectories.stateTrajectory.push_back(state);
    newTargetTrajectories.inputTrajectory.push_back(input);

    detectReflexes(time, reflexEvents);
  }

  targetTrajectories = newTargetTrajectories;

  // Modify the modeSchedule to accomodate the reflex
  /*for (const auto& reflexEvent : reflexEvents) {
    scalar_t reflexTime = reflexEvent.first;
    size_t leg = reflexEvent.second;

    auto it = std::lower_bound(modeSchedule.eventTimes.begin(), modeSchedule.eventTimes.end(), reflexTime);
    size_t index = std::distance(modeSchedule.eventTimes.begin(), it);

    if (index > 0 && index <= modeSchedule.modeSequence.size()) {
        uint32_t currentMode = modeSchedule.modeSequence.at(index - 1);
        currentMode = setLegSwing(currentMode, leg);

        modeSchedule.eventTimes.insert(it, reflexTime);
        modeSchedule.modeSequence.insert(modeSchedule.modeSequence.begin() + index, currentMode);
    } else {
        std::cerr << "[AdaptivePlannerReferenceManager] Reflex insertion index out of bounds!\n";
    }
  }*/
  //setModeSchedule(modeSchedule);

  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
}

void AdaptivePlannerReferenceManager::adjustBasePoseToTerrain(vector_t& state, scalar_t terrainHeight) const {
  vector_t basePose = centroidal_model::getBasePose(state, info_);
  auto normalVector = terrainEstimatorPtr_->getTerrainNormal().normalized();

  scalar_t yaw = basePose(3);
  matrix3_t R;
  R << std::cos(yaw), -std::sin(yaw), 0,
       std::sin(yaw),  std::cos(yaw), 0,
               0,              0,     1;

  vector_t v = R.transpose() * normalVector;
  centroidal_model::getBasePose(state, info_)(4) = std::atan2(v.x(), v.z());
  centroidal_model::getBasePose(state, info_)(2) = terrainHeight + comHeight_ / std::cos(basePose(4));
}

void AdaptivePlannerReferenceManager::detectReflexes(scalar_t time, std::vector<std::pair<scalar_t, size_t>>& reflexEvents) {
  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    const Eigen::Vector3d footForce = contactForcesEstimatorPtr_->getContactForces()[leg];
    const Eigen::Vector3d baseLinearVel = odomEstimatorPtr_->getBaseLinearVelocity();
    const std::string& contactName = contactForcesEstimatorPtr_->getContactNames()[leg];

    Eigen::Vector3d direction = baseLinearVel.norm() > 1e-3 ? baseLinearVel.normalized() : Eigen::Vector3d::UnitX();
    Eigen::Matrix3d swingRotation;
    swingRotation.col(2) = Eigen::Vector3d::UnitZ();
    swingRotation.col(1) = swingRotation.col(2).cross(direction).normalized();
    swingRotation.col(0) = swingRotation.col(1).cross(swingRotation.col(2));

    Eigen::Vector3d contactForceRotated = swingRotation.transpose() * footForce;

    double angle = std::atan2(contactForceRotated.z(), contactForceRotated.x());
    bool forceInsideLimits = angle < kAngleThresholdLower || angle > kAngleThresholdUpper;

    double forceNormXZ = std::hypot(contactForceRotated.x(), contactForceRotated.z());
    bool impact = forceInsideLimits && forceNormXZ > forceThreshold_;

    if (impact) {
      std::cout << "[AdaptivePlannerReferenceManager] Impact detected for " << contactName << std::endl;
      triggerStepReflex(leg, time);
      reflexEvents.emplace_back(time, leg);
    } else if (stepReflexTriggered_[leg] && reflexTriggerTime_[leg] + kResetHysteresys < time) {
      resetStepReflex(leg);
    }
  }
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
    scalar_array_t liftOffHeights(modeSchedule.modeSequence.size());
    scalar_array_t touchDownHeights(modeSchedule.modeSequence.size());
    scalar_array_t maxHeights(modeSchedule.modeSequence.size());

    const auto footPos = endEffectorKinematicsPtr_->getPosition(initState)[leg];
    scalar_t terrainZ = terrainEstimatorPtr_->getTerrainHeightAt(footPos.x(), footPos.y());

    for (size_t i = 0; i < modeSchedule.modeSequence.size(); ++i) {
      bool inSwing = !contactFlagStocks[leg][i];

      liftOffHeights[i] = inSwing ? terrainZ : 0.0;
      touchDownHeights[i] = inSwing ? terrainZ : 0.0;
      maxHeights[i] = computeMaxStepHeight(stepReflexCount_[leg], liftOffHeights[i], stepReflexHeight_, comHeight_);
    }

    liftOffHeightSequence[leg] = liftOffHeights;
    touchDownHeightSequence[leg] = touchDownHeights;
    maxHeightSequence[leg] = maxHeights;
  }

  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);

  // Smooth decay of reflex height over time
  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    if (!stepReflexTriggered_[leg] && stepReflexCount_[leg] > 0) {
      stepReflexCount_[leg] = std::floor(stepReflexCount_[leg] * kReflexDecayFactor);
    }
  }
}

void AdaptivePlannerReferenceManager::triggerStepReflex(size_t leg, scalar_t time) {
  stepReflexTriggered_[leg] = true;
  stepReflexCount_[leg] = std::min(stepReflexCount_[leg] + 1, static_cast<int>(kMaxReflexCount));
  reflexTriggerTime_[leg] = time;
}

void AdaptivePlannerReferenceManager::resetStepReflex(size_t leg) {
  stepReflexTriggered_[leg] = false;
  reflexTriggerTime_[leg] = 0.0;
  //stepReflexCount_[leg] = 0;
  // Keep stepReflexCount_ for decay
}

}  // namespace legged_robot
}  // namespace ocs2

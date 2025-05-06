#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"
#include "wolf_planner_interface/SwingTrajectoryPlannerXY.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

AdaptivePlannerReferenceManager::AdaptivePlannerReferenceManager(PinocchioInterface pinocchioInterface,
                                                                 CentroidalModelInfo info,
                                                                 std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                 std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                                 std::shared_ptr<TerrainEstimator> terrainEstimatorPtr,
                                                                 std::shared_ptr<ContactForcesEstimator> contactForcesEstimatorPtr,
                                                                 const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                 scalar_t comHeight,
                                                                 scalar_t stepReflexHeight,
                                                                 const std::vector<std::string>& contactFrameNames)
    : LeggedReferenceManager(info, gaitSchedulePtr, swingTrajectoryPtr),
      pinocchioInterface_(std::move(pinocchioInterface)),
      terrainEstimatorPtr_(std::move(terrainEstimatorPtr)),
      contactForcesEstimatorPtr_(std::move(contactForcesEstimatorPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      comHeight_(comHeight),
      stepReflexHeight_(stepReflexHeight),
      contactFrameNames_(contactFrameNames) {
  for (auto& reflex : reflexControllers_) {
    reflex.configure(1.0, stepReflexHeight);
  }
}

void AdaptivePlannerReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                       TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  terrainEstimatorPtr_->update();
  contactForcesEstimatorPtr_->update();

  TargetTrajectories newTargetTrajectories;
  const size_t nodeNum = targetTrajectories.timeTrajectory.size();
  for (size_t i = 0; i < nodeNum; ++i) {
    const scalar_t time = targetTrajectories.timeTrajectory[i];
    const vector_t& state = targetTrajectories.getDesiredState(time);
    const vector_t& input = targetTrajectories.getDesiredInput(time);

    pinocchio::forwardKinematics(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), centroidal_model::getGeneralizedCoordinates(state, info_));
    pinocchio::updateFramePlacements(pinocchioInterface_.getModel(), pinocchioInterface_.getData());

    for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
      if (!getContactFlags(time)[leg]) {
        Eigen::Vector3d footForce;
        footForce << contactForcesEstimatorPtr_->getContactForces()[leg][0],
                     contactForcesEstimatorPtr_->getContactForces()[leg][1],
                     contactForcesEstimatorPtr_->getContactForces()[leg][2];

        const auto& footRotation = pinocchioInterface_.getData().oMf[pinocchioInterface_.getModel().getFrameId(contactFrameNames_[leg])].rotation();
        const Eigen::Matrix3d swingFrameRotation = footRotation.transpose();
        const Eigen::Vector3d contactForceSwingFrame = swingFrameRotation * footForce;

        const double angle = std::atan2(contactForceSwingFrame.z(), contactForceSwingFrame.x());
        const double forceNormXZ = std::hypot(contactForceSwingFrame.x(), contactForceSwingFrame.z());

        constexpr double angleMin = -120.0 * M_PI / 180.0;
        constexpr double angleMax =  140.0 * M_PI / 180.0;
        constexpr double forceThreshold = 10.0;

        const bool frontalImpact = (angle < angleMin || angle > angleMax) && forceNormXZ > forceThreshold;

        if (frontalImpact && !reflexControllers_[leg].isActive()) {
          reflexControllers_[leg].trigger(endEffectorKinematicsPtr_->getPosition(state)[leg]);
          std::cout << "[Reflex Triggered] leg: " << leg << ", time: " << time << std::endl;
        }

        reflexControllers_[leg].update(1.0 / 250.0);

        if (reflexControllers_[leg].isActive()) {
          std::cout << "Displacement[" << leg << "] = " << reflexControllers_[leg].getDisplacement().transpose() << std::endl;
        }

      } else {
        std::cout << "[Reflex Reset] leg: " << leg << ", time: " << time << std::endl;
        reflexControllers_[leg].reset();
      }
    }

    newTargetTrajectories.timeTrajectory.push_back(time);
    newTargetTrajectories.stateTrajectory.push_back(state);
    newTargetTrajectories.inputTrajectory.push_back(input);
  }

  targetTrajectories = newTargetTrajectories;
  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
}

void AdaptivePlannerReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                   ModeSchedule& modeSchedule) {
  // Prepare swing foot trajectories (XYZ) with reflex displacement
  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence;
  feet_array_t<scalar_array_t> liftOffXSequence, touchDownXSequence;
  feet_array_t<scalar_array_t> liftOffYSequence, touchDownYSequence;

  const auto footPosAll = endEffectorKinematicsPtr_->getPosition(initState);

  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    scalar_array_t liftOffHeights, touchDownHeights, maxHeights;
    scalar_array_t liftOffXs, touchDownXs, liftOffYs, touchDownYs;

    const Eigen::Vector3d baseFoot = footPosAll[leg];
    const Eigen::Vector3d reflex = reflexControllers_[leg].getDisplacement();

    for (size_t i = 0; i < modeSchedule.modeSequence.size(); ++i) {
      liftOffHeights.push_back(baseFoot.z() + reflex.z());
      touchDownHeights.push_back(baseFoot.z());
      maxHeights.push_back(std::max(baseFoot.z(), baseFoot.z() + reflex.z()));

      liftOffXs.push_back(baseFoot.x() + reflex.x());
      touchDownXs.push_back(baseFoot.x());
      liftOffYs.push_back(baseFoot.y() + reflex.y());
      touchDownYs.push_back(baseFoot.y());
    }

    liftOffHeightSequence[leg] = liftOffHeights;
    touchDownHeightSequence[leg] = touchDownHeights;
    maxHeightSequence[leg] = maxHeights;

    liftOffXSequence[leg] = liftOffXs;
    touchDownXSequence[leg] = touchDownXs;
    liftOffYSequence[leg] = liftOffYs;
    touchDownYSequence[leg] = touchDownYs;
  }

  auto swingPlannerXY = std::dynamic_pointer_cast<SwingTrajectoryPlannerXY>(swingTrajectoryPtr_);
  if (!swingPlannerXY) {
    throw std::runtime_error("[AdaptivePlanner] Invalid swing trajectory planner: XY type required");
  }

  swingPlannerXY->updateXY(modeSchedule,
                           liftOffHeightSequence,
                           touchDownHeightSequence,
                           maxHeightSequence,
                           liftOffXSequence,
                           touchDownXSequence,
                           liftOffYSequence,
                           touchDownYSequence);
}

}  // namespace legged_robot
}  // namespace ocs2
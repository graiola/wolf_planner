#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

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
                                                                 const std::vector<std::string>& contactFrameNames)
    : LeggedReferenceManager(info, gaitSchedulePtr, swingTrajectoryPtr),
      pinocchioInterface_(std::move(pinocchioInterface)),
      terrainEstimatorPtr_(std::move(terrainEstimatorPtr)),
      contactForcesEstimatorPtr_(std::move(contactForcesEstimatorPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      comHeight_(comHeight),
      contactFrameNames_(contactFrameNames),
      stepReflexHeight_(0.05) {
  stepReflexTriggered_.fill(false);
  stepReflexCount_.fill(0);
  reflexTriggerTime_.fill(0.0);
}

void AdaptivePlannerReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                       TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  terrainEstimatorPtr_->update();
  contactForcesEstimatorPtr_->update();
  const scalar_t terrainHeight = terrainEstimatorPtr_->getTerrainCenter().z();

  TargetTrajectories newTargetTrajectories;
  size_t nodeNum = targetTrajectories.timeTrajectory.size();
  for (size_t i = 0; i < nodeNum; ++i) {
    scalar_t time = targetTrajectories.timeTrajectory[i];
    vector_t state = targetTrajectories.getDesiredState(time);
    vector_t input = targetTrajectories.getDesiredInput(time); 

    vector_t pos = centroidal_model::getBasePose(state, info_).head(3);

    auto normalVector = terrainEstimatorPtr_->getTerrainNormal();
    normalVector.normalize();
    matrix3_t R;
    scalar_t z = centroidal_model::getBasePose(state, info_)(3);
    R << cos(z), -sin(z), 0,
         sin(z),  cos(z), 0,
         0,       0,      1;
    vector_t v = R.transpose() * normalVector;
    centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());

    centroidal_model::getBasePose(state, info_)(2) =
        terrainHeight + comHeight_ / cos(centroidal_model::getBasePose(state, info_)(4));

    newTargetTrajectories.timeTrajectory.push_back(time);
    newTargetTrajectories.stateTrajectory.push_back(state);
    newTargetTrajectories.inputTrajectory.push_back(input);

    for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
      if (!getContactFlags(time)[leg] && !stepReflexTriggered_[leg]) {
        Eigen::Vector3d footForce;
        footForce <<  contactForcesEstimatorPtr_->getContactForces()[leg][0], 
                      contactForcesEstimatorPtr_->getContactForces()[leg][1], 
                      contactForcesEstimatorPtr_->getContactForces()[leg][2];
        if (footForce.norm() > 25.0) {
          triggerStepReflex(leg, time);
        }
      }
    }
  }
  targetTrajectories = newTargetTrajectories;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);

  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
}

Eigen::Vector3d AdaptivePlannerReferenceManager::estimateContactForce(size_t leg, const vector_t& state, const vector_t& input) const {
  const auto& model = pinocchioInterface_.getModel();
  pinocchio::Data data(model);  // Create a local non-const Data object

  const vector_t& q = state.head(model.nq);
  const vector_t& v = state.segment(model.nq, model.nv);

  pinocchio::computeAllTerms(model, data, q, v);

  const auto jointTorques = input;
  const auto tau_without_contact = pinocchio::rnea(model, data, q, v, vector_t::Zero(model.nv));
  const auto tau_contact = jointTorques - tau_without_contact;

  pinocchio::FrameIndex frameId = model.getBodyId(contactFrameNames_[leg]);
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nv);
  
  pinocchio::computeFrameJacobian(model, data, q, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J);
  
  Eigen::VectorXd contactForce = (J.transpose()).completeOrthogonalDecomposition().solve(tau_contact);
  return contactForce.head<3>();
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

  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;

  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    scalar_array_t liftOffHeights, touchDownHeights;
    liftOffHeights.resize(modeSchedule.modeSequence.size());
    touchDownHeights.resize(modeSchedule.modeSequence.size());

    for (size_t i = 0; i < modeSchedule.modeSequence.size(); ++i) {
      const bool inSwing = !contactFlagStocks[leg][i];
      const auto footPos = endEffectorKinematicsPtr_->getPosition(initState)[leg];
      const scalar_t terrainZ = terrainEstimatorPtr_->getTerrainHeightAt(footPos.x(), footPos.y());
      liftOffHeights[i] = inSwing ? terrainZ : 0.0;
      touchDownHeights[i] = inSwing ? terrainZ : 0.0;
    }

    if (stepReflexTriggered_[leg] && !contactFlagStocks[leg].back()) {
      double t_trigger = reflexTriggerTime_[leg];
      double t_swing_end = modeSchedule.eventTimes.back();
      double t_peak = 0.5 * (t_trigger + t_swing_end);

      for (size_t i = 0; i < modeSchedule.modeSequence.size(); ++i) {
        double phaseTime = modeSchedule.eventTimes[i];
        if (phaseTime >= t_trigger && phaseTime <= t_peak) {
          double alpha = (phaseTime - t_trigger) / (t_peak - t_trigger);
          liftOffHeights[i] += stepReflexCount_[leg] * stepReflexHeight_ * alpha;
          touchDownHeights[i] += stepReflexCount_[leg] * stepReflexHeight_ * alpha;
        } else if (phaseTime > t_peak && phaseTime <= t_swing_end) {
          double beta = (t_swing_end - phaseTime) / (t_swing_end - t_peak);
          liftOffHeights[i] += stepReflexCount_[leg] * stepReflexHeight_ * beta;
          touchDownHeights[i] += stepReflexCount_[leg] * stepReflexHeight_ * beta;
        }
      }
    }

    liftOffHeightSequence[leg] = liftOffHeights;
    touchDownHeightSequence[leg] = touchDownHeights;
  }
  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}

void AdaptivePlannerReferenceManager::triggerStepReflex(size_t leg, scalar_t time) {
  stepReflexTriggered_[leg] = true;
  stepReflexCount_[leg] += 1;
  reflexTriggerTime_[leg] = time;

  std::cout << "TRIGGER!!!" << std::endl;
}

}  // namespace legged_robot
}  // namespace ocs2

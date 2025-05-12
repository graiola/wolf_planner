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
                                                                     std::shared_ptr<OdomEstimator> odomEstimatorPtr,
                                                                     const EndEffectorKinematics<scalar_t> &endEffectorKinematics,
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
          forceThreshold_(forceThreshold)
    {
      for (auto& reflex : reflexControllers_)
        reflex.configure(1.0, stepReflexHeight);
    }

void AdaptivePlannerReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                        TargetTrajectories &targetTrajectories, ModeSchedule &modeSchedule)
{
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  terrainEstimatorPtr_->update();
  contactForcesEstimatorPtr_->update();
  odomEstimatorPtr_->update();
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

    Eigen::Vector3d footForce, baseLinearVel;

    for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
    {

      const std::string& contactName = contactForcesEstimatorPtr_->getContactNames()[leg];

      // Read foot contact force
      footForce << contactForcesEstimatorPtr_->getContactForces()[leg][0],
                   contactForcesEstimatorPtr_->getContactForces()[leg][1],
                   contactForcesEstimatorPtr_->getContactForces()[leg][2];

      // Read the base linear velocity
      baseLinearVel << odomEstimatorPtr_->getBaseLinearVelocity()[0],
                       odomEstimatorPtr_->getBaseLinearVelocity()[1],
                       odomEstimatorPtr_->getBaseLinearVelocity()[2];

      // Rotate force into foot frame
      //const auto &footRotation = pinocchioInterface_.getData().oMf[pinocchioInterface_.getModel().getFrameId(contactName[leg])].rotation();
      //const Eigen::Vector3d contactForceRotated = footRotation.transpose() * footForce;

      // Rotate force into swing frame taking into account the base velocity
      // Normalize base linear velocity to get movement direction
      Eigen::Vector3d direction = baseLinearVel;
      if (direction.norm() > 1e-3) {
          direction.normalize();
      } else {
          direction = Eigen::Vector3d::UnitX(); // default direction if nearly zero velocity
      }
      // Construct swing frame axes (X = movement, Z = up, Y = side)
      Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
      Eigen::Vector3d y_axis = z_axis.cross(direction).normalized();
      Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized(); // orthogonal correction

      Eigen::Matrix3d swingRotation;
      swingRotation.col(0) = x_axis;
      swingRotation.col(1) = y_axis;
      swingRotation.col(2) = z_axis;

      // Rotate contact force into swing frame
      Eigen::Vector3d contactForceRotated = swingRotation.transpose() * footForce;

      double angle = std::atan2(contactForceRotated.z(), contactForceRotated.x());
      bool forceInsideLimits = false;
      if ((angle < -120.0 * (M_PI / 180.0))||(angle > 140.0 * (M_PI / 180.0)))
        forceInsideLimits = true;

      const double forceNormXZ = std::hypot(contactForceRotated.x(), contactForceRotated.z());

      const bool impact = forceInsideLimits && forceNormXZ > forceThreshold_;

      if (impact && !reflexControllers_[leg].isActive())
      {
        std::cout << "[AdaptivePlannerReferenceManager] TRIGGER reflex for contact "<< contactName << std::endl;
        reflexControllers_[leg].trigger(endEffectorKinematicsPtr_->getPosition(state)[leg]);
      }
      else
      {
        if (!reflexControllers_[leg].isActive())
        {
          //std::cout << "[AdaptivePlannerReferenceManager] RESET reflex for contact "<< contactName << std::endl;
          reflexControllers_[leg].reset();
        }
      }
    }

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

    reflexControllers_[leg].update(1.0 / 250.0);

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

  std::dynamic_pointer_cast<SwingTrajectoryPlannerXY>(swingTrajectoryPtr_)->updateXY(modeSchedule,
                                                                                    liftOffHeightSequence,
                                                                                    touchDownHeightSequence,
                                                                                    maxHeightSequence,
                                                                                    liftOffXSequence,
                                                                                    touchDownXSequence,
                                                                                    liftOffYSequence,
                                                                                    touchDownYSequence);
}

} // namespace legged_robot
} // namespace ocs2

#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AdaptivePlannerReferenceManager::AdaptivePlannerReferenceManager(CentroidalModelInfo info,
                                                             std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             std::shared_ptr<TerrainEstimator> terrainEstimatorPtr,
                                                             scalar_t comHeight)
    : LeggedReferenceManager(info,gaitSchedulePtr,swingTrajectoryPtr),
      info_(std::move(info)),
      terrainEstimatorPtr_(std::move(terrainEstimatorPtr)),
      comHeight_(comHeight)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AdaptivePlannerReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  terrainEstimatorPtr_->update();

  const scalar_t terrainHeight = terrainEstimatorPtr_->getTerrainCenter().z();

  // Modify targetTrajectory to adapt the base posture wrt terrain
  TargetTrajectories newTargetTrajectories;

  size_t nodeNum = targetTrajectories.timeTrajectory.size();
  for (size_t i = 0; i < nodeNum; ++i) {
    scalar_t time  = targetTrajectories.timeTrajectory[i];
    vector_t state = targetTrajectories.getDesiredState(time);
    vector_t input = targetTrajectories.getDesiredInput(time);

    vector_t pos = centroidal_model::getBasePose(state, info_).head(3);

    // Base Orientation
    auto normalVector = terrainEstimatorPtr_->getTerrainNormal();
    normalVector.normalize();
    matrix3_t R;
    scalar_t z = centroidal_model::getBasePose(state, info_)(3);
    R << cos(z), -sin(z), 0,  // clang-format off
         sin(z), cos(z),  0,
         0,      0,       1;  // clang-format on
    vector_t v = R.transpose() * normalVector;
    centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());

    // Base Z Position
    centroidal_model::getBasePose(state, info_)(2) =
        terrainHeight + comHeight_ / cos(centroidal_model::getBasePose(state, info_)(4));

    newTargetTrajectories.timeTrajectory.push_back(time);
    newTargetTrajectories.stateTrajectory.push_back(state);
    newTargetTrajectories.inputTrajectory.push_back(input);
  }
  targetTrajectories = newTargetTrajectories;

  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

}  // namespace legged_robot
}  // namespace ocs2

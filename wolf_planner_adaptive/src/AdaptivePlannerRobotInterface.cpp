#include "wolf_planner_adaptive/AdaptivePlannerRobotInterface.h"

#include "wolf_planner_adaptive/AdaptivePlannerPreComputation.h"
#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"
#include "wolf_planner_adaptive/TerrainEstimator.h"
#include "wolf_planner_adaptive/ContactForcesEstimator.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace wolf_planner {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AdaptivePlannerRobotInterface::AdaptivePlannerRobotInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool useHardFrictionConeConstraint)
  : LeggedInterface(taskFile,urdfFile,referenceFile,useHardFrictionConeConstraint)
{
}

void AdaptivePlannerRobotInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/, const std::string& referenceFile, bool verbose) {
  //auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
  auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlannerXY>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
  auto terrainEstimator = std::make_unique<TerrainEstimator>();
  auto contactForcesEstimator = std::make_unique<ContactForcesEstimator>();
  scalar_t comHeight = 0;
  scalar_t stepReflexHeight = 0;
  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  loadData::loadCppDataType(referenceFile, "stepReflexHeight", stepReflexHeight);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({modelSettings_.contactNames3DoF}, "all_feet");

  referenceManagerPtr_ = std::make_shared<AdaptivePlannerReferenceManager>(*pinocchioInterfacePtr_,centroidalModelInfo_,loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner),
                                                                           std::move(terrainEstimator),std::move(contactForcesEstimator),*eeKinematicsPtr,comHeight,stepReflexHeight,modelSettings_.contactNames3DoF);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AdaptivePlannerRobotInterface::setupPreComputation(const std::string& /*taskFile*/, const std::string& /*urdfFile*/, const std::string& /*referenceFile*/, bool /*verbose*/) {
  problemPtr_->preComputationPtr = std::make_unique<AdaptivePlannerPreComputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_,
        *referenceManagerPtr_->getSwingTrajectoryPlanner(),
        *dynamic_cast<AdaptivePlannerReferenceManager&>(*referenceManagerPtr_).getTerrainEstimator(), modelSettings_);
}

}  // namespace wolf_planner

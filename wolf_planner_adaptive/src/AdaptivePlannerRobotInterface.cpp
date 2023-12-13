#include "wolf_planner_adaptive/AdaptivePlannerRobotInterface.h"

#include "wolf_planner_adaptive/AdaptivePlannerPreComputation.h"
#include "wolf_planner_adaptive/AdaptivePlannerReferenceManager.h"
#include "wolf_planner_adaptive/TerrainEstimator.h"

namespace wolf_planner {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AdaptivePlannerRobotInterface::AdaptivePlannerRobotInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool useHardFrictionConeConstraint)
  : LeggedInterface(taskFile,urdfFile,referenceFile,useHardFrictionConeConstraint)
{
}

void AdaptivePlannerRobotInterface::setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose) {
  auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
  auto terrainEstimator = std::make_unique<TerrainEstimator>();
  scalar_t comHeight = 0;
  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  referenceManagerPtr_ = std::make_shared<AdaptivePlannerReferenceManager>(centroidalModelInfo_,loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner),
                                                                           std::move(terrainEstimator),comHeight);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AdaptivePlannerRobotInterface::setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose) {
  problemPtr_->preComputationPtr = std::make_unique<AdaptivePlannerPreComputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_,
        *referenceManagerPtr_->getSwingTrajectoryPlanner(),
        *dynamic_cast<AdaptivePlannerReferenceManager&>(*referenceManagerPtr_).getTerrainEstimator(), modelSettings_);
}

}  // namespace wolf_planner

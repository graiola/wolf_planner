#include "wolf_planner_adaptive/AdaptivePlannerRobotInterface.h"

namespace wolf_planner {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AdaptivePlannerRobotInterface::setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
  auto terrainEstimator = std::make_unique<TerrainEstimator>();
  scalar_t comHeight = 0;
  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  referenceManagerPtr_ = std::make_shared<LeggedReferenceManager>(centroidalModelInfo_,loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner),
                                                                         std::move(terrainEstimator),comHeight);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AdaptivePlannerRobotInterface::setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                          bool verbose) {
  problemPtr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), *referenceManagerPtr_->getTerrainEstimator(), modelSettings_);
}

}  // namespace wolf_planner

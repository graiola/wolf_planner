#include "wolf_planner_interface/synchronizer/TerrainEstimationReceiver.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TerrainEstimationReceiver::TerrainEstimationReceiver(::ros::NodeHandle nodeHandle, std::shared_ptr<OptimalControlProblem> optmialControlProblemPtr, const std::string& robotName)
    : ptr_(optmialControlProblemPtr), updated_(false) {
  subscriber_ = nodeHandle.subscribe(robotName + "/wolf_controller/terrain_estimation", 1, &TerrainEstimationReceiver::terrainEstimationCallback, this,
                                                    ::ros::TransportHints().udp());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainEstimationReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ReferenceManagerInterface& referenceManager) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mtx_);
    //gaitSchedulePtr_->insertModeSequenceTemplate(receivedGait_, finalTime, timeHorizon);
    updated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainEstimationReceiver::terrainEstimationCallback(const wolf_msgs::TerrainEstimation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  //receivedGait_ = readModeSequenceTemplateMsg(*msg);
  //updated_ = true;
}

}  // namespace legged_robot
}  // namespace ocs2

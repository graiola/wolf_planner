#include "wolf_planner_interface/synchronizer/TerrainEstimationReceiver.h"

namespace wolf_planner {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TerrainEstimationReceiver::TerrainEstimationReceiver(::ros::NodeHandle nodeHandle, std::shared_ptr<LeggedInterface> ptr, const std::string& robotName)
    : ptr_(ptr), updated_(false) {
  terrain_normal_ << 0.0, 0.0, 1.0;
  subscriber_ = nodeHandle.subscribe("/wolf_controller/terrain_estimation", 1, &TerrainEstimationReceiver::terrainEstimationCallback, this,
                                                    ::ros::TransportHints().udp());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainEstimationReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ReferenceManagerInterface& referenceManager) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mtx_);
    ptr_->setTerrainNormal(terrain_normal_);
    updated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainEstimationReceiver::terrainEstimationCallback(const wolf_msgs::TerrainEstimation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  terrain_normal_ << msg->terrain_normal.x, msg->terrain_normal.y, msg->terrain_normal.z;
  updated_ = true;
}

}  // namespace wolf_planner

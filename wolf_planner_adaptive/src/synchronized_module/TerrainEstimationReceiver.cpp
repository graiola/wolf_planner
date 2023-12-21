#include "wolf_planner_adaptive/synchronized_module/TerrainEstimationReceiver.h"

namespace wolf_planner {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TerrainEstimationReceiver::TerrainEstimationReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<TerrainEstimator> ptr, const std::string& robotName)
    : ptr_(ptr), updated_(false) {
  terrainNormal_ << 0.0, 0.0, 1.0;
  subscriber_ = nodeHandle.subscribe("/"+robotName+"/wolf_controller/terrain_estimation", 1, &TerrainEstimationReceiver::terrainEstimationCallback, this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainEstimationReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ReferenceManagerInterface& referenceManager) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mtx_);
    ptr_->setTerrainNormal(terrainNormal_);
    ptr_->setTerrainCenter(terrainCenter_);
    updated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TerrainEstimationReceiver::terrainEstimationCallback(const wolf_msgs::TerrainEstimation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  terrainNormal_ << msg->terrain_normal.x, msg->terrain_normal.y, msg->terrain_normal.z;
  terrainCenter_ << msg->central_point.x,  msg->central_point.y, msg->central_point.z;
  updated_ = true;
}

}  // namespace wolf_planner

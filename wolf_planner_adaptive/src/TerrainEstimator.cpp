#include "wolf_planner_adaptive/TerrainEstimator.h"

#include <iostream>


namespace ocs2 {
namespace legged_robot {

TerrainEstimator::TerrainEstimator()
  :terrainNormal_(0.0,0.0,1.0)
{
}

void TerrainEstimator::update()
{
 // TODO
}

const vector3_t& TerrainEstimator::getTerrainNormal() const
{
  return terrainNormal_;
}

const vector3_t& TerrainEstimator::getTerrainCenter() const
{
  return terrainCenter_;
}

void TerrainEstimator::setTerrainNormal(const vector3_t& terrainNormal)
{
  terrainNormal_ = terrainNormal;
}

void TerrainEstimator::setTerrainCenter(const vector3_t& terrainCenter)
{
  terrainCenter_ = terrainCenter;
}

}  // namespace legged_robot
}  // namespace ocs2

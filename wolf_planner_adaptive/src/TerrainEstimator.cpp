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

scalar_t TerrainEstimator::getTerrainHeightAt(scalar_t x, scalar_t y) const
{
  const auto normal = getTerrainNormal();
  const auto center = getTerrainCenter();

  return center.z() - (normal.x() * (x - center.x()) + normal.y() * (y - center.y())) / normal.z();
}

}  // namespace legged_robot
}  // namespace ocs2

#pragma once

#include <ocs2_legged_robot/common/Types.h>

namespace ocs2 {
namespace legged_robot {

class TerrainEstimator {
 public:

  TerrainEstimator();

  void update();

  void setTerrainNormal(const vector3_t &terrainNormal);

  void setTerrainCenter(const vector3_t& terrainCenter);

  const vector3_t& getTerrainNormal() const;

  const vector3_t& getTerrainCenter() const;

  scalar_t getTerrainHeightAt(scalar_t x, scalar_t y) const;

private:

  vector3_t terrainNormal_;
  vector3_t terrainCenter_;

};

}  // namespace legged_robot
}  // namespace ocs2

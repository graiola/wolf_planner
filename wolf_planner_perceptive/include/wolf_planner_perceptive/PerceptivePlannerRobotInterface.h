#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <wolf_planner_interface/LeggedInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace wolf_planner {
using namespace ocs2;
using namespace legged_robot;

class PerceptivePlannerRobotInterface : public LeggedInterface {
 public:
  PerceptivePlannerRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                  bool useHardFrictionConeConstraint = true);

  ~PerceptivePlannerRobotInterface() override = default;

  virtual void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  std::shared_ptr<grid_map::SignedDistanceField> getSignedDistanceFieldPtr() const { return signedDistanceFieldPtr_; }

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }

  std::shared_ptr<PinocchioSphereInterface> getPinocchioSphereInterfacePtr() const { return pinocchioSphereInterfacePtr_; }

  size_t getNumVertices() const { return numVertices_; }

 protected:

  virtual void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                             bool verbose) override;

  virtual void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                           bool verbose) override;

  size_t numVertices_ = 16;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr_;
  std::shared_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePtr_;
};

}  // namespace wolf_planner

#include "wolf_planner_perceptive_interface/PerceptiveController.h"

#include "wolf_planner_perceptive_interface/synchronized_module/PlanarTerrainReceiver.h"
#include "wolf_planner_perceptive_interface/PerceptiveLeggedInterface.h"
#include "wolf_planner_perceptive_interface/PerceptiveLeggedReferenceManager.h"

using namespace wolf_planner;
using namespace ocs2;
using namespace legged_robot;

void PerceptiveController::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  leggedInterfacePtr_ = std::make_shared<PerceptiveLeggedInterface>(taskFile, urdfFile, referenceFile, verbose);
  leggedInterfacePtr_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void PerceptiveController::setupSynchronizedModules(ros::NodeHandle& nh, std::shared_ptr<MPC_BASE> mpc)
{
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nh,
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterfacePtr_).getPlanarTerrainPtr(),
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterfacePtr_).getSignedDistanceFieldPtr(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation");
  mpc->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void PerceptiveController::setupVisualization(ros::NodeHandle& nh)
{
  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(
       *dynamic_cast<PerceptiveLeggedReferenceManager&>(*leggedInterfacePtr_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
       leggedInterfacePtr_->getCentroidalModelInfo().numThreeDofContacts, nh);

   sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(
       leggedInterfacePtr_->getPinocchioInterface(), leggedInterfacePtr_->getCentroidalModelInfo(),
       *dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterfacePtr_).getPinocchioSphereInterfacePtr(), nh);
}

void PerceptiveController::updateVisualization(const SystemObservation& currentObservation)
{
  footPlacementVisualizationPtr_->update(currentObservation);
  sphereVisualizationPtr_->update(currentObservation);
}

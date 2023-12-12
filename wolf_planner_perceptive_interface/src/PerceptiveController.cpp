#include "wolf_planner_perceptive_interface/PerceptiveController.h"

#include "wolf_planner_perceptive_interface/synchronized_module/PlanarTerrainReceiver.h"
#include "wolf_planner_perceptive_interface/PerceptiveLeggedInterface.h"
#include "wolf_planner_perceptive_interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_sqp/SqpMpc.h>

using namespace wolf_planner;
using namespace ocs2;
using namespace legged_robot;

void PerceptiveController::setup(ros::NodeHandle &nodeHandle, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose, bool visualization)
{
  setupLeggedInterface(taskFile,urdfFile,referenceFile,verbose);
  //setupSynchronizedModules(nodeHandle);
  //if(visualization)
  //  setupVisualization(nodeHandle);
}

void PerceptiveController::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  leggedInterfacePtr_ = std::make_shared<PerceptiveLeggedInterface>(taskFile, urdfFile, referenceFile, verbose);

  // Optimal control problem
  leggedInterfacePtr_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  // MPC
  //mpcPtr_ = std::make_shared<SqpMpc>(leggedInterfacePtr_->mpcSettings(), leggedInterfacePtr_->sqpSettings(),
  //                                   leggedInterfacePtr_->getOptimalControlProblem(), leggedInterfacePtr_->getInitializer());
}

void PerceptiveController::setupSynchronizedModules(ros::NodeHandle& nodeHandle)
{
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nodeHandle,
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterfacePtr_).getPlanarTerrainPtr(),
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterfacePtr_).getSignedDistanceFieldPtr(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation");
  mpcPtr_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void PerceptiveController::setupVisualization(ros::NodeHandle& nodeHandle)
{
  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(
       *dynamic_cast<PerceptiveLeggedReferenceManager&>(*leggedInterfacePtr_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
       leggedInterfacePtr_->getCentroidalModelInfo().numThreeDofContacts, nodeHandle);

   sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(
       leggedInterfacePtr_->getPinocchioInterface(), leggedInterfacePtr_->getCentroidalModelInfo(),
       *dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterfacePtr_).getPinocchioSphereInterfacePtr(), nodeHandle);
}

void PerceptiveController::updateVisualization(const SystemObservation& currentObservation)
{
  footPlacementVisualizationPtr_->update(currentObservation);
  sphereVisualizationPtr_->update(currentObservation);
}

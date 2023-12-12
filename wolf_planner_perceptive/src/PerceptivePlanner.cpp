#include "wolf_planner_perceptive_interface/PerceptivePlanner.h"

#include "wolf_planner_perceptive_interface/synchronized_module/PlanarTerrainReceiver.h"
#include "wolf_planner_perceptive_interface/PerceptiveLeggedInterface.h"
#include "wolf_planner_perceptive_interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_sqp/SqpMpc.h>

using namespace wolf_planner;
using namespace ocs2;
using namespace legged_robot;

void PerceptivePlanner::setup(ros::NodeHandle &nodeHandle, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose, bool visualization)
{
  setupLeggedInterface(taskFile,urdfFile,referenceFile,verbose);
  //setupSynchronizedModules(nodeHandle);
  //if(visualization)
  //  setupVisualization(nodeHandle);
}

void PerceptivePlanner::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose)
{
  leggedInterface_ = std::make_shared<PerceptiveLeggedInterface>(taskFile, urdfFile, referenceFile, verbose);

  // Optimal control problem
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  // MPC
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                     leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
}

void PerceptivePlanner::setupSynchronizedModules(ros::NodeHandle& nodeHandle)
{
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nodeHandle,
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPlanarTerrainPtr(),
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getSignedDistanceFieldPtr(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation");
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void PerceptivePlanner::setupVisualization(ros::NodeHandle& nodeHandle)
{
  footPlacementVisualization_ = std::make_shared<FootPlacementVisualization>(
       *dynamic_cast<PerceptiveLeggedReferenceManager&>(*leggedInterface_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
       leggedInterface_->getCentroidalModelInfo().numThreeDofContacts, nodeHandle);

   sphereVisualization_ = std::make_shared<SphereVisualization>(
       leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
       *dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPinocchioSphereInterfacePtr(), nodeHandle);
}

void PerceptivePlanner::updateVisualization(const SystemObservation& currentObservation)
{
  footPlacementVisualization_->update(currentObservation);
  sphereVisualization_->update(currentObservation);
}

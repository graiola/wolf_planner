#pragma once

#include <mutex>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>

#include <wolf_msgs/TerrainEstimation.h>

namespace ocs2 {
namespace legged_robot {

class TerrainEstimationReceiver : public SolverSynchronizedModule {
 public:
  TerrainEstimationReceiver(::ros::NodeHandle nodeHandle, std::shared_ptr<OptimalControlProblem> optmialControlProblemPtr, const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void terrainEstimationCallback(const wolf_msgs::TerrainEstimation::ConstPtr& msg);

  std::shared_ptr<OptimalControlProblem> ptr_;

  ::ros::Subscriber subscriber_;

  std::mutex mtx_;
  std::atomic_bool updated_;
};

}  // namespace legged_robot
}  // namespace ocs2

// OCS2
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

// WoLF
#include "wolf_msgs/Wrench.h"
#include "wolf_msgs/Cartesian.h"
#include "wolf_msgs/Postural.h"
#include "wolf_msgs/ControllerState.h"

// TbR
#include <legged_interface/LeggedInterface.h>
#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

// ROS
#include <ros/ros.h>

namespace legged {

using namespace ocs2;

class MpcClass {
 public:

  MpcClass() = default;
  ~MpcClass();
  bool init(ros::NodeHandle& controller_nh);
  void update();
  void starting();
  void stopping();
  void retrieveAndPublish();

 protected:

  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose);
  void setupMpc();
  void setupMrt();
  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

  // State Estimation
  SystemObservation currentObservation_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

  // MPC Output
  ros::Publisher mpcWrenchPublisher_lf_;
  ros::Publisher mpcFootPublisher_lf_;
  ros::Publisher mpcWrenchPublisher_lh_;
  ros::Publisher mpcFootPublisher_lh_;
  ros::Publisher mpcWrenchPublisher_rf_;
  ros::Publisher mpcFootPublisher_rf_;
  ros::Publisher mpcWrenchPublisher_rh_;
  ros::Publisher mpcFootPublisher_rh_;
  ros::Publisher mpcBasePublisher_;
  ros::Publisher mpcPosturalPublisher_;

  // Observation Input
  ros::Subscriber mpcObservation_;
  void observationCallback(const ocs2_msgs::mpc_observationConstPtr& msg);

  // Controller state
  ros::Subscriber controllerState_;
  void controllerStateCallback(const wolf_msgs::ControllerStateConstPtr& msg);

 private:
  std::thread mpcThread_;
  std::atomic_bool mpcRunning_{false}, controllerRunning_{false};
  benchmark::RepeatedTimer mpcTimer_;
  std::shared_ptr<SafetyChecker> safetyChecker_;
};

} // namespace legged

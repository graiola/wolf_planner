// OCS2
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

// WoLF
#include "wolf_msgs/Wrench.h"
#include "wolf_msgs/Cartesian.h"
#include "wolf_msgs/Postural.h"

// TbR
#include <legged_interface/LeggedInterface.h>

// ROS
#include <ros/ros.h>

namespace legged {

using namespace ocs2;

class MpcClass {
 public:
  MpcClass() = default;
  ~MpcClass(){
    std::cerr << "########################################################################";
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
  }
  bool init(ros::NodeHandle& controller_nh);
  void update();
  void starting();

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

 private:
  std::atomic_bool mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
};

} // namespace legged

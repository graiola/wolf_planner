// OCS2
#include <ocs2_msgs/mpc_observation.h>

// WoLF msgs
#include <wolf_msgs/Wrench.h>
#include <wolf_msgs/Cartesian.h>
#include <wolf_msgs/Postural.h>
#include <wolf_msgs/ControllerState.h>
#include <wolf_msgs/TerrainEstimation.h>

// WoLF planner interface
#include <wolf_planner_interface/PlannerInterface.h>

// ROS
#include <ros/ros.h>

namespace wolf_planner
{

using namespace ocs2;

class WolfPlannerRos
{

 public:

  WolfPlannerRos() = default;
  bool init();

 protected:

  void updatePolicyAndPublish();

  // WoLF msgs
  wolf_msgs::Wrench forceMsg_lf_, forceMsg_lh_, forceMsg_rf_, forceMsg_rh_;
  wolf_msgs::Cartesian footMsg_lf_, footMsg_lh_, footMsg_rf_, footMsg_rh_;
  wolf_msgs::Cartesian baseMsg_;
  wolf_msgs::Postural posturalMsg_;

  // Interface
  std::shared_ptr<PlannerInterface> planner_;

  // Observation publisher
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

  // System Observation
  SystemObservation observation_;

  // Observation Input
  ros::Subscriber mpcObservation_;
  void observationCallback(const ocs2_msgs::mpc_observationConstPtr& msg);

  // Controller state
  ros::Subscriber controllerState_;
  void controllerStateCallback(const wolf_msgs::ControllerStateConstPtr& msg);

  std::atomic_bool controllerRunning_{false};

};

} // namespace wolf_planner

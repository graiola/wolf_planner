#include "wolf_planner_adaptive/synchronized_module/OdomReceiver.h"

namespace wolf_planner {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OdomReceiver::OdomReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<OdomEstimator> ptr, const std::string& robotName)
    : ptr_(ptr), updated_(false) {
  subscriber_ = nodeHandle.subscribe("/"+robotName+"/odometry/robot", 10, &OdomReceiver::odomCallback, this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OdomReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ReferenceManagerInterface& referenceManager) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mtx_);
    ptr_->setBaseLinearVelocity(base_linear_vel_);
    ptr_->setBaseAngularVelocity(base_angular_vel_);
    updated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OdomReceiver::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);

  base_linear_vel_(0) = msg->twist.twist.linear.x;
  base_linear_vel_(1) = msg->twist.twist.linear.y;
  base_linear_vel_(2) = msg->twist.twist.linear.z;

  base_angular_vel_(0) = msg->twist.twist.angular.x;
  base_angular_vel_(1) = msg->twist.twist.angular.y;
  base_angular_vel_(2) = msg->twist.twist.angular.z;


  updated_ = true;
}

}  // namespace wolf_planner

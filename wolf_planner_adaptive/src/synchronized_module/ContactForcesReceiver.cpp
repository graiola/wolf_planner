#include "wolf_planner_adaptive/synchronized_module/ContactForcesReceiver.h"

namespace wolf_planner {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ContactForcesReceiver::ContactForcesReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<ContactForcesEstimator> ptr, const std::string& robotName)
    : ptr_(ptr), updated_(false) {
  contact_forces_.resize(4);
  contact_states_.resize(4);
  contact_names_.resize(4);
  subscriber_ = nodeHandle.subscribe("/"+robotName+"/wolf_controller/contact_forces", 1, &ContactForcesReceiver::contactForcesCallback, this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ContactForcesReceiver::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                const ReferenceManagerInterface& referenceManager) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mtx_);
    ptr_->setContactForces(contact_forces_);
    ptr_->setContactStates(contact_states_);
    ptr_->setContactNames(contact_names_);
    updated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ContactForcesReceiver::contactForcesCallback(const wolf_msgs::ContactForces::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  for(unsigned int i=0;i<msg->contact_forces.size();i++)
  {
    contact_forces_[i][0] = msg->contact_forces[i].force.x;
    contact_forces_[i][1] = msg->contact_forces[i].force.y;
    contact_forces_[i][2] = msg->contact_forces[i].force.z;

    contact_states_[i] = msg->contact[i];

    contact_names_[i] = msg->name[i];
  }
  updated_ = true;
}

}  // namespace wolf_planner

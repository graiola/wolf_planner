#include "wolf_planner_adaptive/ContactForcesEstimator.h"

#include <iostream>


namespace ocs2 {
namespace legged_robot {

ContactForcesEstimator::ContactForcesEstimator()
{
}

void ContactForcesEstimator::update()
{
 // TODO
}

void ContactForcesEstimator::setContactForces(const std::vector<vector3_t> &contact_forces)
{
  contactForces_ = contact_forces;
}

void ContactForcesEstimator::setContactStates(const std::vector<bool> &contact_states)
{
  contactStates_ = contact_states;
}

const std::vector<vector3_t>& ContactForcesEstimator::getContactForces() const
{
  return contactForces_;
}

const std::vector<bool>& ContactForcesEstimator::getContactStates() const
{
  return contactStates_;
}

}  // namespace legged_robot
}  // namespace ocs2

#pragma once

#include <ocs2_legged_robot/common/Types.h>

namespace ocs2 {
namespace legged_robot {

class ContactForcesEstimator {
 public:

  ContactForcesEstimator();

  void update();

  void setContactForces(const std::vector<vector3_t> &contact_forces);

  const std::vector<vector3_t>& getContactForces() const;

  void setContactStates(const std::vector<bool> &contact_forces);

  const std::vector<bool>& getContactStates() const;

private:

  std::vector<vector3_t> contactForces_;
  std::vector<bool> contactStates_;

};

}  // namespace legged_robot
}  // namespace ocs2

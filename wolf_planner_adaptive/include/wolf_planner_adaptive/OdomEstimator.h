/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <ocs2_legged_robot/common/Types.h>

namespace ocs2 {
namespace legged_robot {

class OdomEstimator {
 public:

  OdomEstimator();

  void update();

  void setBaseLinearVelocity(const vector3_t &base_linear_velocity);
  const vector3_t& getBaseLinearVelocity() const;

  void setBaseAngularVelocity(const vector3_t &base_angular_velocity);
  const vector3_t& getBaseAngularVelocity() const;

private:

  vector3_t baseLinearVel_;
  vector3_t baseAngularVel_;

};

}  // namespace legged_robot
}  // namespace ocs2

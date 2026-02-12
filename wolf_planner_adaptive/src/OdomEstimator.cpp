/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) Gennaro Raiola
 */

#include "wolf_planner_adaptive/OdomEstimator.h"

#include <iostream>


namespace ocs2 {
namespace legged_robot {

OdomEstimator::OdomEstimator()
{
  baseLinearVel_.setZero();
  baseAngularVel_.setZero();
}

void OdomEstimator::update()
{
 // TODO
}

void OdomEstimator::setBaseLinearVelocity(const vector3_t &base_linear_velocity)
{
  baseLinearVel_ = base_linear_velocity;
}

const vector3_t& OdomEstimator::getBaseLinearVelocity() const
{
  return baseLinearVel_;
}

void OdomEstimator::setBaseAngularVelocity(const vector3_t &base_angular_velocity)
{
  baseAngularVel_ = base_angular_velocity;
}

const vector3_t& OdomEstimator::getBaseAngularVelocity() const
{
  return baseAngularVel_;
}


}  // namespace legged_robot
}  // namespace ocs2

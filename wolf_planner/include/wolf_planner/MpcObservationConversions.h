#pragma once

#include <ocs2_mpc/SystemObservation.h>
#include <wolf_msgs/MpcObservation.h>

namespace wolf_planner {
namespace mpc_observation_conversions {

inline wolf_msgs::MpcObservation createMessage(const ocs2::SystemObservation& observation) {
  wolf_msgs::MpcObservation observationMsg;

  observationMsg.time = observation.time;

  observationMsg.state.value.resize(observation.state.rows());
  for (size_t i = 0; i < observation.state.rows(); ++i) {
    observationMsg.state.value[i] = static_cast<float>(observation.state(i));
  }

  observationMsg.input.value.resize(observation.input.rows());
  for (size_t i = 0; i < observation.input.rows(); ++i) {
    observationMsg.input.value[i] = static_cast<float>(observation.input(i));
  }

  observationMsg.mode = observation.mode;

  return observationMsg;
}

inline ocs2::SystemObservation readMessage(const wolf_msgs::MpcObservation& observationMsg) {
  ocs2::SystemObservation observation;

  observation.time = observationMsg.time;

  const auto& state = observationMsg.state.value;
  observation.state = Eigen::Map<const Eigen::VectorXf>(state.data(), state.size()).cast<ocs2::scalar_t>();

  const auto& input = observationMsg.input.value;
  observation.input = Eigen::Map<const Eigen::VectorXf>(input.data(), input.size()).cast<ocs2::scalar_t>();

  observation.mode = observationMsg.mode;

  return observation;
}

}  // namespace mpc_observation_conversions
}  // namespace wolf_planner

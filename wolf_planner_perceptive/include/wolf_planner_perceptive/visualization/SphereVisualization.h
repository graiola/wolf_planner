#pragma once

#include <ros/ros.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>

namespace ocs2 {
namespace legged_robot {

class SphereVisualization {
 public:
  SphereVisualization(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                      const PinocchioSphereInterface& sphereInterface, ros::NodeHandle& nh, scalar_t maxUpdateFrequency = 100.0);
  void update(const SystemObservation& observation);

 private:
  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo centroidalModelInfo_;
  const PinocchioSphereInterface& sphereInterface_;
  ros::Publisher markerPublisher_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace legged_robot
}  // namespace ocs2

#include <ros/ros.h>

#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include <utility>

namespace wolf_planner
{

using namespace ocs2;

class LeggedSelfCollisionVisualization : public GeometryInterfaceVisualization {
 public:
  LeggedSelfCollisionVisualization(PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterface,
                                   const CentroidalModelPinocchioMapping& mapping, ros::NodeHandle& nh, const std::string& tfPrefix, scalar_t maxUpdateFrequency = 50.0)
      : mappingPtr_(mapping.clone()),
        GeometryInterfaceVisualization(std::move(pinocchioInterface), std::move(geometryInterface), nh, tfPrefix+"/world"), // FIXME
        lastTime_(std::numeric_limits<scalar_t>::lowest()),
        minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}
  void update(const SystemObservation& observation) {
    if (observation.time - lastTime_ > minPublishTimeDifference_) {
      lastTime_ = observation.time;

      publishDistances(mappingPtr_->getPinocchioJointPosition(observation.state));
    }
  }

 private:
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace wolf_planner

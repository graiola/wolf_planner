#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Numerics.h>

#include "wolf_planner_adaptive/AdaptivePlannerPreComputation.h"
#include "wolf_planner_interface/SwingTrajectoryPlannerXY.h"

namespace ocs2 {
namespace legged_robot {

AdaptivePlannerPreComputation::AdaptivePlannerPreComputation(
    PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
    const SwingTrajectoryPlanner& swingTrajectoryPlanner,
    const TerrainEstimator& terrainEstimator, ModelSettings settings)
    : LeggedRobotPreComputation(pinocchioInterface, info, swingTrajectoryPlanner, settings),
      terrainEstimatorPtr_(&terrainEstimator) {
  frictionConeConConfigs_.resize(info_.numThreeDofContacts);
  eeXYVelConConfigs_.resize(info_.numThreeDofContacts);
}

AdaptivePlannerPreComputation::AdaptivePlannerPreComputation(const AdaptivePlannerPreComputation& rhs)
    : LeggedRobotPreComputation(rhs), terrainEstimatorPtr_(rhs.terrainEstimatorPtr_) {
  frictionConeConConfigs_.resize(rhs.frictionConeConConfigs_.size());
  eeXYVelConConfigs_.resize(rhs.eeXYVelConConfigs_.size());
}

void AdaptivePlannerPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  auto swingPlannerXY = dynamic_cast<const SwingTrajectoryPlannerXY*>(swingTrajectoryPlannerPtr_);
  if (!swingPlannerXY) {
    throw std::runtime_error("[AdaptivePlannerPreComputation] Swing planner must be SwingTrajectoryPlannerXY.");
  }

  // Z-direction constraint lambda
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(1) << -swingPlannerXY->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << terrainEstimatorPtr_->getTerrainNormal().transpose()).finished();
    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
      config.b(0) -= settings_.positionErrorGain * swingPlannerXY->getZpositionConstraint(footIndex, t);
      config.Ax = settings_.positionErrorGain * (matrix_t(1, 3) << terrainEstimatorPtr_->getTerrainNormal()).finished();
    }
    return config;
  };

  // XY-direction constraint lambda
  auto eeXYVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    Eigen::Vector2d xyVel = swingPlannerXY->getXYvelocityConstraint(footIndex, t);
    config.b = -xyVel;

    // Tangent vectors on the plane
    Eigen::Vector3d normal = terrainEstimatorPtr_->getTerrainNormal();
    Eigen::Matrix<double, 2, 3> tangentBasis;
    tangentBasis.row(0) = Eigen::Vector3d::UnitX() - normal.dot(Eigen::Vector3d::UnitX()) * normal;
    tangentBasis.row(1) = Eigen::Vector3d::UnitY() - normal.dot(Eigen::Vector3d::UnitY()) * normal;
    config.Av = tangentBasis;

    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
      Eigen::Vector2d xyPos = swingPlannerXY->getXYpositionConstraint(footIndex, t);
      config.b -= settings_.positionErrorGain * xyPos;
      config.Ax = settings_.positionErrorGain * tangentBasis;
    }

    return config;
  };

  // Friction cone constraint lambda
  auto frictionConeConConfig = [&](size_t footIndex) {
    FrictionConeConstraint::Config config;
    config.terrainNormal = terrainEstimatorPtr_->getTerrainNormal();
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
      eeXYVelConConfigs_[i] = eeXYVelConConfig(i);
      frictionConeConConfigs_[i] = frictionConeConConfig(i);
    }
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  vector_t q = mappingPtr_->getPinocchioJointPosition(x);
  if (request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGlobalPlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    updateCentroidalDynamics(pinocchioInterface_, info_, q);
    vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
    updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace legged_robot
}  // namespace ocs2

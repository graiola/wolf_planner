#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <wolf_planner_interface/SwingTrajectoryPlanner.h>
#include <wolf_planner_interface/LeggedReferenceManager.h>

#include "wolf_planner_adaptive/TerrainEstimator.h"
#include "wolf_planner_adaptive/ContactForcesEstimator.h"

namespace ocs2 {
namespace legged_robot {

  class StepReflexController {
    public:
     StepReflexController() = default;
   
     void configure(double swingFrequency, double stepHeight, double retractionAngleDeg = 150.0) {
       retractionAngleRad_ = retractionAngleDeg * M_PI / 180.0;
       reflexDuration_ = 0.5 / swingFrequency;
       retractionDuration_ = 0.5 * reflexDuration_;
       Kd_r_ = 10.0 / reflexDuration_;
       Kp_r_ = 0.25 * (Kd_r_ * Kd_r_);
       computeRetractionForce(stepHeight);
       reset();
     }

     const bool& isActive() const { return active_; }
   
     void trigger(const Eigen::Vector3d& initialPosition) {
       t_ = 0.0;
       r0_ = std::sqrt(std::pow(initialPosition.x(), 2) + std::pow(initialPosition.z(), 2));
       r_ = r0_;
       r_dot_ = r_ddot_ = 0.0;
       active_ = true;
     }
   
     void update(double period) {
       if (!active_) return;
   
       if (t_ >= retractionDuration_) {
         Fr_ = 0.0;
         active_ = false;
         return;
       } else {
         Fr_ = Fr_max_;
       }
   
       r_ddot_ = -Kp_r_ * r_ - Kd_r_ * r_dot_ + Fr_;
       r_dot_ += r_ddot_ * period;
       r_ += r_dot_ * period;
   
       double theta = M_PI * t_ / reflexDuration_;
       displacement_(0) = -r_ * std::cos(theta);
       displacement_(1) = 0.0;
       displacement_(2) = r_ * std::sin(theta);

       t_ += period;

       std::cout << t_ << std::endl;
     }
   
     const Eigen::Vector3d& getDisplacement() const { return displacement_; }
   
     void reset() {
       active_ = false;
       displacement_.setZero();
     }
   
    private:
     void computeRetractionForce(double maxRetraction) {
       double lambda = 5.0 / reflexDuration_;
       double t_max = (-retractionDuration_ * lambda * lambda * std::exp(retractionDuration_ * lambda)) /
                      (lambda * lambda * (1.0 - std::exp(retractionDuration_ * lambda)));
       double tmp = (1 - (1 + t_max * lambda) * std::exp(-t_max * lambda)) -
                    (1 - (1 + (t_max - retractionDuration_) * lambda) *
                             std::exp(-(t_max - retractionDuration_) * lambda));
       Fr_max_ = maxRetraction * Kp_r_ / tmp;
     }
   
     // Reflex dynamics
     double reflexDuration_ = 0.0;
     double retractionDuration_ = 0.0;
     double Kp_r_ = 0.0, Kd_r_ = 0.0;
     double r_ = 0.0, r_dot_ = 0.0, r_ddot_ = 0.0, r0_ = 0.0, Fr_ = 0.0, Fr_max_ = 0.0;
     double t0_ = 0.0;
     double retractionAngleRad_ = 0.0;
     double t_;
   
     bool active_ = false;
     Eigen::Vector3d displacement_ = Eigen::Vector3d::Zero();
   };
   

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class AdaptivePlannerReferenceManager : public LeggedReferenceManager {
 public:
  AdaptivePlannerReferenceManager(PinocchioInterface pinocchioInterface,
                                  CentroidalModelInfo info,
                                  std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                  std::shared_ptr<TerrainEstimator> terrainEstimator,
                                  std::shared_ptr<ContactForcesEstimator> contactForcesEstimator,
                                  const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                  scalar_t comHeight,
                                  scalar_t stepReflexHeight,
                                  const std::vector<std::string>& contactFrameNames);

  ~AdaptivePlannerReferenceManager() override = default;

  const std::shared_ptr<TerrainEstimator>& getTerrainEstimator() { return terrainEstimatorPtr_; }

  const std::shared_ptr<ContactForcesEstimator>& getContactForcesEstimator() { return contactForcesEstimatorPtr_; }

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) override;

  void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                    ModeSchedule& modeSchedule);

  // Members
  PinocchioInterface pinocchioInterface_;
  std::shared_ptr<TerrainEstimator> terrainEstimatorPtr_;
  std::shared_ptr<ContactForcesEstimator> contactForcesEstimatorPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  scalar_t comHeight_;

  std::vector<std::string> contactFrameNames_; 

  double stepReflexHeight_;                 // fixed extra height to add (e.g. 0.05 m for 5 cm)
  std::array<bool, 4> stepReflexTriggered_; // whether reflex triggered in current swing for each foot
  std::array<double, 4> reflexTriggerTime_; // time when reflex was triggered (for shaping the offset)
  std::array<StepReflexController, 4> reflexControllers_;

};

}  // namespace legged_robot
}  // namespace ocs2

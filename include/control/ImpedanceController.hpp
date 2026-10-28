#ifndef REHAB_EXO_CONTROL_IMPEDANCECONTROLLER_HPP
#define REHAB_EXO_CONTROL_IMPEDANCECONTROLLER_HPP

#include "utils/Types.hpp"
#include "safety/SafetyMonitor.hpp"
#include <Eigen/Dense>
#include <memory>

namespace rehab_exo {
namespace control {

/**
 * @brief Impedance Controller for rehabilitation exoskeleton
 * 
 * Implements: τ = K*(q_d - q) + D*(q̇_d - q̇) + I*(∫e) + τ_gravity_comp + τ_friction_comp
 * 
 * Adapts to user's residual strength - lower impedance = more compliant (easier to move)
 * Higher impedance = more assistance for patients with less strength
 */
class ImpedanceController {
public:
  struct ImpedanceParams {
    Eigen::VectorXd stiffness;  // K (Nm/rad)
    Eigen::VectorXd damping;    // D (Nm*s/rad)
    Eigen::VectorXd inertia;    // I (kg*m^2) - virtual inertia
    double assist_ratio;        // 0-1, adaptive based on patient progress
  };

  ImpedanceController();
  ~ImpedanceController();

  bool initialize(const ImpedanceParams& params, int num_joints);
  
  // Compute impedance control torque
  utils::ControlCommand compute_torque(const utils::JointState& current_state,
                                      const utils::JointState& desired_state,
                                      const utils::IntentState& intent);
  
  // Adaptive impedance based on patient performance
  void update_adaptive_impedance(const utils::IntentState& intent);
  
  // Gravity and friction compensation
  void set_gravity_compensation(bool enable) { gravity_comp_ = enable; }
  void set_friction_compensation(bool enable) { friction_comp_ = enable; }
  
  // Get/set parameters
  void set_parameters(const ImpedanceParams& params);
  ImpedanceParams get_parameters() const;

private:
  Eigen::VectorXd compute_gravity_torque(const utils::JointState& state) const;
  Eigen::VectorXd compute_friction_torque(const utils::JointState& state) const;
  
  ImpedanceParams params_;
  int num_joints_;
  bool gravity_comp_;
  bool friction_comp_;
  
  // Integral term for steady-state error
  Eigen::VectorXd integral_error_;
  double dt_;
};

} // namespace control
} // namespace rehab_exo

#endif // REHAB_EXO_CONTROL_IMPEDANCECONTROLLER_HPP

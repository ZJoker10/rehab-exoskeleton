#include "control/ImpedanceController.hpp"
#include "utils/Types.hpp"
#include <cmath>
#include <iostream>

namespace rehab_exo {
namespace control {

ImpedanceController::ImpedanceController() 
  : num_joints_(0), gravity_comp_(true), friction_comp_(true), dt_(0.001) {
}

ImpedanceController::~ImpedanceController() = default;

bool ImpedanceController::initialize(const ImpedanceParams& params, int num_joints) {
  num_joints_ = num_joints;
  params_ = params;
  
  // Validate dimensions
  if (params_.stiffness.size() != num_joints ||
      params_.damping.size() != num_joints) {
    std::cerr << "[IMPEDANCE] Invalid parameter dimensions" << std::endl;
    return false;
  }
  
  integral_error_ = Eigen::VectorXd::Zero(num_joints);
  
  std::cout << "[IMPEDANCE] Controller initialized for " << num_joints 
            << " joints, assist_ratio=" << params_.assist_ratio << std::endl;
  return true;
}

utils::ControlCommand ImpedanceController::compute_torque(
    const utils::JointState& current_state,
    const utils::JointState& desired_state,
    const utils::IntentState& intent) {
  
  utils::ControlCommand cmd;
  cmd.timestamp_ns = utils::get_time_ns();
  cmd.desired_torque = Eigen::VectorXd::Zero(num_joints_);
  
  // Position and velocity errors
  Eigen::VectorXd pos_error = desired_state.position - current_state.position;
  Eigen::VectorXd vel_error = desired_state.velocity - current_state.velocity;
  
  // Integral term (anti-windup consideration needed in production)
  integral_error_ += pos_error * dt_;
  
  // Adaptive impedance based on user's intent/residual strength
  double adaptive_assist = params_.assist_ratio * intent.assist_ratio * intent.confidence;
  Eigen::VectorXd K_eff = params_.stiffness * (1.0 + adaptive_assist);
  Eigen::VectorXd D_eff = params_.damping * (1.0 + 0.5 * adaptive_assist);
  
  // Impedance control law
  cmd.desired_torque = K_eff.asDiagonal() * pos_error + 
                       D_eff.asDiagonal() * vel_error;
  
  // Add compensation terms
  if (gravity_comp_) {
    cmd.desired_torque += compute_gravity_torque(current_state);
  }
  if (friction_comp_) {
    cmd.desired_torque += compute_friction_torque(current_state);
  }
  
  // Feed forward from desired acceleration
  cmd.desired_torque += params_.inertia.asDiagonal() * desired_state.acceleration;
  
  // Store for telemetry
  cmd.desired_stiffness = K_eff;
  cmd.desired_damping = D_eff;
  
  return cmd;
}

void ImpedanceController::update_adaptive_impedance(const utils::IntentState& intent) {
  // Increase assistance if user is struggling (low EMG activation but high effort)
  // Decrease assistance as patient progresses
  double emg_mean = intent.emg_activation.mean();
  
  if (emg_mean < 0.2 && intent.confidence > 0.7) {
    // User needs more assistance
    params_.assist_ratio = std::min(1.0, params_.assist_ratio + 0.01);
  } else if (emg_mean > 0.6 && intent.confidence > 0.8) {
    // User doing well, reduce assistance for active rehabilitation
    params_.assist_ratio = std::max(0.0, params_.assist_ratio - 0.005);
  }
}

Eigen::VectorXd ImpedanceController::compute_gravity_torque(const utils::JointState& state) const {
  // Simplified gravity compensation - in production use full dynamic model
  Eigen::VectorXd tau_g = Eigen::VectorXd::Zero(num_joints_);
  // Example: shoulder/elbow gravity terms
  if (num_joints_ >= 2) {
    tau_g(0) = 5.0 * std::cos(state.position(0));
    tau_g(1) = 3.0 * std::cos(state.position(0) + state.position(1));
  }
  return tau_g;
}

Eigen::VectorXd ImpedanceController::compute_friction_torque(const utils::JointState& state) const {
  // Coulomb + viscous friction compensation
  Eigen::VectorXd tau_f = Eigen::VectorXd::Zero(num_joints_);
  double coulomb = 0.5;
  double viscous = 0.1;
  
  for (int i = 0; i < num_joints_; ++i) {
    tau_f(i) = coulomb * std::sign(state.velocity(i)) + viscous * state.velocity(i);
  }
  return tau_f;
}

void ImpedanceController::set_parameters(const ImpedanceParams& params) {
  params_ = params;
}

ImpedanceController::ImpedanceParams ImpedanceController::get_parameters() const {
  return params_;
}

} // namespace control
} // namespace rehab_exo

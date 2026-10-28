#include "hardware_interface/WindowMotorInterface.hpp"
#include "utils/Types.hpp"
#include <cmath>
#include <iostream>

namespace rehab_exo {
namespace hardware_interface {

WindowMotorInterface::WindowMotorInterface(int joint_id, const WindowMotorParams& params)
  : joint_id_(joint_id), params_(params), encoder_count_(0),
    last_encoder_time_(0), last_position_(0), motor_current_(0),
    supply_voltage_(12.0), motor_temp_(25.0),
    limit_switch_min_(false), limit_switch_max_(false) {
}

WindowMotorInterface::~WindowMotorInterface() {
  disable();
}

bool WindowMotorInterface::initialize() {
  std::cout << "[WINDOW_MOTOR] Initializing joint " << joint_id_ 
            << " with gear ratio " << params_.gear_ratio << ":1" << std::endl;
  
  // Configure encoder
  // Configure H-bridge (BTS7960/IBT-2)
  // Configure current sensing (ACS712 or inline resistor)
  // Configure limit switches
  
  encoder_count_ = 0;
  last_encoder_time_ = utils::get_time_ns() / 1e9;
  enabled_ = false;
  
  return true;
}

bool WindowMotorInterface::enable() {
  if (motor_temp_ > 80.0) {
    std::cerr << "[WINDOW_MOTOR] Motor too hot, cannot enable" << std::endl;
    return false;
  }
  enabled_ = true;
  std::cout << "[WINDOW_MOTOR] Joint " << joint_id_ << " enabled" << std::endl;
  return true;
}

void WindowMotorInterface::disable() {
  enabled_ = false;
  write_pwm(0);
  if (params_.self_locking) {
    // Worm gear self-locks, no brake needed
  } else {
    apply_brake();
  }
  std::cout << "[WINDOW_MOTOR] Joint " << joint_id_ << " disabled" << std::endl;
}

bool WindowMotorInterface::set_torque(double torque_nm) {
  if (!enabled_) return false;
  if (check_overcurrent()) {
    disable();
    return false;
  }
  if (limit_switch_min_ && torque_nm < 0) return false;
  if (limit_switch_max_ && torque_nm > 0) return false;
  
  // Convert joint torque to motor torque (accounting for gear ratio and efficiency)
  double efficiency = 0.7; // typical worm gear efficiency
  double motor_torque = (torque_nm / params_.gear_ratio) / efficiency;
  
  // Convert to PWM (voltage control with current limiting)
  // More sophisticated: use current feedback PI control
  double max_motor_torque = params_.stall_torque_nm * (motor_current_ < params_.current_limit_a ? 1.0 : 0.5);
  motor_torque = std::clamp(motor_torque, -max_motor_torque, max_motor_torque);
  
  int duty = static_cast<int>((motor_torque / params_.stall_torque_nm) * 255.0);
  write_pwm(duty);
  
  return true;
}

bool WindowMotorInterface::set_position(double position_rad) {
  // Position control typically done by higher level controller
  // This is a simplified implementation
  return set_torque(0.0);
}

bool WindowMotorInterface::set_velocity(double velocity_rad_s) {
  return set_torque(0.0);
}

utils::JointState WindowMotorInterface::read_state() {
  utils::JointState state;
  state.position = Eigen::VectorXd::Zero(1);
  state.velocity = Eigen::VectorXd::Zero(1);
  state.torque = Eigen::VectorXd::Zero(1);
  state.temperature = Eigen::VectorXd::Constant(1, motor_temp_);
  
  update_encoder();
  state.position(0) = motor_to_joint_position(encoder_count_ / params_.encoder_ppr * 2 * M_PI);
  
  return state;
}

double WindowMotorInterface::read_current() {
  // Read from current sensor (ADC)
  return motor_current_;
}

double WindowMotorInterface::read_temperature() {
  return motor_temp_;
}

bool WindowMotorInterface::check_overcurrent() {
  if (motor_current_ > params_.peak_current_a) {
    std::cerr << "[WINDOW_MOTOR] PEAK OVERCURRENT: " << motor_current_ << "A" << std::endl;
    return true;
  }
  if (motor_current_ > params_.current_limit_a) {
    // Log warning, could derate
  }
  return false;
}

void WindowMotorInterface::apply_brake() {
  // Short H-bridge for dynamic braking
  write_pwm(0);
}

bool WindowMotorInterface::is_at_limit() const {
  return limit_switch_min_ || limit_switch_max_;
}

void WindowMotorInterface::update_encoder() {
  // Quadrature decoder update
  auto now = utils::get_time_ns() / 1e9;
  // Velocity calculation from encoder delta
  last_encoder_time_ = now;
}

double WindowMotorInterface::motor_to_joint_position(double motor_pos) const {
  return motor_pos / params_.gear_ratio;
}

double WindowMotorInterface::motor_to_joint_velocity(double motor_vel) const {
  return motor_vel / params_.gear_ratio;
}

double WindowMotorInterface::joint_to_motor_torque(double joint_torque) const {
  double efficiency = 0.7;
  return joint_torque * params_.gear_ratio * efficiency;
}

void WindowMotorInterface::write_pwm(int duty) {
  // Duty: -255 to 255
  duty = std::clamp(duty, -255, 255);
  // Write to H-bridge driver (BTS7960 has RPWM/LPWM)
}

void WindowMotorInterface::set_direction(bool forward) {
  // Control H-bridge direction
}

} // namespace hardware_interface
} // namespace rehab_exo

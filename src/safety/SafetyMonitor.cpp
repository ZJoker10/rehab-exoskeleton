#include "safety/SafetyMonitor.hpp"
#include "utils/Types.hpp"
#include <algorithm>
#include <iostream>

namespace rehab_exo {
namespace safety {

SafetyMonitor::SafetyMonitor() 
  : state_(utils::SafetyState::OK),
    estop_active_(false) {
  estop_handler_ = std::make_unique<EStopHandler>();
  watchdog_ = std::make_unique<Watchdog>(100); // 100ms timeout
  current_status_ = {utils::SafetyState::OK, "", false, true, true, utils::get_time_ns()};
}

SafetyMonitor::~SafetyMonitor() = default;

bool SafetyMonitor::initialize(const JointLimits& limits) {
  limits_ = limits;
  
  // Validate limits
  if (limits_.pos_min.size() != limits_.pos_max.size() ||
      limits_.vel_max.size() != limits_.torque_max.size()) {
    std::cerr << "[SAFETY] Invalid limit dimensions" << std::endl;
    return false;
  }
  
  // Start watchdog
  watchdog_->start();
  
  std::cout << "[SAFETY] SafetyMonitor initialized with " 
            << limits_.pos_min.size() << " joints" << std::endl;
  return true;
}

utils::SafetyStatus SafetyMonitor::check_safety(const utils::JointState& joint_state) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  
  // E-stop has highest priority
  if (estop_active_ || estop_handler_->is_triggered()) {
    current_status_ = {utils::SafetyState::EMERGENCY_STOP, 
                       fault_reason_.empty() ? "E-STOP ACTIVE" : fault_reason_,
                       true, watchdog_->is_ok(), false, utils::get_time_ns()};
    state_ = utils::SafetyState::EMERGENCY_STOP;
    return current_status_;
  }
  
  // Watchdog check
  if (!watchdog_->is_ok()) {
    register_fault("Watchdog timeout - control loop unresponsive");
    current_status_ = {utils::SafetyState::EMERGENCY_STOP, 
                       fault_reason_, true, false, false, utils::get_time_ns()};
    state_ = utils::SafetyState::EMERGENCY_STOP;
    return current_status_;
  }
  
  // Check all limits
  bool limits_ok = check_position_limits(joint_state) && 
                   check_velocity_limits(joint_state) && 
                   check_torque_limits(joint_state) && 
                   check_temperature_limits(joint_state);
  
  if (!limits_ok) {
    if (state_ != utils::SafetyState::EMERGENCY_STOP) {
      // Escalate to emergency stop on limit violation
      register_fault("Joint limit violation detected");
    }
    current_status_ = {utils::SafetyState::EMERGENCY_STOP, 
                       fault_reason_, false, true, false, utils::get_time_ns()};
    state_ = utils::SafetyState::EMERGENCY_STOP;
    return current_status_;
  }
  
  current_status_ = {utils::SafetyState::OK, "", false, true, true, utils::get_time_ns()};
  state_ = utils::SafetyState::OK;
  return current_status_;
}

utils::ControlCommand SafetyMonitor::enforce_limits(const utils::ControlCommand& cmd,
                                                   const utils::JointState& state) const {
  utils::ControlCommand limited_cmd = cmd;
  size_t n_joints = cmd.desired_torque.size();
  
  // Torque saturation
  for (size_t i = 0; i < n_joints; ++i) {
    limited_cmd.desired_torque(i) = std::clamp(cmd.desired_torque(i),
                                               limits_.torque_min(i),
                                               limits_.torque_max(i));
  }
  
  // Velocity-dependent torque limiting (optional additional safety)
  for (size_t i = 0; i < n_joints; ++i) {
    double vel = std::abs(state.velocity(i));
    if (vel > 0.8 * limits_.vel_max(i)) {
      // Reduce torque near velocity limit
      double reduction = 1.0 - (vel - 0.8*limits_.vel_max(i)) / (0.2*limits_.vel_max(i));
      limited_cmd.desired_torque(i) *= std::max(0.1, reduction);
    }
  }
  
  return limited_cmd;
}

void SafetyMonitor::trigger_estop(const std::string& reason) {
  fault_reason_ = reason;
  estop_active_ = true;
  estop_handler_->trigger();
  std::cerr << "[SAFETY] EMERGENCY STOP TRIGGERED: " << reason << std::endl;
}

bool SafetyMonitor::is_estop_active() const {
  return estop_active_.load() || estop_handler_->is_triggered();
}

void SafetyMonitor::reset_estop() {
  // In production: require two-hand reset, authenticated access
  if (estop_handler_->can_reset()) {
    estop_active_ = false;
    estop_handler_->reset();
    fault_reason_.clear();
    state_ = utils::SafetyState::OK;
    std::cout << "[SAFETY] E-STOP reset" << std::endl;
  } else {
    std::cerr << "[SAFETY] E-STOP reset not authorized" << std::endl;
  }
}

void SafetyMonitor::feed_watchdog() {
  if (watchdog_) watchdog_->feed();
}

bool SafetyMonitor::is_watchdog_ok() const {
  return watchdog_ ? watchdog_->is_ok() : false;
}

utils::SafetyStatus SafetyMonitor::get_status() const {
  std::lock_guard<std::mutex> lock(status_mutex_);
  return current_status_;
}

void SafetyMonitor::register_fault(const std::string& reason) {
  fault_reason_ = reason;
  std::cerr << "[SAFETY] FAULT: " << reason << std::endl;
}

bool SafetyMonitor::check_position_limits(const utils::JointState& state) const {
  for (size_t i = 0; i < state.position.size(); ++i) {
    if (state.position(i) < limits_.pos_min(i) - 0.01 ||
        state.position(i) > limits_.pos_max(i) + 0.01) {
      std::cerr << "[SAFETY] Joint " << i << " position out of bounds: " 
                << state.position(i) << std::endl;
      return false;
    }
  }
  return true;
}

bool SafetyMonitor::check_velocity_limits(const utils::JointState& state) const {
  for (size_t i = 0; i < state.velocity.size(); ++i) {
    if (std::abs(state.velocity(i)) > limits_.vel_max(i) * 1.1) {
      std::cerr << "[SAFETY] Joint " << i << " velocity exceeded: " 
                << state.velocity(i) << std::endl;
      return false;
    }
  }
  return true;
}

bool SafetyMonitor::check_torque_limits(const utils::JointState& state) const {
  for (size_t i = 0; i < state.torque.size(); ++i) {
    if (std::abs(state.torque(i)) > limits_.torque_max(i) * 1.05) {
      std::cerr << "[SAFETY] Joint " << i << " torque exceeded: " 
                << state.torque(i) << std::endl;
      return false;
    }
  }
  return true;
}

bool SafetyMonitor::check_temperature_limits(const utils::JointState& state) const {
  for (size_t i = 0; i < state.temperature.size(); ++i) {
    if (state.temperature(i) > TEMP_CRITICAL) {
      std::cerr << "[SAFETY] Joint " << i << " critical temperature: " 
                << state.temperature(i) << std::endl;
      return false;
    }
    if (state.temperature(i) > TEMP_WARNING) {
      std::cerr << "[SAFETY] Joint " << i << " temperature warning: " 
                << state.temperature(i) << std::endl;
    }
  }
  return true;
}

} // namespace safety
} // namespace rehab_exo

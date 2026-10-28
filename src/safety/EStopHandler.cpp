#include "safety/EStopHandler.hpp"
#include <iostream>

namespace rehab_exo {
namespace safety {

EStopHandler::EStopHandler() 
  : triggered_(false), hardware_triggered_(false), estop_pin_(-1) {
}

EStopHandler::~EStopHandler() = default;

void EStopHandler::trigger() {
  if (!triggered_) {
    triggered_ = true;
    trigger_time_ = std::chrono::steady_clock::now();
    std::cout << "[ESTOP] Hardware/Software E-STOP engaged - actuators disabled" << std::endl;
    // In production: disable motor drivers, apply brake
  }
}

bool EStopHandler::is_triggered() const {
  // Check hardware E-stop
  if (estop_pin_ >= 0 && read_hardware_estop()) {
    return true;
  }
  return triggered_.load();
}

void EStopHandler::reset() {
  std::lock_guard<std::mutex> lock(reset_mutex_);
  triggered_ = false;
  hardware_triggered_ = false;
  std::cout << "[ESTOP] E-STOP reset - system ready" << std::endl;
}

bool EStopHandler::can_reset() const {
  std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(reset_mutex_));
  
  if (!triggered_.load()) return true;
  
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - trigger_time_);
  
  // Cooldown period and manual verification required
  if (elapsed.count() >= RESET_COOLDOWN_SEC) {
    // In production: check if all faults cleared, user authenticated
    return true;
  }
  return false;
}

void EStopHandler::set_hardware_estop_pin(int pin) {
  estop_pin_ = pin;
}

bool EStopHandler::read_hardware_estop() {
  // In production: read GPIO
  return hardware_triggered_.load();
}

} // namespace safety
} // namespace rehab_exo

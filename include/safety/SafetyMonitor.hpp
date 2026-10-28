#ifndef REHAB_EXO_SAFETY_SAFETYMONITOR_HPP
#define REHAB_EXO_SAFETY_SAFETYMONITOR_HPP

#include "utils/Types.hpp"
#include "safety/EStopHandler.hpp"
#include "safety/Watchdog.hpp"
#include <atomic>
#include <mutex>
#include <vector>

namespace rehab_exo {
namespace safety {

/**
 * @brief Multi-layer safety monitor for rehabilitation exoskeleton
 * 
 * Enforces:
 * - Joint position/velocity/torque limits
 * - Emergency stop handling
 * - Hardware/software watchdog monitoring
 * - Fault detection and safe shutdown
 * - Deterministic safety checks at control rate
 */
class SafetyMonitor {
public:
  struct JointLimits {
    Eigen::VectorXd pos_min;
    Eigen::VectorXd pos_max;
    Eigen::VectorXd vel_max;
    Eigen::VectorXd torque_max;
    Eigen::VectorXd torque_min;
  };

  SafetyMonitor();
  ~SafetyMonitor();

  // Initialization
  bool initialize(const JointLimits& limits);
  
  // Safety checks (must run in real-time thread, < 10μs)
  utils::SafetyStatus check_safety(const utils::JointState& joint_state);
  
  // Limit enforcement
  utils::ControlCommand enforce_limits(const utils::ControlCommand& cmd, 
                                      const utils::JointState& state) const;
  
  // E-stop interface
  void trigger_estop(const std::string& reason);
  bool is_estop_active() const;
  void reset_estop(); // Only allowed by authorized personnel with clear checks
  
  // Watchdog
  void feed_watchdog();
  bool is_watchdog_ok() const;
  
  // Status
  utils::SafetyStatus get_status() const;
  
  // Fault registration
  void register_fault(const std::string& reason);

private:
  bool check_position_limits(const utils::JointState& state) const;
  bool check_velocity_limits(const utils::JointState& state) const;
  bool check_torque_limits(const utils::JointState& state) const;
  bool check_temperature_limits(const utils::JointState& state) const;
  
  JointLimits limits_;
  std::atomic<utils::SafetyState> state_;
  std::atomic<bool> estop_active_;
  std::string fault_reason_;
  
  std::unique_ptr<EStopHandler> estop_handler_;
  std::unique_ptr<Watchdog> watchdog_;
  
  mutable std::mutex status_mutex_;
  utils::SafetyStatus current_status_;
  
  // Safety-critical parameters
  static constexpr double TEMP_WARNING = 70.0; // °C
  static constexpr double TEMP_CRITICAL = 85.0; // °C
};

} // namespace safety
} // namespace rehab_exo

#endif // REHAB_EXO_SAFETY_SAFETYMONITOR_HPP

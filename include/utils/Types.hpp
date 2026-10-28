#ifndef REHAB_EXO_UTILS_TYPES_HPP
#define REHAB_EXO_UTILS_TYPES_HPP

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>

namespace rehab_exo {
namespace utils {

// Joint state
struct JointState {
  Eigen::VectorXd position;      // rad
  Eigen::VectorXd velocity;      // rad/s
  Eigen::VectorXd acceleration;  // rad/s^2
  Eigen::VectorXd torque;        // Nm
  Eigen::VectorXd temperature;   // °C
};

// Human intent estimation
struct IntentState {
  Eigen::VectorXd emg_activation;  // normalized 0-1
  Eigen::VectorXd desired_velocity; // rad/s
  Eigen::VectorXd assist_ratio;     // 0-1 (0=full passive, 1=full active assist)
  double confidence;                // 0-1
};

// Control command
struct ControlCommand {
  Eigen::VectorXd desired_torque;   // Nm
  Eigen::VectorXd desired_position; // rad
  Eigen::VectorXd desired_stiffness;// Nm/rad
  Eigen::VectorXd desired_damping;  // Nm*s/rad
  uint64_t timestamp_ns;
};

// Safety status
enum class SafetyState {
  OK,
  WARNING,
  FAULT,
  EMERGENCY_STOP
};

struct SafetyStatus {
  SafetyState state;
  std::string fault_reason;
  bool estop_active;
  bool watchdog_ok;
  bool limits_ok;
  uint64_t timestamp_ns;
};

// Timing utilities
inline uint64_t get_time_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()
  ).count();
}

} // namespace utils
} // namespace rehab_exo

#endif // REHAB_EXO_UTILS_TYPES_HPP

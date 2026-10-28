#ifndef REHAB_EXO_SAFETY_ESTOPHANDLER_HPP
#define REHAB_EXO_SAFETY_ESTOPHANDLER_HPP

#include <atomic>
#include <mutex>
#include <chrono>

namespace rehab_exo {
namespace safety {

/**
 * @brief Hardware and software emergency stop handler
 * 
 * Interfaces with physical E-stop buttons, manages reset authorization,
 * and ensures failsafe behavior (power removal to actuators)
 */
class EStopHandler {
public:
  EStopHandler();
  ~EStopHandler();

  void trigger();
  bool is_triggered() const;
  void reset();
  bool can_reset() const;
  
  // Hardware interface
  void set_hardware_estop_pin(int pin);
  bool read_hardware_estop();

private:
  std::atomic<bool> triggered_;
  std::atomic<bool> hardware_triggered_;
  std::chrono::steady_clock::time_point trigger_time_;
  mutable std::mutex reset_mutex_;
  
  int estop_pin_;
  static constexpr int RESET_COOLDOWN_SEC = 5;
};

} // namespace safety
} // namespace rehab_exo

#endif // REHAB_EXO_SAFETY_ESTOPHANDLER_HPP

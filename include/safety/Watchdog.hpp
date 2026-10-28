#ifndef REHAB_EXO_SAFETY_WATCHDOG_HPP
#define REHAB_EXO_SAFETY_WATCHDOG_HPP

#include <atomic>
#include <chrono>
#include <thread>

namespace rehab_exo {
namespace safety {

/**
 * @brief Hardware/software watchdog for control loop monitoring
 * 
 * Detects control loop hangs, deadline misses, or CPU overload
 * Triggers safe shutdown if control thread doesn't feed watchdog
 */
class Watchdog {
public:
  explicit Watchdog(int timeout_ms = 100);
  ~Watchdog();

  void start();
  void stop();
  void feed();
  bool is_ok() const;
  bool has_timed_out() const;

private:
  void monitor_loop();
  
  int timeout_ms_;
  std::atomic<bool> running_;
  std::atomic<bool> timed_out_;
  std::chrono::steady_clock::time_point last_feed_;
  std::thread monitor_thread_;
};

} // namespace safety
} // namespace rehab_exo

#endif // REHAB_EXO_SAFETY_WATCHDOG_HPP

#ifndef REHAB_EXO_TELEMETRY_THREADSAFELOGGER_HPP
#define REHAB_EXO_TELEMETRY_THREADSAFELOGGER_HPP

#include "utils/RingBuffer.hpp"
#include "utils/Types.hpp"
#include <thread>
#include <atomic>
#include <fstream>
#include <string>
#include <mutex>

namespace rehab_exo {
namespace telemetry {

/**
 * @brief Asynchronous thread-safe logger for high-frequency telemetry
 * 
 * Critical: Logging must not block the real-time control thread (1kHz)
 * Uses lock-free ring buffer + dedicated logging thread
 */
class ThreadSafeLogger {
public:
  struct LogEntry {
    uint64_t timestamp_ns;
    utils::JointState joint_state;
    utils::ControlCommand command;
    utils::IntentState intent;
    utils::SafetyStatus safety_status;
    double cycle_time_us;
  };

  ThreadSafeLogger();
  ~ThreadSafeLogger();

  bool initialize(const std::string& log_file, size_t buffer_size = 100000);
  void start();
  void stop();
  
  // Non-blocking log call - safe from real-time thread
  void log(const LogEntry& entry);
  
  // Flush remaining entries
  void flush();
  
  size_t get_dropped_count() const { return dropped_count_.load(); }
  bool is_overflowing() const { return dropped_count_.load() > 0; }

private:
  void logging_thread_func();
  
  std::unique_ptr<utils::RingBuffer<LogEntry>> buffer_;
  std::thread logging_thread_;
  std::atomic<bool> running_;
  std::ofstream log_file_;
  std::mutex file_mutex_;
  
  std::atomic<size_t> dropped_count_;
  std::atomic<size_t> logged_count_;
  
  static constexpr size_t DEFAULT_BUFFER_SIZE = 100000;
};

} // namespace telemetry
} // namespace rehab_exo

#endif // REHAB_EXO_TELEMETRY_THREADSAFELOGGER_HPP

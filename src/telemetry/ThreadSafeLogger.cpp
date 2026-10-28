#include "telemetry/ThreadSafeLogger.hpp"
#include <iostream>
#include <iomanip>

namespace rehab_exo {
namespace telemetry {

ThreadSafeLogger::ThreadSafeLogger() 
  : running_(false), dropped_count_(0), logged_count_(0) {
}

ThreadSafeLogger::~ThreadSafeLogger() {
  stop();
}

bool ThreadSafeLogger::initialize(const std::string& log_file, size_t buffer_size) {
  buffer_ = std::make_unique<utils::RingBuffer<LogEntry>>(buffer_size);
  log_file_.open(log_file, std::ios::out | std::ios::binary);
  
  if (!log_file_.is_open()) {
    std::cerr << "[LOGGER] Failed to open log file: " << log_file << std::endl;
    return false;
  }
  
  // Write header
  std::lock_guard<std::mutex> lock(file_mutex_);
  log_file_ << "# Rehabilitation Exoskeleton Telemetry Log\n";
  log_file_ << "# timestamp_ns,cycle_time_us,joint0_pos,joint0_vel,...\n";
  log_file_.flush();
  
  std::cout << "[LOGGER] Initialized with buffer size " << buffer_size << std::endl;
  return true;
}

void ThreadSafeLogger::start() {
  if (running_) return;
  running_ = true;
  logging_thread_ = std::thread(&ThreadSafeLogger::logging_thread_func, this);
}

void ThreadSafeLogger::stop() {
  running_ = false;
  if (logging_thread_.joinable()) {
    logging_thread_.join();
  }
  flush();
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

void ThreadSafeLogger::log(const LogEntry& entry) {
  if (!running_) return;
  
  if (buffer_->push(entry)) {
    // OK
  } else {
    dropped_count_++;
    // In production, could trigger warning but don't block control thread
  }
}

void ThreadSafeLogger::flush() {
  // Process remaining entries
  LogEntry entry;
  int count = 0;
  while (buffer_->pop(entry) && count < 10000) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    log_file_ << entry.timestamp_ns << "," << entry.cycle_time_us;
    for (int i = 0; i < entry.joint_state.position.size(); ++i) {
      log_file_ << "," << entry.joint_state.position(i) 
                << "," << entry.joint_state.velocity(i)
                << "," << entry.joint_state.torque(i);
    }
    log_file_ << "," << entry.command.desired_torque(0) << "\n";
    count++;
  }
  log_file_.flush();
}

void ThreadSafeLogger::logging_thread_func() {
  LogEntry entry;
  while (running_ || !buffer_->empty()) {
    if (buffer_->pop(entry)) {
      std::lock_guard<std::mutex> lock(file_mutex_);
      log_file_ << entry.timestamp_ns << "," << entry.cycle_time_us;
      for (int i = 0; i < entry.joint_state.position.size(); ++i) {
        log_file_ << "," << entry.joint_state.position(i) 
                  << "," << entry.joint_state.velocity(i)
                  << "," << entry.joint_state.torque(i);
      }
      if (entry.command.desired_torque.size() > 0) {
        log_file_ << "," << entry.command.desired_torque(0);
      }
      log_file_ << "\n";
      logged_count_++;
    } else {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
}

} // namespace telemetry
} // namespace rehab_exo

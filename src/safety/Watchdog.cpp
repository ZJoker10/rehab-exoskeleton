#include "safety/Watchdog.hpp"
#include <iostream>

namespace rehab_exo {
namespace safety {

Watchdog::Watchdog(int timeout_ms) 
  : timeout_ms_(timeout_ms), running_(false), timed_out_(false) {
  last_feed_ = std::chrono::steady_clock::now();
}

Watchdog::~Watchdog() {
  stop();
}

void Watchdog::start() {
  if (running_) return;
  running_ = true;
  timed_out_ = false;
  last_feed_ = std::chrono::steady_clock::now();
  monitor_thread_ = std::thread(&Watchdog::monitor_loop, this);
}

void Watchdog::stop() {
  running_ = false;
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
}

void Watchdog::feed() {
  last_feed_ = std::chrono::steady_clock::now();
  timed_out_ = false;
}

bool Watchdog::is_ok() const {
  return !timed_out_.load();
}

bool Watchdog::has_timed_out() const {
  return timed_out_.load();
}

void Watchdog::monitor_loop() {
  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feed_);
    
    if (elapsed.count() > timeout_ms_ && !timed_out_) {
      timed_out_ = true;
      std::cerr << "[WATCHDOG] TIMEOUT! Control loop not responding for " 
                << elapsed.count() << "ms" << std::endl;
    }
  }
}

} // namespace safety
} // namespace rehab_exo

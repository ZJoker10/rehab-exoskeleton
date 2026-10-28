#include "control/ImpedanceController.hpp"
#include "safety/SafetyMonitor.hpp"
#include "telemetry/ThreadSafeLogger.hpp"
#include "utils/Types.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

using namespace rehab_exo;

int main(int argc, char** argv) {
  std::cout << "=== Rehabilitation Exoskeleton Control System ===" << std::endl;
  std::cout << "Initializing..." << std::endl;
  
  // Initialize safety monitor
  auto safety_monitor = std::make_shared<safety::SafetyMonitor>();
  safety::SafetyMonitor::JointLimits limits;
  limits.pos_min = Eigen::VectorXd::Constant(2, -1.57);
  limits.pos_max = Eigen::VectorXd::Constant(2, 1.57);
  limits.vel_max = Eigen::VectorXd::Constant(2, 2.0);
  limits.torque_max = Eigen::VectorXd::Constant(2, 50.0);
  limits.torque_min = Eigen::VectorXd::Constant(2, -50.0);
  
  if (!safety_monitor->initialize(limits)) {
    std::cerr << "Failed to initialize safety monitor" << std::endl;
    return 1;
  }
  
  // Initialize impedance controller
  auto impedance_ctrl = std::make_shared<control::ImpedanceController>();
  control::ImpedanceController::ImpedanceParams imp_params;
  imp_params.stiffness = Eigen::VectorXd::Constant(2, 100.0);
  imp_params.damping = Eigen::VectorXd::Constant(2, 10.0);
  imp_params.inertia = Eigen::VectorXd::Constant(2, 0.5);
  imp_params.assist_ratio = 0.5;
  
  if (!impedance_ctrl->initialize(imp_params, 2)) {
    std::cerr << "Failed to initialize impedance controller" << std::endl;
    return 1;
  }
  
  // Initialize telemetry logger
  auto logger = std::make_shared<telemetry::ThreadSafeLogger>();
  if (!logger->initialize("/tmp/exo_telemetry.csv")) {
    std::cerr << "Failed to initialize logger" << std::endl;
    return 1;
  }
  logger->start();
  
  std::cout << "System initialized. Starting 1kHz control loop..." << std::endl;
  
  // 1kHz real-time control loop
  std::atomic<bool> running(true);
  const int CONTROL_PERIOD_US = 1000; // 1ms = 1kHz
  
  // Simulated state
  utils::JointState current_state;
  current_state.position = Eigen::VectorXd::Zero(2);
  current_state.velocity = Eigen::VectorXd::Zero(2);
  current_state.torque = Eigen::VectorXd::Zero(2);
  current_state.temperature = Eigen::VectorXd::Constant(2, 35.0);
  
  utils::JointState desired_state;
  desired_state.position = Eigen::VectorXd::Zero(2);
  desired_state.velocity = Eigen::VectorXd::Zero(2);
  desired_state.acceleration = Eigen::VectorXd::Zero(2);
  
  utils::IntentState intent;
  intent.emg_activation = Eigen::VectorXd::Constant(2, 0.3);
  intent.assist_ratio = Eigen::VectorXd::Constant(2, 0.5);
  intent.confidence = 0.8;
  
  int cycle_count = 0;
  while (running && cycle_count < 10000) { // Run for 10 seconds demo
    auto cycle_start = std::chrono::high_resolution_clock::now();
    
    // Feed watchdog
    safety_monitor->feed_watchdog();
    
    // Safety check
    auto safety_status = safety_monitor->check_safety(current_state);
    if (safety_status.state == utils::SafetyState::EMERGENCY_STOP) {
      std::cerr << "EMERGENCY STOP - Halting control loop" << std::endl;
      break;
    }
    
    // Compute control command
    auto cmd = impedance_ctrl->compute_torque(current_state, desired_state, intent);
    
    // Enforce safety limits
    cmd = safety_monitor->enforce_limits(cmd, current_state);
    
    // Log telemetry (non-blocking)
    telemetry::ThreadSafeLogger::LogEntry entry;
    entry.timestamp_ns = utils::get_time_ns();
    entry.joint_state = current_state;
    entry.command = cmd;
    entry.intent = intent;
    entry.safety_status = safety_status;
    auto cycle_end = std::chrono::high_resolution_clock::now();
    entry.cycle_time_us = std::chrono::duration<double, std::micro>(cycle_end - cycle_start).count();
    logger->log(entry);
    
    // Update state (simplified simulation)
    current_state.position += current_state.velocity * 0.001;
    current_state.velocity += (cmd.desired_torque / 0.5) * 0.001; // simplified dynamics
    current_state.torque = cmd.desired_torque;
    
    // Sleep to maintain 1kHz
    auto sleep_end = cycle_start + std::chrono::microseconds(CONTROL_PERIOD_US);
    std::this_thread::sleep_until(sleep_end);
    
    cycle_count++;
    if (cycle_count % 1000 == 0) {
      std::cout << "Cycle " << cycle_count << " - Status: OK" << std::endl;
    }
  }
  
  logger->stop();
  std::cout << "Control loop stopped. Telemetry saved." << std::endl;
  
  return 0;
}

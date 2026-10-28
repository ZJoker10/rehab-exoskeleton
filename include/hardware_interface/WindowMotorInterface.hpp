#ifndef REHAB_EXO_HARDWARE_INTERFACE_WINDOWMOTORINTERFACE_HPP
#define REHAB_EXO_HARDWARE_INTERFACE_WINDOWMOTORINTERFACE_HPP

#include "utils/Types.hpp"
#include "hardware_interface/JointInterface.hpp"
#include <memory>

namespace rehab_exo {
namespace hardware_interface {

/**
 * @brief Interface for repurposed car window motors
 * 
 * Car window motors are commonly used in DIY exoskeletons due to:
 * - High torque at low RPM with worm gear reduction
 * - Self-locking (prevents backdriving without power)
 * - Compact and affordable
 * - Built-in limit switches in some variants
 * 
 * Challenges:
 * - High gear ratio (typically 50:1 to 300:1) causes high friction
 * - Nonlinear friction, backlash
 * - Poor backdrivability
 * - Current-based overload protection needed
 * - Requires H-bridge driver (e.g., BTS7960, IBT-2) with current sensing
 */
class WindowMotorInterface : public JointInterface {
public:
  struct WindowMotorParams {
    double gear_ratio;           // e.g., 70:1, 100:1
    double no_load_rpm;          // motor side RPM
    double stall_torque_nm;      // motor side stall torque
    double encoder_ppr;          // pulses per revolution (motor side)
    double current_limit_a;      // continuous current limit
    double peak_current_a;       // peak current limit
    bool has_hall_sensor;        // Hall effect encoder
    bool self_locking;           // worm gear
  };

  WindowMotorInterface(int joint_id, const WindowMotorParams& params);
  ~WindowMotorInterface() override;

  bool initialize() override;
  bool enable() override;
  void disable() override;
  
  // Control
  bool set_torque(double torque_nm) override;
  bool set_position(double position_rad) override;
  bool set_velocity(double velocity_rad_s) override;
  
  // Sensing
  utils::JointState read_state() override;
  double read_current() override;
  double read_temperature() override;
  
  // Safety
  bool check_overcurrent();
  void apply_brake(); // For non-self-locking or emergency
  bool is_at_limit() const;

private:
  void update_encoder();
  double motor_to_joint_position(double motor_pos) const;
  double motor_to_joint_velocity(double motor_vel) const;
  double joint_to_motor_torque(double joint_torque) const;
  
  WindowMotorParams params_;
  int joint_id_;
  
  // Encoder
  long encoder_count_;
  double last_encoder_time_;
  double last_position_;
  
  // Current sensing
  double motor_current_;
  double supply_voltage_;
  
  // Thermal
  double motor_temp_;
  
  // Limit switches
  bool limit_switch_min_;
  bool limit_switch_max_;
  
  // H-bridge driver interface (BTS7960/IBT-2)
  void write_pwm(int duty); // -255 to 255
  void set_direction(bool forward);
};

} // namespace hardware_interface
} // namespace rehab_exo

#endif // REHAB_EXO_HARDWARE_INTERFACE_WINDOWMOTORINTERFACE_HPP

# Technical Challenges & Mitigation Strategies

## 1. Low-Latency Real-Time Control & Safety

### Challenges
- **Deterministic timing**: Joint-level torque/impedance control requires 1kHz (1ms period) with < 100μs jitter
- **Safety criticality**: Failure can cause patient injury
- **Multi-threaded coordination**: Control, telemetry, HRI must not interfere

### Mitigations

#### 1.1 Real-Time Scheduling
- **Priority inheritance**: Control thread runs at `SCHED_FIFO`/`SCHED_RR` with highest priority
- **CPU affinity**: Pin control thread to isolated core
- **Memory locking**: `mlockall()` to prevent page faults
- **Preemption control**: Kernel with PREEMPT_RT patches (recommended)

#### 1.2 Safety Monitor (`include/safety/SafetyMonitor.hpp`, `src/safety/SafetyMonitor.cpp`)
```cpp
// Multi-layer checks run every control cycle (< 10μs)
- E-stop: Hardware GPIO + software triggers -> immediate actuator disable
- Watchdog: 100ms timeout, feeds every cycle, triggers E-stop on miss
- Joint limits: Position (±10% margin), velocity (1.1x), torque (1.05x)
- Temperature: Warning at 70°C, critical E-stop at 85°C
- Torque saturation: Per-joint clamping + velocity-dependent reduction
```

#### 1.3 E-Stop Handler (`include/safety/EStopHandler.hpp`)
- **Hardware E-stop**: Direct GPIO to motor drivers (bypass CPU if needed)
- **Software E-stop**: Triggered by safety monitor or supervisor
- **Reset logic**: 5s cooldown + authorization required
- **Fail-safe**: Default state is E-stop engaged

#### 1.4 Watchdog (`include/safety/Watchdog.hpp`)
- Monitors control thread heartbeat
- Separate monitoring thread (lower priority)
- Triggers emergency stop on timeout

---

## 2. Human-Robot Interaction (HRI) & Intent Detection

### Challenges
- **Noisy sensors**: EMG signals have high noise, motion artifacts, crosstalk
- **Variable capability**: Patients have different residual strength, fatigue over time
- **Smooth adaptation**: Must not feel jerky or fight user
- **Transparency**: Should feel natural, not robotic

### Mitigations

#### 2.1 Impedance Control Framework (`include/control/ImpedanceController.hpp`)
Implements the impedance control law:
$$
\tau_{cmd} = K_d (q_d - q) + D_d (\dot{q}_d - \dot{q}) + I_d \ddot{q}_d + \tau_g + \tau_f
$$

**Key features:**
- **Adaptive assistance**: `assist_ratio` adjusts based on EMG activation and confidence
- **Low impedance mode**: For active patients doing exercises
- **High impedance mode**: For passive/more impaired patients
- **Gravity compensation**: Reduces perceived weight
- **Friction compensation**: Reduces stiction

#### 2.2 Intent Estimation (`include/hri/IntentEstimator.hpp` - planned)
```cpp
// Process EMG
- Bandpass filtering (20-450 Hz)
- RMS/MAF envelope extraction
- Normalization to MVC
- Muscle synergy analysis

// Process IMUs
- Orientation estimation (complementary/Kalman filter)
- Motion intention detection
- Gait phase detection

// Fusion
- Bayesian fusion for confidence weighting
- Handle sensor dropout gracefully
```

#### 2.3 Admittance Control Alternative
For force/torque sensor based interaction:
$$
M_d \ddot{x}_d + D_d \dot{x}_d + K_d x_d = F_{ext}
$$
Allows user to guide motion by applying force.

---

## 3. Kinematics, Dynamics & Alignment

### Challenges
- **Anatomical misalignment**: Human joint centers move during motion, robot axes are fixed
- **Knee complex**: Instantaneous center of rotation moves
- **Shoulder girdle**: Highly complex, multiple DOFs
- **Singularities**: Near workspace boundaries
- **Variable geometry**: Different patient sizes

### Mitigations

#### 3.1 Misalignment Compensation
- **Floating axis mechanism**: Allow relative motion between cuffs and links
- **Force-limiting cuffs**: Compliant interfaces to reduce shear forces
- **Self-aligning joints**: Passive DOFs for anatomical alignment
- **Real-time compensation**: Estimate effective joint angles

#### 3.2 Kinematics (`include/kinematics/ExoKinematics.hpp`)
```cpp
// Forward kinematics
X = FK(q) with DH parameters or URDF

// Inverse kinematics (weighted, with constraints)
q = IK(X_d) using:
- Damped Least Squares (DLS) for singularity robustness
- Jacobian transpose for redundancy
- Null space optimization for alignment
- Joint limit avoidance
```

#### 3.3 Dynamics (`include/dynamics/ExoDynamics.hpp`)
- **Recursive Newton-Euler**: Real-time inverse dynamics for compensation
- **Lagrange formulation**: For model-based control
- **Parameter identification**: Patient-specific mass/inertia
- **Friction models**: LuGre or Dahl for accurate compensation

#### 3.4 Closed-Loop Alignment
- Monitor interaction forces/torques at cuffs
- Detect misalignment (excessive shear forces)
- Adjust virtual constraints or provide feedback
- Stop if unsafe misalignment detected

---

## 4. Data Logging & Clinical Telemetry

### Challenges
- **High bandwidth**: 1kHz * (dozens of signals) = ~10KB/s per joint
- **Real-time constraint**: Logging must not block 1kHz control loop
- **Data integrity**: Must not drop safety-critical events
- **Clinical utility**: Need meaningful metrics, not just raw data

### Mitigations

#### 4.1 Asynchronous Logging (`include/telemetry/ThreadSafeLogger.hpp`)
```cpp
Architecture:
1. Control thread: log() -> push to lock-free ring buffer (non-blocking, < 1μs)
2. Dedicated logging thread: pop from buffer, format CSV/binary, write to disk
3. Ring buffer: 100k entries, overwrites on overflow with counter
```

**Key features:**
- **Lock-free where possible**: Ring buffer with mutex only for buffer management
- **Dedicated thread**: Lower priority than control
- **Batch writing**: Reduces syscalls
- **Overflow detection**: `dropped_count_` tracked
- **Binary format option**: For efficiency

#### 4.2 Clinical Metrics (`include/telemetry/ClinicalMetrics.hpp`)
Tracks rehabilitation progress:
- **Range of Motion (ROM)**: Max/min angles per session
- **Movement smoothness**: SPARC, spectral arc length
- **Active participation**: Time with low assist_ratio
- **Muscle activation**: EMG patterns and fatigue
- **Task performance**: Success rate, completion time
- **Compensation patterns**: Detect abnormal strategies

#### 4.3 Data Management
- **Session-based logging**: Auto-increment session IDs
- **Metadata**: Patient ID (anonymized), therapy protocol, timestamps
- **Synchronization**: All data timestamped with same clock
- **Storage**: Local with optional secure upload to clinical DB
- **Privacy**: HIPAA considerations, encryption at rest

---

## Safety Validation Requirements

### Unit Tests
- SafetyMonitor limit enforcement
- E-stop triggering and reset
- Watchdog timeout behavior
- Impedance controller stability

### Integration Tests
- 1kHz loop timing and jitter
- Fault injection (sensor dropout, overcurrent)
- Emergency scenarios

### Clinical Validation
- Benchtop testing with dummy loads
- Range of motion verification
- Force/torque measurement
- Risk assessment per ISO 13485/IEC 62304

## Performance Targets
| Metric | Target | Critical |
|--------|--------|----------|
| Control loop period | 1.0ms ± 0.01ms | Yes |
| Safety check latency | < 10μs | Yes |
| E-stop response | < 1ms | Yes |
| Watchdog timeout | 100ms | Yes |
| Telemetry logging overhead | < 1μs | No |
| Intent estimation latency | < 5ms | No |

---

## 5. Actuation with Repurposed Car Window Motors

### Rationale
Project 2 in the graduation work used car window motors (DC motors with worm gear reduction) due to cost-effectiveness and availability. This approach is common in DIY rehabilitation exoskeletons but introduces specific challenges.

### Challenges

#### 5.1 Gearbox Characteristics
- **High reduction ratio**: Typically 50:1 to 300:1 worm gear reduction
- **Self-locking**: Prevents backdriving when unpowered (safety feature) but limits transparency
- **Low backdrivability**: Hard for patient to move passively with exoskeleton
- **High friction**: Coulomb friction dominates, requires aggressive compensation
- **Backlash**: Can cause control instability
- **Efficiency losses**: 60-80% efficiency due to worm gear friction

#### 5.2 Electrical Considerations
- **High stall current**: 5-15A depending on motor, requires robust drivers
- **Voltage**: Typically 12V or 24V automotive
- **Thermal**: Can overheat with prolonged use
- **H-bridge drivers**: Need BTS7960, IBT-2, or similar with current sensing
- **EMI**: Automotive environment considerations

#### 5.3 Control Challenges
- **Nonlinear friction**: Stick-slip behavior
- **Current control vs voltage control**: Better for torque control but needs sensing
- **Cogging**: At low speeds
- **Compliance**: Limited due to gearbox rigidity

### Mitigations (`include/hardware_interface/WindowMotorInterface.hpp`)

#### 5.3.1 Motor Modeling
```cpp
// Account for gearbox
τ_joint = η * n * τ_motor  // with efficiency η
ω_motor = n * ω_joint      // n = gear ratio

// Friction compensation critical
τ_friction = f_c * sign(ω) + f_v * ω + f_s * sign(ω)*exp(-(ω/ω_s)^2)
```

#### 5.3.2 Current-Based Torque Control
- Measure motor current with shunt resistor or Hall sensor
- Map current to torque: τ_motor ≈ k_t * I (with temperature compensation)
- Closed-loop current control with PI
- Overcurrent protection (peak and continuous)
- Thermal derating

#### 5.3.3 Friction & Backlash Compensation
- **Feedforward friction compensation**: LuGre model or Dahl model
- **Dither**: High-frequency dither to reduce stick-slip
- **Velocity-based compensation**: Stronger at low speeds
- **Coulomb friction observer**: Distinguish friction from load

#### 5.3.4 Safety for Window Motors
```cpp
- Current limiting: Continuous and peak limits
- Thermal monitoring: Temperature sensors or model-based
- Voltage monitoring: Detect weak batteries
- Stall detection: Current spike indicates obstruction
- Braking: Dynamic braking for non-self-locking cases
- Limit switches: Mechanical endstops
```

#### 5.3.5 Backdrivability Solutions
- **Series elastic element**: Add compliance between motor and joint
- **Cable drive**: Can improve backdrivability
- **Lower impedance control**: Compensate for high reflected inertia
- **Gravity balancing**: Springs/counterweights reduce load
- **Use larger gear ratio carefully**: Trade-off between torque and backdrivability

### Implementation Details

**Driver Selection (from Project 2):**
- BTS7960 43A H-bridge module
- IBT-2 motor driver
- Arduino UNO/MEGA for low-level control
- L298N (less ideal for high current)

**Encoder Options:**
- Hall effect sensors in motor (if available)
- Optical encoders on joint side (better for patient position)
- Potentiometers for absolute position
- Quadrature encoders

**Recommended Architecture:**
```
High-level (PC/Jetson) - 1kHz impedance control
  ↓ (CAN/Ethernet/Serial)
Low-level (MCU) - 10kHz current control loop
  ↓ (PWM + direction)
H-bridge driver (BTS7960/IBT-2)
  ↓
Car window motor + gearbox
  ↓
Joint with encoder + limit switches
```

### Testing Considerations
- Measure actual gear ratio and efficiency
- Characterize friction (Coulomb + viscous)
- Test backdrivability
- Current draw under load
- Thermal testing
- Stall torque verification
- Emergency stop with motor inertia

# Rehabilitation Exoskeleton Control System

Production-ready architecture for a lower/upper limb rehabilitation exoskeleton with safety-critical real-time control.

## Key Features
- **1kHz Real-Time Control Loop** - Deterministic torque/impedance/admittance control
- **Multi-Layer Safety** - Hardware E-stop, software watchdog, joint limiters, fault detection
- **HRI & Intent Detection** - EMG/IMU processing with adaptive impedance control
- **Kinematic Compensation** - Anatomical-mechanical misalignment handling
- **Clinical Telemetry** - Asynchronous high-frequency logging without real-time thread blocking

## Quick Start
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
./rehab_exo_node --config ../config/params/exo_config.yaml
```

See [docs/challenges.md](docs/challenges.md) for detailed technical breakdown.

# RoboRehab: A DIY Rehabilitation Exoskeleton

> *Giving people their movement back, one joint at a time.*

Built from repurposed car window motors, a whole lot of C++, and the stubborn determination to make affordable rehab robotics actually work.

## What is this?

RoboRehab is a safety-first, real-time control system for a 1-DOF/2-DOF upper-limb rehabilitation exoskeleton. Born out of a graduation project, it's designed to help patients regain range of motion through guided, adaptive therapy - without breaking the bank.

The goal: take cheap, off-the-shelf car window motors and turn them into a compliant, human-friendly assistive device that *helps* without fighting its user.

## Why window motors?

Because ingenuity beats budget. Car power window motors come with a worm gearbox, plenty of torque for light limb support, and they're practically everywhere. The catch? They're not designed for human-robot interaction. That's exactly the problem we're solving here.

## Features

- **⚡ Blazing fast control** - Deterministic 1kHz real-time loop for smooth, natural-feeling assistance
- **🛡️ Safety first, always** - Multi-layer protection with E-Stop, watchdog, joint limiters, torque saturation & thermal monitoring
- **🤝 Human-aware control** - Impedance & admittance control that adapts to the user's residual strength. More help when needed, less when they're crushing it
- **🧠 Intent detection ready** - Hooks for EMG, IMUs and force/torque sensors to estimate what the user *wants* to do
- **🔧 Misalignment compensation** - Accounts for the fact that human joints don't perfectly line up with robotic ones
- **📊 Clinical-grade telemetry** - Async, non-blocking logging so you can track progress session by session without killing your control loop
- **🔌 Window motor aware** - Custom interface for repurposed window motors with current-based torque control, friction compensation & overcurrent protection

## Architecture at a Glance

```
rehab-exoskeleton/
├── include/           # Clean C++ interfaces
│   ├── control/       # Impedance, admittance & torque controllers
│   ├── safety/        # E-Stop, watchdog & safety monitor
│   ├── hardware_interface/  # Window motor driver interface
│   ├── hri/           # Intent estimation
│   ├── kinematics/    # FK/IK & alignment handling
│   └── telemetry/     # Thread-safe async logging
├── src/               # Implementations
├── config/            # Tunable gains & safety limits (YAML)
├── docs/              # Deep dives into the engineering challenges
└── tests/             # Unit, integration & safety tests
```

## Quick Start

```bash
# Clone it
git clone https://github.com/ZJoker10/rehab-exoskeleton.git
cd rehab-exoskeleton

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Run the demo loop
./rehab_exo_node --config ../config/params/exo_config.yaml
```

## The Philosophy

Rehabilitation shouldn't be locked behind six-figure price tags. This project is about proving that with clever control, proper safety, and a bit of scrapyard magic, we can build something meaningful. 

Every line of code here is written with one question in mind: *"Would I trust this with a patient?"* If the answer isn't a resounding yes, we rethink it.

## Documentation

- [**Engineering Challenges & Solutions**](docs/challenges.md) - The nitty gritty on real-time control, HRI, kinematics, telemetry, and wrangling window motors
- [**Safety Limits Config**](config/safety/safety_limits.yaml) - All the guardrails in one place
- [**Impedance Gains**](config/gains/impedance_gains.yaml) - Tune assistance from passive to near-transparent

## Built With

`C++17` • `Eigen3` • `CMake` • `ROS 2` compatible • `yaml-cpp`

## License

MIT - Use it, learn from it, make it better. Just keep people safe.

---

*Made with care for those learning to move again.*

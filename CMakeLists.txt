cmake_minimum_required(VERSION 3.16)
project(rehab_exoskeleton LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Real-time / safety flags
add_compile_options(-Wall -Wextra -pedantic -Werror -O2)

# Find dependencies
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Safety library
add_library(exo_safety
  src/safety/SafetyMonitor.cpp
  src/safety/EStopHandler.cpp
  src/safety/Watchdog.cpp
)
target_link_libraries(exo_safety PUBLIC Threads::Threads)

# Control library
add_library(exo_control
  src/control/ImpedanceController.cpp
  src/control/TorqueController.cpp
  src/control/AdmittanceController.cpp
  src/control/RealTimeLoop.cpp
)
target_link_libraries(exo_control PUBLIC exo_safety Eigen3::Eigen Threads::Threads)

# Telemetry
add_library(exo_telemetry
  src/telemetry/ThreadSafeLogger.cpp
  src/telemetry/DataRecorder.cpp
  src/telemetry/ClinicalMetrics.cpp
)
target_link_libraries(exo_telemetry PUBLIC Threads::Threads)

# Kinematics/Dynamics
add_library(exo_model
  src/kinematics/ExoKinematics.cpp
  src/dynamics/ExoDynamics.cpp
  src/hri/IntentEstimator.cpp
)
target_link_libraries(exo_model PUBLIC Eigen3::Eigen)

# Hardware interface
add_library(exo_hw
  src/hardware_interface/CanInterface.cpp
  src/hardware_interface/EtherCatInterface.cpp
  src/hardware_interface/JointInterface.cpp
)
target_link_libraries(exo_hw PUBLIC exo_safety Threads::Threads)

# Main application
add_executable(rehab_exo_node src/app/main.cpp)
target_link_libraries(rehab_exo_node
  exo_control
  exo_safety
  exo_telemetry
  exo_model
  exo_hw
  yaml-cpp
  Threads::Threads
)

# Tests
enable_testing()
add_subdirectory(tests)

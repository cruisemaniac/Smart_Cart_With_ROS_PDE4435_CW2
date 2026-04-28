# Logbook - Smart Autonomous Shopping Cart

Student Name: Ashwin Murali Thanalapati (M01037932)
Module: PDE4435 - Robotic System Integration
Role: Sensor & Communication (UWB simulation, data processing, ROS2–Gazebo bridge)

## Project Timeline

### 2026-04-12

- Worked on the UWB simulation setup.
- Added the anchor layout and connected the simulator output into the follow-me pipeline.

### 2026-04-15

- Implemented position estimation using trilateration.
- Added filtering to smooth the UWB readings before they were used by the controller.

### 2026-04-19

- Managed the ROS2-Gazebo bridge configuration.
- Checked that the main sensor and control topics were passing correctly between Gazebo and ROS2.

### 2026-04-23

- Fixed setup issues affecting Gazebo and package execution.
- Re-tested the sensor nodes after the environment changes.

### 2026-04-27

- Did final checks on the sensor pipeline before submission.
- Reviewed the main results for localisation and response time.

## Outcome

- UWB localisation stayed within the project target after filtering.
- The cart was able to keep a stable follow distance in simulation.
- ROS2 and Gazebo communication worked reliably for the main topics.

## Reflection

My main work was on UWB simulation, position estimation, and ROS2-Gazebo communication. The main thing I learned was that filtering and clean topic communication make a big difference when trying to get stable robot behaviour.

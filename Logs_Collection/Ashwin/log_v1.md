# Logbook - Smart Autonomous Shopping Cart

Student Name: Ashwin Murali Thanalapati (M01037932)
Module: PDE4435 - Robotic System Integration
Role: Sensor & Communication (UWB simulation, data processing, ROS2-Gazebo bridge)

## Project Timeline

### 2026-04-12

- Worked on the UWB simulation setup.
- Added the anchor layout and connected the simulator output into the follow-me pipeline.
- Added noise and small bias so simulated UWB readings behaved closer to real sensor behavior.
- Validated that publish rate stayed consistent during simulation runs.
- Verified that simulated person movement from /person/odom produced consistent range updates.
- Checked frame handling so the generated UWB values stayed aligned with cart orientation.

### 2026-04-15

- Implemented position estimation using trilateration.
- Added filtering to smooth the UWB readings before they were used by the controller.
- Tuned early filter settings and tested the warmup behavior to avoid unstable first estimates.
- Checked controller response when the person moved diagonally and when angle changed quickly.
- Compared raw trilateration output with filtered output over repeated runs.
- Confirmed filtered estimates reduced jitter in heading commands during follow mode.

### 2026-04-19

- Managed the ROS2-Gazebo bridge configuration.
- Checked that the main sensor and control topics were passing correctly between Gazebo and ROS2.
- Verified topic directions carefully (GZ_TO_ROS and ROS_TO_GZ) to prevent command/sensor mix-ups.
- Re-ran bridge tests after launch timing changes to avoid startup race issues.
- Confirmed /scan, /odom, /person/odom, and /cmd_vel flows were stable through the bridge.
- Worked with teammates to validate bridge behavior during full launch, not only isolated node tests.

### 2026-04-23

- Fixed setup issues affecting Gazebo and package execution.
- Re-tested the sensor nodes after the environment changes.
- Helped resolve dependency and path issues so all required nodes launched in one flow.
- Rechecked UWB output continuity after each fix to ensure no silent break in pipeline.
- Revalidated bridge config after environment fixes to make sure topic names stayed consistent.
- Checked startup order behavior and confirmed sensor topics were active before follow mode tests.

### 2026-04-27

- Did final checks on the sensor pipeline before submission.
- Reviewed the main results for localisation and response time.
- Finalized parameter values used for demo/presentation consistency.
- Repeated scenario tests (straight follow, side offset, temporary occlusion) for confidence.
- Verified log outputs and RViz markers for final demo walkthrough.
- Cross-checked that the sensor pipeline matched the report claims and presentation flow.

## Outcome

- UWB localisation stayed within the project target after filtering.
- The cart was able to keep a stable follow distance in simulation.
- ROS2 and Gazebo communication worked reliably for the main topics.
- The team had clearer debugging visibility through RViz markers and structured sensor logs.

## Challenges and Fixes

- UWB readings were noisy during fast relative motion.
  Fix: kept Kalman filtering in the loop and confirmed warmup handling before trusting estimates.
- Some launch/bridge startup timing caused missing data at the beginning of runs.
  Fix: retested startup sequence and validated topic activity after launch.
- Integration debugging was slow when outputs were not visible.
  Fix: improved visual and log-based checks to speed up issue detection.

## Reflection

My main work was on UWB simulation, position estimation, and ROS2-Gazebo communication. Expanding this part of the project taught me that sensor quality is not only about hardware; data handling, filtering, and clear topic flow matter just as much. I also learned that small integration details (launch timing, topic direction, startup order) can affect system stability more than expected.

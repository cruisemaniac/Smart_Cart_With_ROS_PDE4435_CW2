> **Note:** This is the initial planning document written during the CW1 ideation phase.
> It reflects early design decisions and does not represent the final implementation.
> Some details have changed: the project uses **ROS 2 Jazzy** (not Humble), a
> **proportional controller** (not DWA), and a **Kalman2D filter** (not EKF).
> See the report and `SETUP_AND_RUN.md` for the actual system.

---

# **Integration of an Autonomous Follow-Me Shopping Cart**
**Module:** PDE4435 - Robotic System Integration 

**Team Members:** Ashwin Murali Thanalapati, Mohammed Shalaby, Jayashanka Anushan, Vignesh Lakshmanasamy 

## **1. What is the problem you are going to solve?**
We are solving the operational and ergonomic limitations of conventional shopping carts in high-volume retail environments. The specific problems include:

- **Physical Strain & Mobility:** Reducing the exertion required by elderly shoppers and parents to manage heavily loaded carts, which studies show significantly reduces mobility efficiency.
- **Maneuverability:** Navigating narrow and dynamic aisles where standard carts lack the agility to make micro-adjustments.
- **Safety Hazards:** Preventing "heel and ankle" impacts caused by poor braking response and high momentum under heavy loads.

## **2. What are the existing similar solutions available?**
Current assistive mobile platforms include:
- **Luggage-Following Robots:** Consumer-grade robots for airports, typically designed for flat, open spaces rather than dense crowds.
- **Warehouse AMRs:** Industrial Autonomous Mobile Robots used for logistics; these are often expensive and require highly structured environments.
- **Standard Service Robots:** Basic "follow-me" systems that use simple differential drive and lack the specialized load-sensing or aisle-navigation capabilities required for retail.

## **3. What differentiates your solution from those existing ones?**
Our solution integrates specialized hardware and software for the unique constraints of a supermarket:
- **Holonomic Motion:** By using **four independently driven omni-wheels**, the cart can move laterally and rotate in place (0 cm turning radius), which is essential for tight aisles and lateral parking.
- **Load-Adaptive Control:** We utilize **weight sensors (load cells)** to monitor the basket payload; the system dynamically reduces maximum speed (capped at 0.8 m/s near humans) to maintain safe braking distances as weight increases.
- **Multi-Modal Localization:** Unlike camera-only systems, we fuse **Ultra-Wideband (UWB)** for user tracking with **LiDAR and Visual Odometry** to ensure stable navigation in metallic shelf environments where signals may be noisy.

## **4. How are you going to build your robot, hardware or simulator (which platform)?**
For Coursework 2, we are adopting a **hybrid implementation** focused on a high-fidelity simulator to evaluate high-level task integration:
- **Simulator:** **Gazebo** will be used for the physics-based world (supermarket aisles and pedestrian actors) and **RViz** for sensor data visualization.
- **Platform:** The robot runs on **ROS 2 Humble Hawksbill**, mirroring the layered architecture of the Raspberry Pi 4B (High-Level) and Arduino Mega (Low-Level) communication bridge.
- **Progress:** We have completed the ROS 2 node graph design and are currently configuring the **Dynamic Window Approach (DWA)** local planner to handle the omni-wheel kinematics in Gazebo.

## **5. What kind of guidance do you need to do your project?**
As we move into the integration phase, the following guidance is required:
- **Kinematic Modeling:** Assistance with the mathematical decomposition of velocities for a **4-wheel holonomic drive** within the ROS 2 `cmd_vel` controller.
- **Sensor Fusion (EKF):** Recommendations on tuning the **Extended Kalman Filter** parameters to prioritize UWB for user-following while relying on LiDAR for static obstacle avoidance.
- **Safety Integration:** Best practices for implementing a **software watchdog timer** that replicates the hardware emergency stop relay to ensure failsafe behavior during simulation crashes.

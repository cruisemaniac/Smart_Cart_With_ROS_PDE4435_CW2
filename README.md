# Smart Cart With ROS — PDE4435 Coursework 2

A ROS 2 Jazzy simulation of an **Autonomous Smart Cart** for indoor retail environments.
The cart uses UWB-based positioning and a Kalman filter to follow a user at 1.0 m distance,
with a LiDAR safety layer that enforces three-zone obstacle avoidance. The simulation runs
in Gazebo Harmonic with a large supermarket world and three autonomous random pedestrians.

---

## Module Information

| Field | Detail |
|---|---|
| Module | PDE4435 — Robotic System Integration |
| Assessment | Coursework 2 |
| Project Title | Autonomous Smart Cart with ROS 2 |

---

## Team Members

| Name | Student ID |
|---|---|
| Ashwin Murali Thanalapati | M01037932 |
| Mohammed Shalaby | M01035318 |
| Jayashanka Anushan | M01037028 |
| Vignesh Lakshmanasamy | M01026685 |

---

## Repository Structure

```
Smart_Cart_With_ROS_PDE4435_CW2/
├── packages/
│   ├── smart_cart_behaviour/       # follow_me, obstacle_stop, teleop nodes
│   ├── smart_cart_description/     # URDF / Xacro robot model
│   ├── smart_cart_gazebo/          # Gazebo world, bridge config, RViz config
│   └── smart_cart_navigation/      # UWB simulator, Kalman filter, navigation state machine
├── config/                         # YAML parameter files
├── Diagrams/                       # Architecture and flowchart diagrams
├── Logs_Collection/                # Individual member logbooks
│   ├── Ashwin/
│   ├── Jayashanka/
│   ├── Shalaby/
│   └── Vignesh/
├── Reports/
│   └── Smart_Cart_CW2_PDE4435_Report_v1_0/   # IEEE LaTeX report
├── docs/                           # Planning and reference documents
├── requirements.txt                # Python dependencies (numpy, pytest)
├── SETUP_AND_RUN.md                # Full setup and run guide
└── README.md
```

---

## Quick Start

See **[SETUP_AND_RUN.md](SETUP_AND_RUN.md)** for the full step-by-step guide to clone,
build, and launch the simulation.

**One-line launch (after setup):**
```bash
ros2 launch smart_cart_behaviour start_all.launch.py
```

**In a second terminal — control the person and cart:**
```bash
ros2 run smart_cart_behaviour teleop_person_node
```

| Key | Action |
|---|---|
| `W / S` | Move person forward / backward |
| `A / D` | Turn person left / right |
| `2` | Activate FOLLOW mode (cart starts following) |
| `1` | STOP cart |
| `3` | IDLE (cart standby) |
| `+ / -` | Adjust person speed |
| `ESC` | Quit |

---

## ROS 2 Packages

| Package | Key Nodes |
|---|---|
| `smart_cart_behaviour` | `follow_me_node`, `obstacle_stop_node`, `teleop_person_node`, `cart_teleop_node`, `random_person_node` |
| `smart_cart_navigation` | `navigation_node`, `uwb_simulator_node` |
| `smart_cart_description` | URDF/Xacro robot model |
| `smart_cart_gazebo` | Gazebo world, ROS-Gz bridge config, RViz config |

---

## Logbook Contribution Guidelines

Each team member records their progress in their assigned folder:

```
Logs_Collection/<MemberName>/log_v1.md
Logs_Collection/<MemberName>/log_v2.md  ...
```

Entries must include the date, tasks completed, and individual contribution.

---

## Report

The IEEE two-column LaTeX report is located in:
```
Reports/Smart_Cart_CW2_PDE4435_Report_v1_0/
```
See [Reports/README_LaTex.md](Reports/README_LaTex.md) for build instructions.

---

## License

For academic coursework use only — PDE4435, Middlesex University.

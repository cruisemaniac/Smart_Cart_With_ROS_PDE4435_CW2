# Smart Cart ROS 2 — Setup & Run Guide

This guide walks you through cloning the repository, building the workspace, and launching the full simulation from scratch.

---

## Prerequisites

Make sure you have the following installed before starting:

- Ubuntu 24.04
- ROS 2 Jazzy (full desktop install)
- Gazebo Harmonic
- Required ROS packages:

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2
```

```
To be install xterm

sudo apt install xterm
```

---

## Step 1 — Create the ROS 2 workspace

If you don't already have a workspace, create one:

```bash
mkdir -p ~/ros/jazzy_ws/src
cd ~/ros/jazzy_ws
```

---

## Step 2 — Clone the repository

Clone the repo **into the `src` folder** of your workspace:

```bash
cd ~/ros/jazzy_ws/src
git clone <your-repo-url> Smart_Cart_With_ROS_PDE4435_CW2
```

After cloning your `src` folder should look like this:

```
jazzy_ws/
└── src/
    └── Smart_Cart_With_ROS_PDE4435_CW2/
        ├── packages/
        │   ├── smart_cart_behaviour/
        │   ├── smart_cart_description/
        │   ├── smart_cart_gazebo/
        │   └── smart_cart_navigation/
        ├── config/
        ├── docs/
        └── README.md
```

---

## Step 3 — Symlink packages so colcon can find them

The ROS packages live inside the `packages/` subfolder. You need to make them visible to `colcon` by symlinking them into `src/`:

```bash
cd ~/ros/jazzy_ws/src

ln -s Smart_Cart_With_ROS_PDE4435_CW2/packages/smart_cart_behaviour     smart_cart_behaviour
ln -s Smart_Cart_With_ROS_PDE4435_CW2/packages/smart_cart_description   smart_cart_description
ln -s Smart_Cart_With_ROS_PDE4435_CW2/packages/smart_cart_gazebo        smart_cart_gazebo
ln -s Smart_Cart_With_ROS_PDE4435_CW2/packages/smart_cart_navigation    smart_cart_navigation
```

After this, your `src/` folder should look like:

```
src/
├── Smart_Cart_With_ROS_PDE4435_CW2/   ← full repo
├── smart_cart_behaviour  → (symlink)
├── smart_cart_description → (symlink)
├── smart_cart_gazebo     → (symlink)
└── smart_cart_navigation → (symlink)
```

> **Why symlinks?** colcon expects packages to be direct children of `src/`. Since this repo stores packages inside a `packages/` subfolder, symlinks bridge that gap without moving files.

---

## Step 4 — Source ROS 2

Always source ROS 2 before building:

```bash
source /opt/ros/jazzy/setup.bash
```

To avoid doing this every time, add it to your shell profile:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 5 — Build the workspace

From the workspace root:

```bash
cd ~/ros/jazzy_ws
colcon build
```

A successful build looks like:

```
Starting >>> smart_cart_behaviour
Starting >>> smart_cart_description
Starting >>> smart_cart_gazebo
Starting >>> smart_cart_navigation
Finished <<< smart_cart_behaviour [~1.5s]
Finished <<< smart_cart_description [~0.6s]
Finished <<< smart_cart_gazebo [~0.5s]
Finished <<< smart_cart_navigation [~1.3s]
Summary: 4 packages finished
```

If any package fails, check that all symlinks in Step 3 are correct.

---

## Step 6 — Source the workspace

After building, source the workspace install:

```bash
source ~/ros/jazzy_ws/install/setup.bash
```

To do this automatically on every new terminal:

```bash
echo "source ~/ros/jazzy_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **Important:** After every `colcon build` you must re-source, or open a fresh terminal.

---

## Step 7 — Launch the full simulation

```bash
ros2 launch smart_cart_behaviour start_all.launch.py
```

This single launch file starts everything in order:

| Delay | What starts |
|---|---|
| Immediately | Gazebo with supermarket world |
| Immediately | Robot State Publisher (loads URDF) |
| After 3s | Robot spawned in Gazebo |
| After 4s | ROS–Gazebo bridge (topics connected) |
| After 5s | `follow_me_node`, `obstacle_stop_node`, `navigation_node` |
| After 6s | RViz2 |

---

## Step 8 — Verify everything is running

Open a new terminal and check nodes and topics:

```bash
# Check all nodes are alive
ros2 node list

# Expected output:
# /follow_me_node
# /obstacle_stop_node
# /navigation_node
# /robot_state_publisher
# /ros_gz_bridge
# /rviz2

# Check topics are active
ros2 topic list

# Key topics you should see:
# /scan           ← LiDAR data from Gazebo
# /cmd_vel_raw    ← velocity from follow_me_node
# /cmd_vel        ← final velocity to robot
# /tf  /tf_static ← transforms
```

---

## Step 9 — Test the follow-me behaviour

The cart uses LiDAR to detect the nearest object within a 60° front arc and follows it at 1.0 m distance. Since there is no person in the simulation by default, you need to spawn a target object.

Open a **new terminal** and run:

```bash
source ~/ros/jazzy_ws/install/setup.bash

ros2 run ros_gz_sim create \
  -name target_person \
  -string "<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='target_person'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry><box><size>0.5 0.5 1.7</size></box></geometry>
      </visual>
      <collision name='collision'>
        <geometry><box><size>0.5 0.5 1.7</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>" \
  -x 1.5 -y 0.0 -z 0.85
```

The cart should drive toward the box and stop at ~1.0 m. Watch the velocity commands:

```bash
ros2 topic echo /cmd_vel_raw
```

---

## Troubleshooting

**`colcon build` fails with "package not found"**
→ Check symlinks in `src/` are pointing to the right folders (Step 3).

**`ros2 launch` says executable not found**
→ You forgot to `source install/setup.bash` after building (Step 6).

**Gazebo opens but robot is not visible**
→ Wait ~5 seconds for the spawn timer. If still missing, check `/robot_description` topic:
```bash
ros2 topic echo /robot_description
```

**`No target detected – cart stopped` repeating**
→ This is normal until you spawn a target object (Step 9).

**RViz2 shows no robot model**
→ In RViz2, set Fixed Frame to `base_link` and add a RobotModel display subscribed to `/robot_description`.

---

## Package overview

| Package | Purpose |
|---|---|
| `smart_cart_behaviour` | `follow_me_node` (LiDAR following), `obstacle_stop_node` (safety stop) |
| `smart_cart_navigation` | `navigation_node` (routes `cmd_vel_raw` → `cmd_vel`) |
| `smart_cart_description` | URDF/Xacro robot model |
| `smart_cart_gazebo` | Gazebo world, bridge config, simulation launch |

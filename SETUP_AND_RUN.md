# Smart Cart ROS 2 — Setup & Run Guide

This guide walks you through cloning the repository, building the workspace,
and launching the full simulation from scratch.

---

## Prerequisites

Make sure you have the following installed before starting:

- Ubuntu 24.04
- ROS 2 Jazzy (full desktop install)
- Gazebo Harmonic
- Python 3 with venv support

**Install required ROS packages:**
```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  python3-venv \
  python3-pip

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

The ROS packages live inside the `packages/` subfolder. You need to make them
visible to `colcon` by symlinking them into `src/`:

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
├── Smart_Cart_With_ROS_PDE4435_CW2/    ← full repo
├── smart_cart_behaviour                → (symlink)
├── smart_cart_description             → (symlink)
├── smart_cart_gazebo                  → (symlink)
└── smart_cart_navigation              → (symlink)
```

> **Why symlinks?** `colcon` expects packages to be direct children of `src/`.
> Since this repo stores packages inside a `packages/` subfolder, symlinks bridge
> that gap without moving files.

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

## Step 5 — Set up the Python virtual environment

```bash
python3 -m venv --system-site-packages ~/.venvs/smart_cart_ros
source ~/.venvs/smart_cart_ros/bin/activate
pip install --upgrade pip
pip install -r ~/ros/jazzy_ws/src/Smart_Cart_With_ROS_PDE4435_CW2/requirements.txt
```

> **Note:** `--system-site-packages` lets the virtual environment see ROS Python
> modules installed via `apt` (xacro, launch, etc.).
>
> **Tip:** Keep the virtual environment outside the workspace tree so `colcon`
> does not try to inspect it during builds.

---

## Step 6 — Build the workspace

From the workspace root, with the virtual environment active:

```bash
cd ~/ros/jazzy_ws
source ~/.venvs/smart_cart_ros/bin/activate
colcon build
```

A successful build looks like:

```
Starting >>> smart_cart_behaviour
Starting >>> smart_cart_description
Starting >>> smart_cart_gazebo
Starting >>> smart_cart_navigation
Finished <<< smart_cart_behaviour  [~2s]
Finished <<< smart_cart_description [~0.8s]
Finished <<< smart_cart_gazebo     [~0.7s]
Finished <<< smart_cart_navigation [~2s]
Summary: 4 packages finished
```

---

## Step 7 — Source the workspace

After building, source the workspace install:

```bash
source ~/ros/jazzy_ws/install/setup.bash
```

To do this automatically in every new terminal:

```bash
echo "source ~/ros/jazzy_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **Important:** After every `colcon build` you must re-source, or open a fresh terminal.

---

## Step 8 — Launch the full simulation

```bash
ros2 launch smart_cart_behaviour start_all.launch.py
```

This single launch file starts everything in sequence:

| Time | What starts |
|---|---|
| t = 0s  | Gazebo Harmonic with large supermarket world + Robot State Publisher |
| t = 6s  | Smart Cart spawned at origin (0, 0) |
| t = 7s  | Main person (user avatar) spawned at (2.0, 0.0) |
| t = 8s  | 3 random autonomous pedestrians spawned in the world |
| t = 9s  | ROS–Gazebo bridge started (topics connected) |
| t = 10s | All ROS 2 nodes started: `follow_me_node`, `obstacle_stop_node`, `navigation_node`, `uwb_simulator_node`, 3× `random_person_node` |
| t = 11s | RViz2 launched with pre-configured display |
| t = 12s | Ready message printed in terminal |

Wait for the **Ready** message before opening the teleop terminal.

---

## Step 9 — Verify everything is running

Open a new terminal and check:

```bash
# Check all nodes are alive
ros2 node list
```

Expected output:
```
/follow_me_node
/navigation_node
/obstacle_stop_node
/random_person_node
/random_person_2_node
/random_person_3_node
/robot_state_publisher
/ros_gz_bridge
/rviz2
/uwb_simulator_node
```

Check key topics are active:
```bash
ros2 topic list
# Key topics you should see:
# /scan              ← LiDAR data from Gazebo
# /uwb/distances     ← simulated UWB anchor ranges
# /cmd_vel_raw       ← follow_me_node velocity output
# /cmd_vel           ← final velocity after obstacle safety filter
# /nav/current_mode  ← IDLE / FOLLOW / STOP
# /person/odom       ← user avatar odometry
```

---

## Step 10 — Control the simulation (teleop)

Open a **new terminal** and run the person teleop node:

```bash
source ~/.venvs/smart_cart_ros/bin/activate
source ~/ros/jazzy_ws/install/setup.bash

ros2 run smart_cart_behaviour teleop_person_node
```

### Person movement

| Key | Action |
|---|---|
| `W` / `↑` | Move person forward |
| `S` / `↓` | Move person backward |
| `A` / `←` | Turn person left |
| `D` / `→` | Turn person right |
| `SPACE` | Stop person |
| `+` / `-` | Speed up / slow down |
| `R` | Reset speed to default (0.6 m/s) |

### Cart remote control

| Key | Action |
|---|---|
| `2` | **FOLLOW** — cart starts following the person at 1.0 m |
| `1` | **STOP** — cart stops immediately |
| `3` | **IDLE** — cart standby, holds position |

**Typical test sequence:**
1. Wait for the Ready message from the launch terminal
2. Run `teleop_person_node` in a new terminal
3. Press `2` to activate FOLLOW mode
4. Use `W/A/S/D` to walk the person around — the cart follows
5. Walk toward the obstacle box to test emergency stop
6. Press `1` to stop the cart, `3` to return to idle

---

## Step 11 — Direct cart testing (optional)

To test raw cart movement independently (bypasses the follow-me pipeline):

```bash
ros2 run smart_cart_behaviour cart_teleop_node
```

This publishes directly to `/cmd_vel`, skipping `follow_me_node` and
`obstacle_stop_node`. Use this to verify the cart URDF and Gazebo
differential drive plugin are working correctly.

| Key | Action |
|---|---|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn left |
| `D` | Turn right |
| `Q / E` | Forward + turn left / right |
| `Z / C` | Backward + turn left / right |
| `SPACE` | Stop |
| `+ / -` | Speed up / down |
| `ESC` | Quit |

---

## Troubleshooting

**`colcon build` fails with "package not found"**
→ Check symlinks in `src/` are pointing to the right folders (Step 3).

**`ros2 launch` says executable not found**
→ You forgot to `source install/setup.bash` after building (Step 7).

**`ModuleNotFoundError: No module named 'xacro'`**
→ Make sure `ros-jazzy-xacro` is installed and you sourced `/opt/ros/jazzy/setup.bash`.

**Gazebo opens but robot is not visible**
→ The cart spawns at t=6s. Wait for the Ready message. If still missing, check:
```bash
ros2 topic echo /robot_description
```

**Cart does not follow the person**
→ Make sure you pressed `2` (FOLLOW) in the teleop terminal. Check the mode:
```bash
ros2 topic echo /nav/current_mode
```

**RViz2 shows no robot model**
→ In RViz2 set Fixed Frame to `odom` and add a RobotModel display subscribed to `/robot_description`.

**`colcon build` starts scanning the virtual environment**
→ Keep the venv outside the workspace tree as shown in Step 5.

---

## Package Overview

| Package | Purpose |
|---|---|
| `smart_cart_behaviour` | `follow_me_node` (UWB tracking + P-controller), `obstacle_stop_node` (LiDAR 3-zone safety), `teleop_person_node` (person + remote control), `cart_teleop_node` (direct cart debug teleop), `random_person_node` (autonomous pedestrian) |
| `smart_cart_navigation` | `navigation_node` (IDLE/FOLLOW/STOP state machine), `uwb_simulator_node` (4-anchor UWB + Kalman filter) |
| `smart_cart_description` | URDF/Xacro differential-drive cart model with LiDAR and depth camera |
| `smart_cart_gazebo` | Large supermarket world, ROS–Gz bridge config, RViz config |

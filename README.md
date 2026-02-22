# MyRobot — ROS 2 Differential-Drive Robot

ROS 2 workspace for a differential-drive robot (Eurobot 2026). Supports real hardware (Arduino + ros2_control) and Gazebo simulation, with Nav2 autonomous navigation.

## Prerequisites

- **ROS 2** (Humble / Jazzy)
- **Gazebo** (Fortress for Humble, Harmonic for Jazzy)
- Nav2, slam_toolbox, robot_localization, ros2_control

## Build

```bash
cd ~/Desktop/copy_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Robot

### 1. Real Robot

```bash
ros2 launch myrobot_bringup real_robot.launch.py
```

Launches hardware interface, ros2_control controllers, and blind navigation.

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use sim clock |

### 2. Simulation (Full Nav Stack)

```bash
ros2 launch myrobot_bringup sim_robot.launch.py
```

Launches Gazebo, controllers, and optionally SLAM/localization + Nav2.

| Parameter | Default | Description |
|---|---|---|
| `use_slam` | `true` | `true` = SLAM, `false` = AMCL localization |
| `run_rviz` | `true` | Launch RViz |

> **Note:** SLAM, localization, and navigation includes are currently commented out in this launch file. Uncomment them to enable.

### 3. Simulation (Blind Navigation)

```bash
ros2 launch myrobot_bringup sim_blind_robot.launch.py
```

Gazebo + controllers + Nav2 blind navigation (no LiDAR, odometry-only localization, static map).

| Parameter | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use sim clock |
| `run_rviz` | `true` | Launch RViz |

---

## Other Launch Files

### myrobot_description

| Launch File | Purpose |
|---|---|
| `gazebo.launch.py` | Spawn robot in Gazebo. Params: `model` (URDF path), `world_name` (default: `arena_world`) |
| `display.launch.py` | View URDF in RViz with joint_state_publisher_gui |

### myrobot_firmware

| Launch File | Purpose |
|---|---|
| `hardware_interface.launch.py` | Start robot_state_publisher + ros2_control controller_manager for real hardware. Param: `use_sim_time` |

### myrobot_controller

| Launch File | Purpose |
|---|---|
| `controller.launch.py` | Spawn `joint_state_broadcaster`, `myrobot_controller`, and `twist_relay` node |

### myrobot_localization

| Launch File | Purpose |
|---|---|
| `global_localization.launch.py` | Map server + AMCL. Params: `map_name`, `use_sim_time`, `amcl_config` |
| `local_localization.launch.py` | Map server + static map→odom TF + EKF. Params: `map_name`, `use_sim_time` |
| `fake_localization.launch.py` | Map server + static TF (no EKF). Params: `map_name`, `use_sim_time` |

### myrobot_mapping

| Launch File | Purpose |
|---|---|
| `slam.launch.py` | slam_toolbox + map_saver_server. Params: `use_sim_time`, `slam_config` |

### myrobot_navigation

| Launch File | Purpose |
|---|---|
| `navigation.launch.py` | Full Nav2 stack (controller, planner, smoother, BT navigator, behaviors). Param: `use_sim_time` |
| `navigation_blind.launch.py` | Nav2 without obstacle detection + local_localization. Params: `use_sim_time`, `map_name` |

Includes a `waypoint_follower.py` script and custom behavior trees in `behavior_tree/`.

### myrobot_vision

| Launch File | Purpose |
|---|---|
| `vision.launch.py` | Camera processing node |

---

## TODO

**Core**
- [ ] ArUco code detection and pose estimation to enhance robot localization
- [ ] Playing strategy — Yellow or Blue side selection
- [ ] Enhance `arena_world` to match the real competition field
- [ ] Adding actuators to grab objects

**Navigation & Autonomy**
- [ ] Timed match logic — 100s game state manager to start/stop actions
- [ ] Multi-waypoint mission planner — scoring-strategy sequencer per side
- [ ] Obstacle avoidance with LiDAR — integrate RPLiDAR for dynamic obstacles

**Hardware & Reliability**
- [ ] Emergency stop / match start cord
- [ ] Battery monitoring node
- [ ] Sensor fusion — combine EKF odometry + ArUco + IMU

**Testing**
- [ ] Automated simulation test — launch sim, send goal, assert arrival
- [ ] Separate Nav2 param profiles for competition vs. testing

**Competition-Specific**
- [ ] Opponent detection & avoidance
- [ ] Score counting node

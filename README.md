# ROS 2 Autonomous Robot Stack

A complete ROS 2 navigation stack for differential-drive robots, featuring simulation, SLAM, and Nav2 autonomous navigation. This project serves as a reusable foundation for robotics teams and educators.

## Features

- **Differential drive robot** with ros2_control support
- **Gazebo simulation** with realistic physics
- **SLAM** using slam_toolbox for map creation
- **Autonomous navigation** with Nav2 stack
- **Real hardware support** via Arduino/ESP32 + micro-ROS
- **Vision capabilities** with camera integration

## Architecture

```text
myrobot_bringup       → System launch files (simulation & real robot)
myrobot_description   → URDF, meshes, Gazebo worlds
myrobot_controller    → ros2_control controllers
myrobot_firmware      → Arduino/ESP32 hardware interface
myrobot_localization  → AMCL, EKF, map server
myrobot_mapping       → SLAM toolbox integration
myrobot_navigation    → Nav2 configuration + behavior trees
myrobot_vision        → Camera processing nodes
```

## Prerequisites

- **ROS 2** (Humble / Jazzy)
- **Gazebo** (Fortress for Humble, Harmonic for Jazzy)
- Required packages:

  ```bash
  sudo apt install ros-$ROS_DISTRO-nav2-bringup \
                   ros-$ROS_DISTRO-slam-toolbox \
                   ros-$ROS_DISTRO-robot-localization \
                   ros-$ROS_DISTRO-ros2-control \
                   ros-$ROS_DISTRO-ros2-controllers
  ```

## Quick Start

### Build the Workspace

```bash
cd ~/myrobot_ws  # or your workspace path
colcon build --symlink-install
source install/setup.bash
```

### Run Simulation (Full Nav Stack)

```bash
ros2 launch myrobot_bringup sim_robot.launch.py
```

This launches:
- Gazebo with the robot spawned
- ros2_control controllers
- SLAM or AMCL localization (toggle with `use_slam` parameter)
- Nav2 navigation stack
- RViz for visualization

**Parameters:**

| Parameter  | Default | Description                                     |
| ---------- | ------- | ----------------------------------------------- |
| `use_slam` | `true`  | `true` = SLAM mode, `false` = AMCL localization |
| `run_rviz` | `true`  | Launch RViz                                     |

### Run Simulation (Blind Navigation)

For environments without LiDAR (odometry-only navigation):

```bash
ros2 launch myrobot_bringup sim_blind_robot.launch.py
```

### Run Real Robot

```bash
ros2 launch myrobot_bringup real_robot.launch.py
```

Connects to real hardware via serial (micro-ROS). Set the correct port in the launch file.

---

## Package Reference

### myrobot_description

| Launch File           | Purpose                                                        |
| --------------------- | -------------------------------------------------------------- |
| `gazebo.launch.py`    | Spawn robot in Gazebo. Params: `model`, `world_name`           |
| `display.launch.py`   | View URDF in RViz with joint state publisher                   |

### myrobot_firmware

| Launch File                  | Purpose                                              |
| ---------------------------- | ---------------------------------------------------- |
| `hardware_interface.launch.py` | ros2_control controller manager for real hardware  |

### myrobot_controller

| Launch File         | Purpose                                                        |
| ------------------- | -------------------------------------------------------------- |
| `controller.launch.py` | Spawn joint_state_broadcaster, diff_drive_controller, twist_relay |

### myrobot_localization

| Launch File                   | Purpose                                  |
| ----------------------------- | ---------------------------------------- |
| `global_localization.launch.py` | Map server + AMCL                        |
| `local_localization.launch.py`  | Map server + static transform + EKF      |
| `fake_localization.launch.py`   | Map server + static TF (no EKF)          |

### myrobot_mapping

| Launch File      | Purpose                            |
| ---------------- | ---------------------------------- |
| `slam.launch.py` | slam_toolbox + map_saver_server    |

### myrobot_navigation

| Launch File              | Purpose                                                        |
| ------------------------ | -------------------------------------------------------------- |
| `navigation.launch.py`   | Full Nav2 stack (planner, controller, smoother, BT navigator)  |
| `navigation_blind.launch.py` | Nav2 without obstacle detection (odometry-only)            |

Includes:

- `waypoint_follower.py` — Sequential goal follower script
- Custom behavior trees in `behavior_tree/`

### myrobot_vision

| Launch File       | Purpose                            |
| ----------------- | ---------------------------------- |
| `vision.launch.py` | Camera processing node with rqt   |

---

## Project Structure

```text
.
├── src/
│   ├── myrobot_bringup/       # Main system launch files
│   ├── myrobot_description/   # Robot URDF, meshes, worlds
│   ├── myrobot_controller/    # Controller configuration
│   ├── myrobot_firmware/      # Arduino communication
│   ├── myrobot_localization/  # Localization (AMCL, EKF)
│   ├── myrobot_mapping/       # SLAM integration
│   ├── myrobot_navigation/    # Nav2 stack + behavior trees
│   └── myrobot_vision/        # Camera processing
├── README.md
└── .gitignore
```

## Usage Examples

### Create a Map

```bash
# 1. Launch SLAM mode
ros2 launch myrobot_bringup sim_robot.launch.py use_slam:=true

# 2. Drive the robot (teleop or RViz)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. Save the map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Navigate to a Goal

```bash
# 1. Launch full stack
ros2 launch myrobot_bringup sim_robot.launch.py use_slam:=false

# 2. Send a goal (from another terminal)
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 1.0, y: 2.0, z: 0.0},
         orientation: {w: 1.0}}}"
```

### Follow Waypoints

```bash
ros2 run myrobot_navigation waypoint_follower
```

---

## Roadmap

### Core Features

- [ ] ArUco marker detection for enhanced localization
- [ ] Dynamic obstacle avoidance with LiDAR
- [ ] Multi-robot support

### Navigation & Autonomy

- [ ] Mission planner with waypoint sequencing
- [ ] Recovery behavior customization
- [ ] Navigation in dynamic environments

### Hardware & Reliability

- [ ] Emergency stop integration
- [ ] Battery monitoring node
- [ ] Sensor fusion (EKF + ArUco + IMU)

### Testing

- [ ] Automated simulation test suite
- [ ] Nav2 parameter tuning profiles

---

## See It In Action

This stack powers competition robots at Eurobot. Check out the competition-specific implementation: [eurobot-robot-2026](https://github.com/YOUR_USERNAME/eurobot-robot-2026)

## License

MIT License - See LICENSE file for details

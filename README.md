# MyRobot ROS2 Stack

Complete ROS 2 autonomous robot stack for differential-drive robots. Simulation-first design with Gazebo, SLAM, Nav2 navigation, and vision processing.

## Overview

This repository provides a modular ROS 2 stack for a differential-drive robot with Gazebo simulation, simultaneous localization and mapping (SLAM), autonomous navigation via Nav2, and camera-based vision processing. The stack uses ros2_control for hardware-agnostic controller management and is designed to be reusable across different robot platforms.

**Target robot:** 2-wheel differential drive with caster wheels, IMU, lidar, and forward-facing camera.

**Tested on:** ROS 2 Jazzy (Ubuntu 24.04)

## Package Overview

| Package | Purpose |
|---------|---------|
| `myrobot_description` | URDF/xacro robot description, Gazebo worlds, meshes, rviz configs |
| `myrobot_controller` | ros2_control controllers: DiffDrive, joint state broadcaster, twist relay |
| `myrobot_bringup` | Simulation launch files |
| `myrobot_mapping` | SLAM toolbox for online map generation |
| `myrobot_localization` | AMCL global localization, EKF for sensor fusion |
| `myrobot_navigation` | Nav2 behavior trees, planner/controller servers, costmaps |
| `myrobot_vision` | Camera calibration, aruco marker detection |

## Features

- **Gazebo simulation** — realistic physics, GPU lidar, IMU noise, camera simulation
- **ros2_control** — DiffDrive controller, joint state broadcaster, configurable velocity/acceleration limits
- **SLAM** — Online SLAM via slam_toolbox, toggle via `use_slam` launch arg
- **Localization** — AMCL global localization
- **Nav2 navigation** — autonomous goal navigation with behavior trees
- **Demo presentation** — (link placeholder)

## Quick Start

### Build

```bash
cd ~/myrobot_ws
colcon build --symlink-install
source install/setup.bash
```

### Run Simulation

```bash
ros2 launch myrobot_bringup sim_robot.launch.py
```

Launch args:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |
| `use_slam` | `true` | Run SLAM (true) or AMCL localization (false) |
| `run_rviz` | `true` | Launch RViz2 |

### Run Display (URDF only, no simulation)

```bash
ros2 launch myrobot_description display.launch.py
```

## Configuration

### Launch Arguments (sim_robot.launch.py)

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |
| `use_slam` | `true` | Run SLAM vs AMCL localization |
| `run_rviz` | `true` | Launch RViz2 |

### Launch Arguments (gazebo.launch.py)

| Argument | Default | Description |
|----------|---------|-------------|
| `model` | `<package>/urdf/robot/my_robot.urdf.xacro` | Robot URDF xacro file |
| `world_name` | `small_house` | Gazebo world name (without extension) |

### Controller Parameters

See `myrobot_controller/config/myrobot_controllers.yaml` for DiffDriveController and JointStateBroadcaster settings:

- Wheel separation: `0.30 m`
- Wheel radius: `0.033 m`
- Velocity/acceleration limits

### Localization Parameters

See `myrobot_localization/config/amcl.yaml` 

### Mapping Parameters

See `myrobot_mapping/config/slam_toolbox.yaml`.

### Navigation Parameters

See `myrobot_navigation/config/` for behavior trees, planners, controllers, and costmaps.

## Project Structure

```
src/
├── myrobot_bringup/
│    └── launch/
│        └── sim_robot.launch.py
│
├── myrobot_controller/
│   ├── launch/
│   │   └── controller.launch.py
│   ├── config/
│   │   └── myrobot_controllers.yaml
│   └── myrobot_controller/
│       └── twist_relay.py
├── myrobot_description/
│   ├── launch/
│   │   ├── gazebo.launch.py
│   │   └── display.launch.py
│   ├── urdf/robot/
│   │   ├── my_robot.urdf.xacro
│   │   ├── my_robot_homemade.xacro
│   │   ├── my_robot_gazebo.xacro
│   │   ├── robot_ros2_control.xacro
│   │   ├── common_properties.xacro
│   │   └── inertial_macros.xacro
│   └── worlds/
│       ├── empty.world
│       └── small_house.world
├── myrobot_localization/
│   ├── launch/
│   │   └── global_localization.launch.py
│   └── config/
│       └── amcl.yaml
├── myrobot_mapping/
│   ├── launch/
│   │   └── slam.launch.py
│   └── config/
│       └── slam_toolbox.yaml
├── myrobot_navigation/
│   ├── launch/
│   │   └── navigation.launch.py
│   └── config/
│       ├── behavior_server.yaml
│       ├── bt_navigator.yaml
│       ├── controller_server.yaml
│       ├── costmap.yaml
│       ├── planner_server.yaml
│       └── smoother_server.yaml
└── myrobot_vision/
    ├── launch/
    │   └── vision.launch.py
    └── config/
        ├── markers.yaml
        └── vision_settings.yaml
```

## Future Work

- Real robot bringup with micro-ROS hardware interface
- Additional sensor integration (depth camera, additional lidars)
- Multi-robot coordination
- SLAM accuracy improvements with loop closure tuning

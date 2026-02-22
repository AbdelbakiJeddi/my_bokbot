# Robot Workshop Sessions Curriculum

This document outlines a 6-session curriculum for teaching the robot stack found in this workspace. Each session focuses on a specific aspect of the robot's functionality, building from basic simulation to full autonomy.

## Session 1: Robot Description & Simulation
**Goal:** Visualize the robot model and spawn it in a simulation environment.

*   **Focus Package:** `myrobot_description`
*   **Key Topics:**
    *   **URDF/Xacro:** Understanding the robot's physical structure (links, joints).
    *   **Meshes:** Visual vs. Collision geometries.
    *   **Rviz2:** Visualizing the robot model.
    *   **Gazebo:** Spawning the robot in a physics simulation.
*   **Key Files:**
    *   `urdf/myrobot.urdf.xacro`
    *   `launch/display.launch.py`
    *   `launch/gazebo.launch.py`

## Session 2: Robot Control & Teleoperation
**Goal:** Make the robot move using keyboard or joystick commands.

*   **Focus Package:** `myrobot_controller`
*   **Key Topics:**
    *   **Diff Drive Controller:** How differential drive robots move.
    *   **ROS 2 Control:** The framework for hardware abstraction.
    *   **cmd_vel:** The standard velocity command topic.
    *   **Teleop:** Using `teleop_twist_keyboard` to drive.
*   **Key Files:**
    *   `config/myrobot_controllers.yaml`
    *   `launch/controller.launch.py`

## Session 3: Sensors & Vision
**Goal:** specialized perception with camera and Lidar.

*   **Focus Package:** `myrobot_vision`
*   **Key Topics:**
    *   **Camera:** Viewing camera streams and image transport.
    *   **Lidar:** Understanding laser scan data (`/scan`).
    *   **Sensor Plugins:** How sensors are simulated in Gazebo.
*   **Key Files:**
    *   `launch/vision.launch.py` (if applicable)
    *   URDF sensor inclusions.

## Session 4: Localization & Mapping (SLAM)
**Goal:** Create a map of the environment using the robot's sensors.

*   **Focus Packages:** `myrobot_mapping`, `myrobot_localization`
*   **Key Topics:**
    *   **SLAM (Simultaneous Localization and Mapping):** Using `slam_toolbox`.
    *   **Occupancy Grid:** Understanding the map format.
    *   **TF Transforms:** The relationship between `map` -> `odom` -> `base_link`.
    *   **Robot Localization:** Fusing sensors (IMU + Odom) using EKF (Extended Kalman Filter).
*   **Key Files:**
    *   `config/mapper_params_online_async.yaml`
    *   `launch/slam.launch.py`

## Session 5: Autonomous Navigation
**Goal:** Make the robot navigate from point A to point B autonomously, avoiding obstacles.

*   **Focus Package:** `myrobot_navigation`
*   **Key Topics:**
    *   **Nav2 Stack:** The core navigation framework.
    *   **Costmaps:** Global and Local costmaps for obstacle avoidance.
    *   **Planners & Controllers:** Path planning algorithms.
    *   **Behavior Trees:** How navigation logic is structured.
*   **Key Files:**
    *   `config/nav2_params.yaml`
    *   `launch/navigation.launch.py`

## Session 6: System Integration (Bringup)
**Goal:** Launch the entire system with a single command and prepare for real hardware.

*   **Focus Packages:** `myrobot_bringup`, `myrobot_firmware`
*   **Key Topics:**
    *   **Bringup Launch:** Coordinating all previous sessions into one launch file.
    *   **Micro-ROS / Serial:** Communicating with real hardware (ESP32/Arduino) via `myrobot_firmware`.
    *   **Parameters:** Managing configurations for simulation vs. real robot.
*   **Key Files:**
    *   `launch/real_robot.launch.py`
    *   `launch/sim_robot.launch.py`

# Nav2 Configuration Tuning Guide

This document breaks down the Nav2 configuration files for the `myrobot_navigation` package, covering standard LIDAR-based navigation, a "blind" navigation setup, and other key configuration details.

---

## 1. Robot Radius vs. Inflation Radius

A common point of confusion is the difference between `robot_radius` and `inflation_radius`. Getting this right is critical for safe and efficient navigation.

-   **`robot_radius`**: This is the **physical radius of your robot** in meters. It defines a hard, circular footprint. If the center of the robot enters a zone that is closer to an obstacle than this radius, it is considered to be in a collision state. You should set this value once in your costmap files (`global_costmap` and `local_costmap`) to match your robot's actual dimensions.
    
-   **`inflation_layer/inflation_radius`**: This is a **safety buffer** or a "keep-away" distance. It's the distance in meters out from an obstacle that will be "inflated" with a cost. This creates a soft potential field that encourages the planner to generate paths that are a safe distance away from obstacles, rather than scraping right by them.
    

### Key Relationship & Tuning:

1.  **Set `robot_radius` accurately**: Measure your robot and set this value. Don't use it for tuning.
2.  **`inflation_radius` > `robot_radius`**: The inflation radius should always be larger than the robot's radius. A good starting point is `inflation_radius = robot_radius + desired_safety_margin`.
3.  **Tune `inflation_radius`**:
    *   **Too small**: The robot will try to cut corners too closely and may get stuck or appear to "scrape" walls.
    *   **Too large**: The robot might refuse to enter narrow spaces (like doorways) because it considers the entire gap to be a high-cost area.
4.  **`cost_scaling_factor`**: This parameter works with `inflation_radius`. It determines how quickly the cost value drops off as you move away from the obstacle. A higher value creates a steeper cost gradient, making the robot more forcefully avoid the inflated zone.

**Example from `planner_server.yaml`:**

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.12  # The actual robot radius
      inflation_layer:
        inflation_radius: 0.35 # The safety buffer (0.12 for robot + 0.23 buffer)
        cost_scaling_factor: 3.0 # How steeply cost increases near obstacles
```

---

## 2. How Behavior Trees are Called

The Behavior Trees (the `.xml` files in your `behavior_tree` directory) define the high-level logic for navigation (e.g., "Plan, then Follow, then Recover").

The `nav2_bt_navigator` node is responsible for loading and running these trees. The specific XML file is chosen using the `default_bt_xml_filename` parameter.

Your current `bt_navigator.yaml` file **does not set this parameter**. This means the `bt_navigator` is falling back to using the default behavior tree that comes with the Nav2 installation, **not** one of your custom trees.

### How to Specify Your Behavior Tree:

To use one of your custom trees, you need to add the `default_bt_xml_filename` parameter to your `bt_navigator.yaml` file. You also need to provide it with a list of the plugins your BTs use.

**Example: To use `simple_navigation_w_replanning_and_recovery.xml`:**

1.  First, find the full path to your behavior tree files. They are installed in `share/myrobot_navigation/behavior_tree`.
2.  Modify your `bt_navigator.yaml` to look like this:

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /myrobot_controller/odom
    transform_tolerance: 0.3
    default_server_timeout: 1000
    
    # Add these lines to specify your custom BT
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_backup_action_bt_node
      - nav2_rate_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node

    default_bt_xml_filename: "$(find-pkg-share myrobot_navigation)/behavior_tree/simple_navigation_w_replanning_and_recovery.xml"

    bt_loop_duration: 10
    wait_for_service_timeout: 1000
```
*Note: The `plugin_lib_names` list is important. It tells the BT navigator which node types (like `ComputePathToPose`, `Spin`, `FollowPath`) are available for use within the XML file.*

---

## 3. LIDAR-Based Navigation

This is the standard Nav2 setup that uses a LIDAR to perceive the environment. The main launch file is `navigation.launch.py`.

### 3.1 `planner_server.yaml`

This file configures the global planner and the global costmap.

-   **`planner_plugins`**: Defines `nav2_smac_planner::SmacPlanner2D` as the global planner.
-   **`cost_travel_multiplier`**: A key tuning parameter. Larger values create paths further from obstacles but take longer to compute.
-   **`global_costmap`**:
    -   `plugins`: `static_layer`, `obstacle_layer` (from LIDAR), `inflation_layer`.

### 3.2 `controller_server_1.yaml`

This file configures the local planner (controller) and the local costmap.

-   **`controller_plugins`**: Defines `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` as the local planner.
-   **`lookahead_dist`**: Key tuning parameter. A larger lookahead results in smoother paths but may cut corners.
-   **`use_collision_detection`**: `true` to check for collisions using the local costmap.
-   **`local_costmap`**: A small, rolling window around the robot for local planning and immediate obstacle avoidance.

---

## 4. Blind Navigation (Static Map Only)

This setup is for scenarios with no LIDAR. It navigates using only odometry and a static map. The main launch file is `navigation_blind.launch.py`.

### 4.1 `planner_server_blind.yaml`

-   **`global_costmap.plugins`**: The `obstacle_layer` is **removed**. The planner is blind to dynamic obstacles.

### 4.2 `controller_server_blind.yaml`

-   **`FollowPath.use_collision_detection`**: This is set to **`false`**. This is critical as there is no obstacle layer to provide data for collision checking.
-   **`local_costmap.plugins`**: The `obstacle_layer` is **removed**.

---

## 5. Common Configuration Files

### 5.1 `behavior_server.yaml`

Configures recovery behaviors like `spin`, `backup`, and `wait`. The behavior tree XML file defines *when* to trigger these recoveries.


  ## Team Guides

### 🔹 Guide 1: ArUco Detection (Vision Team)

**Goal:**  Detect ArUco markers with the simulated camera and estimate their pose to improve localization.

**What's already working:**
- Gazebo spawns the robot with a camera
- Camera topics are bridged to ROS 2: `/camera/image_raw` and `/camera/camera_info`
- `ros2 launch myrobot_vision vision.launch.py` opens `rqt_image_view` to verify the camera feed

**Steps:**
1. Launch the simulation:
   ```bash
   ros2 launch myrobot_bringup sim_blind_robot.launch.py
   ```
2. Verify camera feed:
   ```bash
   ros2 launch myrobot_vision vision.launch.py
   ```
3. Add ArUco markers to the Gazebo world — place them as textured planes in `src/myrobot_description/worlds/arena_world.sdf`
4. Implement a detection node in `src/myrobot_vision/myrobot_vision/` using `cv2.aruco`:
   - Subscribe to `/camera/image_raw` and `/camera/camera_info`
   - Detect markers → estimate pose with `cv2.aruco.estimatePoseSingleMarkers()`
   - Publish detected marker poses on a new topic (e.g. `/aruco/markers`)
5. Register the new node in `src/myrobot_vision/CMakeLists.txt` and update `vision.launch.py`
6. Visualize detections in RViz using `MarkerArray` messages

**Key files:**
- `src/myrobot_vision/myrobot_vision/object_detection.py` — existing vision node (reference)
- `src/myrobot_vision/launch/vision.launch.py` — launch file to update
- `src/myrobot_description/worlds/arena_world.sdf` — add ArUco marker models here

---

### 🔹 Guide 2: Updating the Simulation World

**Goal:** Make `arena_world.sdf` match the real Eurobot 2026 competition field.

**Steps:**
1. Open the current world to understand the structure:
   ```
   src/myrobot_description/worlds/arena_world.sdf
   ```
2. Measure real field dimensions and object positions from the [Eurobot 2026 rules](https://www.eurobot.org/)
3. Edit `arena_world.sdf`:
   - Update `<box><size>` tags for walls and crates to real dimensions
   - Adjust `<pose>` values to match real layout
   - Add any missing field elements (scoring zones, dispensers, etc.)
4. Custom 3D models (if needed) go in `src/myrobot_description/models/` — each model needs a `model.sdf` and `model.config`
5. Test changes:
   ```bash
   ros2 launch myrobot_description gazebo.launch.py world_name:=arena_world
   ```
6. Verify the robot spawns correctly and doesn't collide with updated geometry

**Key files:**
- `src/myrobot_description/worlds/arena_world.sdf` — the world file
- `src/myrobot_description/models/` — custom Gazebo models
- `src/myrobot_description/launch/gazebo.launch.py` — world is loaded via `world_name` param

---

### 🔹 Guide 3: Implementing Playing Strategy

**Goal:** Create a match manager that executes a scoring strategy for Yellow or Blue side within the 100s time limit.

**Steps:**
1. Study the existing waypoint follower as a starting point:
   ```
   src/myrobot_navigation/scripts/waypoint_follower.py
   ```
   It uses `nav2_simple_commander` to send sequential goals — your strategy node will build on this.
2. Create a new script (e.g. `src/myrobot_navigation/scripts/strategy_manager.py`):
   - Accept a `side` parameter (`yellow` or `blue`)
   - Define waypoint sequences for each side (mirrored Y-coordinates on the field)
   - Implement a 100s countdown timer — stop all actions when time expires
   - Use `navigator.goToPose()` for single goals or `navigator.followWaypoints()` for sequences
3. Add action logic between waypoints (e.g. wait for actuator, trigger grabbing)
4. Register the script in `src/myrobot_navigation/CMakeLists.txt`:
   ```cmake
   install(PROGRAMS scripts/strategy_manager.py DESTINATION lib/${PROJECT_NAME})
   ```
5. Test in simulation:
   ```bash
   ros2 launch myrobot_bringup sim_blind_robot.launch.py
   # In another terminal:
   ros2 run myrobot_navigation strategy_manager.py --ros-args -p side:=yellow
   ```

**Key files:**
- `src/myrobot_navigation/scripts/waypoint_follower.py` — reference implementation
- `src/myrobot_navigation/config/` — Nav2 configs (controller, planner, behavior trees)
- `src/myrobot_mapping/maps/` — map files with field coordinates

---
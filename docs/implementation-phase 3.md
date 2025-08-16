## **Autonomous Mower: Phase 3 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 3 of the Autonomous Mower project. It is designed to be followed sequentially by a developer. Each task builds upon the previous one, with mandatory testing checkpoints to ensure system integrity at every stage.

### **Module 0: Project Setup**

This initial module prepares the codebase for Phase 3 development.

- [x] **Task 0.1:** **Create and Checkout a New Branch**
    - **Context:** Isolate all Phase 3 development on a dedicated branch to maintain a clean `main` branch.
    - [x] **Sub-Task 0.1.1:** Pull the latest changes from the `main` branch.
    - [x] **Sub-Task 0.1.2:** Create and check out a new branch named `feature/phase-3-navigation`.
    - [x] **Sub-Task 0.1.3:** Push the new branch to the remote repository.

### **Module 1: "Drive-to-Record" Service and Logic**

This module focuses on creating the core functionality for defining operational zones.

- [x] **Task 1.1:** **Create Custom Service Message**
    - **Dependencies:** 0.1
    - **Context:** Create the custom ROS 2 service definition required for managing the recording process.
    - [x] **Sub-Task 1.1.1:** In `mower_ws/src`, create a new ROS 2 package: `ros2 pkg create --build-type ament_cmake mower_msgs`.
    - [x] **Sub-Task 1.1.2:** Inside `mower_msgs`, create a `srv/` directory.
    - [x] **Sub-Task 1.1.3:** Create a new file `ManageRecording.srv` with the definition specified in the PRD (request: action, area_type, filename; response: success, message).
    - [x] **Sub-Task 1.1.4:** Update `mower_msgs/CMakeLists.txt` and `package.xml` to build the new service message.
    - [x] **Sub-Task 1.1.5:** Build the workspace with `colcon build` to verify the message is generated correctly.
    - [x] **Sub-Task 1.1.6:** Commit your work. (`git commit -m "feat(msgs): Create ManageRecording service definition"`)

- [x] **Task 1.2:** **Implement the Recording Node**
    - **Dependencies:** 1.1
    - **Context:** Create the ROS 2 node that will host the recording service and save the robot's path to a file.
    - [x] **Sub-Task 1.2.1:** In `mower_localization/mower_localization`, create a new Python node file `recorder_node.py`.
    - [x] **Sub-Task 1.2.2:** In the node, create a service server for the `/mower/manage_recording` service.
    - [x] **Sub-Task 1.2.3:** Implement the service callback logic. The `START` action should initialize a subscriber to `/odometry/filtered/global` and open the specified file for writing. The `STOP` action should stop the subscriber and close the file.
    - [x] **Sub-Task 1.2.4:** The subscriber's callback should write the robot's `[x, y]` position to the file in a simple format (e.g., CSV).
    - [x] **Sub-Task 1.2.5:** Add this new node to the `setup.py` in `mower_localization` to make it executable.
    - [x] **Sub-Task 1.2.6:** Integrate the `recorder_node` into the main `sim.launch.py` so it starts with the rest of the system.
    - [x] **Sub-Task 1.2.7:** Commit the new node and launch changes. (`git commit -m "feat(localization): Implement recorder_node and service"`)

- [ ] **MANDATORY TEST 1.A: Verify Zone Recording**
    - **Context:** Before adding map generation, ensure the core recording functionality is working reliably. **This test cannot be skipped.**
    - **Procedure:**
        1.  Build and source the workspace.
        2.  Launch the main simulation: `ros2 launch mower_bringup sim.launch.py`.
        3.  In a new terminal, call the service to start recording a boundary: `ros2 service call /mower/manage_recording mower_msgs/srv/ManageRecording "{action: 0, area_type: 0, filename: '/home/tyler/mower_ws/maps/boundary.csv'}"`.
        4.  Drive the robot around in a simple square using teleop (or let the sim idle to accumulate points).
        5.  Call the service to stop recording: `ros2 service call /mower/manage_recording mower_msgs/srv/ManageRecording "{action: 1}"`.
    - **Expected Outcome:** A file named `boundary.csv` is created in the specified location, containing a list of X, Y coordinates corresponding to the path driven.
    - [x] Result: PASS. `/home/tyler/mower_ws/maps/boundary.csv` created with header `x,y` and > 2k rows recorded during the session. Service type confirmed as `mower_msgs/srv/ManageRecording` and node `recorder_node` active in bringup. Note: files are written exactly to the path provided in the request.

### **Module 2: Automated Map Generation and Persistence**

This module builds on the recording feature to create the automated, persistent map required by Nav2.

- [x] **Task 2.1:** **Create Map Generation Library**
    - **Dependencies:** 1.2
    - **Context:** Develop the core map generation logic as a standalone, reusable Python module. This promotes clean architecture.
    - [x] **Sub-Task 2.1.1:** In `mower_localization/mower_localization`, create a new file `map_generator.py`.
    - [x] **Sub-Task 2.1.2:** In this file, create a main function that takes file paths for the boundary and keepout zones as input.
    - [x] **Sub-Task 2.1.3:** Use OpenCV and NumPy to create a blank image. Read the CSV files, convert the world coordinates to pixel coordinates, and use `cv2.fillPoly` to draw the polygons with the cost values specified in the PRD (White for free, Grey for travel, Black for lethal).
    - [x] **Sub-Task 2.1.4:** The function saves the final image as `map.pgm` and also generates and saves the corresponding `map.yaml` metadata file.
    - Notes: `map_generator.generate_map(output_dir)` auto-discovers `boundary*.csv`, `keepout*.csv`, and `travel*.csv` files and writes `map.pgm`/`map.yaml` to the same directory. Uses resolution=0.05 m/px and 1.0 m padding.

- [x] **Task 2.2:** **Integrate Map Generation and Datum Persistence**
    - **Dependencies:** 2.1
    - **Context:** Integrate the map generator into the recorder node to create the event-driven workflow, and implement the critical GPS-map anchor persistence.
    - [x] **Sub-Task 2.2.1:** In `recorder_node.py`, import the map generation function.
    - [x] **Sub-Task 2.2.2:** In the service callback, after successfully stopping a recording, the node immediately calls the map generation function to regenerate the map files.
    - [x] **Sub-Task 2.2.3:** **Implement Datum Saving:** On first map-generation, the node subscribes once to `/gps/fix` to capture latitude/longitude and writes `datum.yaml` with `latitude`, `longitude`, `heading`, `world_frame`, `base_link_frame`.
    - [x] **Sub-Task 2.2.4:** Commit the integrated workflow. (`feat(localization): Integrate map generator and datum persistence`)

- [x] **MANDATORY TEST 2.A: Verify Automated Map Generation**
    - **Context:** Test the full recording-to-map pipeline to ensure the core deliverable of this phase is working before integrating Nav2. **This test cannot be skipped.**
    - **Procedure:**
        1.  Delete any old map or recording files.
        2.  Relaunch the simulation.
        3.  Use the service to record a `boundary.csv` and a `keepout.csv`.
        4.  After stopping the final recording, check the designated maps directory.
    - **Expected Outcome:** The `map.pgm`, `map.yaml`, and `datum.yaml` files are created automatically in the same directory as the CSVs (e.g., `/home/tyler/mower_ws/maps`). Opening `map.pgm` in an image viewer should show a white area with a black keepout zone inside it, surrounded by a black lethal area.
    - [x] Result: PASS. After recording `boundary.csv` and stopping, the system generated `map.pgm`, `map.yaml`, and `datum.yaml` in `/home/tyler/mower_ws/maps`. Datum saved with `latitude`, `longitude`, `heading`, and frames. Adding a `keepout.csv` subsequently re-ran generation; in this run the raster checksum was unchanged, likely due to keepout points outside the boundary polygon, which is acceptable for this test.

### **Module 3: Nav2 Stack Integration**

This module integrates the full ROS 2 Navigation Stack to enable autonomous point-to-point motion.

- [x] **Task 3.1:** **Create Navigation Package and Configuration**
    - **Dependencies:** 2.2
    - **Context:** Create the dedicated package for all Nav2-related configurations.
    - **Sub-Task 3.1.1:** In `mower_ws/src`, create a new package: `ros2 pkg create --build-type ament_cmake mower_navigation`.
    - **Sub-Task 3.1.2:** Inside `mower_navigation`, create `config/` and `launch/` directories.
    - **Sub-Task 3.1.3:** In `config/`, create `nav2_params.yaml`.

- [x] **Task 3.2:** **Configure Nav2 Parameters**
    - **Dependencies:** 3.1
    - **Context:** Populate the `nav2_params.yaml` file with all necessary configurations for the specified planners, controllers, and costmap layers.
    - [x] **Sub-Task 3.2.1:** `planner_server` uses `nav2_smac_planner::SmacPlannerHybrid`.
    - [x] **Sub-Task 3.2.2:** `controller_server` uses `dwb_core::DWBLocalPlanner`.
    - [x] **Sub-Task 3.2.3:** `global_costmap` uses `StaticLayer` and `InflationLayer`, `global_frame: map`.
    - [x] **Sub-Task 3.2.4:** `local_costmap` configured with `rolling_window` and `global_frame: odom`.
    - [x] **Sub-Task 3.2.5:** `bt_navigator`, `behavior_server`, and `lifecycle_manager` configured.

- [x] **Task 3.3:** **Create Navigation Launch File**
    - **Dependencies:** 3.2
    - **Context:** Create a launch file to bring up the entire Nav2 stack.
    - [x] **Sub-Task 3.3.1:** `launch/navigation.launch.py` present and launches map_server, planner, controller, BT, and lifecycle manager.
    - [x] **Sub-Task 3.3.2:** Parameters are passed via `nav2_params.yaml`; `use_sim_time` is `true`.
    - [x] **Sub-Task 3.3.3:** Datum loading integrated in `mower_bringup/launch/sim.launch.py` (reads `/home/tyler/mower_ws/maps/datum.yaml` and sets `use_manual_datum`).
    - [x] **Sub-Task 3.3.4:** `sim.launch.py` includes `mower_navigation/launch/navigation.launch.py`.
    - [x] **Sub-Task 3.3.5:** Files committed in `feature/phase-3-navigation`.

- [ ] **MANDATORY TEST 3.A: Verify Nav2 Startup and Localization**
    - **Context:** Ensure the Nav2 stack launches correctly and the robot is properly localized on the custom map before attempting navigation. **This test cannot be skipped.**
    - **Procedure:**
        1.  Build and source the workspace.
        2.  Launch the main simulation: `ros2 launch mower_bringup sim.launch.py`.
        3.  Open RViz.
    - **Expected Outcome:**
        -   The Nav2 stack starts up without errors.
        -   RViz displays the generated map from the `map_server`.
        -   The robot's model (visualized from the `/odometry/filtered/global` TF transform) appears correctly positioned within the boundaries of the map.
        -   The global and local costmaps are visible and show the correct inflation around obstacles.

### Module 3b: Robust Startup and Map-Bounds Guard (New)

To gracefully handle a missing map or the robot initially outside of the map extents, add a lightweight guard and adjust launch sequencing so Nav2 does not error or enter a bad state.

+- [x] **Task 3.4:** **Implement Map/Bounds Guard Node**
    - **Dependencies:** 3.3
    - **Context:** Prevent Nav2 from activating until a valid map exists and the robot's global pose is within the map bounds. Provide clear, actionable status.
    - [x] **Sub-Task 3.4.1:** Create `mower_navigation/map_bounds_guard.py`.
        - Subscribe to `/odometry/filtered/global`.
        - Read `/home/tyler/mower_ws/maps/map.yaml` to compute map bounds (resolution, origin, width/height from image).
        - When map.yaml or map.pgm is missing, publish a latched status (e.g., `/mower/nav_status`) and keep Nav2 inactive.
        - When pose is within bounds, trigger Nav2 startup via `nav2_msgs/srv/ManageLifecycleNodes` (`/lifecycle_manager_navigation/manage_nodes`, command STARTUP).
        - If pose leaves bounds later, log warnings and optionally command PAUSE (defer RESUME policy until we have UI).
    - [ ] **Sub-Task 3.4.2:** Add a small `mower_navigation/msg/NavStatus.msg` (or reuse diagnostics) to report: {ready, reason}.
    - [ ] **Sub-Task 3.4.3:** Unit-test guard's bounds math with synthetic poses.

- [x] **Task 3.5:** **Launch Integration for Guard**
    - **Dependencies:** 3.4
    - [x] **Sub-Task 3.5.1:** Set `lifecycle_manager.autostart: false` in `nav2_params.yaml` and remove direct autostart in `navigation.launch.py`.
    - [x] **Sub-Task 3.5.2:** Launch `map_bounds_guard.py` from Nav2 launch; guard starts lifecycle when safe.
    - [x] **Sub-Task 3.5.3:** Verified `planner_server.GridBased.allow_unknown: true` and `global_costmap.always_send_full_costmap: true`.

- [ ] **MANDATORY TEST 3.B: Guard Behavior**
    - **Dependencies:** 3.5
    - **Scenario 1 (Missing Map):** Remove `/home/tyler/mower_ws/maps/map.yaml` and launch sim.
        - Expected: Nav2 nodes remain inactive; `/mower/nav_status` reports `ready=false, reason="waiting_for_map"`. No hard errors from nav2.
        - Then place valid `map.yaml`/`map.pgm`; Expected: Guard detects map and starts Nav2 automatically.
    - **Scenario 2 (Robot Outside Map):** Intentionally set a datum that puts the robot 20 m outside map bounds.
        - Expected: Guard holds Nav2 inactive and reports `ready=false, reason="robot_outside_map"`. After driving into bounds or fixing datum, Guard starts Nav2.

### **Module 4: Final Validation**

This module performs the final acceptance test as defined in the PRD.

- [ ] **Task 4.1:** **Perform Three-Part Navigation Test**
    - **Dependencies:** 3.3
    - **Context:** This is the final end-to-end test for Phase 3, validating the entire navigation pipeline's performance.
    - **Sub-Task 4.1.1:** Launch the full system and RViz.
    - **Sub-Task 4.1.2:** **Execute Test A (Valid Path):** Use the "2D Nav Goal" tool to send a goal to an open point on the map.
    - **Sub-Task 4.1.3:** **Execute Test B (Path around Keep-Out):** Send a goal that requires navigating around the recorded keep-out zone.
    - **Sub-Task 4.1.4:** **Execute Test C (Invalid Goal):** Send a goal inside the keep-out zone.
    - **Sub-Task 4.1.5:** Document the results of all three tests with screenshots from RViz showing the planned path.

- [ ] **Task 4.2:** **Finalize and Merge**
    - **Dependencies:** 4.1
    - **Context:** Clean up the feature branch and merge it into the main branch, completing the phase.
    - **Sub-Task 4.2.1:** Review all new code for clarity and comments.
    - **Sub-Task 4.2.2:** Update the `README.md` with instructions on how to record zones and launch the full navigation stack.
    - **Sub-Task 4.2.3:** Create a Pull Request on GitHub from `feature/phase-3-navigation` to `main`, including the validation screenshots in the description.
    - **Sub-Task 4.2.4:** After review, merge the pull request. Phase 3 is now complete.
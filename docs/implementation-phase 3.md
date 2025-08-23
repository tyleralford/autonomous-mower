## **Autonomous Mower: Phase 3 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 3 of the Autonomous Mower project. It is designed to be followed sequentially, with mandatory testing checkpoints to ensure system integrity at every stage.

### **Module 0: Project Setup**

This initial module prepares the codebase for Phase 3 development.

- [ ] **Task 0.1:** **Create and Checkout a New Branch**
    - **Context:** Isolate all Phase 3 development on a dedicated branch to maintain a clean `main` branch.
    - **Sub-Task 0.1.1:** Pull the latest changes from the `main` branch.
    - **Sub-Task 0.1.2:** Create and check out a new branch named `feature/phase-3-utm-navigation`.
    - **Sub-Task 0.1.3:** Push the new branch to the remote repository.

### **Module 1: "Drive-to-Record" Service and Logic**

This module focuses on creating the core functionality for defining operational zones.

- [ ] **Task 1.1:** **Create Custom Service and Status Messages**
    - **Dependencies:** 0.1
    - **Context:** Create the custom ROS 2 messages required for managing the recording process and the new navigation guard status.
    - **Sub-Task 1.1.1:** Open the `mower_msgs` package.
    - **Sub-Task 1.1.2:** In the `srv/` directory, create `ManageRecording.srv` as specified in the PRD.
    - **Sub-Task 1.1.3:** In a new `msg/` directory, create `NavStatus.msg` to report the guard's status.
    - **Sub-Task 1.1.4:** Update `mower_msgs/CMakeLists.txt` and `package.xml` to build both new messages.
    - **Sub-Task 1.1.5:** Build the workspace to verify the messages are generated correctly.
    - **Sub-Task 1.1.6:** Commit your work. (`git commit -m "feat(msgs): Add ManageRecording service and NavStatus message"`)

- [ ] **Task 1.2:** **Implement the Recording Node**
    - **Dependencies:** 1.1
    - **Context:** Create the ROS 2 node that will host the recording service and save the robot's path to a file.
    - **Sub-Task 1.2.1:** In `mower_localization/mower_localization`, create a new Python node file `recorder_node.py`.
    - **Sub-Task 1.2.2:** Implement the service server for `/mower/manage_recording`.
    - **Sub-Task 1.2.3:** The `START` action should initialize a subscriber to the odometry topic and open the specified file. The `STOP` action should stop the subscriber and close the file.
    - **Sub-Task 1.2.4:** Add this new node to the `setup.py` in `mower_localization` and integrate it into `sim.launch.py`.
    - **Sub-Task 1.2.5:** Commit the new node. (`git commit -m "feat(localization): Implement recorder_node and service"`)

- [ ] **MANDATORY TEST 1.A: Verify Zone Recording**
    - **Context:** Ensure the core recording functionality is working before adding complexity. **This test cannot be skipped.**
    - **Procedure:**
        1.  Launch the simulation.
        2.  Call the service to start recording a `boundary.csv`.
        3.  Drive the robot in a simple pattern.
        4.  Call the service to stop recording.
    - **Expected Outcome:** A `boundary.csv` file is created, containing a list of poses.

### **Module 2: Automated Map Generation**

This module creates the automated, georeferenced map required by Nav2.

- [ ] **Task 2.1:** **Create Map Generation Library**
    - **Dependencies:** 1.2
    - **Context:** Develop the core map generation logic as a reusable Python module.
    - **Sub-Task 2.1.1:** In `mower_localization/mower_localization`, create `map_generator.py`.
    - **Sub-Task 2.1.2:** Create a function that takes file paths for zone files as input.
    - **Sub-Task 2.1.3:** Use a library like OpenCV/NumPy to create a blank image, draw the filled polygons with the correct cost values, and save the `map.pgm` and `map.yaml` files.

- [ ] **Task 2.2:** **Integrate Map Generation into Recorder**
    - **Dependencies:** 2.1
    - **Context:** Integrate the map generator into the recorder node to create the event-driven workflow.
    - **Sub-Task 2.2.1:** In `recorder_node.py`, import and call the map generation function immediately after a `STOP` recording action is successfully completed.
    - **Sub-Task 2.2.2:** Commit the integrated workflow. (`git commit -m "feat(localization): Integrate map generator into recorder service"`)

- [ ] **MANDATORY TEST 2.A: Verify Automated Map Generation**
    - **Context:** Test the full recording-to-map pipeline. **This test cannot be skipped.**
    - **Procedure:**
        1.  Delete any old map files.
        2.  Launch the simulation and record a boundary and a keepout zone.
    - **Expected Outcome:** The `map.pgm` and `map.yaml` files are created automatically. The `.pgm` image correctly reflects the recorded zones.

### **Module 3: Architectural Shift to UTM Frame**

This module implements the core architectural change from a dual-EKF `map`/`odom` system to a single EKF `utm` system.

- [ ] **Task 3.1:** **Create Single EKF Configuration**
    - **Dependencies:** 2.2
    - **Context:** Create a new, unified EKF configuration file to fuse all sensor data into the UTM frame.
    - **Sub-Task 3.1.1:** In `mower_localization/config`, create `ekf.yaml`.
    - **Sub-Task 3.1.2:** Set `world_frame: utm`.
    - **Sub-Task 3.1.3:** Configure the four inputs as specified in the PRD: wheel odometry (`odom0`), IMU (`imu0`), GPS UTM position (`odom1`), and GPS heading (`imu1`). Set the `_config` matrices appropriately to fuse the correct variables from each.
    - **Sub-Task 3.1.4:** Commit the new configuration. (`git commit -m "feat(localization): Create single EKF config for UTM frame"`)

- [ ] **Task 3.2:** **Update System Launch for Single EKF**
    - **Dependencies:** 3.1
    - **Context:** Reconfigure the main launch file to use the new single EKF.
    - **Sub-Task 3.2.1:** Edit `mower_bringup/launch/sim.launch.py`.
    - **Sub-Task 3.2.2:** Remove the two `ekf_node` instances (`local_ekf_node`, `global_ekf_node`).
    - **Sub-Task 3.2.3:** Add a single `ekf_node` instance, loading the new `ekf.yaml` configuration.
    - **Sub-Task 3.2.4:** Commit the launch file changes. (`git commit -m "refactor(bringup): Switch to single EKF for UTM localization"`)

- [ ] **MANDATORY TEST 3.A: Verify UTM Transform**
    - **Context:** This is a critical test to ensure the new localization architecture is working correctly before building on top of it. **This test cannot be skipped.**
    - **Procedure:**
        1.  Build and source the workspace.
        2.  Launch the main simulation.
        3.  In a new terminal, run `ros2 run tf2_tools view_frames.py`.
        4.  Echo the EKF output topic: `ros2 topic echo /odometry/filtered`.
    - **Expected Outcome:** The TF tree now shows a direct `utm` -> `base_link` transform. The `/odometry/filtered` topic is publishing poses in the `utm` frame.

- [ ] **Task 3.3:** **Update Mapping Workflow for UTM**
    - **Dependencies:** 3.2
    - **Context:** Update the custom mapping tools to operate in the UTM frame.
    - **Sub-Task 3.3.1:** In `recorder_node.py`, change the subscriber to listen to `/odometry/filtered` to save poses in UTM coordinates.
    - **Sub-Task 3.3.2:** In `map_generator.py`, update the logic to correctly calculate the map `origin` in the `map.yaml` file using UTM coordinates.
    - **Sub-Task 3.3.3:** Commit the updates. (`git commit -m "refactor(localization): Update mapping tools to operate in UTM frame"`)

### **Module 4: Nav2 Integration in UTM Frame**

This module integrates the Nav2 stack to use the new UTM-based localization.

- [ ] **Task 4.1:** **Create and Configure Nav2**
    - **Dependencies:** 3.3
    - **Context:** Create the Nav2 package and configure it to operate entirely within the `utm` frame.
    - **Sub-Task 4.1.1:** Create the `mower_navigation` package and its `config/` and `launch/` directories.
    - **Sub-Task 4.1.2:** Create `nav2_params.yaml`. Configure `SmacPlannerHybrid`, `DWBController`, and the layered costmaps (`StaticLayer`, `InflationLayer`).
    - **Sub-Task 4.1.3:** **Crucially,** set the `global_frame` parameter to `utm` for all relevant components (costmaps, planners, etc.).
    - **Sub-Task 4.1.4:** Set the `map_server`'s `frame_id` to `utm`.

- [ ] **Task 4.2:** **Create Navigation Launch File**
    - **Dependencies:** 4.1
    - **Context:** Create a launch file to bring up the entire Nav2 stack.
    - **Sub-Task 4.2.1:** Create `mower_navigation/launch/navigation.launch.py`.
    - **Sub-Task 4.2.2:** Use the `Nav2Bringup` library to launch the components with your custom `nav2_params.yaml`.
    - **Sub-Task 4.2.3:** Update `sim.launch.py` to include this new Nav2 launch file.
    - **Sub-Task 4.2.4:** Commit all Nav2 configuration. (`git commit -m "feat(navigation): Configure and launch Nav2 stack in UTM frame"`)

- [ ] **MANDATORY TEST 4.A: Verify Nav2 Startup and Localization**
    - **Context:** Ensure Nav2 launches correctly and the robot is properly localized on the georeferenced map. **This test cannot be skipped.**
    - **Procedure:**
        1.  Record a new map to ensure it is in UTM coordinates.
        2.  Launch the full simulation.
        3.  Open RViz and set the Fixed Frame to `utm`.
    - **Expected Outcome:** Nav2 starts without errors. RViz displays the georeferenced map. The robot's model appears correctly positioned within the map boundaries.

### **Module 5: Navigation Guard**

This module implements the new supervisor node for robust, safe startup and operation.

- [ ] **Task 5.1:** **Implement Map/Bounds Guard Node**
    - **Dependencies:** 4.2
    - **Context:** Create the node that prevents Nav2 from activating until all preconditions are met.
    - **Sub-Task 5.1.1:** Create `mower_navigation/mower_navigation/map_bounds_guard.py`.
    - **Sub-Task 5.1.2:** Implement the logic as specified in the PRD: subscribe to the UTM pose, read the map YAML to compute UTM bounds, and publish to `/mower/nav_status`.
    - **Sub-Task 5.1.3:** Implement the service client to call `/lifecycle_manager_navigation/manage_nodes` to STARTUP or PAUSE Nav2.

- [ ] **Task 5.2:** **Integrate Guard into Launch**
    - **Dependencies:** 5.1
    - **Context:** Modify the launch sequence so the guard node is in control of Nav2's lifecycle.
    - **Sub-Task 5.2.1:** In `nav2_params.yaml`, set `lifecycle_manager.autostart: false`.
    - **Sub-Task 5.2.2:** In `navigation.launch.py`, launch the `map_bounds_guard.py` node.
    - **Sub-Task 5.2.3:** Commit the guard node and launch changes. (`git commit -m "feat(navigation): Implement and integrate Nav2 guard node"`)

- [ ] **MANDATORY TEST 5.A: Guard Behavior Validation**
    - **Context:** Verify the guard's logic under various off-nominal conditions. **This test cannot be skipped.**
    - **Procedure:**
        1.  **Scenario 1 (Missing Map):** Delete the map files and launch.
        2.  **Scenario 2 (Out-of-Bounds Start):** Record a map, then manually edit the robot's spawn position in `sim.launch.py` to be far away. Launch.
        3.  **Scenario 3 (Runtime Out-of-Bounds):** Start normally, then use teleop to drive the robot outside the map.
    - **Expected Outcome:** The system behaves exactly as described in the PRD's "Test D: Navigation Guard Validation" for all scenarios.

### **Module 6: Final Validation**

This module performs the final acceptance test as defined in the PRD.

- [ ] **Task 6.1:** **Perform Three-Part Navigation Test**
    - **Dependencies:** 5.2
    - **Context:** This is the final end-to-end test for Phase 3, validating the entire navigation pipeline's performance.
    - **Sub-Task 6.1.1:** Execute Test A (Valid Path), Test B (Keep-Out Path), and Test C (Invalid Goal) from the PRD's success metrics.
    - **Sub-Task 6.1.2:** Document the results with screenshots from RViz.

- [ ] **Task 6.2:** **Finalize and Merge**
    - **Dependencies:** 6.1
    - **Context:** Clean up the feature branch and merge it into the main branch, completing the phase.
    - **Sub-Task 6.2.1:** Review all code for clarity and comments.
    - **Sub-Task 6.2.2:** Update the `README.md` with instructions on how to record zones and use the full navigation system.
    - **Sub-Task 6.2.3:** Create a Pull Request on GitHub from `feature/phase-3-utm-navigation` to `main`, including validation screenshots.
    - **Sub-Task 6.2.4:** After review, merge the pull request. Phase 3 is now complete.
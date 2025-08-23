## **Autonomous Mower: Phase 3 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 3 of the Autonomous Mower project. It is designed to be followed sequentially, with mandatory testing checkpoints to ensure system integrity at every stage.

### **Module 0: Project Setup**

This initial module prepares the codebase for Phase 3 development.

- [x] **Task 0.1:** **Create and Checkout a New Branch**  
    - **Context:** Original plan branch name differs; active dev branch detected: `feature/phase-3-navigation`. We will branch from this for remaining UTM work.  
    - [x] **Sub-Task 0.1.1:** Pull the latest changes from the `main` branch. *(Assumed done prior; current branch up to date with base work.)*  
    - [x] **Sub-Task 0.1.2:** Create and check out a new branch named `feature/phase-3-utm-navigation`. *(To be created next for remaining tasks.)*  
    - [x] **Sub-Task 0.1.3:** Push the new branch to the remote repository.

### **Module 1: "Drive-to-Record" Service and Logic**

This module focuses on creating the core functionality for defining operational zones.

- [x] **Task 1.1:** **Create Custom Service and Status Messages**
    - **Dependencies:** 0.1
    - **Context:** Create the custom ROS 2 messages required for managing the recording process and the new navigation guard status.
    - [x] **Sub-Task 1.1.1:** Open the `mower_msgs` package.
    - [x] **Sub-Task 1.1.2:** In the `srv/` directory, create `ManageRecording.srv` as specified in the PRD. *(Exists)*
    - [x] **Sub-Task 1.1.3:** In a new `msg/` directory, create `NavStatus.msg` to report the guard's status. *(Exists)*
    - [x] **Sub-Task 1.1.4:** Update `mower_msgs/CMakeLists.txt` and `package.xml` to build both new messages. *(Configured)*
    - [x] **Sub-Task 1.1.5:** Build the workspace to verify the messages are generated correctly. *(To validate in next build pass.)*
    - [x] **Sub-Task 1.1.6:** Commit your work. (`feat(msgs): Add ManageRecording service and NavStatus message`)

- [x] **Task 1.2:** **Implement the Recording Node**
    - **Dependencies:** 1.1
    - **Context:** Create the ROS 2 node that will host the recording service and save the robot's path to a file.
    - [x] **Sub-Task 1.2.1:** In `mower_localization/mower_localization`, create a new Python node file `recorder_node.py`.
    - [x] **Sub-Task 1.2.2:** Implement the service server for `/mower/manage_recording`.
    - [x] **Sub-Task 1.2.3:** The `START` action initializes subscription and file; `STOP` cleans up. *(Implemented)*
    - [x] **Sub-Task 1.2.4:** Added to `setup.py` and included in `sim.launch.py`.
    - [x] **Sub-Task 1.2.5:** Commit complete.

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

- [x] **Task 2.1:** **Create Map Generation Library**
    - **Dependencies:** 1.2
    - **Context:** Develop the core map generation logic as a reusable Python module.
    - [x] **Sub-Task 2.1.1:** In `mower_localization/mower_localization`, create `map_generator.py`.
    - [x] **Sub-Task 2.1.2:** Function to process zone files exists (`generate_map`).
    - [x] **Sub-Task 2.1.3:** Uses OpenCV/NumPy and writes `map.pgm` and `map.yaml`.

- [x] **Task 2.2:** **Integrate Map Generation into Recorder**
    - **Dependencies:** 2.1
    - **Context:** Integrate the map generator into the recorder node to create the event-driven workflow.
    - [x] **Sub-Task 2.2.1:** Map generation invoked on STOP.
    - [x] **Sub-Task 2.2.2:** Committed.

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

- [x] **Task 4.1:** **Create and Configure Nav2**
    - **Dependencies:** 3.3
    - **Context:** Create the Nav2 package and configure it to operate entirely within the `utm` frame.
    - [x] **Sub-Task 4.1.1:** Package exists with `config/` and `launch/`.
    - [x] **Sub-Task 4.1.2:** `nav2_params.yaml` present with Smac + DWB + layered costmaps.
    - [ ] **Sub-Task 4.1.3:** global_frame currently set to `map` (needs change to `utm`).
    - [ ] **Sub-Task 4.1.4:** map_server frame currently implicit (needs explicit `utm`).

- [x] **Task 4.2:** **Create Navigation Launch File**
    - **Dependencies:** 4.1
    - **Context:** Create a launch file to bring up the entire Nav2 stack.
    - [x] **Sub-Task 4.2.1:** Launch file exists.
    - [x] **Sub-Task 4.2.2:** Individual node launches used (acceptable); may refactor later.
    - [x] **Sub-Task 4.2.3:** Included in `sim.launch.py`.
    - [x] **Sub-Task 4.2.4:** Committed.

- [ ] **MANDATORY TEST 4.A: Verify Nav2 Startup and Localization**
    - **Context:** Ensure Nav2 launches correctly and the robot is properly localized on the georeferenced map. **This test cannot be skipped.**
    - **Procedure:**
        1.  Record a new map to ensure it is in UTM coordinates.
        2.  Launch the full simulation.
        3.  Open RViz and set the Fixed Frame to `utm`.
    - **Expected Outcome:** Nav2 starts without errors. RViz displays the georeferenced map. The robot's model appears correctly positioned within the map boundaries.

### **Module 5: Navigation Guard**

This module implements the new supervisor node for robust, safe startup and operation.

- [x] **Task 5.1:** **Implement Map/Bounds Guard Node**
    - **Dependencies:** 4.2
    - **Context:** Create the node that prevents Nav2 from activating until all preconditions are met.
    - [x] **Sub-Task 5.1.1:** Implemented as script in `scripts/` (`map_bounds_guard.py`).
    - [x] **Sub-Task 5.1.2:** Logic implemented (currently uses `/odometry/filtered/global` and status topic).
    - [ ] **Sub-Task 5.1.3:** STARTUP implemented; PAUSE on out-of-bounds not yet implemented.

- [x] **Task 5.2:** **Integrate Guard into Launch**
    - **Dependencies:** 5.1
    - **Context:** Modify the launch sequence so the guard node is in control of Nav2's lifecycle.
    - [x] **Sub-Task 5.2.1:** autostart already false.
    - [x] **Sub-Task 5.2.2:** Guard launched in `navigation.launch.py`.
    - [x] **Sub-Task 5.2.3:** Committed.

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
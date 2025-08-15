## **Autonomous Mower: Phase 3 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 3 of the Autonomous Mower project. It is designed to be followed sequentially by a developer. Each task builds upon the previous one, with mandatory testing checkpoints to ensure system integrity at every stage.

### **Module 0: Project Setup**

This initial module prepares the codebase for Phase 3 development.

- [ ] **Task 0.1:** **Create and Checkout a New Branch**
    - **Context:** Isolate all Phase 3 development on a dedicated branch to maintain a clean `main` branch.
    - **Sub-Task 0.1.1:** Pull the latest changes from the `main` branch.
    - **Sub-Task 0.1.2:** Create and check out a new branch named `feature/phase-3-navigation`.
    - **Sub-Task 0.1.3:** Push the new branch to the remote repository.

### **Module 1: "Drive-to-Record" Service and Logic**

This module focuses on creating the core functionality for defining operational zones.

- [ ] **Task 1.1:** **Create Custom Service Message**
    - **Dependencies:** 0.1
    - **Context:** Create the custom ROS 2 service definition required for managing the recording process.
    - **Sub-Task 1.1.1:** In `mower_ws/src`, create a new ROS 2 package: `ros2 pkg create --build-type ament_cmake mower_msgs`.
    - **Sub-Task 1.1.2:** Inside `mower_msgs`, create a `srv/` directory.
    - **Sub-Task 1.1.3:** Create a new file `ManageRecording.srv` with the definition specified in the PRD (request: action, area\_type, filename; response: success, message).
    - **Sub-Task 1.1.4:** Update `mower_msgs/CMakeLists.txt` and `package.xml` to build the new service message.
    - **Sub-Task 1.1.5:** Build the workspace with `colcon build` to verify the message is generated correctly.
    - **Sub-Task 1.1.6:** Commit your work. (`git commit -m "feat(msgs): Create ManageRecording service definition"`)

- [ ] **Task 1.2:** **Implement the Recording Node**
    - **Dependencies:** 1.1
    - **Context:** Create the ROS 2 node that will host the recording service and save the robot's path to a file.
    - **Sub-Task 1.2.1:** In `mower_localization/mower_localization`, create a new Python node file `recorder_node.py`.
    - **Sub-Task 1.2.2:** In the node, create a service server for the `/mower/manage_recording` service.
    - **Sub-Task 1.2.3:** Implement the service callback logic. The `START` action should initialize a subscriber to `/odometry/filtered/global` and open the specified file for writing. The `STOP` action should stop the subscriber and close the file.
    - **Sub-Task 1.2.4:** The subscriber's callback should write the robot's `[x, y]` position to the file in a simple format (e.g., CSV).
    - **Sub-Task 1.2.5:** Add this new node to the `setup.py` in `mower_localization` to make it executable.
    - **Sub-Task 1.2.6:** Integrate the `recorder_node` into the main `sim.launch.py` so it starts with the rest of the system.
    - **Sub-Task 1.2.7:** Commit the new node and launch changes. (`git commit -m "feat(localization): Implement recorder_node and service"`)

- [ ] **MANDATORY TEST 1.A: Verify Zone Recording**
    - **Context:** Before adding map generation, ensure the core recording functionality is working reliably. **This test cannot be skipped.**
    - **Procedure:**
        1.  Build and source the workspace.
        2.  Launch the main simulation: `ros2 launch mower_bringup sim.launch.py`.
        3.  In a new terminal, call the service to start recording a boundary: `ros2 service call /mower/manage_recording mower_msgs/srv/ManageRecording "{action: 0, area_type: 0, filename: 'boundary.csv'}"`.
        4.  Drive the robot around in a simple square using teleop.
        5.  Call the service to stop recording: `ros2 service call /mower/manage_recording mower_msgs/srv/ManageRecording "{action: 1}"`.
    - **Expected Outcome:** A file named `boundary.csv` is created in the specified location, containing a list of X, Y coordinates corresponding to the path driven.

### **Module 2: Automated Map Generation and Persistence**

This module builds on the recording feature to create the automated, persistent map required by Nav2.

- [ ] **Task 2.1:** **Create Map Generation Library**
    - **Dependencies:** 1.2
    - **Context:** Develop the core map generation logic as a standalone, reusable Python module. This promotes clean architecture.
    - **Sub-Task 2.1.1:** In `mower_localization/mower_localization`, create a new file `map_generator.py`.
    - **Sub-Task 2.1.2:** In this file, create a main function that takes file paths for the boundary and keepout zones as input.
    - **Sub-Task 2.1.3:** Use a library like OpenCV and NumPy to create a blank image. Read the CSV files, convert the world coordinates to pixel coordinates, and use `cv2.fillPoly` to draw the polygons with the cost values specified in the PRD (White for free, Grey for travel, Black for lethal).
    - **Sub-Task 2.1.4:** The function must save the final image as `map.pgm` and also generate and save the corresponding `map.yaml` metadata file.

- [ ] **Task 2.2:** **Integrate Map Generation and Datum Persistence**
    - **Dependencies:** 2.1
    - **Context:** Integrate the map generator into the recorder node to create the event-driven workflow, and implement the critical GPS-map anchor persistence.
    - **Sub-Task 2.2.1:** In `recorder_node.py`, import the map generation function.
    - **Sub-Task 2.2.2:** In the service callback, after successfully stopping a recording, the node must immediately call the map generation function to regenerate the map files.
    - **Sub-Task 2.2.3:** **Implement Datum Saving:** The first time a boundary is recorded, the node must subscribe **once** to the `/odometry/gps` topic from `navsat_transform_node` to get the map's origin datum. This datum must be saved to a persistent file (e.g., `datum.yaml`).
    - **Sub-Task 2.2.4:** Commit the integrated workflow. (`git commit -m "feat(localization): Integrate map generator into recorder service"`)

- [ ] **MANDATORY TEST 2.A: Verify Automated Map Generation**
    - **Context:** Test the full recording-to-map pipeline to ensure the core deliverable of this phase is working before integrating Nav2. **This test cannot be skipped.**
    - **Procedure:**
        1.  Delete any old map or recording files.
        2.  Relaunch the simulation.
        3.  Use the service to record a `boundary.csv` and a `keepout.csv`.
        4.  After stopping the final recording, check the designated maps directory.
    - **Expected Outcome:** The `map.pgm`, `map.yaml`, and `datum.yaml` files are created automatically. Opening `map.pgm` in an image viewer should show a white area with a black keepout zone inside it, surrounded by a black lethal area.

### **Module 3: Nav2 Stack Integration**

This module integrates the full ROS 2 Navigation Stack to enable autonomous point-to-point motion.

- [ ] **Task 3.1:** **Create Navigation Package and Configuration**
    - **Dependencies:** 2.2
    - **Context:** Create the dedicated package for all Nav2-related configurations.
    - **Sub-Task 3.1.1:** In `mower_ws/src`, create a new package: `ros2 pkg create --build-type ament_cmake mower_navigation`.
    - **Sub-Task 3.1.2:** Inside `mower_navigation`, create `config/` and `launch/` directories.
    - **Sub-Task 3.1.3:** In `config/`, create `nav2_params.yaml`.

- [ ] **Task 3.2:** **Configure Nav2 Parameters**
    - **Dependencies:** 3.1
    - **Context:** Populate the `nav2_params.yaml` file with all necessary configurations for the specified planners, controllers, and costmap layers.
    - **Sub-Task 3.2.1:** Configure the `planner_server` to use `SmacPlannerHybrid`.
    - **Sub-Task 3.2.2:** Configure the `controller_server` to use `DWBController`.
    - **Sub-Task 3.2.3:** Configure the `global_costmap`. Set its plugins to use the `StaticLayer` (pointing to `map.pgm`) and the `InflationLayer`. Ensure its global frame is `map`.
    - **Sub-Task 3.2.4:** Configure the `local_costmap`. Set its plugins and ensure its global frame is `odom`.
    - **Sub-Task 3.2.5:** Configure the `behavior_server`, `bt_navigator`, and `lifecycle_manager` with default parameters.

- [ ] **Task 3.3:** **Create Navigation Launch File**
    - **Dependencies:** 3.2
    - **Context:** Create a launch file to bring up the entire Nav2 stack.
    - **Sub-Task 3.3.1:** In `mower_navigation/launch`, create `navigation.launch.py`.
    - **Sub-Task 3.3.2:** Use the `Nav2Bringup` library to include the standard Nav2 launch components, passing your custom `nav2_params.yaml` file as an argument. Ensure `use_sim_time` is set to `True`.
    - **Sub-Task 3.3.3:** **Integrate Datum Loading:** Modify the `navsat_transform_node` launch configuration (in `sim.launch.py`) to load the parameters from the saved `datum.yaml` file. This is the critical step for map persistence.
    - **Sub-Task 3.3.4:** Update the main `sim.launch.py` to include the new `navigation.launch.py`.
    - **Sub-Task 3.3.5:** Commit all Nav2 configuration and launch files. (`git commit -m "feat(navigation): Configure and launch Nav2 stack"`)

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
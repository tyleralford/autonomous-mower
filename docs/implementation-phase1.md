## **Autonomous Mower: Phase 1 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 1 of the Autonomous Mower project. It is designed to be followed sequentially by a developer. Each task builds upon the previous one, with mandatory testing checkpoints to ensure system integrity at every stage.

### **Module 0: Project Setup and Version Control**

This initial module establishes the foundational environment for all future work.

- [x] **Task 0.1:** **Initialize Git Repository**
    - **Context:** Create a centralized repository to track all code changes, manage branches, and collaborate.
    - **Sub-Task 0.1.1:** Create a new repository on GitHub named `autonomous_mower`.
    - **Sub-Task 0.1.2:** Clone the repository to your local development machine.
    - **Sub-Task 0.1.3:** Create and check out a new branch named `feature/phase-1-setup`. All work for this plan will be done on this branch.
    - **Sub-Task 0.1.4:** Create a `.gitignore` file using a standard ROS 2 template to exclude build artifacts, log files, and other generated directories.
    - **Sub-Task 0.1.5:** Create an empty `README.md` file.
    - **Sub-Task 0.1.6:** Commit and push the initial setup. (`git commit -m "Initial project setup with gitignore and README"`)

- [x] **Task 0.2:** **Set Up ROS 2 Workspace**
    - **Dependencies:** 0.1
    - **Context:** Create the ROS 2 workspace that will contain all custom software packages for the project.
    - **Sub-Task 0.2.1:** Inside the repository, create the workspace directory structure: `mower_ws/src`.
    - **Sub-Task 0.2.2:** Perform an initial build of the empty workspace using `colcon build` from the `mower_ws` directory to generate the `build`, `install`, and `log` directories.
    - **Sub-Task 0.2.3:** Add a `README.md` inside `mower_ws` explaining how to source and build the workspace.
    - **Sub-Task 0.2.4:** Commit the workspace structure. (`git commit -m "feat: Create ROS 2 workspace structure"`)

### **Module 1: Robot Description (`mower_description`)**

This module focuses on creating the robot's digital twin. We will build the robot model link-by-link, testing at each stage.

- [ ] **Task 1.1:** **Create Description Package and Base Model**
    - **Dependencies:** 0.2
    - **Context:** Create the package to hold the robot's URDF and establish the base XACRO file with the main chassis link.
    - **Sub-Task 1.1.1:** Navigate to `mower_ws/src` and create a new ROS 2 package: `ros2 pkg create --build-type ament_cmake mower_description`.
    - **Sub-Task 1.1.2:** Inside `mower_description`, create the following directory structure: `urdf/`, `meshes/`, `rviz/`.
    - **Sub-Task 1.1.3:** In the `urdf/` directory, create a main XACRO file: `mower.urdf.xacro`.
    - **Sub-Task 1.1.4:** In `mower.urdf.xacro`, define the `base_link` and the `chassis` link with its visual (as a box) and collision geometries based on the PRD. Define the fixed joint connecting them.
    - **Sub-Task 1.1.5:** Create a basic launch file `display.launch.py` in a new `launch/` directory. This launch file will start the `robot_state_publisher` and `joint_state_publisher_gui` nodes, and launch RViz with a specific config file.
    - **Sub-Task 1.1.6:** Create a basic RViz configuration file `urdf_config.rviz` in the `rviz/` directory and save it. This should display the RobotModel and TF frames.
    - **Sub-Task 1.1.7:** Commit your work. (`git commit -m "feat(description): Create package and base chassis model"`)

- [ ] **MANDATORY TEST 1.A: Visualize Base Model**
    - **Context:** Before adding complexity, ensure the base model and visualization tools are working correctly. **This test cannot be skipped.**
    - **Procedure:**
        1. Build the workspace with `colcon build`.
        2. Source the workspace: `source install/setup.bash`.
        3. Launch the display file: `ros2 launch mower_description display.launch.py`.
    - **Expected Outcome:** RViz opens and displays the `chassis` link correctly. The GUI slider window for joint states also appears. Confirm this before proceeding.

- [ ] **Task 1.2:** **Add Drive Wheels to Model**
    - **Dependencies:** 1.1
    - **Context:** Add the left and right drive wheels to the URDF model.
    - **Sub-Task 1.2.1:** In `mower.urdf.xacro`, define the `left_wheel` and `right_wheel` links. Use cylinder primitives for their visual and collision geometries based on the PRD dimensions.
    - **Sub-Task 1.2.2:** Define the `left_wheel_joint` and `right_wheel_joint`. These must be of type `continuous` and connect each wheel link to `base_link` at the positions specified in the PRD.
    - **Sub-Task 1.2.3:** Commit your work. (`git commit -m "feat(description): Add drive wheel links and joints"`)

- [ ] **MANDATORY TEST 1.B: Visualize Wheels**
    - **Context:** Verify the wheels have been added correctly. **This test cannot be skipped.**
    - **Procedure:** Relaunch the display file: `ros2 launch mower_description display.launch.py`.
    - **Expected Outcome:** The robot model now appears in RViz with both drive wheels correctly positioned. Use the `joint_state_publisher_gui` to move the wheel joint sliders and confirm they rotate as expected.

- [ ] **Task 1.3:** **Add Remaining Links to Model**
    - **Dependencies:** 1.2
    - **Context:** Complete the robot's physical structure by adding the counterweight, front roller, reel, and reel motor.
    - **Sub-Task 1.3.1:** Add the `counterweight` link and its fixed joint.
    - **Sub-Task 1.3.2:** Add the `front_roller` link and its `continuous` joint.
    - **Sub-Task 1.3.3:** Add the `reel` link and its `continuous` joint (`reel_joint`).
    - **Sub-Task 1.3.4:** Add the `reel_motor` link and its fixed joint.
    - **Sub-Task 1.3.5:** For all new links, create visual and collision geometries using simple primitives (box, cylinder) based on the PRD.
    - **Sub-Task 1.3.6:** Relaunch `display.launch.py` and verify in RViz that all links appear correctly and all non-fixed joints can be moved with the GUI sliders.
    - **Sub-Task 1.3.7:** Commit the complete robot structure. (`git commit -m "feat(description): Add all remaining structural links"`)

- [ ] **Task 1.4:** **Add Inertial Properties**
    - **Dependencies:** 1.3
    - **Context:** Add physical properties to the model for a realistic physics simulation.
    - **Sub-Task 1.4.1:** For every link in `mower.urdf.xacro`, add an `<inertial>` tag.
    - **Sub-Task 1.4.2:** Inside each tag, specify the `<mass>` using the value from the PRD.
    - **Sub-Task 1.4.3:** Calculate and specify the `<inertia>` tensor for each link. Assume simple uniform geometry (e.g., box inertia, cylinder inertia) for these calculations.
    - **Sub-Task 1.4.4:** Commit the inertial properties. (`git commit -m "feat(description): Add mass and inertia properties to all links"`)

### **Module 2: Gazebo Simulation Environment**

This module focuses on creating a virtual world and placing the robot model within it.

- [ ] **Task 2.1:** **Create Simulation and Bringup Packages**
    - **Dependencies:** 1.4
    - **Context:** Create the dedicated packages for simulation assets and top-level launch files.
    - **Sub-Task 2.1.1:** In `mower_ws/src`, create `mower_simulation` (`ament_cmake`).
    - **Sub-Task 2.1.2:** In `mower_ws/src`, create `mower_bringup` (`ament_python`).
    - **Sub-Task 2.1.3:** Inside `mower_simulation`, create `worlds/` and `launch/` directories.
    - **Sub-Task 2.1.4:** Commit the new packages. (`git commit -m "feat: Create simulation and bringup packages"`)

- [ ] **Task 2.2:** **Create Gazebo World**
    - **Dependencies:** 2.1
    - **Context:** Define the virtual environment for the robot.
    - **Sub-Task 2.2.1:** In `mower_simulation/worlds`, create a file `lawn.world`.
    - **Sub-Task 2.2.2:** In this file, define a basic world including a ground plane, a sun for lighting, and the default physics engine. Add a simple static box or wall as an obstacle.

- [ ] **Task 2.3:** **Create Simulation Launch File**
    - **Dependencies:** 2.2
    - **Context:** Create a launch file that starts Gazebo and spawns the robot model.
    - **Sub-Task 2.3.1:** In `mower_bringup/launch`, create `sim.launch.py`.
    - **Sub-Task 2.3.2:** This launch file should perform the following actions:
        - Include the `display.launch.py` from `mower_description` to start `robot_state_publisher` and RViz.
        - Launch the Gazebo server (`gzserver`) with the `lawn.world` file.
        - Launch the Gazebo client (`gzclient`).
        - Use the `spawn_entity.py` script from `gazebo_ros` to spawn the robot model (read from the `robot_description` topic) into the simulation.
    - **Sub-Task 2.3.3:** Commit the Gazebo world and launch file. (`git commit -m "feat(simulation): Create Gazebo world and bringup launch file"`)

- [ ] **MANDATORY TEST 2.A: Spawn Robot in Gazebo**
    - **Context:** This is the first full integration test. It verifies that the model is valid for simulation and the launch files are correct. **This test cannot be skipped.**
    - **Procedure:**
        1. Build and source the workspace.
        2. Launch the main simulation file: `ros2 launch mower_bringup sim.launch.py`.
    - **Expected Outcome:**
        - Gazebo and RViz windows open.
        - The robot model appears in the Gazebo world, resting on the ground plane.
        - The robot model appears correctly in RViz.
        - There are no errors in the terminal related to spawning the entity or parsing the URDF.

### **Module 3: `ros2_control` Integration**

This is the most complex module, integrating the control framework. It will be done in careful, verifiable steps.

- [ ] **Task 3.1:** **Add `ros2_control` Tags to URDF**
    - **Dependencies:** 2.3
    - **Context:** Instrument the robot's URDF to make it compatible with `ros2_control` for simulation.
    - **Sub-Task 3.1.1:** Open `mower.urdf.xacro`.
    - **Sub-Task 3.1.2:** Add a `<ros2_control>` tag for the `GazeboSystem` plugin. This tells `ros2_control` how to interface with Gazebo's physics.
    - **Sub-Task 3.1.3:** Within the tag, define the `command_interface` (`velocity`) and `state_interface` (`position` and `velocity`) for the two drive wheel joints (`left_wheel_joint`, `right_wheel_joint`).
    - **Sub-Task 3.1.4:** Define the interfaces for the `reel_joint` as well (command `velocity`, state `position` and `velocity`).
    - **Sub-Task 3.1.5:** Commit the URDF changes. (`git commit -m "feat(control): Add ros2_control tags to URDF for Gazebo"`)

- [ ] **Task 3.2:** **Create Control Package and Configuration**
    - **Dependencies:** 3.1
    - **Context:** Create the package to hold controller configurations and create the YAML file defining which controllers to use.
    - **Sub-Task 3.2.1:** In `mower_ws/src`, create `mower_control` (`ament_cmake`).
    - **Sub-Task 3.2.2:** Inside `mower_control`, create a `config/` directory.
    - **Sub-Task 3.2.3:** In `config/`, create `mower_controllers.yaml`.
    - **Sub-Task 3.2.4:** In the YAML file, define the `controller_manager`.
    - **Sub-Task 3.2.5:** Define the `joint_state_broadcaster`.
    - **Sub-Task 3.2.6:** Define the `diff_drive_controller`. Populate it with all the required parameters from the PRD (wheel names, separation, radius).
    - **Sub-Task 3.2.7:** Define the `joint_trajectory_controller` for the reel, specifying the `reel_joint` and `velocity` interface.
    - **Sub-Task 3.2.8:** Commit the controller package and configuration file. (`git commit -m "feat(control): Create controller config file"`)

- [ ] **Task 3.3:** **Integrate Controllers into Launch File**
    - **Dependencies:** 3.2
    - **Context:** Update the main launch file to load and start the `ros2_control` nodes and the configured controllers.
    - **Sub-Task 3.3.1:** Edit `sim.launch.py` in `mower_bringup`.
    - **Sub-Task 3.3.2:** Add a node to load the `mower_controllers.yaml` file.
    - **Sub-Task 3.3.3:** Add nodes to launch the `controller_manager` (`ros2_control_node`).
    - **Sub-Task 3.3.4:** Add nodes to load and start each controller defined in the YAML file (`spawner` nodes for `joint_state_broadcaster`, `diff_drive_controller`, etc.).
    - **Sub-Task 3.3.5:** **Crucially,** remove the `joint_state_publisher_gui` from the launch process. Its function is now replaced by the `joint_state_broadcaster` which gets state information directly from the simulation.
    - **Sub-Task 3.3.6:** Commit the launch file updates. (`git commit -m "feat(bringup): Integrate ros2_control nodes into main launch"`)

- [ ] **MANDATORY TEST 3.A: Verify Controller Loading**
    - **Context:** Ensure the entire `ros2_control` pipeline is correctly configured and communicating before attempting to move the robot. **This test cannot be skipped.**
    - **Procedure:**
        1. Build and source the workspace.
        2. Launch the main simulation file: `ros2 launch mower_bringup sim.launch.py`.
        3. In a new terminal, check the active topics: `ros2 topic list`.
        4. Echo the `/joint_states` topic: `ros2 topic echo /joint_states`.
    - **Expected Outcome:**
        - The simulation launches without errors.
        - The terminal shows that all controllers have been successfully loaded and started.
        - `ros2 topic list` shows `/joint_states` and the command topics for the controllers (e.g., `/diff_drive_controller/cmd_vel_unstamped`).
        - The `/joint_states` topic should be publishing the positions and velocities of all joints. Manually move the robot in Gazebo by dragging it, and confirm the values on the topic change.

### **Module 4: Teleoperation and Final Validation**

This final module achieves the primary goal of manual control and completes the PRD requirements.

- [ ] **Task 4.1:** **Add Teleoperation Launch**
    - **Dependencies:** 3.3
    - **Context:** Add a keyboard teleoperation node to the system to provide manual control inputs.
    - **Sub-Task 4.1.1:** In `mower_bringup/launch`, create a new launch file `teleop.launch.py`.
    - **Sub-Task 4.1.2:** This file should launch the `teleop_twist_keyboard` node.
    - **Sub-Task 4.1.3:** Use a remapping in the launch file to ensure the output topic of the teleop node (`/cmd_vel`) is correctly mapped to the input of the `diff_drive_controller` (`/diff_drive_controller/cmd_vel_unstamped`).
    - **Sub-Task 4.1.4:** Commit the teleop launch file. (`git commit -m "feat(bringup): Add keyboard teleop launch file"`)

- [ ] **MANDATORY TEST 4.A: Full System End-to-End Validation**
    - **Context:** This is the final acceptance test for Phase 1. It validates all requirements specified in the PRD. **This test cannot be skipped.**
    - **Procedure:**
        1. Build and source the workspace.
        2. In one terminal, launch the main simulation: `ros2 launch mower_bringup sim.launch.py`.
        3. In a second terminal, launch the teleop node: `ros2 launch mower_bringup teleop.launch.py`.
        4. With the teleop terminal active, use the keyboard keys (i, j, k, l, etc.) to drive the robot in Gazebo.
        5. In a third terminal, manually publish a velocity command to the reel controller to verify it spins. Example command: `ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["reel_joint"], points: [{positions: [0], velocities: [1.0], time_from_start: {sec: 1}}]}'`
    - **Expected Outcome:**
        - **Success Metric 1:** The robot model in Gazebo moves smoothly in response to keyboard commands (forward, backward, turning).
        - **Success Metric 2:** The robot model in RViz mirrors the movement in Gazebo.
        - **Success Metric 3:** The odometry trail in RViz (`/odom` topic) correctly visualizes the robot's path as you drive it.
        - **Success Metric 4:** Publishing the command to the reel controller causes the reel link to spin in both Gazebo and RViz.
        - **Success Metric 5:** The entire system is stable and runs without errors.

- [ ] **Task 4.2:** **Finalize and Merge**
    - **Dependencies:** 4.1
    - **Context:** Clean up the feature branch and merge it into the main branch, completing the phase.
    - **Sub-Task 4.2.1:** Review all code for clarity and comments.
    - **Sub-Task 4.2.2:** Update the main `README.md` file with instructions on how to launch the simulation and teleop nodes.
    - **Sub-Task 4.2.3:** Create a Pull Request on GitHub from `feature/phase-1-setup` to `main`.
    - **Sub-Task 4.2.4:** After review, merge the pull request. Phase 1 is now complete.
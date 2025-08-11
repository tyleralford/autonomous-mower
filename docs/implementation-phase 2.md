## **Autonomous Mower: Phase 2 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 2 of the Autonomous Mower project. It is designed to be followed sequentially by a developer. Each task builds upon the previous one, with mandatory testing checkpoints to ensure system integrity at every stage.

### **Module 0: Project Setup**

This initial module prepares the codebase for Phase 2 development.

- [x] **Task 0.1:** **Create and Checkout a New Branch**
    - **Context:** Isolate all Phase 2 development on a dedicated branch to maintain a clean `main` branch.
    - [x] **Sub-Task 0.1.1:** Pull the latest changes from the `main` branch.
    - [x] **Sub-Task 0.1.2:** Create and check out a new branch named `feature/phase-2-sensors-ekf`.
    - [x] **Sub-Task 0.1.3:** Push the new branch to the remote repository.

### **Module 1: Sensor Integration into the URDF Model**

This module focuses on adding the required sensors to the robot's description and verifying they produce data in the simulation.

- [x] **Task 1.1:** **Add Simulated IMU**
    - **Dependencies:** 0.1
    - **Context:** Add an Inertial Measurement Unit to the robot model to provide acceleration and angular velocity data.
    - [x] **Sub-Task 1.1.1:** In `mower_description/urdf/components/`, create a new file `sensors.xacro`.
    - [x] **Sub-Task 1.1.2:** In `sensors.xacro`, define a new link `imu_link` and a fixed joint to attach it to the `chassis` link.
    - [x] **Sub-Task 1.1.3:** In the same file, add the `gazebo_ros_imu_sensor` plugin. Configure it to publish to the `/imu/data_raw` topic and set its `frame_id` to `imu_link`.
    - [x] **Sub-Task 1.1.4:** Configure the plugin with basic Gaussian noise for all sensor axes. Enable the debug visualization flag.
    - [x] **Sub-Task 1.1.5:** Update `mower.urdf.xacro` to include the new `sensors.xacro` file.
    - [x] **Sub-Task 1.1.6:** Commit your work. (`git commit -m "feat(description): Add IMU sensor to model"`)

- [x] **MANDATORY TEST 1.A: Verify IMU Data Publication** (✅ COMPLETED - IMU sensor integrated and publishing at 70Hz with noise)
    - **Context:** Before proceeding, ensure the IMU is correctly integrated and publishing data. **This test cannot be skipped.**
    - **Procedure:**
        1. Build and source the workspace.
        2. Launch the main simulation: `ros2 launch mower_bringup sim.launch.py`.
        3. In a new terminal, check the topic list for `/imu/data_raw`.
        4. Echo the topic: `ros2 topic echo /imu/data_raw`.
    - **Expected Outcome:** The topic exists and publishes `sensor_msgs/Imu` messages at a steady rate. The data should change when you manually move or rotate the robot in the Gazebo interface.

- [x] **Task 1.2:** **Add Simulated Dual GPS** (✅ COMPLETED)
    - **Dependencies:** 1.1
    - **Context:** Add two GPS sensors to the model. These will provide raw location data and be used to calculate an absolute heading.
    - [x] **Sub-Task 1.2.1:** In `sensors.xacro`, define two new links: `gps_left_link` and `gps_right_link`.
    - [x] **Sub-Task 1.2.2:** Attach them with fixed joints to the `chassis` link. Place them symmetrically along the Y-axis with a **43cm baseline** (e.g., at `y=+0.215` and `y=-0.215` at an appropriate X/Z offset).
    - [x] **Sub-Task 1.2.3:** Add two instances of the NavSat sensor using native Gazebo Harmonic format.
    - [x] **Sub-Task 1.2.4:** Configure the first plugin to publish to `/gps/left/fix` with `frame_id: gps_left_link`.
    - [x] **Sub-Task 1.2.5:** Configure the second plugin to publish to `/gps/right/fix` with `frame_id: gps_right_link`.
    - [x] **Sub-Task 1.2.6:** Configure both plugins with basic Gaussian noise and enable their debug visualization.
    - [x] **Sub-Task 1.2.7:** Commit your work. (`git commit -m "Add dual GPS sensor configuration for Task 1.2"`)

- [x] **MANDATORY TEST 1.B: Verify GPS Data Publication** (✅ COMPLETED - Dual GPS sensors publishing at 20Hz)
    - **Context:** Ensure both GPS sensors are publishing data independently. **This test cannot be skipped.**
    - **Procedure:**
        1. Relaunch the simulation: `ros2 launch mower_bringup sim.launch.py`.
        2. Check for the `/gps/left/fix` and `/gps/right/fix` topics.
        3. Echo both topics to confirm they are publishing `sensor_msgs/NavSatFix` messages.
    - **Expected Outcome:** Both topics exist and are publishing valid latitude/longitude data.

### **Module 2: Custom GPS Heading Calculation Node**

This module focuses on creating the new ROS 2 node that provides the absolute heading measurement required by the global EKF.

- [x] **Task 2.1:** **Create Localization Package and Node Skeleton** (✅ COMPLETED)
    - **Dependencies:** 1.2
    - **Context:** Create the package that will hold all state estimation configurations and the new custom node.
    - [x] **Sub-Task 2.1.1:** In `mower_ws/src`, create a new ROS 2 package: `ros2 pkg create --build-type ament_python mower_localization`.
    - [x] **Sub-Task 2.1.2:** Inside `mower_localization`, create a node file named `gps_heading_node.py`.
    - [x] **Sub-Task 2.1.3:** In the node, create the basic structure: two subscribers for the left and right GPS topics, and one publisher for the `/gps/heading` topic.
    - [x] **Sub-Task 2.1.4:** Commit the new package and node skeleton. (`git commit -m "Create package and gps_heading_node skeleton. Complete Task 2.1 Sub-tasks 2.1.1-2.1.4"`)

- [x] **Task 2.2:** **Implement Heading Calculation Logic** (✅ COMPLETED)
    - **Dependencies:** 2.1
    - **Context:** Implement the core logic of the node to calculate and publish the heading.
    - [x] **Sub-Task 2.2.1:** In the GPS callback, store the latest messages from both sensors.
    - [x] **Sub-Task 2.2.2:** In a timer callback (e.g., at 20 Hz), calculate the vector between the two GPS positions. Use `math.atan2` to get the heading angle in radians.
    - [x] **Sub-Task 2.2.3:** Add **1.5707963 radians (90 degrees)** to the calculated angle to perform the mandatory yaw offset correction.
    - [x] **Sub-Task 2.2.4:** Convert the final angle into a quaternion.
    - [x] **Sub-Task 2.2.5:** Populate a `sensor_msgs/Imu` message with the calculated orientation and a fixed, reasonable orientation covariance.
    - [x] **Sub-Task 2.2.6:** Publish the message.
    - [x] **Sub-Task 2.2.7:** Commit the implemented logic. (`git commit -m "Implement GPS heading calculation logic"`)

- [x] **Task 2.3:** **Integrate Heading Node into Launch** (✅ COMPLETED)
    - **Dependencies:** 2.2
    - **Context:** Add the new node to the main simulation launch file so it runs as part of the system.
    - [x] **Sub-Task 2.3.1:** Edit `mower_bringup/launch/sim.launch.py`.
    - [x] **Sub-Task 2.3.2:** Add a `Node` action to execute the `gps_heading_node.py`.
    - [x] **Sub-Task 2.3.3:** Commit the launch file update. (`git commit -m "Add gps_heading_node to main launch"`)

- [x] **MANDATORY TEST 2.A: Verify Heading Publication** (✅ COMPLETED)
    - **Context:** Test the custom node's logic in isolation before feeding its data to the EKF. **This test cannot be skipped.**
    - **Procedure:**
        1. Build and source the workspace.
        2. Relaunch the simulation: `ros2 launch mower_bringup sim.launch.py`.
        3. Echo the `/gps/heading` topic.
        4. In Gazebo, rotate the robot model on the spot.
    - **Expected Outcome:** The `/gps/heading` topic publishes `sensor_msgs/Imu` messages. The `orientation` quaternion in the message should change in correspondence with the robot's rotation in Gazebo.

### **Module 3: State Estimation Pipeline (`robot_localization`)**

This module implements the core dual-EKF setup.

- [ ] **Task 3.1:** **Create EKF Configuration Files**
    - **Dependencies:** 2.3
    - **Context:** Create the YAML files that will configure the two EKF nodes.
    - [ ] **Sub-Task 3.1.1:** In `mower_localization`, create a `config/` directory.
    - [ ] **Sub-Task 3.1.2:** Inside `config/`, create three files: `ekf_local.yaml`, `ekf_global.yaml`, and `navsat_transform.yaml`.
    - [ ] **Sub-Task 3.1.3:** Commit the empty config files. (`git commit -m "feat(localization): Add EKF config files"`)

- [ ] **Task 3.2:** **Configure and Integrate Local EKF**
    - **Dependencies:** 3.1
    - **Context:** Set up the first EKF to provide a smooth, continuous odometry based on relative sensors.
    - [ ] **Sub-Task 3.2.1:** Edit `ekf_local.yaml`. Configure the `frequency`, `world_frame` (`odom`), and the inputs for wheel odometry (`odom0`) and the IMU (`imu0`), including their respective `_config` matrices for fusing velocity and angular rate.
    - [ ] **Sub-Task 3.2.2:** Edit `sim.launch.py` to launch the `ekf_node` from `robot_localization`, giving it a unique name (`local_ekf_node`) and passing the `ekf_local.yaml` configuration to it.
    - [ ] **Sub-Task 3.2.3:** **Crucially,** edit the `diff_drive_controller` parameters in `mower_controllers.yaml` and set `enable_odom_tf: false`. The EKF will now be responsible for this transform.
    - [ ] **Sub-Task 3.2.4:** Commit the local EKF configuration. (`git commit -m "feat(localization): Configure and launch local EKF"`)

- [ ] **MANDATORY TEST 3.A: Verify Local EKF Operation**
    - **Context:** Ensure the local EKF is correctly fusing wheel odometry and IMU data. **This test cannot be skipped.**
    - **Procedure:**
        1. Relaunch the simulation.
        2. Use `ros2 topic echo /odometry/filtered/local` to see the output.
        3. In RViz, display the TF tree and the odometry output.
        4. Drive the robot using teleop.
    - **Expected Outcome:** The `/odometry/filtered/local` topic is active. The `odom` -> `base_link` transform is now being published by the EKF. The pose estimate in RViz should appear smooth.

- [ ] **Task 3.3:** **Configure and Integrate `navsat_transform_node`**
    - **Dependencies:** 3.2
    - **Context:** Set up the node that converts GPS lat/lon to the Cartesian `map` frame.
    - [ ] **Sub-Task 3.3.1:** Edit `navsat_transform.yaml`. Configure the topic subscriptions and, most importantly, set `yaw_offset: 1.5707963`.
    - [ ] **Sub-Task 3.3.2:** Edit `sim.launch.py` to launch the `navsat_transform_node` from `robot_localization` with its configuration file.
    - [ ] **Sub-Task 3.3.3:** Commit the configuration. (`git commit -m "feat(localization): Configure and launch navsat_transform_node"`)

- [ ] **Task 3.4:** **Configure and Integrate Global EKF**
    - **Dependencies:** 3.3
    - **Context:** Set up the second EKF to provide a globally accurate, drift-corrected pose.
    - [ ] **Sub-Task 3.4.1:** Edit `ekf_global.yaml`. Configure the `world_frame` (`map`).
    - [ ] **Sub-Task 3.4.2:** Configure the inputs to fuse the output of the local EKF (`odom0`), the output of `navsat_transform_node` (`odom1`), and the output of the custom heading node (`imu0`). Set the `_config` matrices to fuse only the appropriate values from each (e.g., only XY from `odom1`, only Yaw from `imu0`).
    - [ ] **Sub-Task 3.4.3:** Edit `sim.launch.py` to launch a second `ekf_node` with a different name (`global_ekf_node`) and the `ekf_global.yaml` configuration.
    - [ ] **Sub-Task 3.4.4:** Commit the global EKF configuration. (`git commit -m "feat(localization): Configure and launch global EKF"`)

- [ ] **MANDATORY TEST 3.B: Verify Full TF Tree**
    - **Context:** Statically verify that all nodes are running and the complete transformation chain is present. **This test cannot be skipped.**
    - **Procedure:**
        1. Relaunch the simulation.
        2. Check that all nodes are active.
        3. In a new terminal, run `ros2 run tf2_tools view_frames.py`.
    - **Expected Outcome:** The generated PDF shows a complete and correct TF tree: `map` -> `odom` -> `base_link`. There should be no disconnected branches.

### **Module 4: Final Validation**

This module performs the final acceptance test as defined in the PRD.

- [ ] **Task 4.1:** **Perform Loop-Closure Validation Test**
    - **Dependencies:** 3.4
    - **Context:** This is the final end-to-end test for Phase 2, validating the entire state estimation pipeline's performance.
    - [ ] **Sub-Task 4.1.1:** In RViz, create a clean configuration to visualize the required elements for the test. Add two `Path` displays.
    - [ ] **Sub-Task 4.1.2:** Configure the first Path to display `/odometry/filtered/local` in the `odom` fixed frame.
    - [ ] **Sub-Task 4.1.3:** Configure the second Path to display `/odometry/filtered/global` in the `map` fixed frame.
    - [ ] **Sub-Task 4.1.4:** Save this configuration to `mower_localization/rviz/ekf_test.rviz`.
    - [ ] **Sub-Task 4.1.5:** Launch the simulation and RViz.
    - [ ] **Sub-Task 4.1.6:** Perform the loop-closure driving test as described in the PRD's success metrics. Record screenshots or a short video for documentation.
    - [ ] **Sub-Task 4.1.7:** Confirm that the global path shows minimal drift while the local path shows significant drift.

- [ ] **Task 4.2:** **Finalize and Merge**
    - **Dependencies:** 4.1
    - **Context:** Clean up the feature branch and merge it into the main branch, completing the phase.
    - [ ] **Sub-Task 4.2.1:** Review all new code for clarity and comments.
    - [ ] **Sub-Task 4.2.2:** Update the `README.md` file with instructions on how to view the new sensor data and EKF outputs.
    - [ ] **Sub-Task 4.2.3:** Create a Pull Request on GitHub from `feature/phase-2-sensors-ekf` to `main`, including the validation screenshots in the description.
    - [ ] **Sub-Task 4.2.4:** After review, merge the pull request. Phase 2 is now complete.
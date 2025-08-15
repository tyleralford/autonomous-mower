## **Autonomous Mower: Phase 2 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 2 of the Autonomous Mower project. It is designed to be followed sequentially by a developer. Each task builds upon the previous one, with mandatory testing checkpoints to ensure system integrity at every stage.

## Status Update — 2025-08-15

- Simulation launches cleanly with bridges for Clock, IMU, and GPS. Model pose bridging removed.
- Ground Truth Heading Node updated to consume IMU (/imu) and republish noisy heading on /gps/heading at 20 Hz.
- navsat_transform configured with yaw_offset: 0.0 and use_odometry_yaw: true; /odometry/gps updating.
- EKF nodes (local and global) are running and fusing inputs; Module 4 loop-closure test completed.

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
    - [x] **Sub-Task 1.2.1:** In `sensors.xacro`, define a new link: `gps_link`.
    - [x] **Sub-Task 1.2.2:** Attach it with a fixed joint to the `chassis` link.
    - [x] **Sub-Task 1.2.3:** Add an instance of the NavSat sensor using native Gazebo Harmonic format.
    - [x] **Sub-Task 1.2.4:** Configure the plugin to publish to `/gps/fix` with `frame_id: gps_link`.
    - [x] **Sub-Task 1.2.5:** Configure the plugin with basic Gaussian noise and enable its debug visualization.
    - [x] **Sub-Task 1.2.6:** Commit your work. (`git commit -m "Add dual GPS sensor configuration for Task 1.2"`)

- [x] **MANDATORY TEST 1.B: Verify GPS Data Publication** (✅ COMPLETED - Dual GPS sensors publishing at 20Hz)
    - **Context:** Ensure the GPS sensor is publishing data. **This test cannot be skipped.**
    - **Procedure:**
        1. Relaunch the simulation: `ros2 launch mower_bringup sim.launch.py`.
        2. Check for the `/gps/fix` topic.
        3. Echo the topic to confirm it is publishing `sensor_msgs/NavSatFix` messages.
    - **Expected Outcome:** The topic exists and is publishing valid latitude/longitude data.

### **Module 2: Simulated GPS Heading Republisher**

This module focuses on creating the new ROS 2 node that provides the absolute heading measurement required by the global EKF.

- [x] **Task 2.1:** **Create Localization Package and Node Skeleton** (✅ COMPLETED)
    - **Dependencies:** 1.2
    - **Context:** Create the package that will hold all state estimation configurations and the new custom node.
    - [x] **Sub-Task 2.1.1:** In `mower_ws/src`, create a new ROS 2 package: `ros2 pkg create --build-type ament_python mower_localization`.
    - [x] **Sub-Task 2.1.2:** Inside `mower_localization`, create a node file named `gps_heading_node.py`.
    - [x] **Sub-Task 2.1.3:** In the node, create the basic structure: two subscribers for the left and right GPS topics, and one publisher for the `/gps/heading` topic.
    - [x] **Sub-Task 2.1.4:** Commit the new package and node skeleton. (`git commit -m "Create package and gps_heading_node skeleton. Complete Task 2.1 Sub-tasks 2.1.1-2.1.4"`)

- [x] **Task 2.2:** **Implement Heading Republisher Logic** (✅ COMPLETED)
    - **Dependencies:** 2.1
    - **Context:** Implement the core logic of the node to calculate and publish the heading.
    - [x] **Sub-Task 2.2.1:** Subscribe to `/imu` and cache the latest orientation.
    - [x] **Sub-Task 2.2.2:** In a 20 Hz timer, extract yaw, add small Gaussian noise, and create a quaternion.
    - [x] **Sub-Task 2.2.3:** Publish a `sensor_msgs/Imu` on `/gps/heading` with orientation and covariance.
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

- [x] **Task 3.1:** **Create EKF Configuration Files** (✅ COMPLETED)
    - **Dependencies:** 2.3
    - **Context:** Create the YAML files that will configure the two EKF nodes.
    - [x] **Sub-Task 3.1.1:** In `mower_localization`, create a `config/` directory.
    - [x] **Sub-Task 3.1.2:** Inside `config/`, create three files: `ekf_local.yaml`, `ekf_global.yaml`, and `navsat_transform.yaml`.
    - [x] **Sub-Task 3.1.3:** Commit the empty config files. (`git commit -m "Add EKF config files"`)

- [x] **Task 3.2:** **Configure and Integrate Local EKF**
    - **Dependencies:** 3.1
    - **Context:** Set up the first EKF to provide a smooth, continuous odometry based on relative sensors.
    - [x] **Sub-Task 3.2.1:** Edit `ekf_local.yaml`. Configure the `frequency`, `world_frame` (`odom`), and the inputs for wheel odometry (`odom0`) and the IMU (`imu0`), including their respective `_config` matrices for fusing velocity and angular rate.
    - [x] **Sub-Task 3.2.2:** Edit `sim.launch.py` to launch the `ekf_node` from `robot_localization`, giving it a unique name (`local_ekf_node`) and passing the `ekf_local.yaml` configuration to it.
    - [x] **Sub-Task 3.2.3:** **Crucially,** edit the `diff_drive_controller` parameters in `mower_controllers.yaml` and set `enable_odom_tf: false`. The EKF will now be responsible for this transform.
    - [x] **Sub-Task 3.2.4:** Commit the local EKF configuration. (`git commit -m "feat(localization): Configure and launch local EKF"`)

- [x] **MANDATORY TEST 3.A: Verify Local EKF Operation** (✅ COMPLETED)
    - **Context:** Ensure the local EKF is correctly fusing wheel odometry and IMU data. **This test cannot be skipped.**
    - **Procedure:**
        1. Relaunch the simulation.
        2. Use `ros2 topic echo /odometry/filtered/local` to see the output.
        3. In RViz, display the TF tree and the odometry output.
        4. Drive the robot using teleop.
    - **Outcome:** `/odometry/filtered/local` is publishing; `odom -> base_link` TF provided by EKF.

- [x] **Task 3.3:** **Configure and Integrate `navsat_transform_node`**
    - **Dependencies:** 3.2
    - **Context:** Set up the node that converts GPS lat/lon to the Cartesian `map` frame.
    - [x] **Sub-Task 3.3.1:** Edit `navsat_transform.yaml`. Configure the topic subscriptions and set `yaw_offset: 0.0`, `use_odometry_yaw: true`.
    - [x] **Sub-Task 3.3.2:** Edit `sim.launch.py` to launch the `navsat_transform_node` from `robot_localization` with its configuration file.
    - [x] **Sub-Task 3.3.3:** Commit the configuration. (`git commit -m "feat(localization): Configure and launch navsat_transform_node"`)

- [x] **Task 3.4:** **Configure and Integrate Global EKF**
    - **Dependencies:** 3.3
    - **Context:** Set up the second EKF to provide a globally accurate, drift-corrected pose.
    - [x] **Sub-Task 3.4.1:** Edit `ekf_global.yaml`. Configure the `world_frame` (`map`).
    - [x] **Sub-Task 3.4.2:** Configure the inputs to fuse the output of the local EKF (`odom0`), the output of `navsat_transform_node` (`odom1`), and the output of the custom heading node (`imu0`). Set the `_config` matrices to fuse only the appropriate values from each (e.g., only XY from `odom1`, only Yaw from `imu0`).
    - [x] **Sub-Task 3.4.3:** Edit `sim.launch.py` to launch a second `ekf_node` with a different name (`global_ekf_node`) and the `ekf_global.yaml` configuration.
    - [x] **Sub-Task 3.4.4:** Commit the global EKF configuration. (`git commit -m "feat(localization): Configure and launch global EKF"`)

- [x] **MANDATORY TEST 3.B: Verify Full TF Tree** (✅ COMPLETED)
    - **Context:** Statically verify that all nodes are running and the complete transformation chain is present. **This test cannot be skipped.**
    - **Procedure:**
        1. Relaunch the simulation.
        2. Check that all nodes are active.
        3. In a new terminal, run `ros2 run tf2_tools view_frames.py`.
    - **Outcome:** TF chain present: `map` -> `odom` -> `base_link`. Global and local odometry topics active.

### **Module 4: Final Validation**

This module performs the final acceptance test as defined in the PRD.

- [x] **Task 4.1:** **Perform Loop-Closure Validation Test** (✅ COMPLETED)
    - **Dependencies:** 3.4
    - **Context:** This is the final end-to-end test for Phase 2, validating the entire state estimation pipeline's performance.
    - [x] **Sub-Task 4.1.1:** In RViz, create a clean configuration to visualize the required elements for the test. Add two `Path` displays.
    - [x] **Sub-Task 4.1.2:** Configure the first Path to display `/odometry/filtered/local` in the `odom` fixed frame.
    - [x] **Sub-Task 4.1.3:** Configure the second Path to display `/odometry/filtered/global` in the `map` fixed frame.
    - [x] **Sub-Task 4.1.4:** Save this configuration to `mower_localization/rviz/ekf_test.rviz`.
    - [x] **Sub-Task 4.1.5:** Launch the simulation and RViz.
    - [x] **Sub-Task 4.1.6:** Perform the loop-closure driving test and record artifacts.
    - [x] **Sub-Task 4.1.7:** Confirm results: global path minimal drift; local path visible drift.

- [ ] **Task 4.2:** **Finalize and Merge**
    - **Dependencies:** 4.1
    - **Context:** Clean up the feature branch and merge it into the main branch, completing the phase.
    - [x] **Sub-Task 4.2.1:** Review all new code for clarity and comments.
    - [x] **Sub-Task 4.2.2:** Update the `README.md` file with instructions on how to view the new sensor data and EKF outputs.
    - [ ] **Sub-Task 4.2.3:** Create a Pull Request on GitHub from `feature/phase-2-sensors-ekf` to `main`, including the validation screenshots in the description.
    - [ ] **Sub-Task 4.2.4:** After review, merge the pull request. Phase 2 is now complete.

### **Module 5: Refine Heading Calculation for Simulation**

This module replaces the dual-GPS heading calculation with a more stable simulation-only method that derives heading from the simulator's ground truth pose.

- [x] **Task 5.1:** **Create Ground Truth Heading Node**
    - **Context:** Create a new node that subscribes to the ground truth pose from Gazebo, adds noise, and publishes it as a standard sensor message for the EKF.
    - [x] **Sub-Task 5.1.1:** In `mower_localization/mower_localization/`, create a new file `ground_truth_heading_node.py`.
    - [x] **Sub-Task 5.1.2:** The node subscribes to `/imu` for orientation and repackages yaw as a heading.
    - [x] **Sub-Task 5.1.3:** In the timer callback, extract yaw, add noise, and publish as `/gps/heading`.
    - [x] **Sub-Task 5.1.4:** Apply a small amount of Gaussian noise to the quaternion to simulate a high-quality GPS-based heading sensor.
    - [x] **Sub-Task 5.1.5:** Publish the resulting orientation in a `sensor_msgs/Imu` message to the existing `/gps/heading` topic. Ensure a realistic covariance is set.

- [x] **Task 5.2:** **Update System Integration**
    - **Context:** Modify the launch files and robot description to remove the now-redundant components and integrate the new heading node.
    - [x] **Sub-Task 5.2.1:** In `sensors.xacro`, ensure a single `gps_link` and one NavSat sensor exist.
    - [x] **Sub-Task 5.2.2:** In `sim.launch.py`, ensure only one consolidated `gz_bridge` instance is used.
    - [x] **Sub-Task 5.2.3:** In `sim.launch.py`, use `ground_truth_heading_node` and remove any model pose dependencies.
    - [x] **Sub-Task 5.2.4:** In `navsat_transform.yaml`, ensure the input GPS topic is `/gps/fix`.

- [x] **MANDATORY TEST 5.A: Verify New Heading Publication**
    - **Context:** Ensure the new node is correctly publishing a stable, noisy heading based on ground truth.
    - **Procedure:**
        1. Build and source the workspace.
        2. Relaunch the simulation: `ros2 launch mower_bringup sim.launch.py`.
        3. Echo the `/gps/heading` topic.
        4. In Gazebo, rotate the robot model on the spot.
    - **Expected Outcome:** The `/gps/heading` topic publishes `sensor_msgs/Imu` messages. The orientation should closely track the robot's rotation in Gazebo but with slight variations due to the added noise.

## Notes
The current simulation configuration uses IMU orientation to emulate a GPS heading sensor at 20 Hz on `/gps/heading`. This is a simulation-only substitution for future hardware. The model pose bridge has been removed to simplify the pipeline and avoid reliance on simulator-only topics.

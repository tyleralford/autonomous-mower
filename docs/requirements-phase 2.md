# **Product Requirements Document: Autonomous Mower - Phase 2**

## **1. Introduction / Overview**

This document provides a complete set of requirements for **Phase 2: Sensor Integration and State Estimation** of the Autonomous Mower Development Plan. Building upon the validated simulation platform from Phase 1, the objective of this phase is to integrate simulated sensors (IMU, GPS) into the robot model and to establish a robust, dual-Extended Kalman Filter (EKF) system.

This deliverable is critical for providing the accurate and reliable pose (position and orientation) information that is the foundation for all subsequent autonomous navigation and mapping capabilities. This PRD is a self-contained specification for a developer to implement this phase.

## **2. Goals / Objectives**

The principal objective of Phase 2 is to **develop and validate a robust state estimation pipeline that fuses multiple sensor sources to provide two distinct, reliable pose estimates: a smooth, continuous local estimate and a globally accurate, non-drifting global estimate.** The final output will be a system capable of accurately tracking the robot's pose within a world-fixed coordinate system, ready for the introduction of mapping and navigation algorithms in Phase 3.

## **3. Target Audience / User Personas**

*   **User:** Internal Robotics Developer
*   **Role & Goal:** The developer is the primary user of the Phase 2 deliverable. Their goal is to build and validate a robust sensor fusion pipeline that will be foundational for all future autonomy work. They need to ensure the system is correctly configured to handle realistic sensor data and produce stable, accurate outputs.

## **4. User Stories / Use Cases**

*   "As a developer, I want to add simulated IMU and GPS sensors to the robot's URDF model so I can generate realistic sensor data for testing."
*   "As a developer, I want to configure and launch a dual-EKF system to produce two distinct pose estimates: one for smooth local movement (`odom` frame) and one for global accuracy (`map` frame)."
*   "As a developer, I want to visualize the sensor data and the output of both EKFs in Gazebo and RViz to confirm they are functioning correctly and correcting for drift over time."

## **5. Functional Requirements**

### **FR1: Sensor Integration (URDF)**
*   The `mower.urdf.xacro` model **must** be updated to include the following simulated sensors:
    *   **IMU:** An IMU link and the `GazeboRosImuSensor` plugin **must** be added.
    *   **Dual GPS:** Two separate GPS links and `GazeboRosGpsSensor` plugins **must** be added. The two sensors **must** be placed with a fixed, known baseline of **43cm** between them.
*   All sensor plugins **must** be configured with basic Gaussian noise to simulate real-world imperfections.
*   All sensor plugins **must** be configured to enable their debug visualization flags so that their status can be observed within the Gazebo simulation environment.

### **FR2: Custom GPS Heading Node**
*   A new ROS 2 Python or C++ node **must** be created to provide an absolute heading measurement.
*   This node **must** subscribe to the `sensor_msgs/NavSatFix` topics from both simulated GPS sensors.
*   The node **must** calculate the heading vector between the two GPS positions.
*   The node **must** apply a yaw offset of **+90 degrees (1.5707963 radians)** to the calculated heading to conform with the REP-103 standard (aligning a North-up heading to a ROS-standard East-up heading).
*   The node **must** publish the final, corrected orientation as a `sensor_msgs/Imu` message to the `/gps/heading` topic. Only the `orientation` and `orientation_covariance` fields of the message need to be populated.

### **FR3: Dual-EKF Configuration (`robot_localization`)**
*   Two instances of the `robot_localization` `ekf_node` **must** be configured and launched.
*   **Local EKF (`ekf_local.yaml`):**
    *   **Purpose:** Provide a smooth, continuous, locally accurate pose.
    *   **World Frame:** `odom`.
    *   **Fused Inputs:** The node **must** be configured to fuse only continuous, relative sensor data:
        1.  Wheel odometry from the `diff_drive_controller` (`nav_msgs/Odometry`).
        2.  Angular velocity and linear acceleration from the IMU (`sensor_msgs/Imu`).
    *   **Output:** Publishes the `odom` -> `base_link` transform.
*   **Global EKF (`ekf_global.yaml`):**
    *   **Purpose:** Provide a globally accurate, drift-free pose.
    *   **World Frame:** `map`.
    *   **Fused Inputs:** The node **must** be configured to fuse the following inputs:
        1.  The full odometry output from the Local EKF.
        2.  The absolute XY position from the `navsat_transform_node` (`nav_msgs/Odometry`).
        3.  The absolute orientation from the custom GPS heading node (`/gps/heading`).
    *   **Output:** Publishes the `map` -> `odom` transform.

### **FR4: `navsat_transform_node` Configuration**
*   The `navsat_transform_node` from the `robot_localization` package **must** be configured and launched.
*   The node **must** be configured with a `yaw_offset` of **1.5707963 radians (90 degrees)** to correctly align the GPS datum with the ROS standard coordinate frames.

## **6. Non-Functional Requirements**

### **NFR1: Coordinate Frame Standards**
*   The entire system **must** adhere strictly to the coordinate frame conventions defined in **REP-105**, establishing the `map` -> `odom` -> `base_link` transformation hierarchy.

### **NFR2: Performance & Publish Rates**
*   The system **must** be configured with the following sensor and node publish rates:
    *   **IMU Plugin:** 70 Hz
    *   **GPS Plugins:** 20 Hz
    *   **EKF Nodes:** ~50 Hz (e.g., 50.0)

## **7. Design Considerations / Mockups**

This phase is purely for backend and simulation development. No graphical user interface is required. The primary visual interfaces for validation will be the Gazebo simulator (for sensor debug visuals) and RViz (for visualizing TF frames and odometry paths).

## **8. Success Metrics**

The successful completion of Phase 2 will be verified by a formal validation test.

*   **Procedure:**
    1.  Launch the complete simulation stack including all sensors and EKF nodes.
    2.  In RViz, display the TF tree and two `rviz_plugins/Path` visualizations: one for the local EKF output (`/odometry/filtered/local`, fixed frame `odom`) and one for the global EKF output (`/odometry/filtered/global`, fixed frame `map`).
    3.  Using keyboard teleoperation, drive the robot in a large, closed-loop pattern (e.g., a large rectangle or figure-eight), returning to the exact starting location in the Gazebo world.
*   **Expected Outcomes:**
    *   [ ] The `map` -> `odom` -> `base_link` transform chain is complete and stable in the TF tree.
    *   [ ] The **local path** (in the `odom` frame) is smooth and continuous but shows significant, visible drift upon returning to the start. The end of the path does not align with the beginning.
    *   [ ] The **global path** (in the `map` frame) may show small, discrete jumps as GPS data is fused, but upon returning to the start, its endpoint aligns very closely with its starting point, demonstrating successful drift correction. This is the primary success criterion.

## **9. Out of Scope / Future Considerations**

To ensure Phase 2 remains focused, the following capabilities are explicitly **out of scope** for this PRD:

*   Any form of mapping (SLAM) or localization within a pre-existing map (AMCL).
*   Any autonomous navigation or motion planning (Nav2).
*   High-level mission logic (Behavior Trees).
*   The web-based User Interface.
*   Integration with physical hardware.
# **Product Requirements Document: Autonomous Mower - Phase 1 (Draft 2)**

## **1. Introduction / Overview**

This document provides a complete set of requirements for **Phase 1: Foundational Simulation and Manual Control** of the Autonomous Mower Development Plan. The objective of this phase is to produce a high-fidelity digital twin of the mower within a simulated environment. This deliverable will serve as the primary development and testing platform for all subsequent software development.

This PRD is a self-contained specification. It details all functional requirements, technical constraints, physical dimensions, and controller configurations necessary for a developer to successfully complete this phase.

## **2. Goals / Objectives**

The principal objective of Phase 1 is to **create a developer-ready, visually and kinematically accurate simulation of the mower that is fully operational for basic manual control.** The final output will be a stable, reliable, and easy-to-use simulation tool that validates the robot's fundamental design and control structure, enabling future development of autonomous capabilities.

## **3. Target Audience / User Personas**

*   **User:** Internal Robotics Developer
*   **Role & Goal:** The developer is the primary user of the Phase 1 deliverable. They need a stable and accurate simulation environment to serve as the development and testing platform for the entire ROS 2 software stack. Their goal is to have a reliable tool that abstracts away the physical hardware, allowing them to focus on building and validating autonomy software in later phases.

## **4. User Stories / Use Cases**

*   "As a developer, I want to launch a single command to start the entire simulation environment (Gazebo world, robot model, and control nodes) so I can begin testing quickly."
*   "As a developer, I want to drive the robot's chassis in the simulation using my keyboard to manually test its basic mobility and the correctness of the differential drive controller."
*   "As a developer, I want to view the robot's TF tree (the relationship between all its parts) and its odometry trail in RViz to visually confirm the control system is publishing the correct state information."

## **5. Functional Requirements**

### **FR1: Robot Modeling (URDF/XACRO)**
The system **must** include a complete URDF/XACRO model of the mower with the following specifications:

*   **Coordinate System:** The model **must** use the ROS standard coordinate system (+X Forward, +Y Left, +Z Up). The central `base_link` frame **must** be located at ground level, centered between the two drive wheels.
*   **Physical Properties:** The model **must** accurately implement all link dimensions and masses as specified below. Any missing data required for a complete model **must** be identified and included.

| Link Name | Description | Mass (kg) | Dimensions / Position | Joint to Parent |
| :--- | :--- | :--- | :--- | :--- |
| `base_link` | The central reference frame of the robot. | - | Origin (0, 0, 0) | - |
| `chassis` | The main body of the robot. | 2.27 | L:19cm, W:44cm, H:22cm. | Fixed to `base_link` |
| `counterweight` | Rear counterweight for balance. | 9.07 | L:21cm, W:13cm, H:13cm. | Fixed to `chassis` |
| `right_wheel` | Right drive wheel with hub motor. | 1.81 | Ø19.5cm, W:3.6cm. At (0, -23cm, 9.75cm) | Continuous, revolute |
| `left_wheel` | Left drive wheel with hub motor. | 1.81 | Ø19.5cm, W:3.6cm. At (0, 23cm, 9.75cm) | Continuous, revolute |
| `front_roller` | Front caster roller. | 0.45 | Ø8.89cm, W:33cm. At (48cm, 0, 4.445cm) | Continuous, revolute |
| `reel` | The cutting reel assembly. | 11.34 | Ø17.5cm, W:47cm. At (30cm, 0, 8.75cm) | Continuous, revolute |
| `reel_motor` | The motor driving the reel. | 1.0 (est.) | At (21.8cm, -23.5cm, 17.55cm) | Fixed to `chassis` |

*   **Inertial Properties:** For a realistic physics simulation, inertia tensors for each link **must** be calculated and included in the model. These values should be derived based on the assumption of simple, uniform geometric shapes (e.g., box, cylinder, sphere).

### **FR2: Simulation Environment**
*   The system **must** provide a Gazebo world for the robot to operate in. The world should include a ground plane and simple static obstacles (e.g., a wall, a slope) to facilitate basic testing.
*   A single, top-level launch file **must** be provided to start the Gazebo environment and spawn the complete robot model.

### **FR3: Manual Control**
*   The robot model **must** be controllable for manual driving using the standard `teleop_twist_keyboard` ROS 2 package.
*   Chassis control **must** be achieved by publishing standard `geometry_msgs/Twist` messages to the `/cmd_vel` topic.
*   The cutting reel (`reel_joint`) **must** be controllable (i.e., made to spin). This functionality will be verified by manually publishing a standard ROS 2 message to its designated controller.

### **FR4: ros2_control Integration**
The simulation **must** use the `ros2_control` framework for hardware abstraction. The controllers **must** be configured with the following parameters:

*   **Differential Drive Controller:**
    *   **Controller Type:** `diff_drive_controller`
    *   **Left Wheel Joint Name:** `left_wheel_joint`
    *   **Right Wheel Joint Name:** `right_wheel_joint`
    *   **Wheel Separation:** **0.46 m**
    *   **Wheel Radius:** **0.0975 m**
*   **Reel Controller:**
    *   **Controller Type:** `joint_trajectory_controller` (or equivalent velocity controller).
    *   **Controlled Joint:** `reel_joint`
    *   **Interface Type:** `velocity`
    *   **Transmission:** The controller configuration **must** account for the physical **20:76** belt reduction between the motor and the reel.

### **FR5: Visualization and State Publishing**
*   The system **must** publish the robot's state, including the transform tree (TF) and joint states.
*   The `diff_drive_controller` **must** publish odometry data as a `nav_msgs/Odometry` message to the `/odom` topic.
*   A launch file **must** be provided to start RViz with a pre-loaded configuration to display the robot model, its TF tree, and its odometry trail.

## **6. Non-Functional Requirements**

### **NFR1: Code Structure & Maintainability**
*   To ensure long-term maintainability and modularity, the project **must** adhere to the following package structure within the `mower_ws/src` workspace:
    *   `mower_bringup`
    *   `mower_description`
    *   `mower_control`
    *   `mower_simulation`
    *   `mower_localization`
    *   `mower_navigation`
    *   `mower_mission_control`
    *   `mower_ui`
    *   `mower_msgs`

### **NFR2: Technical Constraints**
*   **ROS 2 Framework:** The software stack **must** use `ROS 2 Jazzy`.
*   **Simulator:** The simulator **must** be `Gazebo`.
*   **Control Abstraction:** The control framework **must** be `ros2_control`, utilizing the `gz_ros2_control` package for simulation.
*   **Robot Model Format:** The robot model **must** be written in `URDF` and `XACRO`.

## **7. Design Considerations / Mockups**

This phase is purely for backend and simulation development. No graphical user interface (GUI) is required. The primary visual interfaces will be the Gazebo simulator and the RViz visualization tool.

## **8. Success Metrics**

The successful completion of Phase 1 will be verified when all of the following criteria are met:

*   [ ] The main simulation launch file (`sim.launch.py`) executes without errors.
*   [ ] The robot model loads correctly in the Gazebo simulation environment and the RViz visualization tool, reflecting all specified physical properties.
*   [ ] A developer can demonstrate smooth manual control of the robot (forward, backward, left turn, right turn) using keyboard teleoperation.
*   [ ] A developer can demonstrate that the cutting reel can be commanded to spin by publishing a ROS 2 message.
*   [ ] The `/odom` topic is being published with odometry data, and the `odom` -> `base_link` transform in the TF tree is correctly updated and visualized in RViz during movement.

## **9. Out of Scope / Future Considerations**

To ensure Phase 1 remains focused, the following capabilities are explicitly **out of scope** for this PRD:

*   Integration, processing, or simulation of any sensors beyond the implicit wheel encoders (e.g., no IMU, GPS).
*   Any form of autonomous behavior, including mapping (SLAM), localization (AMCL), or navigation (Nav2).
*   Development of the web-based User Interface.
*   PID tuning for physical hardware.
*   Any integration with physical hardware. These will be addressed in subsequent phases.
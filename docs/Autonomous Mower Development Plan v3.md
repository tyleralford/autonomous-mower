# **A Comprehensive Development Plan for a ROS 2-Based Autonomous Lawn Mower**

This document presents a comprehensive, phased development plan for an autonomous lawn mower based on the ROS 2 Jazzy framework. The plan is designed to guide a development team through the entire software lifecycle, from initial simulation and modeling to final hardware integration and deployment. By structuring the project into six discrete and verifiable phases, this plan aims to systematically de-risk development, ensure modularity, and produce a robust, production-ready software stack. This report will serve as the primary engineering blueprint, detailing architectural decisions, implementation strategies, and testing protocols for each component of the system.

## **System Architecture and Development Environment**

This initial section establishes the foundational principles and infrastructure for the entire project. A well-defined architecture and development environment are critical for team collaboration, code quality, and long-term maintainability. A significant portion of failures in complex robotics projects stems not from algorithmic flaws but from poor software engineering practices, leading to integration challenges, untestable code, and difficulty in collaboration. By adopting the best practices from the ROS 2 community from the outset, a scaffold for success is created. Investing time in this foundational phase, even before writing a single line of control code, will dramatically accelerate development in later phases and reduce debugging time. It transforms the project from a hobbyist experiment into a professional engineering endeavor.

### **ROS 2 Workspace and Package Structure**

A structured approach to code organization is paramount for managing the complexity of an autonomous system. The project will be contained within a single ROS 2 workspace and divided into multiple, functionally distinct packages to enforce a clear separation of concerns.

#### **Workspace Setup**

A single ROS 2 workspace, named mower\_ws, will be established to contain all custom packages. This follows standard ROS 2 practice, which recommends a dedicated directory for each new project. The workspace will have a src directory where all package source code will reside. The initial setup involves creating the directory structure and performing an initial build with colcon build to generate the build, install, and log directories.

#### **Package Organization**

The project will be organized into a series of modular packages. This architecture enforces separation of concerns, a key principle for large ROS projects that enhances maintainability and reusability. This structure is inspired by successful open-source projects like OpenMowerROS2 and ros2\_diff\_drive\_robot. The modular package structure isolates concerns, making each part independently buildable and testable. The following packages will be created within the mower\_ws/src directory:

*   **mower\_bringup**: Contains top-level launch files for starting the robot in different configurations (e.g., simulation, hardware). This package will orchestrate the launching of nodes from all other packages, making it the primary entry point for running the system. Good launch file design involves creating a top-level file that includes other, more specific launch files, promoting reusability.
*   **mower\_description**: Contains the robot's Unified Robot Description Format (URDF) and XACRO model files, mesh files for visualization, and Gazebo-specific configurations. This package encapsulates the physical and visual representation of the robot.
*   **mower\_control**: Implements the ros2\_control hardware interface for the physical robot and contains controller configurations in .yaml files. This package abstracts the low-level hardware control from the rest of the system.
*   **mower\_simulation**: Holds Gazebo world files, 3D models for the environment, and simulation-specific launch files. This keeps all simulation-related assets organized in one place.
*   **mower\_localization**: Contains configuration files (.yaml) and launch files for the robot\_localization package, specifically for the unified EKF setup and the navsat\_transform\_node.
*   **mower\_navigation**: Contains configuration for the Nav2 stack, including costmap parameters, planner settings, behavior tree XMLs, and saved map files (.pgm, .yaml).
*   **mower\_mission\_control**: Implements the high-level mission logic using BehaviorTree.CPP. This package will contain the C++ source code for custom Behavior Tree nodes that define mower-specific actions and conditions.
*   **mower\_ui**: Contains the source code for the locally hosted web user interface, including the backend server (e.g., Python or Node.js) and the frontend React application.
*   **mower\_msgs**: Defines any custom ROS 2 messages, services, or actions required for communication between packages. This is crucial for establishing a clear and stable API between the various subsystems.

### **Core Data Types and Custom Interface Definitions**

While standard ROS 2 message types (sensor\_msgs, nav\_msgs, geometry\_msgs) will be used wherever possible, the project's unique requirements necessitate custom interfaces. These will be defined in the mower\_msgs package, creating a clear and version-controlled API contract between the different software modules.
**Example Custom Interface Definitions:**

*   **MowerStatus.msg**: A comprehensive telemetry message designed to provide a snapshot of the robot's state to the UI and other monitoring tools.
    `# MowerStatus.msg`
    `std_msgs/Header header`
    `float32 battery_voltage`
    `float32 left_driver_temp`
    `float32 right_driver_temp`
    `uint8 gps_fix_type   # 0:NO_FIX, 1:FIX, 2:RTK_FLOAT, 3:RTK_FIXED`
    `bool cutter_active`
    `string current_state # e.g., "IDLE", "MOWING", "DOCKING"`

*   **StartMowing.action**: A ROS 2 action for initiating and managing a mowing job. Actions are suitable for long-running tasks that require feedback.
    `# StartMowing.action`
    `# Goal`
    `geometry_msgs/Polygon mow_area`
    `string mowing_pattern`
    `float32 cutting_height`
    `---`
    `# Result`
    `bool success`
    `string message`
    `---`
    `# Feedback`
    `float32 percent_complete`
    `string current_status`

*   **DefineMapArea.srv**: A service that allows the UI to send user-defined polygons to the navigation system to update the keep-out zone map.
    `# DefineMapArea.srv`
    `# Request`
    `uint8 area_type # 0:MOW_AREA, 1:KEEPOUT_ZONE, 2:TRAVEL_PATH`
    `geometry_msgs/Polygon area`
    `---`
    `# Response`
    `bool success`

#### **Table 1: ROS 2 Package Overview**

The following table provides a clear, high-level map of the entire software architecture, defining the responsibility of each component. This is invaluable for developers to understand where to find or add specific functionality.

| Package Name | Primary Language | Purpose | Key Dependencies |
| :--- | :--- | :--- | :--- |
| mower\_bringup | Python (Launch) | Top-level launch files to start the system in various modes (sim, hardware). | All other mower\_\* packages |
| mower\_description | URDF/XACRO | Contains the robot's physical model, sensor placements, and visual/collision meshes. | urdf, xacro |
| mower\_control | C++ | ros2\_control hardware interface implementation and controller configurations. | ros2\_control, hardware\_interface |
| mower\_simulation | SDF/XML | Gazebo world files and simulation-specific assets. | gazebo\_ros, gz\_ros2\_control |
| mower\_localization | YAML/Launch | Configuration for robot\_localization (unified EKF) and navsat\_transform\_node. | robot\_localization |
| mower\_navigation | YAML/Launch/XML | Nav2 stack configuration (costmaps, planners, behavior trees) and map files. | nav2\_bringup |
| mower\_mission\_control | C++ | High-level mission logic via Behavior Trees; custom BT nodes for mower-specific actions. | behaviortree\_cpp\_v4, nav2\_behavior\_tree |
| mower\_ui | JavaScript/Python | Backend server and frontend React application for user interaction. | foxglove\_bridge, React |
| mower\_msgs | msg/srv/action | Custom interface definitions for communication between packages. | rosidl\_default\_generators |

## **Phase I: Foundational Simulation and Manual Control**

**Goal:** Create a visually and kinematically accurate simulation of the robot and establish basic manual control. This phase validates the robot's physical model and the low-level drive system, forming the bedrock upon which all subsequent autonomy will be built.

### **Robot Modeling with URDF and XACRO**

The first step in any robotics project is to create a digital twin of the physical system. The Unified Robot Description Format (URDF) is the standard for this in ROS. A detailed URDF model is not just for visualization; it is fundamental for physics simulation, collision detection, and kinematic calculations.
A URDF file will be created in the mower\_description package. It will define the robot's structure through a series of link and joint elements.

*   **Links**: These represent the rigid bodies of the robot. For this project, the primary links will be base\_link (the main chassis), left\_wheel\_link, right\_wheel\_link, caster\_wheel\_link, and cutter\_reel\_link.
*   **Joints**: These define the kinematic relationships between links. The left\_wheel\_joint and right\_wheel\_joint will be of type continuous to allow for infinite rotation, which is necessary for a differential drive robot. The cutter and caster joints will likely be revolute or fixed, depending on the design.
*   **Visuals and Collisions**: Each link will contain both \<visual\> and \<collision\> elements. The \<visual\> tag defines what the robot looks like in visualization tools like RViz, while the \<collision\> tag defines the geometry used by the physics engine for collision checking. Initially, simple geometric primitives (box, cylinder, sphere) will be used for rapid prototyping. These can later be replaced with detailed 3D mesh files (e.g., .stl or .dae) for higher visual fidelity and more accurate collision modeling.
*   **Inertial Properties**: For a realistic physics simulation in Gazebo, each link must have an \<inertial\> tag defining its mass and inertia tensor. Accurate values are crucial for simulating realistic acceleration, braking, and turning dynamics. These will be estimated based on the CAD model of the physical robot.

To manage the complexity of the URDF file and promote reusability, it will be written using XACRO (XML Macros). XACRO allows for the use of constants (e.g., wheel\_radius, chassis\_width), mathematical expressions, and macros. For instance, a single macro can define a wheel, which can then be instantiated for both the left and right sides, reducing code duplication and making the model easier to modify.

### **Gazebo Simulation Environment Setup**

With the robot model defined, a virtual environment is needed to test it. Gazebo is the chosen simulator due to its tight integration with ROS 2 and its realistic physics engine.
A simple Gazebo world file (e.g., lawn.world) will be created in the mower\_simulation package. This SDF (Simulation Description Format) file will define the environment, including a ground plane with grass texture, a sun model for lighting, and global physics properties like gravity. To facilitate testing, the world will include basic obstacles such as a tree, a wall, and a sloped area to simulate a typical residential lawn.
The URDF model must be augmented with Gazebo-specific information using the \<gazebo\> tag. This is where the simulation interfaces for sensors and actuators are defined via plugins. For example, material properties like friction for the wheels will be defined here to ensure realistic interaction with the ground plane.
A top-level launch file in mower\_bringup will be responsible for starting the simulation. It will launch the Gazebo server, spawn the robot model by calling the spawn\_entity.py script from the gazebo\_ros package, and start visualization tools. This launch file will read the URDF content from the robot\_description topic, which is populated by the robot\_state\_publisher node.

### **ros2\_control Integration for Differential Drive**

The decision to use ros2\_control from the very beginning, even in simulation, is a strategic choice that establishes a clean separation between high-level control logic and hardware-specific implementation. While a simpler Gazebo plugin like libgazebo\_ros\_diff\_drive.so could be used , it tightly couples the simulation to a specific plugin's logic. The ros2\_control framework provides a standardized, more powerful alternative that is designed for seamless sim-to-real transition. The controller becomes a separate, reusable component, and the simulation plugin is treated as just one possible "hardware interface." In Phase VI, this simulation plugin will be swapped for a custom real-world hardware interface, but the diff\_drive\_controller and all the logic above it will remain *unchanged*. This architecture minimizes the changes required to move from simulation to the physical robot, drastically reducing the "sim-to-real gap."
The integration involves three key steps:

1.  **URDF Configuration**: The robot's XACRO file will be augmented with \<ros2\_control\> tags. Inside this tag, a \<plugin\> will be specified for the hardware interface. For the simulation phase, this will be the gz\_ros2\_control/GazeboSystem plugin. This plugin acts as the bridge between the ros2\_control framework and Gazebo's physics engine.
2.  **Joint Interfaces**: Within the \<ros2\_control\> tag, each controllable joint (the two drive wheels) will have its interfaces defined. The diff\_drive\_controller requires a velocity command interface and position and velocity state interfaces. These tell ros2\_control what can be commanded and what can be read from the "hardware" (in this case, the Gazebo simulation).
3.  **Controller Configuration**: A mower\_controllers.yaml file in the mower\_control package will define the controllers to be loaded by the controller\_manager.
    *   **joint\_state\_broadcaster**: This essential controller reads the state of all registered joints from the hardware interface and publishes them to the /joint\_states topic. This topic is consumed by robot\_state\_publisher to update the robot's TF tree, which is critical for visualization and localization.
    *   **diff\_drive\_controller**: This is the core motion controller. It subscribes to a /cmd\_vel topic (of type geometry\_msgs/Twist) and uses the robot's kinematics (wheel separation and radius) to convert the desired linear and angular velocities into individual velocity commands for the left and right wheels. It also uses the feedback from the wheel state interfaces (simulated encoders) to calculate and publish odometry information on an /odom topic.

### **Basic Keyboard Teleoperation**

To provide a simple method for manual control and initial testing, the standard teleop\_twist\_keyboard package will be used. A launch file will start this node and use ROS 2's remapping feature to connect its output Twist message topic to the input topic expected by the diff\_drive\_controller. This allows a developer to drive the robot around the Gazebo world using keyboard commands, providing immediate feedback on the correctness of the model and control setup.

### **Phase I Testing and Validation**

The successful completion of Phase I will be verified through a clear, repeatable test procedure.

*   **Objective**: Verify that the robot model spawns correctly in Gazebo and can be driven accurately and smoothly using manual keyboard commands.
*   **Procedure**:
    1.  Execute the main simulation launch file (sim.launch.py) from the mower\_bringup package.
    2.  Confirm that the Gazebo window opens, the lawn world is loaded, and the robot model appears without errors.
    3.  Confirm that RViz launches and displays the robot model correctly, rendered from the TF data published by robot\_state\_publisher.
    4.  Launch the teleoperation node.
    5.  Issue forward, backward, and turning commands from the keyboard.
*   **Expected Outcome**: The robot model in Gazebo must move in response to the commands. The motion should be smooth, and the wheels should rotate in the correct directions. The /odom topic published by the diff\_drive\_controller should show continuously updating pose information that corresponds to the robot's movement. The TF tree in RViz should show the odom frame and the moving base\_link frame.

#### **Table 2: diff\_drive\_controller Parameter Reference**

This table provides a quick reference for the essential parameters needed to configure the differential drive controller, which is central to the robot's mobility. This consolidates information from official documentation into a project-specific context.

| Parameter | Type | Example Value | Description |
| :--- | :--- | :--- | :--- |
| left\_wheel\_names | string\_array | \['left\_wheel\_joint'\] | Name of the joint(s) for the left side of the drive train. |
| right\_wheel\_names | string\_array | \['right\_wheel\_joint'\] | Name of the joint(s) for the right side of the drive train. |
| wheel\_separation | double | 0.5 | Distance between the center of the left and right wheels (meters). Critical for turning kinematics. |
| wheel\_radius | double | 0.15 | Radius of the drive wheels (meters). Critical for calculating linear velocity from angular velocity. |
| odom\_frame\_id | string | odom | The name of the odometry frame to be published in the odometry message and TF. |
| base\_frame\_id | string | base\_link | The name of the robot's base frame. The controller will publish the odom \-\> base\_link transform. |
| publish\_rate | double | 50.0 | Frequency (Hz) at which odometry is published. |
| use\_stamped\_vel | bool | false | Whether to subscribe to TwistStamped (true) or Twist (false) messages for velocity commands. |
| pose\_covariance\_diagonal | double\_array | \[0.001,...\] | Diagonal elements of the 6x6 pose covariance matrix in the published odometry message. |
| twist\_covariance\_diagonal | double\_array | \[0.001,...\] | Diagonal elements of the 6x6 twist covariance matrix in the published odometry message. |
| open\_loop | bool | false | If true, odometry is calculated from commanded velocities instead of feedback from state interfaces. |
| enable\_odom\_tf | bool | true | Whether the controller should publish the odom \-\> base\_link transform. This will be used by the local EKF. |

## **Phase II: Sensor Integration and State Estimation**

**Goal:** Integrate all simulated sensors into the Gazebo model and establish a robust, unified Extended Kalman Filter (EKF) state estimation system. This phase is critical for providing a single, accurate, and reliable pose, which is the foundation for all autonomous navigation.

### **Gazebo Sensor Plugin Configuration**

To simulate the robot's sensor suite, Gazebo plugins will be added to the URDF model. These plugins interface with Gazebo's physics and rendering engines to generate realistic sensor data and publish it on ROS 2 topics.

*   **IMU Plugin**: The GazeboRosImuSensor plugin will be added to the URDF, attached to a dedicated imu\_link that is fixed to the base\_link. This plugin simulates an Inertial Measurement Unit by publishing sensor\_msgs/msg/Imu messages. The plugin's configuration will include noise parameters to simulate the drift and inaccuracies inherent in real-world IMUs.
*   **GPS Plugin**: The GazeboRosGpsSensor plugin will be used to provide standard GPS data. It will be configured to publish sensor\_msgs/msg/NavSatFix messages containing latitude, longitude, and altitude, with noise parameters configured to simulate varying signal quality.
*   **Wheel Encoders**: The simulation of wheel encoders is implicitly and accurately handled by the gz\_ros2\_control plugin configured in Phase I. It reads the ground-truth joint state from Gazebo's physics engine, serving as perfect simulated encoder feedback.

### **Establishing the TF2 Transformation Tree (REP-105)**

A consistent and standardized coordinate frame tree is essential. The system will adhere strictly to the conventions defined in ROS Enhancement Proposal (REP) 105. The core transformation tree will have the simplified, robust structure `map` -> `base_link`:

*   **base\_link**: This frame is rigidly attached to the robot's chassis. Transforms from `base_link` to all other rigidly attached frames (e.g., `imu_link`, `gps_link`) are static and published by `robot_state_publisher`.
*   **map**: This is a world-fixed, GPS-anchored frame. It is globally accurate and drift-free, representing the origin of the operational area. The transform from `map` to `base_link` is dynamic and will be published by the state estimation filter.

### **Unified EKF State Estimation with `robot_localization`**

Fusing multiple sensor sources with an Extended Kalman Filter (EKF) provides a much more accurate and reliable pose estimate than any single source. This project will use a single, unified EKF to produce one authoritative pose for the entire system.

The `robot_localization` package provides a highly configurable implementation of EKF. A single instance of its `ekf_node` will be configured in the `mower_localization` package to:
*   **Set the World Frame**: The `world_frame` parameter will be set to `map`.
*   **Fuse All Inputs**: The filter will be configured to fuse four sources of data:
    1.  **Wheel Odometry**: High-frequency relative velocity from the `diff_drive_controller`.
    2.  **IMU**: High-frequency relative orientation and angular velocity.
    3.  **GPS (Absolute Position)**: Absolute position from the `navsat_transform_node`.
    4.  **GPS (Absolute Heading)**: An absolute heading measurement (simulated in this phase) to anchor the robot's orientation.
*   **Produce the Final Pose**: The node's primary responsibility is to publish the `map` -> `base_link` transform and a globally accurate odometry message on `/odometry/filtered`.

### **Configuring `navsat_transform_node` for GPS Integration**

The `navsat_transform_node`, also from the `robot_localization` package, is a critical utility that converts geodetic coordinates (latitude, longitude) from a GPS into the robot's Cartesian `map` frame. It uses the first valid set of messages to establish the origin (the datum) of the `map` frame, anchoring the robot's coordinate system to a real-world geodetic point. Its output is a `nav_msgs/Odometry` message containing the robot's position in the `map` frame, which is then consumed by the EKF.

### **Phase II Testing and Validation**

A series of tests will be conducted to validate the entire state estimation pipeline.

*   **Objective**: Verify that the EKF is running correctly and producing a stable, globally-accurate pose estimate that corrects for the drift of raw wheel odometry.
*   **Procedure**:
    1.  Launch the full simulation stack, including the `ekf_node` and `navsat_transform_node`.
    2.  In RViz, visualize two paths: one from the raw wheel odometry (`/odom`) and one from the EKF's filtered output (`/odometry/filtered`).
    3.  Using teleoperation, drive the robot in a large, closed-loop pattern, returning to the starting point.
*   **Expected Outcomes**:
    *   **Raw Odometry Path**: This path should show significant, visible drift; the end of the path will not align with the start.
    *   **Filtered (map) Path**: This path should be globally accurate. Upon returning to the start, the end of this path should align very closely with the beginning, demonstrating the effectiveness of the global correction from the GPS and heading sensors.
    *   **TF Tree**: The `map` -> `base_link` transform chain must be complete and stable throughout the test.

## **Phase III: User-Defined Mapping and GPS-Based Navigation**

**Goal:** Integrate the ROS 2 Navigation Stack (Nav2) to empower the robot with core autonomous capabilities: creating a map of its operational zones and navigating to specified goal points within those zones using its high-accuracy GPS-based pose.

### **Integrating the Nav2 Stack**

Nav2 is the production-grade navigation solution in ROS 2. A central navigation launch file will be created to launch the Nav2 Lifecycle Manager and all required servers (planner, controller, behavior). The robot's authoritative pose for Nav2 will be provided directly by the filtered EKF output from Phase II, operating entirely within the GPS-anchored `map` frame.

### **Map Generation via "Drive-to-Record"**

For an outdoor robot, a map of user-defined operational zones is more effective than traditional SLAM. This project will use a "drive-to-record" methodology.

1.  **Recording Mode**: A developer will initiate a "recording session" via a ROS 2 service. Using teleoperation, they will drive the robot around the perimeter of the desired mowing area and any internal keep-out zones. A node will record the robot's GPS-corrected pose from the EKF.
2.  **Map Generation Script**: A separate offline script will process the recorded waypoint files. This script will generate a standard Nav2 costmap (`.pgm` image file and `.yaml` metadata file), where the area inside the main perimeter is marked as free space, and keep-out zones are marked as lethal space.

### **Map Management and Layered Costmaps**

The generated map defines the static boundaries of the robot's world. Nav2 will be configured to use a layered costmap architecture, which is critical for future extensibility.

*   **Static Layer**: Loads the `.pgm` map file created by the map generation script.
*   **Inflation Layer**: Adds an inflation radius around all lethal obstacles.
*   **Future Extensibility**: This layered architecture is a key design choice. In the future, real-time obstacle avoidance can be seamlessly added by inserting a new sensor layer without re-architecting the core navigation logic.

### **Point-to-Point Navigation**

With a map of operational zones and a reliable pose from the EKF, the robot can navigate autonomously. When a goal point is provided, the Nav2 planner server will generate a path that respects the map boundaries, and the controller server will execute that path, guiding the robot to its destination.

### **Phase III Testing and Validation**

This phase concludes with end-to-end navigation tests.

*   **Objective**: Verify that the robot can create a valid operational map and reliably navigate to a goal point within that map using its GPS-based localization.
*   **Procedure**:
    1.  Launch the full simulation stack.
    2.  Use the "drive-to-record" feature to define a lawn area with a keep-out zone.
    3.  Run the map generator script.
    4.  Restart the system with the full navigation stack, loading the new map.
    5.  In RViz, set a navigation goal that requires navigating around the keep-out zone.
*   **Expected Outcomes**:
    *   Nav2's planner should generate a global plan that remains entirely within the "free space" and avoids the keep-out zone.
    *   The robot should successfully and accurately reach the specified goal pose in simulation.

## **Phase IV: Mission Control and Coverage Path Planning**

**Goal:** Implement the high-level intelligence for the mower, enabling it to execute a full mowing job using a coverage path planning algorithm, managed by a robust Behavior Tree. This phase transitions the robot from simple point-to-point navigation to performing a complete, autonomous task.

### **High-Level Logic: A Behavior Tree-Based Architecture**

The core logic will be governed by a Behavior Tree (BT), using the BehaviorTree.CPP v4 library. A new C++ node, `mower_mission_control_node`, will load the main BT definition from an XML file and "tick" the tree to drive the robot's behavior.

### **Designing the Mower's Core Behavior Tree**

A library of custom C++ BT nodes will be developed to bridge the gap between the abstract logic of the BT and the concrete ROS 2 interfaces.

*   **Condition Nodes**: Examples: IsBatteryLow, IsMowingJobActive.
*   **Action Nodes**: Examples: GoToDock (calls Nav2), GenerateCoveragePath, FollowCoveragePath.

### **Coverage Path Planning (CPP) for Mowing Patterns**

The Boustrophedon ("the way of the ox") decomposition algorithm will be used to generate mowing patterns. A `GenerateCoveragePath` BT action node will take the user-defined mowing area polygon as input and output an ordered list of waypoints. A `FollowCoveragePath` action node will then use Nav2's Waypoint Follower to execute the pattern.

### **Executing a Full Mowing Mission**

A complete mowing mission will be a Sequence of actions: GenerateCoveragePath, NavigateToStart, EngageCutter, FollowCoveragePath, DisengageCutter, and GoToDock.

### **Phase IV Testing and Validation**

*   **Objective**: Verify the mission controller can generate and execute a full coverage path within a user-defined zone.
*   **Procedure**: Launch the full system and send a "StartMowing" action goal.
*   **Expected Outcomes**: The robot should generate a Boustrophedon path, navigate to the start, follow the path, and return to the dock.

#### **Table 3: Behavior Tree vs. Finite State Machine Comparison**

This table formally justifies the architectural decision to use Behavior Trees over Finite State Machines for the project's high-level logic, providing a clear, at-a-glance comparison for stakeholders.

| Feature | Finite State Machine (FSM) | Behavior Tree (BT) | Justification for Mower |
| :--- | :--- | :--- | :--- |
| **Structure** | Graph of states and explicit transitions. | Hierarchical tree of tasks (nodes). | BT's hierarchy naturally models complex tasks (Mow Job \-\> Follow Path \-\> Get Next Waypoint). |
| **Scalability** | Becomes exponentially complex ("state explosion") with new states/conditions. | Highly modular; new behaviors can be added as new subtrees or nodes with minimal impact. | The mower will have many states (mowing, docking, rain delay, obstacle wait). BTs scale better. |
| **Reactivity** | Requires transitions from every state to handle global events (e.g., E-Stop, Pause). | Can handle global events at a high level in the tree (e.g., a ReactiveSequence) that preempts lower-level tasks. | A ReactiveSequence can constantly check for "Pause" or "Battery Low", making the system instantly responsive without complex transition logic. |
| **Reusability** | States are monolithic and hard to reuse. | Nodes (e.g., NavigateTo, CheckBattery) are self-contained and highly reusable across different trees. | A library of custom BT nodes can be built and composed in different ways for different missions (e.g., "Mow Lawn" vs. "Edge Trim"). |
| **Development Tooling** | Often requires custom graphical editors or manual coding. | Excellent tooling like Groot for visualization and debugging BehaviorTree.CPP XML files. | Groot provides live visualization of the tree's execution, which is invaluable for debugging complex mission logic. |

## **Phase V: Web User Interface and System Telemetry**

**Goal:** Develop a comprehensive, touch-optimized web-based user interface for full command, control, and monitoring of the lawn mower. This UI will be the primary method of interaction for the end-user.

### **Web UI Architecture: ROS-to-Web Communication**

*   **Backend/Bridge**: The foxglove\_bridge will be used as the WebSocket server.
*   **Frontend**: A Single-Page Application (SPA) will be developed using the **React** JavaScript framework.
*   **Hosting**: The web application will be served by a lightweight web server on the robot's onboard computer, which will create its own Wi-Fi hotspot.

### **UI Component: Interactive Map and Zone Editor**

The map is the central element of the UI. It will be implemented in two stages:

1.  **Initial Implementation (Drive-to-Record)**: The UI will provide controls to start and stop a "drive-to-record" session while the user manually drives the robot with a virtual joystick.
2.  **Enhanced Implementation (Polygon Editing)**: The UI will be enhanced to display the generated zone polygons overlaid on a satellite map background. Users will be able to directly manipulate the vertices of these polygons to fine-tune the boundaries.

### **UI Component: Job Configuration and Control**

The UI will provide a simple interface for managing mowing jobs, including selecting a mowing area, choosing a pattern, and controlling the mission with "Start," "Pause," and "Stop" buttons.

### **UI Component: Telemetry Dashboard and Visualization**

A dashboard will display key telemetry data (battery, GPS status, etc.). A virtual joystick will provide manual control.

### **Phase V Testing and Validation**

*   **Objective**: Ensure the web UI is fully functional, intuitive, and reliably controls all aspects of the robot's operation.
*   **Procedure**: Systematically test every feature, including the "drive-to-record" workflow and a full mission launch, on multiple touch-based devices.
*   **Expected Outcome**: The UI provides complete, stable, and reliable control over the robot.

## **Phase VI: Hardware Integration and Deployment**

**Goal:** Transition the entire software stack from the simulated environment to the physical robot hardware, validate its real-world performance, and prepare it for deployment.

### **Implementing a ros2\_control Hardware Interface**

A custom C++ class, `MowerHardwareInterface`, will be created to bridge the generic `ros2_control` framework with the robot's specific hardware (motor controllers, IMU), implementing the standard `hardware_interface::SystemInterface`.

### **PID Tuning for Drive Motor Control**

The low-level motor controllers will be tuned to accurately track the velocity setpoints from the `diff_drive_controller`. Tools like `rqt_plot` will be used to graph commanded vs. actual velocity to adjust PID gains for a fast and stable response.

### **Onboard Computer Setup and System Launch**

An onboard computer will be set up with Ubuntu Server and ROS 2 Jazzy. The main launch file will be configured as a `systemd` service to start automatically on boot.

### **Bridging Simulation and Reality: Final Validation**

A comprehensive, end-to-end system test will be performed on the physical hardware in a real-world environment.

*   **Objective**: To validate that the system's real-world performance matches its performance in simulation.
*   **Procedure**:
    1.  **Sensor Validation**: Power on the robot and verify all real sensor data is being published correctly.
    2.  **Teleoperation Test**: Manually drive the robot with the UI to confirm smooth motion.
    3.  **Zone Recording and Navigation Test**: Use the "drive-to-record" feature on a real lawn to define an operational area and a keep-out zone. Test point-to-point navigation around the zone.
    4.  **Full Mission Test**: Launch a full mowing job from the web UI on a small patch of grass.
*   **Expected Outcome**: The physical robot successfully and safely performs all tasks previously validated in simulation, from map creation to autonomous mowing.

## **Conclusion**

This comprehensive development plan outlines a structured, phased approach to building a sophisticated autonomous lawn mower. By leveraging the power and modularity of the ROS 2 ecosystem, adhering to professional software engineering best practices, and systematically progressing from a high-fidelity simulation to physical hardware, this plan provides a clear and robust roadmap for creating a feature-rich and reliable robotic system. The key architectural decisions—a modular package structure, the use of ros2\_control for hardware abstraction, a unified EKF for GPS-based state estimation, Behavior Trees for mission logic, and a modern web-based UI—are designed to manage complexity, ensure scalability, and minimize the sim-to-real gap. The successful completion of these six phases will result in a fully functional prototype and a mature software stack.


#### **Works cited**

1\. ROS 2 developer guide — ROS 2 Documentation: Jazzy ..., https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Developer-Guide.html 2\. Managing large projects — ROS 2 Documentation: Jazzy ..., https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html 3\. ROS2 Project Structure \- Robotics Stack Exchange, https://robotics.stackexchange.com/questions/98344/ros2-project-structure 4\. Creating a workspace — ROS 2 Documentation: Jazzy documentation, https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html 5\. jkaflik/OpenMowerNext: Just another software stack for ... \- GitHub, https://github.com/jkaflik/OpenMowerROS2 6\. gurselturkeri/ros2\_diff\_drive\_robot: Differential drive mobile ... \- GitHub, https://github.com/gurselturkeri/ros2\_diff\_drive\_robot 7\. How to Create URDF and Launch Files in ROS2 and Display Them in Rviz, https://aleksandarhaber.com/how-to-create-urdf-and-launch-files-in-ros2-and-display-them-in-rviz/ 8\. URDF — ROS 2 Documentation: Foxy documentation, https://docs.ros.org/en/foxy/Tutorials/URDF/URDF-Main.html 9\. ros-controls/ros2\_control\_demos: This repository aims at providing examples to illustrate ros2\_control and ros2\_controllers \- GitHub, https://github.com/ros-controls/ros2\_control\_demos 10\. A ROS 2-based Navigation and Simulation Stack for the Robotino \- arXiv, https://arxiv.org/html/2411.09441v1 11\. Setup CI/CD for a ROS2 project using Github | by Shantanuparab ..., https://medium.com/@shantanuparab99/setup-ci-cd-for-a-ros2-project-using-github-121d62bae348 12\. Tutorials — ROS 2 Documentation: Jazzy documentation, https://docs.ros.org/en/jazzy/Tutorials.html 13\. URDF Tutorials — ROS 2 Documentation: Foxy documentation, https://ftp.udx.icscoe.jp/ros/ros\_docs\_mirror/en/foxy/Tutorials/URDF/URDF-Main.html 14\. ROS2 URDF Tutorial \- Describe Any Robot (Links and Joints) \- YouTube, https://www.youtube.com/watch?v=LsKL8N5Iwkw 15\. Create a URDF with ROS2 \[1H Crash Course\] \- YouTube, https://www.youtube.com/watch?v=dZ\_CyyEvBE0 16\. Tutorial : Gazebo plugins in ROS \- Gazebo, https://classic.gazebosim.org/tutorials?tut=ros\_gzplugins 17\. Setting Up Odometry \- Gazebo Classic — Nav2 1.0.0 documentation, https://docs.nav2.org/setup\_guides/odom/setup\_odom\_gz\_classic.html 18\. Class GazeboRosDiffDrive — gazebo\_plugins 3.5.2 documentation, https://docs.ros.org/en/rolling/p/gazebo\_plugins/generated/classgazebo\_\_plugins\_1\_1GazeboRosDiffDrive.html 19\. Building a ros2\_control System: ROS2 Control with the JetBot Part 2 \- AWS, https://builder.aws.com/content/2dk9wWbrwCB7PDXhblRFe0iaBbK/building-a-ros2\_control-system-ros2-control-with-the-jetbot-part-2?trk=db22a1cc-3925-4660-b03b-76ae662f8606\&sc\_channel=el 20\. Tutorial 7: Control — 240AR060 \- Introduction to ROS, https://sir.upc.edu/projects/ros2tutorials/7-control/index.html 21\. ROS2 Jazzy Tutorial: Basics of ros2\_control library and Robot Simulation in Gazebo, https://aleksandarhaber.com/ros2-jazzy-tutorial-basics-of-ros2\_control-library-and-robot-simulation-in-gazebo/ 22\. Writing a Hardware Component — ROS2\_Control: Rolling Jul 2025 ..., https://control.ros.org/rolling/doc/ros2\_control/hardware\_interface/doc/writing\_new\_hardware\_component.html 23\. DiffBot — ROS2\_Control: Humble Aug 2025 documentation, https://control.ros.org/humble/doc/ros2\_control\_demos/example\_2/doc/userdoc.html 24\. How to use the Gazebo differential drive plugin in ROS 2 \- The Construct, https://www.theconstruct.ai/how-to-use-the-gazebo-differential-drive-plugin-in-ros-2/ 25\. diff\_drive\_controller — ROS2\_Control: Humble Aug 2025 documentation, https://control.ros.org/humble/doc/ros2\_controllers/diff\_drive\_controller/doc/userdoc.html 26\. diff\_drive\_controller — ROS2\_Control: Rolling Aug 2025 documentation, https://control.ros.org/rolling/doc/ros2\_controllers/diff\_drive\_controller/doc/userdoc.html 27\. teleop\_twist\_keyboard \- ROS Index, https://index.ros.org/r/teleop\_twist\_keyboard/ 28\. Get CM level accuracy using hector gps plugin in Gazebo \- ROS Answers archive, https://answers.ros.org/question/386920/ 29\. REP 105 \-- Coordinate Frames for Mobile Platforms (ROS.org), https://www.ros.org/reps/rep-0105.html 30\. Setting Up Transformations — Nav2 1.0.0 documentation, https://docs.nav2.org/setup\_guides/transformation/setup\_transforms.html 31\. Navigation Concepts — Nav2 1.0.0 documentation, https://docs.nav2.org/concepts/index.html 32\. navigation \- robot\_localization dual-EKF:How the two ekf nodes ..., https://robotics.stackexchange.com/questions/90869/robot-localization-dual-ekfhow-the-two-ekf-nodes-work-together 33\. Integrating GPS Data — robot\_localization 2.6.12 documentation, http://docs.ros.org/en/melodic/api/robot\_localization/html/integrating\_gps.html 34\. How to Use GPS With the Robot Localization Package – ROS 2 \- Automatic Addison, https://automaticaddison.com/how-to-use-gps-with-the-robot-localization-package-ros-2/ 35\. ROS 2 Navigation — ROS 2 workshop documentation \- Read the Docs, https://ros2-industrial-workshop.readthedocs.io/en/latest/\_source/navigation/ROS2-Navigation.html 36\. ROS2 Nav2 Tutorial \- The Robotics Back-End, https://roboticsbackend.com/ros2-nav2-tutorial/ 37\. Nav2 Mapping with SLAM Toolbox | ROS2 Developers Open Class \#137 \- YouTube, https://www.youtube.com/watch?v=rZOxPGCn4QM 38\. Navigation in ROS2 \- HSHL Mechatronik, https://wiki.hshl.de/wiki/index.php/Navigation\_in\_ROS2 39\. TheOnceAndFutureSmalltalker/ros\_map\_editor: Qt based tool for editing maps generated by ROS gmapping package. \- GitHub, https://github.com/TheOnceAndFutureSmalltalker/ros\_map\_editor 40\. CyberAgentAILab/nav2-keepout-zone-map-creator \- GitHub, https://github.com/CyberAgentAILab/nav2-keepout-zone-map-creator 41\. Navigating with Keepout Zones — Nav2 1.0.0 documentation, https://docs.nav2.org/tutorials/docs/navigation2\_with\_keepout\_filter.html 42\. ROS2 Tutorial: Navigation with keepout zones \- Robots and Robotics, https://robotics.snowcron.com/robotics\_ros2/keepout\_zones\_map.htm 43\. \[ROS Q\&A\] 136 \- How to edit a map generated with gmapping \- The Construct, https://www.theconstruct.ai/ros-qa-136-how-to-edit-a-map-generated-with-gmapping/ 44\. Localization | Clearpath Robotics Documentation, https://docs.clearpathrobotics.com/docs/ros/tutorials/navigation\_demos/localization 45\. State Machines vs Behavior Trees ... \- Polymath Robotics Blog, https://www.polymathrobotics.com/blog/state-machines-vs-behavior-trees 46\. Finite State Machine and Behavior Tree Fusion | by Abdullah Ahmet Askin \- Medium, https://medium.com/@abdullahahmetaskin/finite-state-machine-and-behavior-tree-fusion-3fcce33566 47\. ROS Package: behaviortree\_cpp, https://index.ros.org/p/behaviortree\_cpp/ 48\. Integration with ROS2 | BehaviorTree.CPP, https://www.behaviortree.dev/docs/ros2\_integration/ 49\. fields2cover — fields2cover 2.0.0 documentation, https://docs.ros.org/en/iron/p/fields2cover/ 50\. tbharathchandra/LawnBot: Autonomous Lawn Mowing Robot Built using ROS, Boustrophedon Coverage Planning Algorithm \- GitHub, https://github.com/tbharathchandra/LawnBot 51\. Coverage Path Planning: The Boustrophedon Cellular Decomposition \- Carnegie Mellon University's Robotics Institute, https://www.ri.cmu.edu/pub\_files/pub4/choset\_howie\_1997\_3/choset\_howie\_1997\_3.pdf 52\. Using Rosbridge with ROS 2 \- Foxglove, https://foxglove.dev/blog/using-rosbridge-with-ros2 53\. User Interfaces \- Programming Multiple Robots with ROS 2, https://osrf.github.io/ros2multirobotbook/ui.html 54\. CDonosoK/ros2\_teleoperation: ROS2 package implementing a teleoperation interface using QT \- GitHub, https://github.com/CDonosoK/ros2\_teleoperation 55\. Add TeleOp to your ROS2 Robot \- Medium, https://medium.com/@peter.gaston/add-teleop-to-your-ros2-robot-5b7b0a5606ce 56\. PID Controller — ROS2\_Control: Humble Jul 2025 documentation, https://control.ros.org/humble/doc/ros2\_controllers/pid\_controller/doc/userdoc.html 57\. PID Controller — ROS2\_Control: Rolling Aug 2025 documentation, https://control.ros.org/rolling/doc/ros2\_controllers/pid\_controller/doc/userdoc.html 58\. \[ROS Q\&A\] 112 \- How to tune a PID with ROS Control \- The Construct, https://www.theconstruct.ai/ros-qa-112-how-to-tune-a-pid-with-ros-control/ 59\. pid\_tuning \- ROS Wiki, http://wiki.ros.org/pid\_tuning
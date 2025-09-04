# **Product Requirements Document: Autonomous Mower - Phase 3**

## **1. Introduction / Overview**

This document provides a complete set of requirements for **Phase 3: User-Defined Mapping and GPS-Based Navigation** of the Autonomous Mower Development Plan. This phase builds upon the robust state estimation system developed in Phase 2. The objective is to empower the robot with the ability to create its own operational map through a manual recording process and to autonomously navigate within this map using a globally-referenced (UTM) coordinate system and the ROS 2 Navigation Stack (Nav2).

A key aspect of this phase is ensuring the navigation system is robust and fault-tolerant, gracefully handling off-nominal conditions such as starting without a map or outside of the defined operational area.

This PRD is a self-contained specification that details all functional requirements, architectural decisions, and success criteria necessary for a developer to successfully implement this phase.

## **2. Goals / Objectives**

The principal objective of Phase 3 is to **develop a complete, robust, and safe GPS-based autonomous navigation system that operates within a georeferenced (UTM) frame.** The final output will be a system where a developer can:
1.  Manually drive the robot to record operational zones, with all coordinates saved in the UTM frame.
2.  Have the system automatically generate a persistent, georeferenced navigation map from these recordings.
3.  Trust the system to remain safely inactive until all preconditions for navigation are met.
4.  Command the robot to autonomously navigate to any valid point within the defined map, with the robot correctly respecting all zone limitations.

## **3. Target Audience / User Personas**

*   **User:** Internal Robotics Developer
*   **Role & Goal:** The developer is the primary user of the Phase 3 deliverable. Their goal is to build and validate the core autonomous navigation capabilities of the robot. They need to create a system that can understand its operational environment via manual definition and reliably navigate within it, setting the stage for task-specific logic (like mowing) in later phases.

## **4. User Stories / Use Cases**

*   "As a developer, I want to run a service to start and stop recording the robot's path to define distinct operational zones, with all data saved in UTM coordinates."
*   "As a developer, I want the system to automatically process my recorded zone files into a persistent, georeferenced Nav2 map as soon as I finish recording."
*   "As a developer, I want to configure a single EKF to fuse all sensor data and produce a robust pose estimate in the `utm` frame."
*   "As a developer, I want to launch the Nav2 stack and see that the robot's UTM-based position is correctly localized within its georeferenced map."
*   "As a developer, I want to send a navigation goal in RViz and have the robot plan and execute a path that respects all defined zones."
*   "As a developer, I want the navigation system to remain safely inactive if the robot is started without a map or outside of the map's boundaries, preventing errors."
*   "As a developer, I want to receive a clear status message explaining why the navigation system is not yet active."
*   "As a developer, I want the navigation system to automatically pause if the robot inadvertently drives outside of the defined map area."

## **5. Functional Requirements**

### **FR1: "Drive-to-Record" Zone Creation**
*   A ROS 2 service named `/mower/manage_recording` **must** be created to handle the recording of operational zones.
*   The service **must** be of a custom type, `ManageRecording.srv`.
*   When a recording session is active, a node **must** subscribe to the robot's fused pose in the UTM frame (from `/odometry/filtered`) and save the sequence of UTM poses to the specified file.

### **FR2: Automated Georeferenced Map Generation**
*   The `/mower/manage_recording` service node **must** trigger an internal map generation function immediately upon the successful completion of a `STOP` action.
*   The map generation function **must** process all existing recorded zone files (containing UTM coordinates) to produce a single, consolidated navigation map (`map.pgm`, `map.yaml`).
*   When writing the `map.yaml` file, the `origin` field **must** be set to the correct UTM coordinate of the map's bottom-left corner, making the map inherently georeferenced.
*   The cost hierarchy in the `.pgm` file **must** be as follows:
    *   **Lethal Cost (Black):** The area outside the main `boundary` polygon and the area inside all `keepout` polygons.
    *   **Medium Cost (Grey):** The area inside any `travel_area` polygons.
    *   **Free Space (White):** The remaining area inside the `boundary` polygon.

### **FR3: Single EKF (UTM Frame) Configuration**
*   The dual-EKF system **must** be replaced with a single `robot_localization` `ekf_node`.
*   The EKF's `world_frame` **must** be set to `utm`.
*   The EKF **must** be configured to fuse data from four sources:
    1.  **Wheel Odometry:** For high-frequency relative velocity updates.
    2.  **IMU:** For high-frequency relative orientation and angular velocity updates.
    3.  **GPS (UTM Position):** For authoritative absolute position updates (from `navsat_transform_node`).
    4.  **GPS (True Heading):** For authoritative absolute orientation updates (from the simulated heading node).
*   The primary output of this node **must** be the `utm` -> `base_link` transform.

### **FR4: Nav2 Stack Integration in UTM Frame**
*   The system **must** launch and manage the core ROS 2 Navigation Stack (Nav2).
*   All components of the Nav2 stack (Map Server, Planner Server, Controller Server, BT Navigator, Costmaps) **must** be configured to use `map` as their global frame.
*   The Nav2 stack **must** be configured with the following specific components:
    *   **Global Planner:** `SmacPlannerHybrid`
    *   **Local Planner / Controller:** `DWBController`
*   The Nav2 costmap **must** be configured to load the georeferenced map generated by the "Drive-to-Record" process.

### **FR5: Navigation Guard and Lifecycle Management**
*   A "Map/Bounds Guard" node **must** supervise the Nav2 stack.
*   **Startup Logic:** The guard node **must** prevent the Nav2 stack from activating until a valid map exists and the robot's initial UTM pose is within the map's UTM boundaries.
*   **Runtime Logic:** If the robot's UTM pose moves outside the map boundaries, the guard node **must** actively send a service call to **PAUSE** the Nav2 lifecycle nodes.
*   **Status Reporting:** The guard node **must** publish a `mower_msgs/msg/NavStatus` message to a latched topic `/mower/nav_status`, reporting the system's readiness and a reason string.

## **6. Non-Functional Requirements**

### **NFR1: Coordinate Frame Standards**
*   The system's primary transform tree **must** be `utm` -> `base_link`. 

### **NFR2: Extensibility**
*   The Nav2 costmap configuration **must** use a layered approach (e.g., `StaticLayer`, `InflationLayer`). This is a mandatory architectural requirement to ensure that future real-time obstacle avoidance sensors can be easily integrated.

### **NFR3: Robustness**
*   The system **must** handle off-nominal startup conditions (missing map, out-of-bounds start) without crashing.
*   The `autostart` parameter in the Nav2 configuration **must** be set to `false` to enable external lifecycle management by the guard node.

## **7. Design Considerations / Mockups**

This phase is purely for backend and simulation development. The primary visual interface for testing and validation will be RViz. The `/mower/nav_status` topic is designed for future integration into a user-facing UI in Phase V.

## **8. Success Metrics**

The successful completion of Phase 3 will be verified by a formal, multi-part acceptance test.

*   **Setup:**
    1.  Use the `/mower/manage_recording` service to record and generate a georeferenced map.
    2.  Launch the full navigation stack.
    3.  In RViz, set the "Fixed Frame" to `utm`. Display the robot's pose, the Nav2 costmap, and the global plan topic.
*   **Test A: Valid Path in Free Space**
    *   **Action:** Use the "2D Nav Goal" tool in RViz to set a goal in an open area of the map.
    *   **Expected Outcome:** Nav2 successfully generates a smooth path to the goal, and the robot navigates to the destination.
*   **Test B: Path Respecting Keep-Out Zone**
    *   **Action:** Set a goal that requires the robot to navigate around the defined keep-out zone.
    *   **Expected Outcome:** The generated global path **must not** enter the lethal cost area of the keep-out zone.
*   **Test C: Invalid Goal Handling**
    *   **Action:** Set a goal either inside a keep-out zone or outside the main boundary.
    *   **Expected Outcome:** The planner **must** fail to find a valid path. The robot **must not** move.
*   **Test D: Navigation Guard Validation**
    *   **Scenario 1 (Missing Map):** Launch the simulation without a map.
        *   **Expected Outcome:** Nav2 nodes remain inactive. `/mower/nav_status` reports `ready=false, reason="waiting_for_map"`.
    *   **Scenario 2 (Out-of-Bounds Start):** Launch with a map but with the robot's starting UTM position outside the map's boundaries.
        *   **Expected Outcome:** Nav2 nodes remain inactive. `/mower/nav_status` reports `ready=false, reason="robot_outside_map"`.
    *   **Scenario 3 (Runtime Out-of-Bounds):** Start normally and drive the robot outside the map boundary.
        *   **Expected Outcome:** The Nav2 nodes are automatically paused.

## **9. Out of Scope / Future Considerations**

To ensure Phase 3 remains focused, the following capabilities are explicitly **out of scope** for this PRD:

*   **Full Area Coverage:** Point-to-point navigation is in scope, but executing a full mowing pattern is out of scope and will be addressed in Phase IV.
*   **Real-Time Obstacle Avoidance:** The system will only navigate based on its pre-defined map.
*   **Automatic "Resume" Logic:** The guard will pause Nav2, but a policy for automatically resuming is out of scope until the UI is developed in Phase V.
*   **Web User Interface:** All interactions for this phase will be via ROS 2 services and RViz.
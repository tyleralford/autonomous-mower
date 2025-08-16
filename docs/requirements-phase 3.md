# **Product Requirements Document: Autonomous Mower - Phase 3**

## **1. Introduction / Overview**

This document provides a complete set of requirements for **Phase 3: User-Defined Mapping and GPS-Based Navigation** of the Autonomous Mower Development Plan. This phase builds upon the robust state estimation system developed in Phase 2. The objective is to empower the robot with the ability to create its own operational map through a manual recording process and to autonomously navigate between points within this map using the ROS 2 Navigation Stack (Nav2).

This PRD is a self-contained specification that details all functional requirements, architectural decisions, and success criteria necessary for a developer to successfully implement this phase.

## **2. Goals / Objectives**

The principal objective of Phase 3 is to **develop a complete, GPS-based autonomous navigation system that operates within user-defined boundaries.** The final output will be a system where a developer can:
1.  Manually drive the robot to record operational zones (boundaries, keep-out areas).
2.  Have the system automatically generate a persistent, GPS-anchored navigation map from these recordings.
3.  Command the robot to autonomously navigate to any valid point within the defined map, with the robot correctly respecting all zone limitations.

## **3. Target Audience / User Personas**

*   **User:** Internal Robotics Developer
*   **Role & Goal:** The developer is the primary user of the Phase 3 deliverable. Their goal is to build and validate the core autonomous navigation capabilities of the robot. They need to create a system that can understand its operational environment via manual definition and reliably navigate within it, setting the stage for task-specific logic (like mowing) in later phases.

## **4. User Stories / Use Cases**

*   "As a developer, I want to run a service to start and stop recording the robot's path to define distinct operational zones (boundaries, keepouts, travel paths)."
*   "As a developer, I want the system to automatically process my recorded zone files into a persistent, GPS-anchored Nav2 map as soon as I finish recording."
*   "As a developer, I want to launch the Nav2 stack, have it load my custom map, and see that the robot's GPS-based position is correctly localized within it, regardless of where the robot started."
*   "As a developer, I want to send a navigation goal in RViz and have the robot plan and execute a path that respects all defined zones."

## **5. Functional Requirements**

### **FR1: "Drive-to-Record" Zone Creation**
*   A ROS 2 service named `/mower/manage_recording` **must** be created to handle the recording of operational zones.
*   The service **must** be of a custom type, `ManageRecording.srv`, with the following definition:
    *   **Request:** `uint8 action` (e.g., `START=0`, `STOP=1`), `uint8 area_type` (e.g., `BOUNDARY=0`, `KEEPOUT=1`, `TRAVEL=2`), `string filename`.
    *   **Response:** `bool success`, `string message`.
*   When a recording session is active, a node **must** subscribe to the robot's global pose (`/odometry/filtered/global`) and save the sequence of poses to the specified file (e.g., in `.csv` format).

### **FR2: Automated Map Generation**
*   The `/mower/manage_recording` service node **must** trigger an internal map generation function immediately upon the successful completion of a `STOP` action.
*   The map generation function **must** process all existing recorded zone files (`boundary`, `keepout`, `travel`) to produce a single, consolidated navigation map.
*   The output **must** be a standard Nav2 map, consisting of:
    *   A `.pgm` image file with pixel values corresponding to cost.
    *   A `.yaml` file containing the map's metadata (resolution, origin, etc.).
*   The cost hierarchy in the `.pgm` file **must** be as follows:
    *   **Lethal Cost (Black):** The area outside the main `boundary` polygon and the area inside all `keepout` polygons.
    *   **Medium Cost (Grey):** The area inside any `travel_area` polygons.
    *   **Free Space (White):** The remaining area inside the `boundary` polygon.

### **FR3: Persistent GPS-Map Anchor**
*   The map generation process **must** also save the `navsat_transform_node`'s datum (the GPS origin of the map) to a persistent file.
*   The main navigation launch file **must** be configured to load this saved datum on startup, ensuring the `map` frame is consistently anchored to the same real-world GPS coordinates across all sessions.

### **FR4: Nav2 Stack Integration**
*   The system **must** launch and manage the core ROS 2 Navigation Stack (Nav2).
*   Nav2 **must** be configured to get the robot's global pose directly from the EKF's topic: `/odometry/filtered/global`. No other localization source (e.g., AMCL) shall be used.
*   The Nav2 stack **must** be configured with the following specific components:
    *   **Global Planner:** `SmacPlannerHybrid`
    *   **Local Planner / Controller:** `DWBController`
*   The Nav2 costmap **must** be configured to load the map generated by the "Drive-to-Record" process.

## **6. Non-Functional Requirements**

### **NFR1: Extensibility**
*   The Nav2 costmap configuration **must** use a layered approach (e.g., `StaticLayer` for the generated map, `InflationLayer` for buffering). This is a mandatory architectural requirement to ensure that future real-time obstacle avoidance sensors can be easily integrated by adding a new layer without re-architecting the navigation system.

### **NFR2: Usability**
*   The map generation process **must** be fully automated and integrated into the system. It shall not require any manual, offline steps from the developer after a zone is recorded.

### **NFR3: Robust Startup Behavior**
*   The system **must** start gracefully when a map is missing or when the robot's initial pose is outside the map bounds. Nav2 shall not crash or enter a bad lifecycle state; instead, a status shall indicate the reason Nav2 is inactive and automatically start Nav2 when conditions become valid.

## **7. Design Considerations / Mockups**

This phase is purely for backend and simulation development. No graphical user interface (GUI) is required. The primary visual interface for testing and validation will be RViz.

## **8. Success Metrics**

The successful completion of Phase 3 will be verified by a formal, three-part acceptance test.

*   **Setup:**
    1.  Use the `/mower/manage_recording` service to record and generate a map with a main boundary and at least one internal keep-out zone.
    2.  Launch the full navigation stack.
    3.  In RViz, display the robot's pose, the Nav2 costmap, and the global plan topic.
*   **Test A: Valid Path in Free Space**
    *   **Action:** Use the "2D Nav Goal" tool in RViz to set a goal in an open area of the map.
    *   **Expected Outcome:** Nav2 successfully generates a smooth path to the goal, and the robot navigates to the destination.
*   **Test B: Path Respecting Keep-Out Zone**
    *   **Action:** Set a goal that requires the robot to navigate around the defined keep-out zone.
    *   **Expected Outcome:** The generated global path **must not** enter the lethal cost area of the keep-out zone. The robot successfully navigates to the destination.
*   **Test C: Invalid Goal Handling**
    *   **Action:** Set a goal either inside a keep-out zone or outside the main boundary.
    *   **Expected Outcome:** The planner **must** fail to find a valid path, and an error message should be logged. The robot **must not** move.

*   **Test D: Graceful Startup Without Map**
    *   **Action:** Delete/map not present at startup.
    *   **Expected Outcome:** Nav2 nodes remain inactive without error spam; a system status indicates `waiting_for_map`. When the map files are provided during runtime, Nav2 autostarts and becomes operational.

*   **Test E: Robot Outside Map Bounds**
    *   **Action:** Start with the robot's global pose outside the generated map extents.
    *   **Expected Outcome:** Nav2 remains inactive with status `robot_outside_map`. After the robot is within bounds (or datum corrected), Nav2 autostarts and normal navigation proceeds.

## **9. Out of Scope / Future Considerations**

To ensure Phase 3 remains focused, the following capabilities are explicitly **out of scope** for this PRD:

*   **Full Area Coverage:** Point-to-point navigation is in scope, but executing a full mowing pattern that covers an entire area is out of scope and will be addressed in Phase IV.
*   **Real-Time Obstacle Avoidance:** The system will only navigate based on its pre-defined map. It will not have sensors or logic to react to unexpected obstacles.
*   **Web User Interface:** All interactions for this phase will be via ROS 2 services and RViz. The user-facing UI is part of Phase V.
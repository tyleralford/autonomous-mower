# **Product Requirements Document: Autonomous Mower - Phase 4**

## **1. Introduction / Overview**

This document provides a complete set of requirements for **Phase 4: Mission Control and Coverage Path Planning** of the Autonomous Mower Development Plan. This phase builds upon the robust point-to-point navigation system developed in Phase 3. The objective is to implement the high-level intelligence or "brain" of the robot, enabling it to accept, plan, and execute a complete, autonomous mowing mission from start to finish.

This will be accomplished using a reactive Behavior Tree architecture, which will manage the robot's state, handle real-time events, and execute the complex sequence of actions required for a full mowing job. This PRD is a self-contained specification for a developer to implement this phase.

## **2. Goals / Objectives**

The principal objective of Phase 4 is to **develop a robust and reactive mission control system capable of planning and executing a complete, configurable, end-to-end mowing task.** The final output will be a system where a developer can:
1.  Initiate a mowing job with a single command, specifying a target area and detailed mowing parameters.
2.  Trust the robot to autonomously navigate to the area, perform a full coverage mowing pattern with perimeters, and return to its starting point.
3.  Observe the robot reacting appropriately to external commands (e.g., Pause/Resume) and internal state changes (e.g., Low Battery) during the mission.

## **3. Target Audience / User Personas**

*   **User:** Internal Robotics Developer
*   **Role & Goal:** The developer is the primary user of the Phase 4 deliverable. Their goal is to implement and validate the core task-oriented logic of the robot. They need to create a system that can translate a high-level command ("mow this area") into a sequence of navigation and hardware actions, while safely handling a wide range of potential interruptions and failures.

## **4. User Stories / Use Cases**

*   "As a developer, I want to send a single ROS 2 Action goal to command the robot to perform a complete mowing job with specific parameters like overlap, pattern angle, and perimeters."
*   "As a developer, I want the robot to provide continuous feedback during a mowing job, reporting its current status (e.g., 'mowing perimeter') and the overall progress."
*   "As a developer, I want the mission controller to be architected as a hierarchical Behavior Tree, so its logic is modular, scalable, and easy to debug."
*   "As a developer, I want the robot to automatically pause its mission and return to a dock if its battery level drops below a critical threshold."
*   "As a developer, I want the robot to safely handle a loss of GPS signal or a navigation failure by pausing, retrying, and reporting an error if it cannot recover."
*   "As a developer, I want to be able to pause, resume, or completely stop a mowing job at any time using ROS 2 commands."

## **5. Functional Requirements**

### **FR1: "Start Mowing" ROS 2 Action**
*   A comprehensive ROS 2 Action named `StartMowing.action` **must** be created to serve as the primary interface for initiating and managing mowing jobs.
*   The Action definition **must** include:
    *   **Goal:** The `mow_area` polygon and all configurable mowing parameters: `float32 overlap_percent`, `float32 pattern_angle`, `uint8 num_perimeters`, etc.
    *   **Result:** A `bool success` flag and a `string message`.
    *   **Feedback:** `float32 percent_complete` and `string current_status`.

### **FR2: Behavior Tree (BT) Mission Controller**
*   The high-level mission logic **must** be implemented using a Behavior Tree, leveraging the `BehaviorTree.CPP v4` library.
*   A new `mower_mission_control` package **must** be created to house the BT logic, XML files, and custom C++ BT nodes.
*   The BT architecture **must** be hierarchical and reactive. High-level nodes that check for critical events (e.g., Pause commanded, Low Battery) **must** be able to interrupt the main mowing sequence.
*   The main mowing sequence **must** be: Navigate to Mow Area -> Execute Mowing Pattern -> Navigate to Dock.

### **FR3: Custom Behavior Tree Nodes**
*   A library of custom C++ BT nodes **must** be developed to connect the BT to the ROS 2 ecosystem. This library **must** include, at a minimum:
    *   **Conditions:** `IsBatteryLow`, `IsGpsGood`, `IsJobPaused`.
    *   **Actions:** `GenerateCoveragePath`, `FollowWaypoints` (interfaces with Nav2's Waypoint Follower), `NavigateToPose` (interfaces with Nav2's NavigateToPose action), `EngageCutter`, `DisengageCutter`, `ReportError`.

### **FR4: Coverage Path Planner**
*   The `GenerateCoveragePath` BT node **must** implement a Boustrophedon ("back-and-forth") coverage algorithm.
*   **Input:** The node **must** accept the `mow_area` polygon and all mowing parameters from the `StartMowing` action goal.
*   **Output:** The node **must** produce an ordered list of `geometry_msgs/PoseStamped` waypoints, which will be passed to the `FollowWaypoints` action.

### **FR5: Mission Reactivity and Error Handling**
*   The mission control system **must** correctly handle the following events:
    *   **Pause/Resume:** An external command can pause the current action and all motion. A subsequent command can resume the mission from where it left off.
    *   **Low Battery:** If the battery state falls below a configurable threshold, the robot **must** cancel the current mowing job, disengage its cutter, and execute a "Return to Dock" sequence.
    *   **Poor/Lost GPS:** If the GPS fix quality degrades below a configurable threshold, the robot **must** pause all motion and wait for the signal to recover.
    *   **Navigation Failure:** If a call to a Nav2 action (e.g., `NavigateToPose` or `FollowWaypoints`) fails, the system **must** retry the action a configurable number of times (e.g., 3 times) before aborting the entire job and reporting an error.
    *   **Stop/Reset:** An external command can completely cancel the current job and return the BT to an idle state.

## **6. Non-Functional Requirements**

### **NFR1: Hardware Abstraction & Sim-to-Real**
*   To ensure a smooth transition to hardware, the BT nodes **must** interact with sensors through standardized ROS 2 interfaces, not direct simulation data.
*   **Battery Interface:** The `IsBatteryLow` node **must** get battery data by subscribing to the `/battery_state` topic, which publishes standard `sensor_msgs/BatteryState` messages.
*   **GPS Interface:** The `IsGpsGood` node **must** get GPS quality data by subscribing to the `/gps/fix` topic, which publishes standard `sensor_msgs/NavSatFix` messages.

### **NFR2: Simulation Support**
*   To enable development and testing without physical hardware, a new "Battery Simulator" node **must** be created. This node's sole responsibility is to publish fake `sensor_msgs/BatteryState` messages to the `/battery_state` topic, with a service to allow a developer to manually set the battery level for testing purposes.

## **7. Design Considerations / Mockups**

This phase is purely for backend and mission logic development. No graphical user interface (GUI) is required. All interactions (starting jobs, pausing, etc.) will be performed via ROS 2 command-line tools (`ros2 action send_goal`, etc.). The `StartMowing.action` feedback is designed for future consumption by the UI in Phase V.

## **8. Success Metrics**

The successful completion of Phase 4 will be verified by a formal, multi-part acceptance test.

*   **Setup:**
    1.  Launch the full system, including the new `mower_mission_control_node` and the `battery_simulator_node`.
    2.  Use a pre-recorded map from Phase 3.
*   **Test A: Full Mission Execution**
    *   **Action:** Use `ros2 action send_goal` to start a mowing job with specific parameters (e.g., 2 perimeters, 45-degree angle).
    *   **Expected Outcome:** The robot successfully navigates to the start of the mow area, executes the complete mowing pattern (perimeters first, then Boustrophedon fill), and returns to its starting dock. The action's feedback topic must continuously report status and progress.
*   **Test B: Pause and Resume**
    *   **Action:** During the execution of the mowing pattern from Test A, send a "Pause" command, wait at least 5 seconds, then send a "Resume" command.
    *   **Expected Outcome:** The robot stops all motion and pauses its BT when commanded. It successfully resumes the mowing pattern from the point of interruption.
*   **Test C: Low Battery Abort**
    *   **Action:** Start a new mowing job. While the robot is mowing, use the battery simulator's service to set the battery level to a "low" state.
    *   **Expected Outcome:** The robot immediately cancels the mowing job, disengages its cutter, and autonomously navigates back to its starting dock.

## **9. Out of Scope / Future Considerations**

*   **Real-Time Obstacle Avoidance:** The coverage planner and mission controller will operate based on the static map from Phase 3 only.
*   **Web User Interface:** All interactions are via the command line. The UI for mission control is part of Phase V.
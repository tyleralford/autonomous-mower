## **Autonomous Mower: Phase 4 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 4 of the Autonomous Mower project. It is designed to be followed sequentially by a developer. Each task builds upon the previous one, with mandatory testing checkpoints to ensure system integrity at every stage.

### **Module 0: Project Setup**

This initial module prepares the codebase for Phase 4 development.

- [ ] **Task 0.1:** **Create and Checkout a New Branch**
    - **Context:** Isolate all Phase 4 development on a dedicated branch.
    - **Sub-Task 0.1.1:** Pull the latest changes from the `main` branch.
    - **Sub-Task 0.1.2:** Create and check out a new branch named `feature/phase-4-mission-control`.
    - **Sub-Task 0.1.3:** Push the new branch to the remote repository.

### **Module 1: Foundational Interfaces and Packages**

This module creates the scaffolding for the mission control system: the ROS 2 Action definition and the C++ package that will house the logic.

- [ ] **Task 1.1:** **Create the `StartMowing` Action Message**
    - **Dependencies:** 0.1
    - **Context:** Define the primary ROS 2 interface for starting and managing mowing jobs.
    - **Sub-Task 1.1.1:** In the `mower_msgs` package, create an `action/` directory.
    - **Sub-Task 1.1.2:** Create a new file `StartMowing.action` with the definition specified in the PRD (Goal, Result, Feedback).
    - **Sub-Task 1.1.3:** Update `mower_msgs/CMakeLists.txt` and `package.xml` to build the new action message.
    - **Sub-Task 1.1.4:** Build the workspace to verify the message is generated correctly.
    - **Sub-Task 1.1.5:** Commit your work. (`git commit -m "feat(msgs): Create StartMowing action definition"`)

- [ ] **Task 1.2:** **Create the Mission Control Package**
    - **Dependencies:** 1.1
    - **Context:** Create the C++ package that will contain the Behavior Tree (BT) nodes and the main mission control executable.
    - **Sub-Task 1.2.1:** In `mower_ws/src`, create a new package: `ros2 pkg create --build-type ament_cmake mower_mission_control`.
    - **Sub-Task 1.2.2:** Inside `mower_mission_control`, create the following directory structure: `src/`, `include/mower_mission_control/`, and `bt_xml/`.
    - **Sub-Task 1.2.3:** Commit the new package structure. (`git commit -m "feat: Create mower_mission_control package"`)

### **Module 2: Simulation Support**

This module creates the necessary simulation nodes to test reactive behaviors without hardware.

- [ ] **Task 2.1:** **Implement the Battery Simulator Node**
    - **Dependencies:** 1.2
    - **Context:** Create a simple node to simulate the robot's battery, allowing for testing of the `IsBatteryLow` condition.
    - **Sub-Task 2.1.1:** In `mower_simulation/scripts`, create a new Python node `battery_simulator_node.py`.
    - **Sub-Task 2.1.2:** The node must publish `sensor_msgs/BatteryState` messages to the `/battery_state` topic at a regular interval (e.g., 1 Hz).
    - **Sub-Task 2.1.3:** The node must host a service (e.g., `/mower/set_battery_level`) that allows a developer to set the simulated battery voltage or percentage.
    - **Sub-Task 2.1.4:** Add the new node to `mower_simulation/setup.py` and integrate it into the main `sim.launch.py`.
    - **Sub-Task 2.1.5:** Commit the new simulator node. (`git commit -m "feat(simulation): Implement battery simulator node"`)

- [ ] **MANDATORY TEST 2.A: Verify Battery Simulation**
    - **Context:** Ensure the battery simulator is working correctly before it is used as a dependency for testing the BT. **This test cannot be skipped.**
    - **Procedure:**
        1.  Launch the full simulation.
        2.  Check that the `/battery_state` topic is being published.
        3.  Call the `/mower/set_battery_level` service to set a new battery level.
    - **Expected Outcome:** The messages published on `/battery_state` should reflect the new value set via the service call.

### **Module 3: Core Behavior Tree Implementation**

This module implements the main BT runner and the library of custom nodes that will form the building blocks of the mission logic.

- [ ] **Task 3.1:** **Implement the Main Mission Control Node**
    - **Dependencies:** 2.1
    - **Context:** Create the main C++ node that will load, register, and "tick" the Behavior Tree.
    - **Sub-Task 3.1.1:** In `mower_mission_control/src`, create `mission_control_node.cpp`.
    - **Sub-Task 3.1.2:** This node should use the `BehaviorTreeFactory` to register custom nodes (which will be created next) and load a BT definition from an XML file.
    - **Sub-Task 3.1.3:** Create a timer that "ticks" the loaded tree at a regular frequency (e.g., 10 Hz).
    - **Sub-Task 3.1.4:** In `mower_mission_control/bt_xml`, create a simple `mower_main.xml` with a placeholder node (e.g., a `PrintMessage` node) to test the loading mechanism.
    - **Sub-Task 3.1.5:** Add the executable to the `CMakeLists.txt` and integrate it into the main `sim.launch.py`.

- [ ] **MANDATORY TEST 3.A: Verify BT Loading and Ticking**
    - **Context:** Ensure the core BT engine is running before adding complex custom nodes. **This test cannot be skipped.**
    - **Procedure:**
        1.  Launch the full simulation.
    - **Expected Outcome:** The `mission_control_node` starts without errors. The placeholder message from the simple BT XML is printed to the console repeatedly, confirming the tree is being ticked.

- [ ] **Task 3.2:** **Implement Simple Condition and Action Nodes**
    - **Dependencies:** 3.1
    - **Context:** Create the first set of custom C++ BT nodes for basic checks and actions.
    - **Sub-Task 3.2.1:** Create header and source files for each new node (e.g., `is_battery_low.hpp`, `is_gps_good.hpp`, `engage_cutter.hpp`).
    - **Sub-Task 3.2.2:** Implement the nodes. `IsBatteryLow` and `IsGpsGood` will subscribe to their respective topics and return `SUCCESS` or `FAILURE`. `EngageCutter` will log a message (functionality will be expanded later).
    - **Sub-Task 3.2.3:** Export these nodes as plugins using `pluginlib` and update the `CMakeLists.txt` and `package.xml` accordingly.
    - **Sub-Task 3.2.4:** Register the new nodes in `mission_control_node.cpp`.
    - **Sub-Task 3.2.5:** Commit the new nodes. (`git commit -m "feat(bt_nodes): Implement simple condition and action nodes"`)

- [ ] **Task 3.3:** **Implement Nav2 Interfacing Action Nodes**
    - **Dependencies:** 3.2
    - **Context:** Create the complex BT nodes that interface with Nav2's action servers.
    - **Sub-Task 3.3.1:** Create nodes for `NavigateToPose` and `FollowWaypoints`.
    - **Sub-Task 3.3.2:** Implement these nodes by inheriting from the `nav2_behavior_tree::BtActionNode` base class, which simplifies interfacing with ROS 2 Action servers.
    - **Sub-Task 3.3.3:** Export and register these nodes as plugins.
    - **Sub-Task 3.3.4:** Commit the Nav2 nodes. (`git commit -m "feat(bt_nodes): Implement Nav2 interfacing nodes"`)

- [ ] **MANDATORY TEST 3.B: Verify Custom Node Functionality**
    - **Context:** Test each new node in isolation using simple test trees. **This test cannot be skipped.**
    - **Procedure:**
        1.  Create temporary, simple BT XML files for testing each node (e.g., `test_nav.xml` that just calls `NavigateToPose`).
        2.  Launch the system and send a navigation goal via the test tree.
        3.  Use the battery simulator to trigger the `IsBatteryLow` condition and verify the BT logic branches correctly.
    - **Expected Outcome:** The `NavigateToPose` node successfully commands the robot to move. The condition nodes correctly return `SUCCESS`/`FAILURE` based on simulated sensor data.

### **Module 4: Coverage Path Planner**

This module implements the core logic for generating the mowing pattern.

- [ ] **Task 4.1:** **Implement the `GenerateCoveragePath` BT Node**
    - **Dependencies:** 3.3
    - **Context:** Create the C++ BT node that contains the Boustrophedon path generation algorithm.
    - **Sub-Task 4.1.1:** Create the `generate_coverage_path.hpp` and `.cpp` files.
    - **Sub-Task 4.1.2:** The node must read the mowing area polygon and all mowing parameters from the BT's input blackboard.
    - **Sub-Task 4.1.3:** Implement the logic to generate the perimeter paths based on the `num_perimeters` parameter.
    - **Sub-Task 4.1.4:** Implement the Boustrophedon decomposition algorithm to generate the back-and-forth lanes for the remaining area.
    - **Sub-Task 4.1.5:** Combine all paths into a single, ordered list of `geometry_msgs/PoseStamped` waypoints.
    - **Sub-Task 4.1.6:** Write the final waypoint list to the BT's output blackboard.
    - **Sub-Task 4.1.7:** Export and register this node as a plugin.
    - **Sub-Task 4.1.8:** Commit the planner node. (`git commit -m "feat(bt_nodes): Implement GenerateCoveragePath node"`)

- [ ] **MANDATORY TEST 4.A: Verify Path Generation**
    - **Context:** Ensure the planner can generate a valid and correct path before attempting to execute it. **This test cannot be skipped.**
    - **Procedure:**
        1.  Create a test BT XML that runs only the `GenerateCoveragePath` node with hardcoded input parameters.
        2.  Launch the system and tick the test tree.
        3.  The node should log the generated waypoints or allow inspection via debug topics.
        4.  Visualize the generated waypoints in RViz to confirm the pattern is correct.
    - **Expected Outcome:** For a given input polygon, the node generates a valid, non-empty list of waypoints that correctly represents the desired perimeter and Boustrophedon pattern.

### **Module 5: Full Mission Integration**

This module assembles all the created components into the final, reactive mission logic.

- [ ] **Task 5.1:** **Assemble the Full Behavior Tree**
    - **Dependencies:** 4.1
    - **Context:** Update `mower_main.xml` to implement the complete, hierarchical, and reactive mission logic described in the PRD.
    - **Sub-Task 5.1.1:** Use a high-level `ReactiveFallback` or `ReactiveSequence` to constantly check for events like "Pause," "Low Battery," or "Bad GPS."
    - **Sub-Task 5.1.2:** Implement the main mowing sequence (`NavigateToPose` -> `GenerateCoveragePath` -> `FollowWaypoints` -> `NavigateToPose`) as a subtree that can be interrupted by the reactive layer.
    - **Sub-Task 5.1.3:** Implement the logic for handling Pause/Resume and Stop commands using blackboard variables.
    - **Sub-Task 5.1.4:** Implement the navigation failure recovery logic (e.g., using a `Retry` decorator node).

- [ ] **Task 5.2:** **Implement the `StartMowing` Action Server**
    - **Dependencies:** 5.1
    - **Context:** Implement the ROS 2 Action server in the `mission_control_node` that acts as the entry point for all jobs.
    - **Sub-Task 5.2.1:** In `mission_control_node.cpp`, create an action server for `StartMowing`.
    - **Sub-Task 5.2.2:** The `handle_goal` callback should accept or reject a new job.
    - **Sub-Task 5.2.3:** The `handle_accepted` callback should take the goal parameters, write them to the BT blackboard, and begin ticking the main tree.
    - **Sub-Task 5.2.4:** While the tree is running, the node must periodically read status from the blackboard and publish it as action feedback.
    - **Sub-Task 5.2.5:** When the BT finishes (returns `SUCCESS` or `FAILURE`), the action server must report the final result.
    - **Sub-Task 5.2.6:** Commit the fully integrated mission logic. (`git commit -m "feat(mission_control): Integrate full BT logic and action server"`)

### **Module 6: Final Validation**

This module performs the final acceptance test as defined in the PRD.

- [ ] **Task 6.1:** **Perform Multi-Part Acceptance Test**
    - **Dependencies:** 5.2
    - **Context:** This is the final end-to-end test for Phase 4, validating the entire mission control system.
    - **Sub-Task 6.1.1:** **Execute Test A (Full Mission):** Use `ros2 action send_goal` to start a complete mowing job.
    - **Sub-Task 6.1.2:** **Execute Test B (Pause/Resume):** During the mission, use a separate command-line call to set a "paused" flag on the BT blackboard and then clear it.
    - **Sub-Task 6.1.3:** **Execute Test C (Low Battery Abort):** During the mission, use the battery simulator's service to trigger a low-battery state.
    - **Sub-tASK 6.1.4:** Document the results of all three tests with screenshots and terminal logs.

- [ ] **Task 6.2:** **Finalize and Merge**
    - **Dependencies:** 6.1
    - **Context:** Clean up the feature branch and merge it into the main branch, completing the phase.
    - **Sub-Task 6.2.1:** Review all code for clarity and comments.
    - **Sub-Task 6.2.2:** Update the `README.md` with instructions on how to start a mowing job and interact with the mission controller.
    - **Sub-Task 6.2.3:** Create a Pull Request on GitHub from `feature/phase-4-mission-control` to `main`, including validation artifacts.
    - **Sub-Task 6.2.4:** After review, merge the pull request. Phase 4 is now complete.
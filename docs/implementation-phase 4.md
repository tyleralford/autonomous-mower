## **Autonomous Mower: Phase 4 Implementation Plan**

This document provides a detailed, step-by-step plan for executing Phase 4 of the Autonomous Mower project. It is designed to be followed sequentially by a junior developer. Each task builds upon the previous one, with mandatory testing checkpoints to ensure system integrity at every stage.

### **Module 0: Project Setup and Interfaces**

This module establishes the foundational packages and ROS 2 interfaces for the mission control system.

- [ ] **Task 0.1:** **Create and Checkout a New Branch**
    - **Context:** Isolate all Phase 4 development on a dedicated branch.
    - **Sub-Task 0.1.1:** Pull the latest changes from the `main` branch.
    - **Sub-Task 0.1.2:** Create and check out a new branch named `feature/phase-4-mission-control`.
    - **Sub-Task 0.1.3:** Push the new branch to the remote repository.

- [ ] **Task 0.2:** **Define the "Start Mowing" Action**
    - **Dependencies:** 0.1
    - **Context:** Create the custom ROS 2 Action that will be the primary entry point for starting and managing mowing jobs.
    - **Sub-Task 0.2.1:** In the `mower_msgs` package, create an `action/` directory.
    - **Sub-Task 0.2.2:** Create a new file `StartMowing.action` with the full definition specified in the PRD (goal, result, feedback).
    - **Sub-Task 0.2.3:** Update `mower_msgs/CMakeLists.txt` and `package.xml` to build the new action message.
    - **Sub-Task 0.2.4:** Build the workspace (`colcon build`) and verify the new message interfaces are generated correctly (`ros2 interface show mower_msgs/action/StartMowing`).
    - **Sub-Task 0.2.5:** Commit your work. (`git commit -m "feat(msgs): Create StartMowing action definition"`)

- [ ] **Task 0.3:** **Create Mission Control Package**
    - **Dependencies:** 0.2
    - **Context:** Create the new C++ package that will house all the mission logic, including the Behavior Tree nodes.
    - **Sub-Task 0.3.1:** In `mower_ws/src`, create a new ROS 2 package: `ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclcpp_action nav2_msgs mower_msgs behaviortree_cpp_v4 --node-name mission_control_node mower_mission_control`.
    - **Sub-Task 0.3.2:** Inside `mower_mission_control`, create the following directory structure: `bt_xml/` for Behavior Tree definitions and `include/mower_mission_control/` and `src/` for C++ node source files.
    - **Sub-Task 0.3.3:** Commit the new package structure. (`git commit -m "feat(mission): Create mower_mission_control package"`)

### **Module 1: Hardware Simulators and Interfaces**

This module creates the simulated hardware interfaces required to test the mission logic without a physical robot.

- [ ] **Task 1.1:** **Implement Battery Simulator Node**
    - **Dependencies:** 0.3
    - **Context:** To test the `IsBatteryLow` condition, we need a way to simulate the battery's state. This node will publish fake battery data and provide a service to control it during tests.
    - **Sub-Task 1.1.1:** In a new package (e.g., `mower_simulation`), create a Python node `battery_simulator_node.py`.
    - **Sub-Task 1.1.2:** The node should continuously publish a `sensor_msgs/BatteryState` message to the `/battery_state` topic.
    - **Sub-Task 1.1.3:** Create a ROS 2 service (e.g., `/mower/set_battery_level`) that allows a developer to send a float value (e.g., 0.0 to 1.0) to set the `percentage` field of the published message.
    - **Sub-Task 1.1.4:** Add the node to the main `sim.launch.py` so it starts with the system.
    - **Sub-Task 1.1.5:** Commit the new simulator node. (`git commit -m "feat(simulation): Implement battery simulator node"`)

- [ ] **MANDATORY TEST 1.A: Verify Hardware Interfaces**
    - **Context:** Before building any mission logic, ensure the required sensor topics are available and behaving as expected. **This test cannot be skipped.**
    - **Procedure:**
        1.  Build and source the workspace.
        2.  Launch the main simulation: `ros2 launch mower_bringup sim.launch.py`.
        3.  In a new terminal, check the topic list for `/battery_state` and `/gps/fix`.
        4.  Echo both topics and verify they are publishing the correct message types.
        5.  Call the `/mower/set_battery_level` service with a value of `0.5` and confirm the echoed `/battery_state` message updates accordingly.
    - **Expected Outcome:** All required sensor topics are active and publishing valid data, and the battery simulator is controllable.

### **Module 2: Core Behavior Tree Scaffolding**

This module focuses on creating the C++ node that runs the Behavior Tree and implementing the first, most basic navigation action.

- [ ] **Task 2.1:** **Implement the Behavior Tree Manager Node**
    - **Dependencies:** 1.1
    - **Context:** This C++ node is the "engine" that loads and runs our Behavior Tree. It will also host the `StartMowing` action server.
    - **Sub-Task 2.1.1:** In `mower_mission_control/src/mission_control_node.cpp`, set up a basic ROS 2 node.
    - **Sub-Task 2.1.2:** Use the `behaviortree_cpp_v4` library to create a `BT::BehaviorTreeFactory`.
    - **Sub-Task 2.1.3:** Implement the `StartMowing` action server. For now, the `handle_goal` callback can simply accept any new goal.
    - **Sub-Task 2.1.4:** In the `execute` callback of the action server, load a simple BT from an XML file (e.g., `mower_mission_control/bt_xml/main.xml`).
    - **Sub-Task 2.1.5:** Create a simple `main.xml` with only a `<BehaviorTree ID="MainTree"><AlwaysSuccess/></BehaviorTree>` tag.
    - **Sub-Task 2.1.6:** The node should "tick" the tree at a regular rate (e.g., 10 Hz) as long as the action is active.
    - **Sub-Task 2.1.7:** Add the node to the main `sim.launch.py`.
    - **Sub-Task 2.1.8:** Commit the BT manager node. (`git commit -m "feat(mission): Implement BT manager and action server"`)

- [ ] **Task 2.2:** **Create a "NavigateToPose" BT Action Node**
    - **Dependencies:** 2.1
    - **Context:** This is our first custom BT node. It will connect the Behavior Tree to the Nav2 stack, allowing the BT to command the robot to move. This is a critical integration point.
    - **Reference:** [Nav2 Documentation on BT Nodes](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2_bt_plugin.html), [BT.CPP ROS2 Integration](https://www.behaviortree.dev/docs/ros2_integration/)
    - **Sub-Task 2.2.1:** In `mower_mission_control`, create a new C++ class `NavigateToPoseNode` that inherits from `BT::RosActionNode<nav2_msgs::action::NavigateToPose>`.
    - **Sub-Task 2.2.2:** This node will take a goal pose from the BT's "blackboard" as an input port.
    - **Sub-Task 2.2.3:** In the `mission_control_node.cpp`, register this new custom node with the `BT::BehaviorTreeFactory` using a name like "NavigateToPose".
    - **Sub-Task 2.2.4:** Update `main.xml` to use this new node. For testing, you can hardcode a goal pose on the blackboard.
    - **Sub-Task 2.2.5:** Commit the new BT node. (`git commit -m "feat(mission): Implement NavigateToPose BT action node"`)

- [ ] **MANDATORY TEST 2.A: Verify BT-Controlled Navigation**
    - **Context:** This test is the "first light" for the mission controller, proving that the Behavior Tree can successfully command the robot to move using the Nav2 stack. **This test cannot be skipped.**
    - **Procedure:**
        1.  Build and source the workspace.
        2.  Launch the main simulation.
        3.  In a new terminal, send a simple goal to the `/mower/start_mowing` action server (the goal content doesn't matter yet).
        4.  Observe the robot's behavior in RViz.
    - **Expected Outcome:** Upon receiving the action goal, the Behavior Tree should tick the `NavigateToPose` node, which should in turn call the Nav2 action. The robot should autonomously navigate to the hardcoded goal pose in the simulation.

### **Module 3: Coverage Path Planning Logic**

This module implements the core algorithm for generating the mowing pattern.

- [ ] **Task 3.1:** **Implement Boustrophedon Planner**
    - **Dependencies:** 2.2
    - **Context:** Create the standalone, non-ROS logic for the coverage path planner. This makes it easy to test the algorithm in isolation.
    - **Sub-Task 3.1.1:** In a new utility library within `mower_mission_control`, create a function that implements the Boustrophedon decomposition algorithm.
    - **Sub-Task 3.1.2:** The function should take a polygon and all mowing parameters (overlap, angle, perimeters, etc.) as input.
    - **Sub-Task 3.1.3:** The function's output should be a list of poses (e.g., a `std::vector` of pose objects).
    - **Sub-Task 3.1.4:** Write a simple, non-ROS unit test to verify the planner's output for a simple rectangular polygon.
    - **Sub-Task 3.1.5:** Commit the planner library. (`git commit -m "feat(mission): Implement Boustrophedon coverage planner library"`)

- [ ] **Task 3.2:** **Create "GenerateCoveragePath" BT Node**
    - **Dependencies:** 3.1
    - **Context:** Wrap the planner logic in a new BT node so it can be called from the Behavior Tree.
    - **Sub-Task 3.2.1:** Create a new C++ class `GenerateCoveragePathNode` that inherits from `BT::SyncActionNode`.
    - **Sub-Task 3.2.2:** The node should take the mowing area polygon and all parameters from input ports on the blackboard.
    - **Sub-Task 3.2.3:** The node's `tick()` method should call the Boustrophedon planner function.
    - **Sub-Task 3.2.4:** It should write the resulting list of waypoints to an output port on the blackboard.
    - **Sub-Task 3.2.5:** Register this new node with the `BT::BehaviorTreeFactory`.
    - **Sub-Task 3.2.6:** Commit the new BT node. (`git commit -m "feat(mission): Implement GenerateCoveragePath BT node"`)

### **Module 4: Executing the Full Mowing Mission**

This module assembles the components into a complete, end-to-end mowing sequence.

- [ ] **Task 4.1:** **Create "FollowWaypoints" BT Action Node**
    - **Dependencies:** 3.2
    - **Context:** Create the BT node that will execute the path generated by the planner.
    - **Reference:** [Nav2 Documentation on FollowWaypoints](https://navigation.ros.org/tutorials/docs/navigation2_waypoints.html)
    - **Sub-Task 4.1.1:** Create a new C++ class `FollowWaypointsNode` that inherits from `BT::RosActionNode<nav2_msgs::action::FollowWaypoints>`.
    - **Sub-Task 4.1.2:** The node will take the list of waypoints from an input port on the blackboard.
    - **Sub-Task 4.1.3:** Register this new node with the `BT::BehaviorTreeFactory`.
    - **Sub-Task 4.1.4:** Commit the new BT node. (`git commit -m "feat(mission): Implement FollowWaypoints BT action node"`)

- [ ] **Task 4.2:** **Assemble the Mowing Sequence in BT XML**
    - **Dependencies:** 4.1
    - **Context:** Modify the main Behavior Tree XML to define the complete, sequential mowing logic.
    - **Sub-Task 4.2.1:** In `main.xml`, create a `<Sequence>` node for the main mowing task.
    - **Sub-Task 4.2.2:** The sequence should contain the following nodes in order:
        1.  `NavigateToPose` (to get to the start of the mow area).
        2.  `GenerateCoveragePath`.
        3.  `FollowWaypoints`.
        4.  `NavigateToPose` (to return to the dock).
    - **Sub-Task 4.2.3:** In the `mission_control_node`, update the `StartMowing` action server to take the goal parameters and write them to the blackboard for the BT to use.
    - **Sub-Task 4.2.4:** Commit the updated BT XML. (`git commit -m "feat(mission): Assemble full mowing sequence in BT XML"`)

- [ ] **MANDATORY TEST 4.A: Verify Full Mission Execution**
    - **Context:** This test validates that the robot can perform a complete, end-to-end mowing job, without any reactivity yet. **This test cannot be skipped.**
    - **Procedure:**
        1.  Launch the full system.
        2.  Use `ros2 action send_goal` to send a `StartMowing` goal with a defined mow area polygon and simple parameters (e.g., 0 perimeters, 0-degree angle).
        3.  Observe the robot in RViz.
    - **Expected Outcome:** The robot navigates to the start of the area, generates a visible Boustrophedon path, follows the waypoints to cover the area, and returns to its starting point.

### **Module 5: Adding Reactivity**

This module adds the final layer of intelligence, allowing the mission to be interrupted by external events.

- [ ] **Task 5.1:** **Implement Condition Nodes**
    - **Dependencies:** 4.2
    - **Context:** Create the BT nodes that will check for critical events.
    - **Sub-Task 5.1.1:** Create a C++ class `IsBatteryLowNode` that subscribes to `/battery_state` and returns `SUCCESS` if the battery is low, `FAILURE` otherwise.
    - **Sub-Task 5.1.2:** Create a C++ class `IsGpsGoodNode` that subscribes to `/gps/fix` and returns `SUCCESS` if the fix quality is acceptable, `FAILURE` otherwise.
    - **Sub-Task 5.1.3:** Create a C++ class `IsJobPausedNode` that checks a variable on the blackboard and returns `SUCCESS` if the job is paused.
    - **Sub-Task 5.1.4:** Register all new condition nodes with the `BT::BehaviorTreeFactory`.
    - **Sub-Task 5.1.5:** Commit the condition nodes. (`git commit -m "feat(mission): Implement BT condition nodes for reactivity"`)

- [ ] **Task 5.2:** **Implement Pause/Resume/Stop Logic**
    - **Dependencies:** 5.1
    - **Context:** Create the ROS 2 interface for controlling the mission externally.
    - **Sub-Task 5.2.1:** In the `mission_control_node`, create services (e.g., `/mower/pause_mission`, `/mower/resume_mission`).
    - **Sub-Task 5.2.2:** The service callbacks should simply set a `is_paused` variable on the BT blackboard.
    - **Sub-Task 5.2.3:** The `StartMowing` action server's `cancel_callback` should handle the "Stop" command by halting the tree.

- [ ] **Task 5.3:** **Update BT XML with Reactive Logic**
    - **Dependencies:** 5.2
    - **Context:** Re-structure the main BT to use reactive nodes, making it interruptible.
    - **Sub-Task 5.3.1:** In `main.xml`, wrap the main mowing `<Sequence>` in a `ReactiveFallback`.
    - **Sub-Task 5.3.2:** This fallback's children should be, in order:
        1.  A sequence to handle a low battery event (e.g., log message, return to dock).
        2.  A sequence to handle a pause command.
        3.  The main mowing `<Sequence>` itself.
    - **Sub-Task 5.3.3:** This structure ensures that if the battery is low, that behavior is executed. If not, it checks if the job is paused. If not, it executes the mowing task.
    - **Sub-Task 5.3.4:** Commit the final reactive BT. (`git commit -m "refactor(mission): Restructure BT XML for reactive logic"`)

### **Module 6: Final Validation**

This module performs the final acceptance test as defined in the PRD.

- [ ] **Task 6.1:** **Perform Full Acceptance Test**
    - **Dependencies:** 5.3
    - **Context:** This is the final end-to-end test for Phase 4, validating all requirements.
    - **Sub-Task 6.1.1:** Execute **Test A (Full Mission)** from the PRD.
    - **Sub-Task 6.1.2:** Execute **Test B (Pause and Resume)** from the PRD by calling the new pause/resume services during the mission.
    - **Sub-Task 6.1.3:** Execute **Test C (Low Battery Abort)** from the PRD by using the battery simulator's service to trigger a low battery state.
    - **Sub-Task 6.1.4:** Document the results of all three tests with screenshots or videos.

- [ ] **Task 6.2:** **Finalize and Merge**
    - **Dependencies:** 6.1
    - **Context:** Clean up the feature branch and merge it into the main branch, completing the phase.
    - **Sub-Task 6.2.1:** Review all new code for clarity, comments, and documentation.
    - **Sub-Task 6.2.2:** Update the `README.md` with instructions on how to start and interact with a mowing mission.
    - **Sub-Task 6.2.3:** Create a Pull Request on GitHub from `feature/phase-4-mission-control` to `main`, including validation artifacts.
    - **Sub-Task 6.2.4:** After review, merge the pull request. Phase 4 is now complete.
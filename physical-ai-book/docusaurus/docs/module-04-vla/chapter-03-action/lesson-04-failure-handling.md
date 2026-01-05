---
id: lesson-04-failure-handling
title: "Lesson 4: Robustness - Reacting to Failures"
sidebar_position: 4
description: Implement strategies for the Robot Executive to detect, report, and recover from execution failures.
---

### Lesson Objective
To integrate mechanisms into the Robot Executive for detecting when a low-level ROS 2 action (like a motion primitive) fails, reporting that failure, and potentially implementing recovery strategies or requesting human intervention.

### Prerequisites
- Completion of Lesson 3: "Integrated Grasp Planning".
- Understanding of exception handling in Python.

### Concept Explanation
In the real world, things go wrong. A motion primitive might fail (e.g., "gripper couldn't grasp object"), a sensor might malfunction, or the robot might encounter an unexpected obstacle. A truly autonomous robot needs to be **robust**, meaning it can detect and react intelligently to these failures.

The Robot Executive, being the orchestrator, is the ideal place to implement failure handling. Its strategies can include:

1.  **Failure Detection**:
    -   Monitoring the result of ROS 2 Actions: If a primitive returns `success=False`.
    -   Timeouts: If a primitive doesn't complete within an expected time frame.
    -   Sensor monitoring: Detecting if the robot is stuck or if an object isn't where it's supposed to be.

2.  **Failure Reporting**:
    -   Logging the error clearly.
    -   Sending a status update back to the LLM or a human operator. This is where the feedback loops become critical.

3.  **Recovery Strategies**:
    -   **Retry**: Simply try the failed action again (e.g., if a network glitch caused an action server to timeout).
    -   **Alternative Action**: If the LLM has provided alternative steps, try one of those.
    -   **Fallback State**: Move the robot to a safe, known state (e.g., stop motors, raise arm).
    -   **Request Human Help**: Report the failure and ask the human for new instructions.

This lesson emphasizes that a robot doesn't have to be perfect; it has to be able to *handle imperfection*.

### Real-World Analogy
Imagine a pilot flying a plane with an autopilot system.
- The **autopilot** (Robot Executive) is given a flight plan (LLM sequence).
- If an **engine fails** (action failure), the autopilot detects it.
- It doesn't just crash. It **reports** the failure to the pilot.
- It might try a **recovery strategy** (e.g., shutting down the engine and trying to restart).
- If it can't recover, it asks the **human pilot for help**.
This layered approach ensures safety and allows for human oversight when needed.

### Hands-On Task
**Task**: Modify your `robot_executive.py` to handle simulated failures of the `drive_forward` primitive.

1.  **Modify `drive_forward_server.py`**: Introduce a simulated failure. For example, make the `drive_forward` action server randomly fail 20% of the time, or fail if the distance requested is too large.
2.  **Modify `robot_executive.py`**:
    -   Add logic to check the `success` field of the action result.
    -   If `success` is `False`, log the error and send a message to a `robot_status` topic indicating the failure.
    -   Implement a simple retry mechanism (e.g., retry once).
    -   If the retry also fails, publish a message asking the user for help.
3.  **Run and Test**:
    - Launch your modified `drive_forward_server.py`.
    - Launch your `robot_executive.py`.
    - Send `move_base` commands. Observe how the Executive handles the simulated failures, retries, and eventually reports a need for help.

### Python + ROS 2 Code Example
#### `drive_forward_server.py` (with simulated failure)
```python
# drive_forward_server.py (excerpt with simulated failure)

# ... (imports) ...
import random

class DriveForwardActionServer(Node):
    # ... (__init__) ...

    def execute_callback(self, goal_handle):
        # ... (logging and feedback) ...
        
        # Simulate a random failure
        if random.random() < 0.2: # 20% chance of failure
            goal_handle.abort() # Indicate failure
            self.get_logger().error('Simulated DriveForward Goal Aborted: Random failure!')
            result = DriveForward.Result()
            result.sequence = [0] # Indicate failure with '0'
            return result

        # Simulate failure if distance is too large
        if goal_handle.request.order > 5: # If distance > 5m, simulate failure
            goal_handle.abort()
            self.get_logger().error('Simulated DriveForward Goal Aborted: Distance too large!')
            result = DriveForward.Result()
            result.sequence = [0]
            return result

        # ... (rest of successful execution) ...

        goal_handle.succeed()
        result = DriveForward.Result()
        result.sequence = [1] # Indicate success with a '1'
        self.get_logger().info('DriveForward Goal Succeeded')
        return result

# ... (main) ...
```

#### `robot_executive.py` (updated for failure handling)
```python
# robot_executive.py (excerpt with failure handling)

# ... (imports) ...
# Assuming DriveForward action is defined and built
# from robot_motion_interfaces.action import DriveForward # Replace with your actual DriveForward Action
from rclpy.action import ActionClient
import threading

class RobotExecutive(Node):
    def __init__(self):
        super().__init__('robot_executive')
        # ... (other initializations) ...
        self.get_logger().info('Robot Executive node has been started.')

        # Publisher for status updates / help requests
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Action client for DriveForward primitive
        self._drive_forward_action_client = ActionClient(self, DriveForward, 'drive_forward')

    # ... (command_callback) ...

    async def _send_drive_forward_goal(self, distance_meters):
        self._drive_forward_action_client.wait_for_server()
        goal_msg = DriveForward.Goal()
        goal_msg.order = int(distance_meters) # Using 'order' as distance for example
        
        self.get_logger().info(f"Executive sending DriveForward goal: {distance_meters} meters.")
        
        # Send goal asynchronously
        send_goal_future = self._drive_forward_action_client.send_goal_async(goal_msg)
        
        # Wait for the server to accept/reject the goal
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('DriveForward goal rejected :(')
            return False

        self.get_logger().info('DriveForward goal accepted :)')
        
        # Wait for the result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        if result.sequence[0] == 1: # Assuming 1 for success, 0 for failure
            self.get_logger().info(f'DriveForward Succeeded: Traveled to {result.sequence}')
            return True
        else:
            self.get_logger().error(f'DriveForward Failed: Result indicated failure.')
            return False

    def _execute_move_base(self, params: dict):
        direction = params.get("direction", "forward")
        distance = params.get("distance_meters", 0.0)
        
        # For simplicity, we only handle 'forward' and map to DriveForward for now
        if direction == "forward" and distance > 0:
            for retry_count in range(2): # Try once, then retry once
                self.get_logger().info(f"Attempting to move {distance}m forward (Retry {retry_count}).")
                # Need to run async action in a separate thread if executive is synchronous
                # For this example, we'll block briefly
                success = rclpy.spin_until_future_complete(self, threading.Thread(target=lambda: self._send_drive_forward_goal(distance)).start())
                if success:
                    self.get_logger().info(f"Successfully moved {distance}m forward.")
                    return
                else:
                    self.get_logger().warn(f"Move forward failed on attempt {retry_count}. Retrying...")
            
            self.get_logger().error(f"Failed to move {distance}m forward after retries. Publishing help request.")
            status_msg = String()
            status_msg.data = f"EXECUTION_FAILURE: Could not move {distance}m. Human intervention needed."
            self.status_publisher.publish(status_msg)
        else:
            self.get_logger().warn(f"Move direction '{direction}' or distance '{distance}' not supported by current executive.")

def main(args=None):
    rclpy.init(args=args)
    # Important: Need to handle async actions properly.
    # A simple spin won't work if actions are async and you need to wait for them.
    # For a real application, consider an Executor:
    # from rclpy.executors import MultiThreadedExecutor
    # executor = MultiThreadedExecutor()
    # robot_executive = RobotExecutive()
    # executor.add_node(robot_executive)
    # executor.spin()
    
    robot_executive = RobotExecutive()
    rclpy.spin(robot_executive) # Will block but we are simulating the async calls
    robot_executive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Not having clear success/failure conditions from your motion primitives. Your Executive needs a reliable way to know if an action succeeded or failed.
- **Mistake**: Implementing a retry loop that retries indefinitely. Always put a limit on retries.
- **Tip**: Design your recovery strategies to be safe. If a robot is mid-movement and an action fails, its first priority should often be to stop and enter a safe state.
- **Tip**: Use a separate ROS 2 topic (e.g., `/robot_status`) for the Executive to publish its status, errors, and requests for human help. This allows a separate monitoring node or UI to subscribe and display this information.

### Mini Assessment
1.  What is the main goal of failure handling in a VLA system?
    a) To avoid writing any error-prone code.
    b) To enable the robot to detect, report, and react intelligently to execution problems.
    c) To ensure the robot never makes mistakes.
2.  Which of the following is NOT a common way for the Executive to detect a primitive action failure?
    a) Monitoring the `success` field of an Action result.
    b) Checking the robot's battery level every second.
    c) Implementing a timeout if an action doesn't complete within expected time.
3.  If a `drive_forward` action primitive fails due to a temporary communication glitch, what is a simple recovery strategy the Executive could employ?
    a) Crash the robot.
    b) Immediately ask the human for help.
    c) Retry the action a limited number of times.
4.  Why is it important to have a `robot_status` topic for the Executive to publish on?
    a) To make the robot seem more talkative.
    b) To allow other nodes or a human UI to monitor the robot's progress and errors.
    c) To store internal debugging information.
5.  What should a robot's first priority be if it detects a critical failure during a complex movement?
    a) Continue the movement to complete the task.
    b) Stop and enter a safe state.
    c) Wait for human input without doing anything.

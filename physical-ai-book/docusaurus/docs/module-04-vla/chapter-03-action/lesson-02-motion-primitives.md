---
id: lesson-02-motion-primitives
title: "Lesson 2: Implementing Motion Primitives"
sidebar_position: 2
description: Develop reusable, low-level robot behaviors (motion primitives) that can be triggered by the Robot Executive.
---

### Lesson Objective
To create a set of robust, low-level motion primitives (e.g., "drive_forward", "turn_in_place") that the Robot Executive can call to execute complex LLM-generated plans.

### Prerequisites
- Completion of Lesson 1: "The Robot Executive - Orchestrating ROS 2".
- Understanding of PID control (optional, for more advanced primitives).

### Concept Explanation
While the LLM is great at high-level planning, it shouldn't directly control individual motor commands. Instead, the Executive should call upon a library of pre-defined, robust **motion primitives**. These are highly engineered, low-level behaviors that handle the intricacies of robot control.

Examples of motion primitives:
-   `drive_forward(distance)`: Drives the robot straight for a specified distance. This primitive handles motor commands, odometry feedback, and stopping accurately.
-   `turn_in_place(angle)`: Rotates the robot by a specific angle.
-   `grasp_object(object_id)`: Executes a sequence to approach an object, open the gripper, close it, and lift.

Motion primitives are typically implemented as separate ROS 2 nodes or as functions within a dedicated "Robot Driver" node. The Robot Executive then sends goals to these primitives (often as ROS 2 Actions or Services), waits for their completion, and incorporates their results into the overall plan.

By using motion primitives, we create a layered architecture:
-   **LLM**: High-level task planning ("Go to the kitchen").
-   **Robot Executive**: Translates plan into a sequence of motion primitives ("drive_forward(5)", "turn_in_place(90)", "drive_forward(2)").
-   **Motion Primitive**: Executes the low-level motor control for each primitive.

### Real-World Analogy
Think of a music conductor (the Robot Executive) leading an orchestra.
- The **score** (LLM plan) says "Play a grand symphony."
- The **conductor** (Executive) knows the symphony needs violins to play "this" melody and cellos to play "that" bassline.
- The **musicians** (motion primitives) know exactly how to play their instruments (perform their low-level movements) perfectly.
The conductor doesn't tell each violinist which finger to place where; they just tell them to "play the melody."

### Hands-On Task
**Task**: Implement a basic `drive_forward` motion primitive as a ROS 2 Action Server.

1.  **Create Action Definition**: Define a custom ROS 2 Action `DriveForward.action` in a new package (e.g., `robot_motion_interfaces`).
    ```
    # Goal
    float32 distance_meters
    ---
    # Result
    bool success
    float32 final_x
    float32 final_y
    ---
    # Feedback
    float32 current_distance_traveled
    ```
2.  **Build Action Package**: `colcon build --packages-select robot_motion_interfaces`.
3.  **Create Primitive Server**: Create a Python script `drive_forward_server.py` in your `ros2_ws/src` directory. This will be an Action Server for `DriveForward.action`.
4.  **Modify Executive**: Update your `robot_executive.py` from Lesson 1 to send a goal to this `DriveForward` Action Server when it receives a `move_base` command with a `distance_meters` parameter.
5.  **Run and Test**:
    - Run the `drive_forward_server.py`.
    - Run the `robot_executive.py`.
    - Publish a command to the executive: `ros2 topic pub --once /robot_commands ... "move_base", "distance_meters": 2.0 ...`
    - Observe the executive sending the goal, receiving feedback, and getting the result.

### Python + ROS 2 Code Example
#### `DriveForward.action` (in `robot_motion_interfaces/action/`)
```
# Goal
float32 distance_meters
---
# Result
bool success
float32 final_x
float32 final_y
---
# Feedback
float32 current_distance_traveled
```

#### `drive_forward_server.py` (Action Server)
```python
# drive_forward_server.py

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
import math

# Import our custom action interface
# from robot_motion_interfaces.action import DriveForward # This would be if we created a custom action

# For simplicity, we will use a generic action interface
# In a real system, you'd define custom actions based on your needs
from example_interfaces.action import Fibonacci as DriveForward # Using Fibonacci as a stand-in for custom action

class DriveForwardActionServer(Node):
    def __init__(self):
        super().__init__('drive_forward_server')
        self._action_server = ActionServer(
            self,
            DriveForward,
            'drive_forward',
            self.execute_callback)
        self.get_logger().info('DriveForward Action Server ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing DriveForward goal: {goal_handle.request.order} meters (simulated as order of Fibonacci)')
        
        # In a real robot, this is where you'd send commands to motors,
        # read odometry, and control PID loops to achieve the distance.
        
        feedback_msg = DriveForward.Feedback()
        feedback_msg.sequence = [] # Use sequence as current_distance_traveled for this example

        distance_to_travel = goal_handle.request.order # Using order as distance_meters
        current_distance = 0.0
        step_distance = 0.1 # Simulate moving 0.1 meters at a time
        
        while current_distance < distance_to_travel and not goal_handle.is_cancel_requested:
            current_distance += step_distance
            feedback_msg.sequence.append(int(current_distance * 10)) # Store as int for Fibonacci.sequence type
            self.get_logger().info(f'Feedback: Traveled {current_distance:.2f}/{distance_to_travel:.2f} meters')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5) # Simulate time taken to move

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('DriveForward Goal Canceled')
            return DriveForward.Result()

        goal_handle.succeed()
        result = DriveForward.Result()
        result.sequence = [1] # Indicate success with a '1' in sequence
        self.get_logger().info('DriveForward Goal Succeeded')
        return result

def main(args=None):
    rclpy.init(args=args)
    server = DriveForwardActionServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: The Robot Executive tries to directly control motors. This bypasses the carefully engineered motion primitives and is prone to errors.
- **Mistake**: Not handling units correctly. Ensure distances are in meters, angles in radians, velocities in m/s and rad/s, consistently.
- **Tip**: Test each motion primitive independently first. Use `ros2 action send_goal` from the command line to verify it behaves correctly before integrating it with the Executive.

### Mini Assessment
1.  What is a "motion primitive"?
    a) A simple robot motor.
    b) A low-level, reusable robot behavior (e.g., "drive_forward").
    c) A high-level plan from the LLM.
2.  Why use motion primitives instead of the Executive directly controlling motors?
    a) It makes the LLM's job easier.
    b) Motion primitives are more robust, engineered for low-level control, and reusable.
    c) It's faster to write.
3.  What ROS 2 primitive is typically used for motion primitives that involve continuous feedback and might be long-running?
    a) Topics.
    b) Services.
    c) Actions.
4.  If your `drive_forward` primitive is published a goal with `distance_meters=5.0`, but the robot only moves 2.0 meters, what is likely the problem?
    a) The LLM made a mistake.
    b) The low-level control loop within the `drive_forward` primitive is not accurately measuring or executing the distance.
    c) The robot's battery is dead.
5.  What layer of the architecture is responsible for translating the LLM plan into a sequence of motion primitive calls?
    a) The individual motor controllers.
    b) The Robot Executive.
    c) The ASR system.

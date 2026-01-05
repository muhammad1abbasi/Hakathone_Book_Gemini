---
id: lesson-04-actions
title: "Lesson 4: Actions - The Asynchronous Task Manager"
sidebar_position: 4
description: Learn to use ROS 2 Actions for long-running, feedback-driven, and preemptible tasks.
---

### Lesson Objective
To understand the structure of a ROS 2 **Action** (goal, feedback, result) and implement a simple Action Server and Client.

### Prerequisites
- Completion of Lesson 3: "Services".

### Concept Explanation
We've learned that Topics are for continuous data streams and Services are for quick, synchronous transactions. But what about tasks that take a long time to complete, where we want updates along the way, and we might want to cancel them? For this, ROS 2 provides **Actions**.

An Action is the most complex communication type, and it's made of three parts:
1.  **The Goal**: The client sends a goal to the Action Server (e.g., "rotate the robot 90 degrees"). The server can accept or reject the goal.
2.  **The Feedback**: While the server is executing the goal, it can send a stream of feedback messages back to the client (e.g., "I have rotated 10 degrees... 20 degrees... 30 degrees...").
3.  **The Result**: When the task is finished, the server sends a single result message back to the client (e.g., "Finished rotation. Final heading is 90.2 degrees.").

This is an **asynchronous** pattern. The client doesn't have to wait. It can send the goal and go do other work, while listening for feedback and the final result in the background. Actions are also **preemptible**, meaning the client can send a cancel request at any time.

### Real-World Analogy
An **Action** is like ordering a pizza for delivery.
- The **Action Server** is the pizza place.
- The **Action Client** is you.
- The **Goal** is your order ("I'd like a large pepperoni pizza delivered to 123 Main St."). The pizza place accepts your goal.
- The **Feedback** is the stream of updates you get from the delivery app ("The kitchen is preparing your order... Your driver is on the way... Your driver is 5 minutes away...").
- The **Result** is the pizza arriving at your door, and the final status ("Your pizza has been delivered.").
- **Preemption** is you calling the pizza place halfway through to say, "Never mind, cancel my order."

### Hands-On Task
**Task**: Create an Action that simulates computing the Fibonacci sequence.

1.  **Action Definition**: ROS 2 provides a built-in action `example_interfaces/action/Fibonacci`.
    - **Goal**: an integer `order`.
    - **Feedback**: the current sequence.
    - **Result**: the final sequence.
2.  **Create Server File**: In `ros2_ws/src`, create `simple_action_server.py` and add the server code.
3.  **Create Client File**: In the same directory, create `simple_action_client.py` and add the client code.
4.  **Run the Nodes**:
    - Open a terminal, navigate to `ros2_ws/src`, and run the server:
      ```bash
      source /opt/ros/humble/setup.bash
      python3 simple_action_server.py
      ```
    - Open a **second terminal**, navigate to `ros2_ws/src`, and run the client:
      ```bash
      source /opt/ros/humble/setup.bash
      python3 simple_action_client.py
      ```
5.  **Observe**: Watch the client terminal. You'll see it send the goal, then receive periodic feedback as the sequence is computed, and finally receive the full result.

### Python + ROS 2 Code Example

#### Action Server
```python
# simple_action_server.py
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class SimpleActionServer(Node):
    def __init__(self):
        super().__init__('simple_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info("Action server is ready.")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    simple_action_server = SimpleActionServer()
    rclpy.spin(simple_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Action Client
```python
# simple_action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__('simple_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        
        self.get_logger().info("Sending goal request...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = SimpleActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Forgetting that Actions are asynchronous. You can't just send a goal and expect the result to be there on the next line of code. You have to use "futures" and "callbacks" to handle the responses when they arrive.
- **Tip**: The `ros2 action` command-line tool is your best friend for debugging. You can `list` actions, `info` about them, and even `send_goal` from the command line to test your server without needing to write a client first.

### Mini Assessment
1.  An Action is the best choice for which scenario?
    a) Getting a robot's current battery level once.
    b) Streaming camera images continuously.
    c) Commanding a robot to navigate to a goal location, which takes 30 seconds and requires progress updates.
2.  What are the three main parts of an Action definition?
    a) Request, Response, Acknowledgment
    b) Goal, Feedback, Result
    c) Start, Middle, End
3.  What does "preemptible" mean in the context of an Action?
    a) The task cannot be stopped once it starts.
    b) The client can cancel the goal while it is executing.
    c) The server can change the goal halfway through.
4.  In the pizza delivery analogy, what does the continuous stream of app notifications represent?
    a) The Goal
    b) The Feedback
    c) The Result
5.  Why is an Action "asynchronous"?
    a) The client must wait for the result before doing anything else.
    b) The client can send a goal and then perform other tasks while waiting for the result.
    c) It only works in the morning.
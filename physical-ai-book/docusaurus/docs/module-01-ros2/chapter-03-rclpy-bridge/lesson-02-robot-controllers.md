---
id: lesson-02-robot-controllers
title: "Lesson 2: Creating a Simple Robot Controller"
sidebar_position: 2
description: Write a node that subscribes to velocity commands and publishes mock robot state.
---

### Lesson Objective
To build a basic "robot controller" node that simulates a robot's movement by subscribing to velocity commands and publishing its changing position (odometry).

### Prerequisites
- Completion of Lesson 1: "Structuring Nodes with Python Classes".

### Concept Explanation
A real robot controller is a complex piece of software that translates high-level commands (like "move forward at 0.5 m/s") into low-level electrical signals sent to the motors. It also reads from wheel encoders to calculate how far the robot has actually moved. This position data is typically published as **odometry**.

We can simulate this behavior with a simple node. Our controller will:
1.  **Subscribe** to a topic of type `geometry_msgs/msg/Twist`. The `Twist` message is the standard way to represent velocity in ROS. It has a `linear` component (movement in x, y, z) and an `angular` component (rotation around x, y, z).
2.  **Publish** on a topic of type `nav_msgs/msg/Odometry`. The `Odometry` message represents the robot's estimated position and orientation in the world.
3.  **Have a Game Loop**: A timer will run at a high frequency (e.g., 50 Hz). In each tick of this "game loop," it will update the robot's position based on the last received velocity command.

This creates a closed loop. Another node can send `Twist` commands to move the robot, and listen to the `/odom` topic to see where the robot is.

### Real-World Analogy
This is exactly like playing a video game character.
- Your **joystick** sends velocity commands (a `Twist` message).
- The **game engine** (our controller node) receives these commands.
- In every frame of the game (our timer loop), the engine updates your character's X/Y coordinates based on your joystick input.
- It then re-renders your character on the screen at the new position (publishing the `Odometry` message).

### Hands-On Task
**Task**: Create a simulated differential drive robot controller. A differential drive robot has two wheels that can spin at different speeds, allowing it to move forward and turn.

1.  **Create Controller File**: In `ros2_ws/src`, create a file named `simple_robot_controller.py` and add the code below.
2.  **Run the Controller**:
    ```bash
    source /opt/ros/humble/setup.bash
    cd ros2_ws/src
    python3 simple_robot_controller.py
    ```
    The node will start, but nothing will happen yet because no one is publishing velocity commands.
3.  **Send a Command**: Open a **second terminal**. We will use the `ros2` CLI to send a single command to make the robot move forward.
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ```
4.  **Observe**: In the controller's terminal, you'll see the odometry being published, with the `x` position increasing over time.
5.  **Echo Odometry**: In a **third terminal**, watch the odometry data being published:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 topic echo /odom
    ```

### Python + ROS 2 Code Example
```python
# simple_robot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        
        # State of the robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd_vel = Twist()

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Timer for the main control loop
        self.timer = self.create_timer(0.02, self.controller_loop) # 50 Hz

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg

    def controller_loop(self):
        # Update robot's position based on last velocity command
        dt = 0.02  # Time step
        self.x += self.last_cmd_vel.linear.x * math.cos(self.theta) * dt
        self.y += self.last_cmd_vel.linear.x * math.sin(self.theta) * dt
        self.theta += self.last_cmd_vel.angular.z * dt

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        # Orientation in quaternion
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f'Published Odom: x={self.x:.2f}, y={self.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Using the wrong message types. `/cmd_vel` almost always uses `geometry_msgs/msg/Twist`. Odometry uses `nav_msgs/msg/Odometry`. Using the wrong one will cause errors.
- **Tip**: When your robot isn't moving as expected, use `ros2 topic echo /cmd_vel` to see if the velocity commands are actually being published correctly by your other nodes. Then, `echo /odom` to see if the controller is correctly calculating its position. This helps isolate the problem.

### Mini Assessment
1.  What is the standard ROS 2 message type for sending velocity commands?
    a) `std_msgs/msg/String`
    b) `geometry_msgs/msg/Twist`
    c) `nav_msgs/msg/Odometry`
2.  What does "odometry" represent?
    a) The robot's desired velocity.
    b) The robot's estimated position and orientation.
    c) The robot's battery level.
3.  In the controller, why do we use a high-frequency timer for the main loop?
    a) To make the code more complex.
    b) To smoothly update the robot's position over time, like frames in a game.
    c) To save battery power.
4.  If you want the robot to turn left, what value in the `Twist` message would you set?
    a) A positive `linear.x`.
    b) A negative `linear.x`.
    c) A positive `angular.z`.
5.  Which command would you use to send a single velocity command from the terminal?
    a) `ros2 topic echo /cmd_vel`
    b) `ros2 topic pub /cmd_vel ...`
    c) `ros2 service call /cmd_vel ...`
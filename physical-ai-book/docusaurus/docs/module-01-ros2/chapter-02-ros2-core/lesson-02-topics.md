---
id: lesson-02-topics
title: "Lesson 2: Topics - The One-to-Many Broadcast"
sidebar_position: 2
description: Learn to use ROS 2 Topics for continuous data streaming using a publisher/subscriber pattern.
---

### Lesson Objective
To create two nodes: a **publisher** that continuously broadcasts data on a **topic**, and a **subscriber** that listens to that data.

### Prerequisites
- Completion of Lesson 1: "Nodes".
- You should have a `ros2_ws/src` directory.

### Concept Explanation
**Topics** are the most common way for nodes to communicate in ROS 2. They work like a radio station. One node (the **publisher**) broadcasts data on a specific channel (the **topic**), and any other node (the **subscriber**) can tune into that channel to receive the data.

This is a one-to-many communication system. You can have:
- One publisher and many subscribers.
- Many publishers on the same topic.
- A node can be both a publisher and a subscriber at the same time (even on the same topic).

This pattern is perfect for continuous data streams, like sensor data (camera images, LiDAR scans) or the robot's current position (odometry).

The data that is sent on a topic must have a specific **message type**. ROS 2 provides many standard message types (like `String`, `Int32`, `Twist` for velocity), and you can also create your own.

### Real-World Analogy
A **Topic** is like a public radio station, for example, a weather station.
- The **Publisher** is the meteorologist at the station, continuously broadcasting the current temperature.
- The **Topic Name** is the radio frequency (e.g., "101.5 FM").
- The **Message Type** is the kind of data being sent (e.g., a number representing the temperature).
- **Subscribers** are anyone with a radio who tunes into 101.5 FM to hear the temperature. They don't need to know who the meteorologist is; they just need to know the frequency.

### Hands-On Task
**Task**: Create a publisher node and a subscriber node to communicate over a topic.

1.  **Create Publisher File**: In your `ros2_ws/src` directory, create a file named `simple_publisher.py` and add the Python code from the first example below.
2.  **Create Subscriber File**: In the same directory, create another file named `simple_subscriber.py` and add the code from the second example.
3.  **Run the Nodes**:
    - Open a terminal, navigate to `ros2_ws/src`, and run the publisher:
      ```bash
      source /opt/ros/humble/setup.bash
      python3 simple_publisher.py
      ```
    - Open a **second terminal**, navigate to `ros2_ws/src`, and run the subscriber:
      ```bash
      source /opt/ros/humble/setup.bash
      python3 simple_subscriber.py
      ```
4.  **Observe**: In the subscriber's terminal, you will see the "Hello World" messages being received from the publisher.
5.  **Inspect (Optional)**: Open a **third terminal** and use the `ros2` command-line tool to see what's happening.
    - List topics: `ros2 topic list` (you should see `/chatter`).
    - Echo topic data: `ros2 topic echo /chatter`.

### Python + ROS 2 Code Example

#### Publisher
```python
# simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber
```python
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Publisher and Subscriber use a different topic name or message type. They must match exactly for communication to work.
- **Tip**: Use the `ros2 topic` command-line tools to debug. `ros2 topic list` shows all active topics. `ros2 topic info /topic_name` shows who is publishing and subscribing. `ros2 topic echo /topic_name` shows the data being published in real-time. This is extremely useful for checking if a publisher is working correctly, even before you've written a subscriber.

### Mini Assessment
1.  The Topic communication pattern is best for:
    a) A single request and a single response.
    b) Long-running tasks with feedback.
    c) Continuous data streams.
2.  In the radio station analogy, what does the Topic Name represent?
    a) The meteorologist.
    b) The radio frequency.
    c) The listener.
3.  What happens if a publisher sends a message but no subscribers are running?
    a) The publisher node crashes.
    b) The message is sent out but nobody receives it.
    c) The ROS Master stores the message for later.
4.  In the code, what is the purpose of `from std_msgs.msg import String`?
    a) It imports the Node class.
    b) It imports the message type we will use for our topic.
    c) It is a standard Python library for text.
5.  Which command would you use to see the raw data being sent on the `/chatter` topic?
    a) `ros2 topic list`
    b) `ros2 topic info /chatter`
    c) `ros2 topic echo /chatter`
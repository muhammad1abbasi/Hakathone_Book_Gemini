---
id: lesson-04-debugging-tools
title: "Lesson 4: Debugging with ROS 2 Tools"
sidebar_position: 4
description: Learn to use command-line tools like rqt_graph, ros2 topic, and ros2 node to inspect a live ROS 2 system.
---

### Lesson Objective
To become proficient in using ROS 2's command-line and GUI tools to visualize, inspect, and debug a running system of nodes.

### Prerequisites
- Completion of the previous lessons in this chapter.
- Have your publisher/subscriber or service/client nodes handy to run.

### Concept Explanation
When your robotic system grows to include many nodes, debugging becomes difficult. You can't just use `print()` statements everywhere. ROS 2 provides powerful tools to help you understand what's happening in the ROS Graph.

**Key Debugging Tools:**

1.  **`rqt_graph`**: This is a GUI tool that visualizes the ROS Graph. It shows you which nodes are running, what topics they are publishing or subscribing to, and how they are all connected. It's the first tool you should use to get a high-level overview of your system.

2.  **`ros2 node`**: This command lets you interact with nodes.
    - `ros2 node list`: Shows all currently running nodes.
    - `ros2 node info /node_name`: Shows a node's publishers, subscribers, services, and actions.

3.  **`ros2 topic`**: Lets you interact with topics.
    - `ros2 topic list`: Lists all active topics.
    - `ros2 topic echo /topic_name`: Prints the messages being published on a topic to the console. Incredibly useful for seeing sensor data or publisher output.
    - `ros2 topic pub ...`: Publish a single message to a topic from the command line. Great for testing a subscriber without needing to write a full publisher node.
    - `ros2 topic info /topic_name`: Shows the message type and number of publishers/subscribers.

4.  **`ros2 service`** and **`ros2 action`**: Similar tools for listing, getting info about, and calling services or sending goals to actions directly from the command line.

### Real-World Analogy
Debugging a ROS system is like being a detective at a crime scene.
- **`rqt_graph`** is your corkboard with pictures and strings, showing you who all the suspects are (nodes) and who they've been talking to (topics). It gives you the big picture of the relationships.
- **`ros2 topic echo`** is like tapping a phone line. It lets you listen in on a specific conversation between two suspects to hear exactly what they are saying (the data).
- **`ros2 node info`** is like getting a background check on a suspect. It tells you all their known associations (their publishers, subscribers, etc.).
- **`ros2 topic pub`** is like feeding an anonymous tip to one of the suspects to see how they react. You can inject data into the system to trigger a specific behavior.

### Hands-On Task
**Task**: Use the debugging tools to inspect the publisher/subscriber pair from Lesson 2.

1.  **Run Your Nodes**: Open two terminals. In one, run `simple_publisher.py`. In the other, run `simple_subscriber.py`.
2.  **Use `rqt_graph`**:
    - Open a third terminal and source your ROS 2 setup file.
    - Run the command: `rqt_graph`.
    - A window will pop up. You should see the `/simple_publisher` node, the `/simple_subscriber` node, and an arrow connecting them via the `/chatter` topic.
3.  **Use Node Tools**:
    - In the third terminal, run `ros2 node list`. You should see your two nodes listed, plus others that `rqt_graph` might be running.
    - Get info on your publisher: `ros2 node info /simple_publisher`.
4.  **Use Topic Tools**:
    - Run `ros2 topic list` to see `/chatter` and `/parameter_events`.
    - Run `ros2 topic echo /chatter` to see the "Hello World" messages printed in your third terminal, proving that the data is flowing through the ROS system.
5.  **Publish from the Command Line**:
    - Stop your `simple_publisher.py` script (Ctrl+C).
    - In the third terminal, run this command:
      ```bash
      ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'Hello from the command line'}"
      ```
    - Observe that your `simple_subscriber.py` node, which is still running, receives and prints this new message.

### Common Mistakes & Debugging Tips
- **Mistake**: Forgetting to source the `setup.bash` file in every new terminal you open. If ROS 2 commands are "not found," this is the first thing to check.
- **Tip**: `rqt_graph` is your best first step when a system isn't working. It will immediately show you if a node isn't running or if it's not connected to the topic you expect. If you see a node but no arrows, you likely have a mismatch in topic names or message types.
- **Tip**: Use `ros2 topic echo` to confirm that a node is publishing data *before* you even start writing the subscriber for it. This helps you build and test your system incrementally.

### Mini Assessment
1.  Which tool provides a visual GUI map of your running nodes and topics?
    a) `ros2 node list`
    b) `rqt_graph`
    c) `ros2 topic echo`
2.  You have a publisher and a subscriber, but the subscriber isn't receiving any messages. What is the BEST first step to debug this?
    a) Rewrite the subscriber from scratch.
    b) Use `ros2 topic echo` on the topic to see if the publisher is sending any data at all.
    c) Check the robot's battery.
3.  Which command would you use to see a list of all running nodes?
    a) `ros2 topic list`
    b) `ros2 node info`
    c) `ros2 node list`
4.  What is `ros2 topic pub` useful for?
    a) Viewing data on a topic.
    b) Getting information about a topic's message type.
    c) Sending a single message from the command line to test a subscriber.
5.  If `rqt_graph` shows your two nodes but no connection between them, what is a likely problem?
    a) The nodes are running on different computers.
    b) There is a typo in the topic name in one of the nodes.
    c) The nodes are not written in the same programming language.
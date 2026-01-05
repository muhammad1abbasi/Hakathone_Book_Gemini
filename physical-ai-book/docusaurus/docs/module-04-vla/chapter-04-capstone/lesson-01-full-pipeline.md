---
id: lesson-01-full-pipeline
title: "Lesson 1: Integrating the Full VLA Pipeline"
sidebar_position: 1
description: Bringing together the Voice, Language, and Action components into a cohesive, end-to-end VLA system.
---

### Lesson Objective
To integrate all components developed in previous chapters (Whisper ASR, LLM for planning, Robot Executive for action) into a single, functional VLA pipeline capable of responding to voice commands.

### Prerequisites
- Completion of Module 4, Chapter 3 (Action).
- A running ROS 2 environment with your `robot_executive.py` and at least one motion primitive server (e.g., `drive_forward_server.py`).

### Concept Explanation
The VLA pipeline is a chain of interconnected ROS 2 nodes, each performing a specialized task:

1.  **Audio Capture Node**: (From Lesson 4.1.1) Listens to the microphone, performs wake word detection, and publishes audio chunks to a ROS 2 topic.
2.  **ASR Node (Whisper)**: (From Lesson 4.1.1) Subscribes to audio chunks, transcribes them using Whisper, and publishes the text to a `voice_commands` topic.
3.  **LLM Planner Node**: (From Lesson 4.2.3) Subscribes to `voice_commands`, uses an LLM to parse intent and generate an action sequence (JSON), and publishes this sequence to a `robot_commands` topic.
4.  **Robot Executive Node**: (From Lesson 4.3.1) Subscribes to `robot_commands`, translates the JSON sequence into ROS 2 calls (to motion primitive action servers), and manages execution.
5.  **Motion Primitive Servers**: (From Lesson 4.3.2) Receive action goals from the Executive, perform low-level robot control, and send back feedback and results.

This forms a complete closed loop: **Human Voice -> Robot Action -> Physical World Change**.

### Real-World Analogy
This is like a complex command chain in a military operation.
- **Human Commander** (You) gives a voice order.
- **Radio Operator** (ASR Node) transcribes the order and sends it up the chain.
- **Strategic Planner** (LLM Planner) receives the text, understands the intent, and creates a plan (sequence of actions).
- **Tactical Commander** (Robot Executive) receives the plan and dispatches specialized units (Motion Primitive Servers) to execute each step.
- **Field Units** (Motion Primitive Servers) perform the actual tasks.

### Hands-On Task
**Task**: Connect a mock audio input to the LLM planner and Executive to demonstrate the full VLA pipeline.

1.  **Run Core Components**: Ensure your `robot_executive.py` and `drive_forward_server.py` (or other motion primitive servers) are running in separate terminals.
2.  **Create Mock ASR Node**: Create a new Python script `mock_asr_node.py` that simulates the ASR. It will either take text input from the console or from a pre-recorded audio file and publish it to the `voice_commands` topic.
3.  **Create LLM Planner Node**: Adapt your `action_sequencer.py` from Lesson 4.2.3 into a ROS 2 node that subscribes to `voice_commands` and publishes to `robot_commands`.
4.  **Launch All Nodes**: Use a ROS 2 launch file to start your `mock_asr_node`, `llm_planner_node`, `robot_executive`, and `drive_forward_server`.
5.  **Test the Pipeline**: Speak a command to your `mock_asr_node` (or provide text input). Observe the full chain of events as the LLM plans and the Executive commands the motion primitive.

### Python + ROS 2 Code Example
*(Due to the length and complexity of full node implementations, this example focuses on the conceptual ROS 2 graph and the main launch file.)*

#### `mock_asr_node.py` (simulated ASR)
```python
# mock_asr_node.py (in ros2_ws/src)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MockASRNode(Node):
    def __init__(self):
        super().__init__('mock_asr_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.get_logger().info('Mock ASR Node ready. Type commands and press Enter:')

        # Use a timer to simulate periodic checks or just wait for input
        self.timer = self.create_timer(0.1, self.publish_command_from_input)
        self._input_thread = threading.Thread(target=self._get_input_loop)
        self._input_thread.daemon = True
        self._input_thread.start()
        self._latest_command = None

    def _get_input_loop(self):
        while rclpy.ok():
            command = input("You (mock audio): ")
            self._latest_command = command
            time.sleep(0.1)

    def publish_command_from_input(self):
        if self._latest_command:
            msg = String()
            msg.data = self._latest_command
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published mock voice command: "{msg.data}"')
            self._latest_command = None # Clear after publishing

def main(args=None):
    rclpy.init(args=args)
    mock_asr_node = MockASRNode()
    rclpy.spin(mock_asr_node)
    mock_asr_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import threading
    main()
```

#### `llm_planner_node.py` (ROS 2 Node version of `action_sequencer.py`)
```python
# llm_planner_node.py (in ros2_ws/src)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import openai

openai.api_key = os.getenv("OPENAI_API_KEY")

if not openai.api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set or not provided.")

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.get_logger().info('LLM Planner Node has been started.')

        # Subscriber for voice commands from ASR
        self.asr_subscription = self.create_subscription(
            String,
            'voice_commands',
            self.asr_command_callback,
            10)
        self.asr_subscription

        # Publisher for robot commands to Executive
        self.executive_publisher = self.create_publisher(String, 'robot_commands', 10)
        
        # Define available actions (same as Lesson 4.2.3)
        self.available_actions = [
            {"name": "move_base", "description": "Move the robot to a target (x, y, theta) or in a direction for a distance.",
             "parameters": {"x": float, "y": float, "theta": float, "direction": ["forward", "backward", "left", "right"], "distance_meters": float}},
            {"name": "detect_object", "description": "Detect objects of a certain type/color in the environment.",
             "parameters": {"object_name": str, "color": str}},
            {"name": "pick_object", "description": "Pick up a detected object.",
             "parameters": {"object_id": str}},
            {"name": "place_object", "description": "Place an object at a target (x, y, z) or on a surface.",
             "parameters": {"x": float, "y": float, "z": float, "surface_name": str}},
            {"name": "report_status", "description": "Report current status of the robot.",
             "parameters": {"status_type": ["battery", "location", "temperature"]}},
            {"name": "gripper_control", "description": "Open or close the robot's gripper.",
             "parameters": {"command": ["open", "close"]}}
        ]

    def asr_command_callback(self, msg):
        human_command = msg.data
        self.get_logger().info(f'Received ASR command: "{human_command}"')
        
        # Call the parse_and_plan function (from Lesson 4.2.4)
        result = self._parse_and_plan_with_llm(human_command)
        
        # Publish the result for the Robot Executive
        output_msg = String()
        output_msg.data = json.dumps(result)
        self.executive_publisher.publish(output_msg)
        self.get_logger().info(f"Published command for executive: {output_msg.data}")

    def _parse_and_plan_with_llm(self, command: str) -> dict:
        prompt = f"""
        You are a robust robot command parser and task planner.
        The user said: "{command}".
        
        Your goal is to perform one of the following:
        1. If the command is clear, directly generate a JSON list of ROS 2 action calls using the available actions.
        2. If the command is ambiguous or parameters are missing, generate a JSON object to ask for clarification.
        
        Available Actions (use only these, and provide all necessary parameters):
        {json.dumps(self.available_actions, indent=2)}
        
        Output Format:
        - If clear, return: {{ "type": "action_sequence", "actions": [ {{ "name": "...", "parameters": {{...}} }} ] }}
        - If ambiguous, return: {{ "type": "clarification_needed", "question": "..." }}
        
        Example 1: Clear command
        User: "Move forward 2 meters."
        Output: {{ "type": "action_sequence", "actions": [ {{ "name": "move_base", "parameters": {{ "direction": "forward", "distance_meters": 2.0 }} }} ] }}
        
        Example 2: Ambiguous command
        User: "Pick up the block."
        Output: {{ "type": "clarification_needed", "question": "Which block are you referring to? Please specify color or location." }}
        
        Your JSON response:
        """
        
        try:
            response = openai.chat.completions.create(
                model="gpt-3.5-turbo-1106",
                messages=[
                    {"role": "system", "content": "You are a helpful robot command parser and task planner."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.2,
                response_format={"type": "json_object"}
            )
            parsed_output = json.loads(response.choices[0].message.content.strip())
            return parsed_output
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON from LLM: {e}")
            return {"type": "error", "message": "JSON decoding failed."}
        except Exception as e:
            self.get_logger().error(f"Error during LLM processing: {e}")
            return {"type": "error", "message": str(e)}


def main(args=None):
    rclpy.init(args=args)
    llm_planner_node = LLMPlannerNode()
    rclpy.spin(llm_planner_node)
    llm_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Not having all nodes running. If the ASR node isn't publishing, the LLM planner won't receive anything. Use `rqt_graph` to visualize the entire pipeline and ensure all connections are active.
- **Mistake**: Incorrect topic names or message types between nodes. Always double-check your `create_publisher` and `create_subscription` calls.
- **Tip**: Test each part of the pipeline individually before trying to run the whole thing. Test ASR, then LLM planner, then Executive. This helps isolate where problems might be.

### Mini Assessment
1.  What is the final output of the ASR node in the VLA pipeline?
    a) Raw audio data.
    b) A ROS 2 action sequence.
    c) Text transcription published to a topic.
2.  What is the input to the Robot Executive node?
    a) Raw audio data.
    b) LLM-generated JSON action sequences.
    c) Physical sensor data.
3.  Which ROS 2 tool is essential for visualizing the entire VLA pipeline and its connections?
    a) `ros2 topic echo`
    b) `rqt_graph`
    c) `ros2 node list`
4.  If the LLM Planner Node isn't receiving any commands, what is the most likely culprit?
    a) The Robot Executive is not running.
    b) The ASR Node is not publishing to the `voice_commands` topic, or the subscription is incorrect.
    c) The motion primitive servers are not running.
5.  Why is a modular design (separate nodes for ASR, LLM, Executive) beneficial for the VLA pipeline?
    a) It makes the code harder to debug.
    b) It allows for independent development, testing, and replacement of individual components.
    c) It increases latency.

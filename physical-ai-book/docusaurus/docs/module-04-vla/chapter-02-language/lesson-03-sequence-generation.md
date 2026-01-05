--- 
id: lesson-03-sequence-generation
title: "Lesson 3: LLM to ROS Action Sequence Generation"
sidebar_position: 3
description: Translating parsed commands into a sequence of specific ROS 2 actions for the robot.
---

### Lesson Objective
To leverage an LLM to translate a high-level intent and its extracted parameters into a precise, ordered sequence of low-level ROS 2 actions that the robot can execute.

### Prerequisites
- Completion of Lesson 2: "Extracting Parameters for Robot Actions".
- Familiarity with ROS 2 Actions (from Module 1).

### Concept Explanation
We have the user's intent and the relevant parameters. Now, we need to convert this high-level understanding into a concrete, executable plan for the robot. This plan will be a sequence of ROS 2 Actions (or Services, or Topics, depending on the need). The LLM can also help here, acting as a **task planner**.

We can prompt the LLM to output a list of discrete steps, each corresponding to a known ROS 2 Action. For example, if the intent is `grasp_object` with `object_name: "block"` and `color: "red"`, the LLM might generate a plan like:
1.  `find_object(color="red", type="block")` (ROS 2 Service)
2.  `move_to_object(object_id=...)` (ROS 2 Action)
3.  `grasp(object_id=...)` (ROS 2 Action)
4.  `move_arm_to_home_position()` (ROS 2 Action)

The trick is to provide the LLM with a "toolset" – a description of the available ROS 2 Actions/Services, their parameters, and what they do. This is a form of **Function Calling** or **Tool Use** with LLMs. The LLM then acts as a reasoning engine, selecting and ordering the appropriate tools.

### Real-World Analogy
Imagine you're instructing a chef with a very specific set of kitchen tools.
- You say: "Make me a simple pasta dish."
- The **LLM** (the chef's brain) would take this high-level goal and break it down into a sequence of operations using available tools:
    1.  `boil_water(amount=...)` (using the `pot` tool)
    2.  `add_pasta(type=...)` (using the `pasta_box` tool)
    3.  `make_sauce(type=...)` (using `sauce_ingredients` tool)
    4.  `combine_and_serve()`
The chef understands the final goal and knows how to use the available tools to achieve it.

### Hands-On Task
**Task**: Use an LLM to generate a sequence of ROS 2-like actions for a robot to "grab the blue cube and place it on the red mat."

1.  **Modify `intent_parser.py`**: Open your `intent_parser.py` script.
2.  **Add Action Definitions**: Define a list of mock ROS 2 actions/services that your robot supports, including their parameters.
3.  **Update Prompt**: Add these action definitions to the LLM's prompt and ask it to generate a sequence of calls.
4.  **Run the Script**:
    ```bash
    python3 intent_parser.py
    ```
    Try the command "Grab the blue cube and place it on the red mat." Observe the generated sequence of actions.

### Python + ROS 2 Code Example
```python
# action_sequencer.py (building on intent_parser.py)

import openai
import os
import json

openai.api_key = os.getenv("OPENAI_API_KEY")

if not openai.api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set or not provided.")

def generate_action_sequence(parsed_command: dict) -> list[dict]:
    """
    Uses an LLM to generate a sequence of ROS 2 actions based on a parsed command.
    """
    intent = parsed_command.get("intent", "unknown")
    params = parsed_command.get("parameters", {})

    # Define the available robot actions (this would typically come from a robot's capabilities)
    available_actions = [
        {"name": "move_base", "description": "Move the robot to a target (x, y, theta) or in a direction for a distance.",
         "parameters": {"x": float, "y": float, "theta": float, "direction": str, "distance_meters": float}},
        {"name": "detect_object", "description": "Detect objects of a certain type/color in the environment.",
         "parameters": {"object_name": str, "color": str}},
        {"name": "pick_object", "description": "Pick up a detected object.",
         "parameters": {"object_id": str}},
        {"name": "place_object", "description": "Place an object at a target (x, y, z) or on a surface.",
         "parameters": {"x": float, "y": float, "z": float, "surface_name": str}},
        {"name": "report_status", "description": "Report current status of the robot.",
         "parameters": {"status_type": str}},
        {"name": "gripper_control", "description": "Open or close the robot's gripper.",
         "parameters": {"command": ["open", "close"]}}
    ]

    # Dynamically build the prompt based on the parsed command
    prompt_messages = [
        {"role": "system", "content": "You are a robot task planner. Your goal is to convert high-level commands into a sequence of precise, low-level ROS 2 actions using the available tools."},
        {"role": "user", "content": f"The user wants the robot to perform the following intent: '{intent}' with parameters: {json.dumps(params)}. "
                                      "Using the following available actions, generate a JSON list of action calls:\n"
                                      f"Available Actions: {json.dumps(available_actions, indent=2)}\n\n"
                                      "Respond only with a JSON array of action objects. Each object should have 'name' and 'parameters'. "
                                      "Ensure all necessary parameters for an action are present. Infer parameters if needed. "
                                      "Example:\n"
                                      "["
                                      "  { \"name\": \"move_base\", \"parameters\": { \"direction\": \"forward\", \"distance_meters\": 1.0 } }"
                                      "]
                                      "\n\nGenerate the action sequence for the command:"}
    ]

    try:
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo-1106", # Use a model with function calling capabilities
            messages=prompt_messages,
            temperature=0.0,
            response_format={"type": "json_object"}
        )
        # The model might return a JSON object with a key, so we need to extract the array
        action_sequence = json.loads(response.choices[0].message.content.strip())
        return action_sequence
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON from LLM action sequence: {e}")
        return []
    except Exception as e:
        print(f"Error during LLM action sequence generation: {e}")
        return []

def main():
    print("Robot Action Sequencer. Type 'exit' to quit.")
    while True:
        user_command = input("Enter robot command (e.g., 'grab the blue box'): ")
        if user_command.lower() == 'exit':
            break
        
        # First, parse the command (from previous lesson)
        # For simplicity, we'll hardcode a parsed command here for demonstration
        # In a real system, you'd call parse_command() from Lesson 2
        if "blue box" in user_command.lower():
            parsed_cmd = {"intent": "grasp_object", "parameters": {"object_name": "box", "color": "blue"}}
        elif "move forward" in user_command.lower():
            parsed_cmd = {"intent": "move_base", "parameters": {"direction": "forward", "distance_meters": 1.0}}
        else:
            parsed_cmd = {"intent": "unknown", "parameters": {}}

        print(f"Parsed Command: {json.dumps(parsed_cmd, indent=2)}")
        
        if parsed_cmd["intent"] != "unknown":
            action_sequence = generate_action_sequence(parsed_cmd)
            print(f"Generated Action Sequence:\n{json.dumps(action_sequence, indent=2)}\n")
        else:
            print("Cannot generate action sequence for unknown intent.\n")

if __name__ == "__main__":
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: The LLM generates actions or parameters not defined in your `available_actions` toolset. Your prompt needs to be very clear about only using the provided tools.
- **Mistake**: The LLM generates a valid action but with incorrect or missing parameters. This requires refining the parameter descriptions in your `available_actions` or providing more examples.
- **Tip**: Test your `available_actions` definitions thoroughly. Make sure they cover all the basic capabilities your robot needs.
- **Tip**: When testing, provide the LLM with the *exact* JSON output from your previous `parse_command` step. This ensures a clean pipeline.

### Mini Assessment
1.  What is the role of the LLM in "sequence generation"?
    a) To directly control the robot's motors.
    b) To translate high-level intent into a precise, ordered list of robot actions.
    c) To transcribe speech to text.
2.  What does it mean to provide the LLM with a "toolset"?
    a) Giving the LLM physical tools.
    b) Describing the available robot actions/services and their parameters to the LLM.
    c) Installing new software on the robot.
3.  Why is JSON a suitable output format for the action sequence?
    a) It's easy for robots to read aloud.
    b) It provides a structured format that can be directly consumed by a Python robot executive.
    c) It makes the output shorter.
4.  If the LLM generates an action sequence that includes an action not present in your `available_actions`, what is the problem?
    a) The robot does not have enough battery.
    b) The prompt instructing the LLM to use only available tools was insufficient.
    c) The user spoke too fast.
5.  What is "Function Calling" with LLMs useful for in this context?
    a) To make the LLM call your phone.
    b) To enable the LLM to interact with external tools and APIs, like robot actions.
    c) To debug the LLM's internal thoughts.

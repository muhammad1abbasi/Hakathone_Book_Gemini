```python
# robust_action_sequencer.py (building on action_sequencer.py)

import openai
import os
import json

openai.api_key = os.getenv("OPENAI_API_KEY")

if not openai.api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set or not provided.")

def parse_and_plan(command: str) -> dict:
    """
    Uses an LLM to detect intent, extract parameters, and generate an action sequence,
    including ambiguity detection and clarification requests.
    """
    
    # Define the available robot actions (this would typically come from a robot's capabilities)
    available_actions = [
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

    prompt = f"""
    You are a robust robot command parser and task planner.
    The user said: "{command}".
    
    Your goal is to perform one of the following:
    1. If the command is clear, directly generate a JSON list of ROS 2 action calls using the available actions.
    2. If the command is ambiguous or parameters are missing, generate a JSON object to ask for clarification.
    
    Available Actions (use only these, and provide all necessary parameters):
    {json.dumps(available_actions, indent=2)}
    
    Output Format:
    - If clear, return: {{ "type": "action_sequence", "actions": [ {{ "name": "...", "parameters": {{...}} }} ] }}
    - If ambiguous, return: {{ "type": "clarification_needed", "question": "..." }}
    
    Example 1: Clear command
    User: "Move forward 2 meters."
    Output: {{ "type": "action_sequence", "actions": [ {{ "name": "move_base", "parameters": {{ "direction": "forward", "distance_meters": 2.0 }} }} ] }}
    
    Example 2: Ambiguous command
    User: "Pick up the block."
    Output: {{ "type": "clarification_needed", "question": "Which block are you referring to? Please specify color or location." }}
    
    Example 3: Unclear command, unknown intent
    User: "Tell me a joke."
    Output: {{ "type": "clarification_needed", "question": "My apologies, I am a robot assistant and cannot tell jokes. I can help with robot control." }}
    
    Your JSON response:
    """
    
    try:
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo-1106", # Use a model with function calling capabilities or good JSON mode
            messages=[
                {"role": "system", "content": "You are a helpful robot command parser and task planner."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.2, # Allow some creativity for clarification questions
            response_format={"type": "json_object"}
        )
        parsed_output = json.loads(response.choices[0].message.content.strip())
        return parsed_output
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON from LLM: {e}")
        return {"type": "error", "message": "JSON decoding failed."}
    except Exception as e:
        print(f"Error during LLM processing: {e}")
        return {"type": "error", "message": str(e)}

def main():
    print("Robust Robot Command Processor. Type 'exit' to quit.")
    while True:
        user_command = input("Enter robot command: ")
        if user_command.lower() == 'exit':
            break
        
        result = parse_and_plan(user_command)
        
        if result.get("type") == "action_sequence":
            print(f"Robot ready to execute actions:\n{json.dumps(result['actions'], indent=2)}\n")
        elif result.get("type") == "clarification_needed":
            print(f"Clarification needed: {result['question']}\n")
        elif result.get("type") == "error":
            print(f"Error: {result['message']}\n")
        else:
            print(f"Unexpected LLM output type: {json.dumps(result, indent=2)}\n")

if __name__ == "__main__":
    main()
```
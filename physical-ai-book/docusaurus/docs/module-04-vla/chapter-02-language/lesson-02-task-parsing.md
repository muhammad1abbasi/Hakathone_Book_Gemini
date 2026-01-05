--- 
id: lesson-02-task-parsing
title: "Lesson 2: Extracting Parameters for Robot Actions"
sidebar_position: 2
description: Using LLMs to parse natural language commands and extract structured parameters (e.g., direction, distance, object name) for robot actions.
---

### Lesson Objective
To use an LLM to not only detect the user's intent but also extract relevant parameters (like "5 meters", "red block") from a natural language command in a structured format (e.g., JSON).

### Prerequisites
- Completion of Lesson 1: "Detecting User Intent with LLMs".
- Familiarity with JSON format.

### Concept Explanation
Once we know the user's intent (e.g., `move_robot`), the next crucial step is to understand the specifics: *how* should the robot move? *where* should it go? *what* object should it grasp? This is where **Task Parsing** comes in. An LLM can be prompted to extract structured information (parameters) from a natural language command.

The key is to ask the LLM for output in a format that your robot's control system can easily consume, such as JSON. This is more robust than trying to parse free-form text.

For example, if the intent is `move_robot`, we might ask the LLM to output:
```json
{
  "intent": "move_robot",
  "parameters": {
    "direction": "forward",
    "distance_meters": 5.0
  }
}
```

Prompt engineering for parameter extraction involves:
1.  **Clear Instructions**: Explicitly tell the LLM what parameters to look for.
2.  **Output Format**: Specify the desired output format (e.g., "Respond only in JSON.").
3.  **Examples (Few-Shot Learning)**: Providing a few examples of input commands and their corresponding JSON output can significantly improve performance.

### Real-World Analogy
Imagine you are a personal assistant taking notes for your boss.
- Your boss says: "Please schedule a meeting with John for next Tuesday at 3 PM about the Q4 report."
- **Intent Detection** is realizing the boss wants to "schedule_meeting."
- **Task Parsing** is you extracting the details: `person="John"`, `day="next Tuesday"`, `time="3 PM"`, `topic="Q4 report"`.
You convert the natural, conversational instruction into a structured list of parameters that can be used to fill out a calendar entry.

### Hands-On Task
**Task**: Extend the intent detector to extract parameters in JSON format.

1.  **Modify `intent_detector.py`**: Open your `intent_detector.py` script from the previous lesson.
2.  **Update `detect_intent` function**: Change its name to `parse_command` and modify the prompt to ask for JSON output including parameters.
3.  **Add `json` import**: Add `import json` to the top of your script.
4.  **Run the Script**:
    ```bash
    python3 intent_detector.py
    ```
    Try commands like "Robot, go back two meters", "Grab the blue cup on the table", "Report your battery status". Observe the structured JSON output.

### Python + ROS 2 Code Example
```python
# intent_parser.py (formerly intent_detector.py)

import openai
import os
import json

# Set your OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")

if not openai.api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set or not provided.")

def parse_command(command: str) -> dict:
    """
    Uses an LLM to detect intent and extract parameters from a natural language command,
    responding in JSON format.
    """
    prompt = f"""
    You are an expert robot command parser.
    The user said: "{command}".
    
    Your task is to identify the core intent and extract any relevant parameters.
    
    Here are the possible intents and their expected parameters:
    - move_robot: {{ "direction": ["forward", "backward", "left", "right"], "distance_meters": float }}
    - grasp_object: {{ "object_name": string, "color": string (optional), "location": string (optional) }}
    - report_status: {{ "status_type": ["battery", "location", "temperature"] }}
    - stop_robot: {{}}
    - unknown: {{ "reason": string }}
    
    Respond only in JSON format. For parameters, try to infer reasonable defaults if not explicitly stated
    (e.g., if "move forward" assume a small default distance). If a parameter cannot be inferred, omit it.
    If the command is unclear, use the "unknown" intent.
    
    Example 1: "Move forward 5 meters"
    {{ "intent": "move_robot", "parameters": {{ "direction": "forward", "distance_meters": 5.0 }} }}
    
    Example 2: "Grab the red block"
    {{ "intent": "grasp_object", "parameters": {{ "object_name": "block", "color": "red" }} }}
    
    Example 3: "Stop!"
    {{ "intent": "stop_robot", "parameters": {{}} }}
    
    Example 4: "Tell me about the weather"
    {{ "intent": "unknown", "parameters": {{ "reason": "Weather is not a robot-related command." }} }}
    
    Your JSON response:
    """
    
    try:
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo", # or "gpt-4" for better parameter extraction
            messages=[
                {"role": "system", "content": "You are a helpful robot command parser."}, 
                {"role": "user", "content": prompt}
            ],
            temperature=0.0, # Keep temperature low for deterministic parsing
            response_format={"type": "json_object"} # Ensure JSON output
        )
        parsed_output = json.loads(response.choices[0].message.content.strip())
        return parsed_output
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON from LLM: {e}")
        return {"intent": "error", "parameters": {"reason": "JSON decoding failed."}}
    except Exception as e:
        print(f"Error during LLM command parsing: {e}")
        return {"intent": "error", "parameters": {"reason": str(e)}}

def main():
    print("Robot Command Parser. Type 'exit' to quit.")
    while True:
        user_command = input("Enter robot command: ")
        if user_command.lower() == 'exit':
            break
        
        parsed_result = parse_command(user_command)
        print(f"Parsed Result:\n{json.dumps(parsed_result, indent=2)}\n")

if __name__ == "__main__":
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: The LLM output is not valid JSON. This usually means your prompt needs to be more explicit about the JSON format, or you might need a stronger model (like GPT-4). The `response_format={"type": "json_object"}` parameter helps enforce this.
- **Mistake**: Missing parameters or incorrect parameter types. Refine your prompt examples (few-shot learning) to cover more edge cases and variations.
- **Tip**: Test with a wide variety of commands, including ambiguous ones, to see how robust your parser is. If a command is too vague, the LLM might struggle to extract parameters.

### Mini Assessment
1.  What is the main goal of Task Parsing?
    a) To translate the command into a different language.
    b) To extract structured parameters from a natural language command.
    c) To generate a summary of the command.
2.  Why is JSON a good format for LLM output in this context?
    a) It's human-readable.
    b) It's easily consumed by programming languages for structured data.
    c) It makes the LLM response longer.
3.  What is "few-shot learning" in prompt engineering?
    a) Giving the LLM a few chances to answer.
    b) Providing examples of input/output in the prompt to guide the LLM.
    c) Limiting the LLM to only a few parameters.
4.  If the LLM frequently misses a specific parameter (e.g., `object_name`), what should you do to improve its performance?
    a) Reduce the `temperature` to 0.
    b) Provide more explicit instructions and examples in the prompt for that parameter.
    c) Use a simpler LLM model.
5.  What parameter should be used with the OpenAI API to try and force the output to be JSON?
    a) `output_format="json"`
    b) `response_format={"type": "json_object"}`
    c) `json_only=True`
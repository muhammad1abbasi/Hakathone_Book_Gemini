---
id: lesson-01-intent-detection
title: "Lesson 1: Detecting User Intent with LLMs"
sidebar_position: 1
description: Leveraging LLMs to classify natural language commands and extract the user's core intent.
---

### Lesson Objective
To use a Large Language Model (LLM) to classify a natural language command into a predefined set of robot intents (e.g., "move," "grasp," "report").

### Prerequisites
- Completion of Module 4, Chapter 1 (Voice Interfaces).
- Basic understanding of LLMs and API interaction (e.g., OpenAI API).

### Concept Explanation
After converting speech to text with Whisper, the next step in our VLA pipeline is to understand what the user *wants* the robot to do. This is called **Intent Detection**. An LLM is incredibly good at this.

Instead of writing complex rule-based systems, we can simply ask the LLM to identify the user's intent from a given text command. This is achieved through **prompt engineering**, where we craft a clear instruction for the LLM.

A typical prompt for intent detection might look like this:
```
"The user said: '{command}'. What is their core intent? Choose from: 'move_robot', 'grasp_object', 'report_status', 'stop_robot'."
```

The LLM will then output one of the predefined intents. This structured output is crucial because it allows our robot's control system to take specific actions. For example, if the intent is `move_robot`, we know to expect parameters like `direction` and `distance`.

### Real-World Analogy
Think of a customer service hotline where you press "1" for Sales, "2" for Support, "3" for Billing.
- **Your spoken command** (e.g., "I need help with my bill") is the input.
- The **LLM** acts as the automated system, listening to your request.
- **Intent Detection** is like the system classifying your request as "Billing," directing you to the correct department.
This process simplifies a complex natural language input into a discrete, actionable category.

### Hands-On Task
**Task**: Use an LLM to detect the intent from a user command.

1.  **Get an OpenAI API Key**: If you don't have one, sign up at OpenAI and obtain an API key.
2.  **Install OpenAI Library**:
    ```bash
    pip install openai
    ```
3.  **Create Script**: Create a Python script named `intent_detector.py`.
4.  **Write the Code**: Copy the Python code from the example below. Replace `YOUR_OPENAI_API_KEY` with your actual key.
5.  **Run the Script**:
    ```bash
    python3 intent_detector.py
    ```
    The script will ask you for a command. Try phrases like "Move forward five meters", "Pick up the red block", or "What is my current battery level?". Observe the LLM's detected intent.

### Python + ROS 2 Code Example
```python
# intent_detector.py

import openai
import os

# Set your OpenAI API key (replace with your actual key or use environment variable)
# openai.api_key = "YOUR_OPENAI_API_KEY"
# OR
# Set environment variable: export OPENAI_API_KEY='your_key_here'
openai.api_key = os.getenv("OPENAI_API_KEY")

if not openai.api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set or not provided.")

def detect_intent(command: str) -> str:
    """
    Uses an LLM to detect the core intent from a natural language command.
    """
    robot_intents = [
        "move_robot",
        "grasp_object",
        "report_status",
        "stop_robot",
        "unknown" # Fallback for unclear commands
    ]
    
    prompt = f"""
    You are an expert robot assistant.
    The user wants the robot to perform an action based on the following command: "{command}".
    What is the primary, overarching intent of this command? 
    
    Choose from the following predefined intents: {', '.join(robot_intents)}.
    Respond only with the chosen intent, no other text or explanation.
    """
    
    try:
        response = openai.chat.completions.create(
            model="gpt-3.5-turbo", # or "gpt-4" for better results
            messages=[
                {"role": "system", "content": "You are a helpful robot assistant focused on intent detection."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.0, # Keep temperature low for deterministic intent detection
            max_tokens=20 # Keep it short for just the intent word
        )
        intent = response.choices[0].message.content.strip()
        return intent if intent in robot_intents else "unknown"
    except Exception as e:
        print(f"Error during LLM intent detection: {e}")
        return "error"

def main():
    print("Robot Intent Detector. Type 'exit' to quit.")
    while True:
        user_command = input("Enter robot command: ")
        if user_command.lower() == 'exit':
            break
        
        detected_intent = detect_intent(user_command)
        print(f"Detected Intent: {detected_intent}\n")

if __name__ == "__main__":
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Not setting the `OPENAI_API_KEY` correctly. This will result in an authentication error.
- **Mistake**: The LLM returns an intent not in your predefined list. This often means your prompt is unclear or your intent list is not comprehensive enough for the user's possible commands. Add a "fallback" intent like `unknown`.
- **Tip**: Keep the `temperature` parameter low (e.g., 0.0 or 0.1) for intent detection. This makes the LLM's responses more deterministic and less creative, which is desirable for classification tasks.

### Mini Assessment
1.  What is "Intent Detection" in the context of VLA robotics?
    a) Converting speech to text.
    b) Classifying a natural language command into a predefined robot goal.
    c) Generating a robot's movement path.
2.  Why is structured output from the LLM (e.g., a single intent word) important for robotics?
    a) It makes the output more human-readable.
    b) It allows the robot's control system to take specific, predefined actions.
    c) It's required by the OpenAI API.
3.  What is "prompt engineering" in this context?
    a) Designing a robot's physical structure.
    b) Crafting clear instructions for the LLM to get desired output.
    c) Writing Python code for ROS 2.
4.  Why is it recommended to keep the `temperature` parameter low for intent detection with LLMs?
    a) To make the LLM's responses more creative.
    b) To make the LLM's responses more deterministic and reliable for classification.
    c) To save computational cost.
5.  If an LLM returns an intent like "dance" but it's not in your `robot_intents` list, what should you do?
    a) Add "dance" to your robot's capabilities and the intent list.
    b) Consider if "dance" maps to an existing intent or add a fallback "unknown" intent.
    c) Ignore the command.

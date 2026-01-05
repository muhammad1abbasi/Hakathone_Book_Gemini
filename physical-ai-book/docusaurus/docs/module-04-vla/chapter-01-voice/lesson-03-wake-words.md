---
id: lesson-03-wake-words
title: "Lesson 3: Implementing Wake Words"
sidebar_position: 3
description: Integrate wake word detection (e.g., "Hey Robot") to activate your voice interface on demand.
---

### Lesson Objective
To implement a simple wake word detection system using a lightweight library, allowing the robot's voice interface to activate only when addressed.

### Prerequisites
- Completion of Lesson 2: "Audio Preprocessing for Robust ASR".
- Python 3.9+ installed.

### Concept Explanation
For a robot to be truly interactive, it shouldn't be constantly listening and transcribing everything. This is computationally expensive, generates a lot of irrelevant data, and can be a privacy concern. Instead, most voice assistants use a **wake word** (or hotword) system.

A wake word detector is a small, lightweight model that continuously listens for a specific phrase (e.g., "Hey Robot," "Computer," "Alexa"). Once the wake word is detected, it "wakes up" the more resource-intensive ASR system (like Whisper) to start transcribing the subsequent speech. This makes the system much more efficient and user-friendly.

Popular open-source wake word libraries include:
- **Picovoice Porcupine**: Highly accurate, supports custom wake words, runs on edge devices.
- **Mycroft Precise**: Another open-source option, uses deep learning.
- **Mozilla DeepSpeech (for custom training)**: Requires more effort for custom wake words.

For this lesson, we will use a simple, illustrative approach, but in a real system, you'd integrate a specialized wake word engine. The concept is that audio is constantly monitored, but only if the wake word is heard is the audio buffered and sent to Whisper.

### Real-World Analogy
Think of a dog. You don't want your dog to respond to every single sound in the house. Instead, it's trained to respond to its name.
- Your dog constantly **listens** (the lightweight wake word detector).
- When it hears its **name** (the wake word), it perks up and pays attention.
- Then, it starts to **process your commands** (the ASR system, Whisper).
This prevents constant attention and unnecessary processing.

### Hands-On Task
**Task**: Create a simple Python script that simulates wake word detection, then activates the ASR.

1.  **Install Libraries**:
    ```bash
    pip install pydub SpeechRecognition
    ```
2.  **Create Wake Word Simulation Script**: Create a Python script named `wake_word_simulator.py`.
3.  **Write the Code**: Copy the Python code from the example below. This script will listen to your microphone. When it detects a phrase like "Hey Robot", it will then start transcribing for a short period.
4.  **Run the Script**:
    ```bash
    python3 wake_word_simulator.py
    ```
    Speak "Hey Robot" followed by a command. Observe how it only transcribes after the wake word.

### Python + ROS 2 Code Example
```python
# wake_word_simulator.py

import speech_recognition as sr
import whisper
import time
from datetime import datetime, timedelta

# Load Whisper model (or use a smaller one like 'tiny' for faster processing)
asr_model = whisper.load_model("base")

# Simulated wake word detection function
# In a real system, this would be a specialized, lightweight model (e.g., Porcupine)
def detect_wake_word(audio_chunk, wake_phrase="hey robot"):
    try:
        # A lightweight ASR might be used for simple wake word detection
        # or a dedicated model. For this example, we'll use a fast ASR pass.
        # Note: This is computationally more intensive than a true wake word engine.
        temp_result = asr_model.transcribe(audio_chunk, language='en', fp16=False, initial_prompt=wake_phrase, verbose=False)
        transcription = temp_result['text'].strip().lower()
        if wake_phrase in transcription:
            print(f"Wake word detected! (Transcription: '{transcription}')")
            return True
        return False
    except Exception:
        return False

def main():
    r = sr.Recognizer()
    r.pause_threshold = 0.5 # Shorter pause for more continuous listening
    r.energy_threshold = 700 # Adjust based on ambient noise

    print("Listening for wake word 'Hey Robot'...")

    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        listening_for_command = False
        last_wake_word_time = datetime.now()

        while True:
            try:
                audio = r.listen(source, phrase_time_limit=3) # Listen for up to 3 seconds
                
                audio_np = np.frombuffer(audio.frame_data, dtype=np.int16).astype(np.float32) / 32768.0
                audio_np = whisper.audio.pad_or_trim(audio_np)
                
                # If not currently listening for a command, check for wake word
                if not listening_for_command:
                    if detect_wake_word(audio_np):
                        listening_for_command = True
                        last_wake_word_time = datetime.now()
                        print("Wake word received. Listening for command...")
                elif listening_for_command:
                    # After wake word, transcribe the command
                    command_text = asr_model.transcribe(audio_np, language='en', fp16=False)["text"].strip()
                    if command_text:
                        print(f"Command received: {command_text}")
                        # Here, you would send the command to an LLM or directly to the robot
                    
                    # If silence or a short command, stop listening for command after a timeout
                    if datetime.now() - last_wake_word_time > timedelta(seconds=5): # Listen for 5 seconds after wake word
                        listening_for_command = False
                        print("Timeout. Listening for wake word again...")

            except sr.WaitTimeoutError:
                # No speech detected in the phrase_time_limit
                if listening_for_command and datetime.now() - last_wake_word_time > timedelta(seconds=3):
                    listening_for_command = False
                    print("Silence detected. Listening for wake word again...")
                # else: print("...") # uncomment to see constant listening
            except Exception as e:
                print(f"Error: {e}")
            
            time.sleep(0.1) # Prevent busy-waiting

if __name__ == "__main__":
    import numpy as np
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: Using a full ASR model for continuous wake word detection. This is inefficient and will consume too many resources. Dedicated wake word engines are optimized for this.
- **Tip**: Adjust `energy_threshold` and `pause_threshold` in `speech_recognition` to suit your environment's noise level and your speaking patterns. Test in the actual robot environment.
- **Tip**: For real-time applications, processing audio in short chunks is essential to minimize latency.

### Mini Assessment
1.  What is the primary purpose of a wake word system in a voice interface?
    a) To make the robot move faster.
    b) To activate the voice interface efficiently and on-demand.
    c) To translate commands into different languages.
2.  Which of the following is an advantage of using a wake word system?
    a) It requires more powerful hardware.
    b) It reduces computational cost and improves privacy.
    c) It makes the robot's voice louder.
3.  What happens after a wake word is successfully detected?
    a) The robot immediately executes a predefined action.
    b) The more resource-intensive ASR system activates to transcribe the subsequent command.
    c) The robot goes to sleep.
4.  In a real wake word system, is a full ASR model typically used for continuous detection?
    a) Yes, always.
    b) No, a more lightweight, specialized model is used.
    c) Only for very short wake words.
5.  If your wake word system is constantly activating without you speaking the wake word, what might you need to adjust?
    a) The robot's color.
    b) The `energy_threshold` in `speech_recognition`.
    c) The model size of Whisper.

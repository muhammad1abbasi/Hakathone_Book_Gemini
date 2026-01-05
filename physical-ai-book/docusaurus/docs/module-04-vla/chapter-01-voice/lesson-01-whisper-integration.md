--- 
id: lesson-01-whisper-integration
title: "Lesson 1: Integrating OpenAI Whisper"
sidebar_position: 1
description: Setting up and using OpenAI's Whisper model for accurate speech-to-text conversion.
---

### Lesson Objective
To install OpenAI Whisper, transcribe an audio file, and set up a basic Python script for real-time audio transcription.

### Prerequisites
- Python 3.9+ installed.
- `pip` package manager.
- Basic understanding of Python scripting.
- (Optional) Access to a microphone for live transcription.

### Concept Explanation
**OpenAI Whisper** is a general-purpose Automatic Speech Recognition (ASR) model. It's trained on a massive dataset of diverse audio and performs exceptionally well on various tasks like transcribing speech, identifying languages, and even translating. For robotics, Whisper provides a robust way to convert spoken human commands into text that an LLM (and subsequently, our robot) can understand.

Whisper comes in several model sizes (tiny, base, small, medium, large), trading off accuracy for speed and computational requirements. For real-time robotics, often a smaller model like `base` or `small` is sufficient and performs much faster.

The core steps for using Whisper are:
1.  **Installation**: Install the `openai-whisper` Python package.
2.  **Model Loading**: Load a specific Whisper model (e.g., `whisper.load_model("base")`). This downloads the model weights the first time.
3.  **Transcription**: Pass an audio file or a NumPy array of audio data to the model's `transcribe()` method.

For real-time applications, you typically integrate Whisper with an audio recording library (like `pyaudio`) to capture microphone input in chunks and feed it to Whisper.

### Real-World Analogy
Think of Whisper as a highly skilled stenographer that can understand many languages and accents.
- You speak a command to your robot.
- Whisper is listening, and like a stenographer, it quickly converts your spoken words into written text.
- This text can then be passed to a lawyer (our LLM) to understand the meaning and decide what to do next.

### Hands-On Task
**Task**: Transcribe an audio file and set up a basic real-time transcriber.

1.  **Install Whisper**:
    ```bash
    pip install openai-whisper
    # For real-time audio capture
    pip install pyaudio SpeechRecognition
    ```
2.  **Download Sample Audio**: Find a short `.wav` or `.mp3` audio file online, or record your own speaking a command like "Robot, go to the kitchen." Save it as `command.wav` in your working directory.
3.  **Create Transcription Script**: Create a Python script named `whisper_transcriber.py`.
4.  **Write the Code**: Copy the Python code from the example below.
5.  **Run the Script**:
    ```bash
    python3 whisper_transcriber.py
    ```
    You should see the transcription of your audio file printed to the console, followed by real-time transcription of any speech detected from your microphone.

### Python + ROS 2 Code Example
```python
# whisper_transcriber.py

import whisper
import speech_recognition as sr
import time

# --- Part 1: Transcribe an Audio File ---
print("--- Part 1: Transcribing an Audio File ---")
try:
    model = whisper.load_model("base") # or "small", "medium" etc.
    result = model.transcribe("command.wav") # Make sure command.wav exists
    print(f"File Transcription: {result['text']}\n")
except FileNotFoundError:
    print("Error: command.wav not found. Please create or download a sample audio file.")
except Exception as e:
    print(f"Error during file transcription: {e}")


# --- Part 2: Basic Real-time Microphone Transcription ---
# This is a simplified example. For robust ROS 2 integration,
# you would typically use a ROS 2 Audio Capture node.

print("--- Part 2: Real-time Microphone Transcription (Speak now) ---")
r = sr.Recognizer()
r.pause_threshold = 0.8 # seconds of non-speaking audio before a phrase is considered complete
r.energy_threshold = 400 # minimum audio energy to consider for speech

with sr.Microphone() as source:
    print("Adjusting for ambient noise, please wait...")
    r.adjust_for_ambient_noise(source)
    print("Say something!")
    
    try:
        while True:
            audio = r.listen(source) # Listen for speech
            print("Processing audio...")
            
            # Using Whisper for transcription
            # sr.AudioData objects can be directly passed to whisper.transcribe
            audio_np = whisper.audio.load_audio(audio.frame_data, sr=whisper.audio.SAMPLE_RATE)
            transcription = model.transcribe(audio_np)["text"]
            
            if transcription.strip(): # Only print if not empty
                print(f"You said: {transcription}")
            
            time.sleep(0.1) # Small delay to prevent CPU overuse
            
    except KeyboardInterrupt:
        print("\nStopping real-time transcription.")
    except Exception as e:
        print(f"Error during real-time transcription: {e}")

```

### Common Mistakes & Debugging Tips
- **Mistake**: Incorrectly installing `pyaudio` on Linux. You might need `sudo apt-get install portaudio19-dev python3-pyaudio`.
- **Mistake**: Using too large a Whisper model for real-time applications. Larger models are more accurate but much slower. Start with `base` or `small` for live processing.
- **Tip**: If real-time transcription is choppy, try adjusting `r.pause_threshold` (how long of silence before it thinks you're done speaking) and `r.energy_threshold` (how loud the sound needs to be to be considered speech).

### Mini Assessment
1.  What is the primary function of OpenAI Whisper in VLA robotics?
    a) Generating robot movements.
    b) Converting spoken language into text.
    c) Understanding visual scenes.
2.  Which of the following is NOT a model size for Whisper?
    a) tiny
    b) huge
    c) large
3.  Why might you choose a smaller Whisper model (e.g., "base") for real-time applications?
    a) It is more accurate.
    b) It is faster and requires less computational power.
    c) It supports more languages.
4.  What Python package is commonly used to capture microphone input for real-time transcription?
    a) `numpy`
    b) `speech_recognition` (which uses `pyaudio`)
    c) `whisper`
5.  What does ASR stand for?
    a) Advanced Speech Recognition
    b) Automatic Speech Recognition
    c) Autonomous Speech Robotics

--- 
id: lesson-02-audio-preprocessing
title: "Lesson 2: Audio Preprocessing for Robust ASR"
sidebar_position: 2
description: Techniques for filtering noise, normalizing volume, and handling audio segments to improve ASR accuracy.
---

### Lesson Objective
To apply common audio preprocessing techniques (noise reduction, volume normalization) to improve the accuracy of Whisper's speech-to-text transcription in a robotics environment.

### Prerequisites
- Completion of Lesson 1: "Integrating OpenAI Whisper".
- Familiarity with NumPy for array manipulation.

### Concept Explanation
Real-world robotics environments are often noisy. Fans, motor hum, background conversations, and ambient sounds can significantly degrade the performance of any ASR system, including Whisper. **Audio preprocessing** involves applying digital signal processing (DSP) techniques to clean up the audio before feeding it to the ASR model, thereby improving transcription accuracy.

Common preprocessing techniques include:
1.  **Noise Reduction**: Removing or minimizing static, hum, or other persistent background noises. Libraries like `pydub` or `noisereduce` can be very effective.
2.  **Volume Normalization**: Adjusting the audio's volume to a consistent level. This helps Whisper perform consistently, regardless of how loud or soft the original speech was.
3.  **Silence Removal/Segmentation**: Removing long periods of silence from the audio or segmenting audio into discrete speech chunks. This can reduce processing time and prevent the ASR from "listening" to nothing.
4.  **Resampling**: Ensuring the audio is at the optimal sample rate (e.g., 16 kHz for Whisper) and bit depth.

These techniques are crucial for making your voice interface robust enough for practical robotics applications.

### Real-World Analogy
Imagine you are trying to hear a friend speak in a very crowded, noisy room.
- **Noise Reduction** is like moving to a quieter corner or asking people to quiet down.
- **Volume Normalization** is like asking your friend to speak up if they're too quiet, or asking them to lower their voice if they're shouting.
- **Silence Removal** is like cutting out all the times your friend paused and didn't say anything, so you only get the spoken words.
All these actions help you understand what your friend is saying more clearly.

### Hands-On Task
**Task**: Apply basic noise reduction and volume normalization to an audio file.

1.  **Install Libraries**:
    ```bash
    pip install pydub noisereduce soundfile
    ```
2.  **Get a Noisy Audio File**: Record a new `command_noisy.wav` file where you speak a command while there's some background noise (e.g., a fan, music playing quietly).
3.  **Create Preprocessing Script**: Create a Python script named `audio_processor.py`.
4.  **Write the Code**: Copy the Python code from the example below.
5.  **Run the Script**:
    ```bash
    python3 audio_processor.py
    ```
    Compare the transcription of the original noisy file with the processed file. You should see an improvement in accuracy.

### Python + ROS 2 Code Example
```python
# audio_processor.py

import whisper
from pydub import AudioSegment
from pydub.silence import split_on_silence
import noisereduce as nr
import soundfile as sf
import numpy as np

# Load Whisper model
model = whisper.load_model("base")

def transcribe_audio(audio_path):
    """Transcribes an audio file using Whisper."""
    try:
        result = model.transcribe(audio_path)
        return result['text']
    except Exception as e:
        return f"Transcription Error: {e}"

def process_audio(input_path, output_path="processed_audio.wav"):
    """Applies noise reduction and normalization to an audio file."""
    print(f"Processing {input_path}...")
    audio = AudioSegment.from_wav(input_path)

    # Convert to NumPy array for noisereduce
    samples = np.array(audio.get_array_of_samples())
    if audio.sample_width == 2: # 16-bit audio
        samples = samples.astype(np.int16)
    elif audio.sample_width == 4: # 32-bit audio
        samples = samples.astype(np.int32)
    
    # Noise Reduction
    # We need to compute noise profile from a silent part, or assume it.
    # For simplicity, we assume noise is present throughout for this example.
    # In real apps, you'd isolate a silent segment.
    reduced_noise_samples = nr.reduce_noise(y=samples, sr=audio.frame_rate, 
                                            stationary=True, prop_decrease=0.8)
    
    # Convert back to AudioSegment
    processed_audio = AudioSegment(
        reduced_noise_samples.tobytes(), 
        frame_rate=audio.frame_rate,
        sample_width=audio.sample_width,
        channels=audio.channels
    )

    # Volume Normalization (e.g., to -20 dBFS)
    normalized_audio = processed_audio.normalize(headroom=2.0) # Adjusts peak to -2dBFS

    # Export processed audio
    normalized_audio.export(output_path, format="wav")
    print(f"Processed audio saved to {output_path}")
    return output_path

def main():
    input_file = "command_noisy.wav" # Ensure this file exists

    print(f"Original transcription of {input_file}:")
    original_text = transcribe_audio(input_file)
    print(original_text)

    processed_file = process_audio(input_file)

    print(f"\nTranscription of processed {processed_file}:")
    processed_text = transcribe_audio(processed_file)
    print(processed_text)

if __name__ == '__main__':
    main()

```

### Common Mistakes & Debugging Tips
- **Mistake**: Over-processing audio. Too much noise reduction can remove parts of the speech, and excessive normalization can introduce distortion. Find a balance.
- **Tip**: Always listen to the processed audio. Sometimes what looks good on a waveform plot doesn't sound good to a human ear or for an ASR model.
- **Tip**: For real-time applications, processing audio in small chunks (`chunk_size` for `pyaudio`) is crucial to minimize latency. You'd apply these techniques to each incoming chunk.

### Mini Assessment
1.  What is the main goal of audio preprocessing for ASR?
    a) To make the audio louder.
    b) To clean up audio to improve transcription accuracy.
    c) To convert audio into video.
2.  Which technique adjusts the audio's volume to a consistent level?
    a) Noise reduction.
    b) Silence removal.
    c) Volume normalization.
3.  Why is noise reduction important in robotics environments?
    a) Robots are sensitive to loud noises.
    b) Background noise can significantly degrade ASR performance.
    c) It makes the robot's voice sound better.
4.  What happens if you use too much noise reduction?
    a) It makes the audio sound clearer.
    b) It can remove parts of the actual speech.
    c) It increases the file size.
5.  What is the recommended sample rate for Whisper?
    a) 44.1 kHz
    b) 8 kHz
    c) 16 kHz

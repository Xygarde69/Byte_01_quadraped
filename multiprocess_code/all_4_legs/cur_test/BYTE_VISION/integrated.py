# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VISION/integrated.py
# STATUS    ·  PROTOTYPE · vision (BYTE_VISION)
# ────────────────────────────────────────────────────────────────────────
# Voice + vision agent: a voice thread (speech_recognition + pyttsx3,
# 'Yes?' prompt) answers through Google Gemini, while a vision thread
# runs YOLOv8 on RealSense frames.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 integrated.py
# NOTE      ·  SECURITY: contains a hard-coded Google API key (line ~13)
#              that is in git history and on the remote. Revoke it and
#              read a new one from an environment variable.
# ════════════════════════════════════════════════════════════════════════
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import threading
import time
import speech_recognition as sr
import pyttsx3
import google.generativeai as genai
import os

# --- 1. CONFIGURATION ---
# REPLACE WITH YOUR ACTUAL API KEY
genai.configure(api_key=os.environ["GEMINI_API_KEY"])
# Using 'gemini-pro' as it is the most stable for your current library version
model_llm = None
try:
    print("Searching for available models...")
    for m in genai.list_models():
        if 'generateContent' in m.supported_generation_methods:
            print(f" Found and using model: {m.name}")
            model_llm = genai.GenerativeModel(m.name)
            break
except Exception as e:
    print(f"Could not list models: {e}")

if model_llm is None:
    print("❌ CRITICAL: No compatible models found. Check your API key or internet.")

WAKE_WORD = "hello"

# --- 2. GLOBAL SHARED MEMORY ---
shared_vision_state = {}
vision_lock = threading.Lock()

# --- 3. THE MOUTH (Thread-Safe Version) ---
def speak(text):
    """Initializes a fresh engine for every call to avoid threading crashes."""
    print(f"🤖 Agent: {text}")
    def _run_tts():
        try:
            engine = pyttsx3.init()
            engine.setProperty('rate', 175)
            # Optional: Change to female voice if available
            # voices = engine.getProperty('voices')
            # engine.setProperty('voice', voices[1].id)
            engine.say(text)
            engine.runAndWait()
            engine.stop()
        except Exception as e:
            print(f"TTS Error: {e}")
            
    threading.Thread(target=_run_tts).start()

# --- 4. THE BRAIN (LLM Logic) ---
def process_with_llm(user_input):
    global model_llm
    if model_llm is None:
        return "My brain is not initialized. Please check the terminal."
    
    with vision_lock:
        current_view = shared_vision_state.copy()
    
    # Context injection for the robot's personality
    prompt = f"""
    You are Byte 01, an advanced autonomous dog robot located at SRM KTR, Chennai.
    You have two jobs:
    1. Answer questions about your environment using the provided Camera Data.
    2. Answer general questions (weather, math, facts) using your vast AI knowledge.
    
    Current User Question: "{user_input}"
    
    Live Camera Data (Object: Distance in meters):
    {current_view}
    
    INSTRUCTIONS:
    - Be conversational and brief (max 2 sentences).
    - If asked about objects/distances, use the Camera Data.
    - If asked about the weather or general facts, answer directly. (Note: It is March 2026 in Chennai).
    - Do NOT say you don't have access to data; you are an AI, use your training.
    """
    
    try:
        response = model_llm.generate_content(prompt)
        try:
            return response.text.replace('*', '') # Clean for TTS
        except:
            return "I can see the objects in front of me, but I'm having a moment of confusion regarding that question."
    except Exception as e:
        print(f"LLM Error: {e}")
        return "I am having trouble accessing my cloud brain right now."

# --- 5. THE EAR (Voice Thread) ---
def voice_loop():
    recognizer = sr.Recognizer()
    # We move the microphone context INSIDE the loop to refresh it
    
    print(f"🎙️ System Ready. Say '{WAKE_WORD}' to interact.")
    speak("System online. I am ready.")

    while True:
        try:
            with sr.Microphone() as source:
                # Refresh calibration slightly to handle ambient noise changes
                recognizer.adjust_for_ambient_noise(source, duration=0.5)
                
                print(f"--- Waiting for '{WAKE_WORD}' ---")
                audio = recognizer.listen(source, timeout=None, phrase_time_limit=3)
            
            text = recognizer.recognize_google(audio).lower()
            print(f"Heard: {text}")

            if WAKE_WORD in text:
                speak("Yes?") 
                
                # Small pause to let the "Yes?" finish speaking before listening
                time.sleep(0.5) 

                with sr.Microphone() as source:
                    print("Listening for command...")
                    command_audio = recognizer.listen(source, timeout=5, phrase_time_limit=8)
                
                command_text = recognizer.recognize_google(command_audio)
                print(f"🗣️ User: {command_text}")
                
                answer = process_with_llm(command_text)
                speak(answer)
                
                # Give the agent time to finish speaking before the loop restarts
                time.sleep(2) 
                
        except Exception as e:
            # If it didn't hear anything or timed out, just silently restart the loop
            continue

# --- 6. THE EYE (Vision & Depth Thread) ---
def vision_loop():
    global shared_vision_state
    
    # Using Medium model for better accuracy-speed balance
    model = YOLO("yolov8m-seg.pt") 

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    
    align = rs.align(rs.stream.color)
    spatial_filter = rs.spatial_filter()
    hole_filling_filter = rs.hole_filling_filter()

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame: continue

            # Apply your high-accuracy filters
            filtered_depth = spatial_filter.process(depth_frame)
            filtered_depth = hole_filling_filter.process(filtered_depth)
            
            depth_image = np.asanyarray(filtered_depth.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Local Inference
            results = model(color_image, stream=True, verbose=False)
            frame_data = {}

            for r in results:
                if r.masks is not None:
                    for i, mask_points in enumerate(r.masks.xy):
                        # Create Stencil
                        pts = np.array(mask_points, dtype=np.int32)
                        stencil = np.zeros(depth_image.shape, dtype=np.uint8)
                        cv2.fillPoly(stencil, [pts], 255)

                        # Calculate Depth
                        object_depths = depth_image[stencil == 255]
                        valid = object_depths[object_depths > 0]

                        if len(valid) > 0:
                            dist = np.median(valid) * depth_scale
                            name = model.names[int(r.boxes.cls[i])]
                            frame_data[name] = round(float(dist), 2)

                            # Visuals
                            x1, y1, x2, y2 = map(int, r.boxes.xyxy[i])
                            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(color_image, f"{name}: {dist:.2f}m", (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Update Shared Memory
            with vision_lock:
                shared_vision_state = frame_data

            cv2.imshow('Byte01 - Vision System', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

# --- 7. MAIN START ---
if __name__ == "__main__":
    # Start Voice Assistant in background
    voice_thread = threading.Thread(target=voice_loop, daemon=True)
    voice_thread.start()

    # Start Vision System (Main Thread)
    vision_loop()
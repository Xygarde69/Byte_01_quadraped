# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VISION/enrollment.py
# STATUS    ·  PROTOTYPE · vision (BYTE_VISION)
# ────────────────────────────────────────────────────────────────────────
# Enroll a person: asks for a name in the terminal, then captures face
# images with an Intel RealSense camera into team_members/.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 enrollment.py
# ════════════════════════════════════════════════════════════════════════
import pyrealsense2 as rs
import numpy as np
import cv2
import face_recognition
import os

# --- INITIAL SETUP ---
SAVE_PATH = "team_members"
if not os.path.exists(SAVE_PATH):
    os.makedirs(SAVE_PATH)

# Get the name from the terminal
person_name = input("Enter the name of the person to enroll: ").strip().replace(" ", "_")

# Configure RealSense Pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print(f"\n--- Enrollment Started for: {person_name} ---")
print("Controls:")
print("  [S] - Save Image")
print("  [Q] - Quit/Finish")

try:
    while True:
        # Get frames from RealSense
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert to numpy array
        frame = np.asanyarray(color_frame.get_data())
        
        # Display frame (we create a copy for drawing so we save the clean version)
        display_frame = frame.copy()

        # Face Detection for the Preview Box
        # We shrink the image to 1/4 size for faster box rendering
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        face_locations = face_recognition.face_locations(rgb_small_frame)

        # Draw the box on the display frame
        for (top, right, bottom, left) in face_locations:
            # Scale back up
            top *= 4; right *= 4; bottom *= 4; left *= 4
            cv2.rectangle(display_frame, (left, top), (right, bottom), (0, 255, 0), 2)
            cv2.putText(display_frame, "Face Detected", (left, top - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Overlay UI text
        cv2.putText(display_frame, f"Target: {person_name}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imshow('Byte-01 Enrollment Tool', display_frame)

        key = cv2.waitKey(1) & 0xFF

        # Save Logic
        if key == ord('s'):
            file_name = f"{SAVE_PATH}/{person_name}.jpg"
            
            # If file exists, create a numbered version
            counter = 1
            while os.path.exists(file_name):
                file_name = f"{SAVE_PATH}/{person_name}_{counter}.jpg"
                counter += 1
            
            # Save the CLEAN frame (without the green box)
            cv2.imwrite(file_name, frame)
            print(f"Captured: {file_name}")

        elif key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
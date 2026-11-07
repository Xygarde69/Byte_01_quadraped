# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VISION/vision.py
# STATUS    ·  PROTOTYPE · vision (BYTE_VISION)
# ────────────────────────────────────────────────────────────────────────
# Face recognition against the photos in team_members/, using a
# RealSense colour+depth stream (depth is used to estimate how far away
# the face is).
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 vision.py
# ════════════════════════════════════════════════════════════════════════
import pyrealsense2 as rs
import numpy as np
import cv2
import face_recognition
import os

# --- 1. PREP ---
KNOWN_DIR = "team_members"
known_encodings = []
known_names = []

for file in os.listdir(KNOWN_DIR):
    if file.lower().endswith(('.png', '.jpg', '.jpeg')):
        img = face_recognition.load_image_file(os.path.join(KNOWN_DIR, file))
        encs = face_recognition.face_encodings(img)
        if encs:
            known_encodings.append(encs[0])
            known_names.append(os.path.splitext(file)[0].split('_')[0])

# --- 2. REALSENSE SETUP ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)

align = rs.align(rs.stream.color)

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame: continue

        frame = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            top *= 4; right *= 4; bottom *= 4; left *= 4
            
            # --- BULLETPROOF 3D CHECK ---
            is_real_3d = False
            
            # 1. CENTER CROP: Only look at the inner 50% of the box (The "Nose/Cheek" zone)
            height = bottom - top
            width = right - left
            c_top = top + int(height * 0.25)
            c_bot = bottom - int(height * 0.25)
            c_left = left + int(width * 0.25)
            c_right = right - int(width * 0.25)
            
            # Ensure we don't go out of bounds
            if c_top >= 0 and c_bot < 480 and c_left >= 0 and c_right < 640:
                face_depth = depth_image[c_top:c_bot, c_left:c_right]
                total_pixels = face_depth.size
                valid_pixels = face_depth[face_depth > 0]
                
                # 2. IR DENSITY CHECK & GAP CHECK
                if total_pixels > 0:
                    valid_ratio = valid_pixels.size / total_pixels
                    
                    # A real face reflects IR well (>70% valid). A phone screen scatters it.
                    if valid_ratio > 0.60 and valid_pixels.size > 50:
                        near_point = np.percentile(valid_pixels, 10) 
                        far_point = np.percentile(valid_pixels, 90)  
                        depth_gap = far_point - near_point
                        
                        # Gap must be between 30mm and 250mm
                        if 20 < depth_gap < 300: 
                            is_real_3d = True

            # --- RECOGNITION LOGIC ---
            matches = face_recognition.compare_faces(known_encodings, face_encoding, 0.5)
            name = "UNKNOWN"
            
            if True in matches:
                if is_real_3d:
                    best_match = np.argmin(face_recognition.face_distance(known_encodings, face_encoding))
                    name = known_names[best_match]
                    hud_color = (120, 255, 120) # Success Green
                else:
                    name = "SPOOF (2D REFLECTION)"
                    hud_color = (0, 0, 255) # Warning Red
            else:
                hud_color = (200, 200, 200) # Neutral Gray

            # --- MINIMALIST UI ---
            cv2.rectangle(frame, (left, top), (right, bottom), hud_color, 1)
            label_y = bottom + 25
            cv2.rectangle(frame, (left, label_y - 15), (right, label_y + 10), (0, 0, 0), -1)
            cv2.putText(frame, f"ID: {name.upper()}", (left + 5, label_y + 2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('Byte-01 Biometric HUD', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
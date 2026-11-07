# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VISION/object_detection.py
# STATUS    ·  PROTOTYPE · vision (BYTE_VISION)
# ────────────────────────────────────────────────────────────────────────
# YOLOv8n-seg object detection/segmentation on an Intel RealSense
# colour+depth stream (nano model chosen for the Pi).
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 object_detection.py
# ════════════════════════════════════════════════════════════════════════
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time

def main():
    # 0. Enable RealSense logging to see the internal errors
    rs.log_to_console(rs.log_severity.error)

    # 1. Load YOLOv8 Nano (Crucial for Pi)
    print("Loading YOLOv8n-seg...")
    model = YOLO("yolov8n-seg.pt") 

    # 2. Initialize Intel RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    
    # CHANGE: Dropped to 15 FPS to prevent buffer overflow
    # If it still crashes, try 640, 480 at 6 FPS or 424, 240 at 15 FPS
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    print("Starting pipeline...")
    profile = pipeline.start(config)

    # Set the buffer size to 1 (Don't let frames pile up!)
    # This prevents the "Processing Block" error
    sensor = profile.get_device().query_sensors()[0]
    sensor.set_option(rs.option.frames_queue_size, 1)

    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)

    # Post-processing filters
    spatial_filter = rs.spatial_filter()
    hole_filling_filter = rs.hole_filling_filter()

    try:
        while True:
            # 3. Wait for frames with a timeout
            frames = pipeline.wait_for_frames(5000)
            if not frames:
                continue

            # Defensive Check: Ensure we have both frames before aligning
            depth_f = frames.get_depth_frame()
            color_f = frames.get_color_frame()
            if not depth_f or not color_f:
                continue

            try:
                # This is where your error was happening
                aligned_frames = align.process(frames)
            except RuntimeError as e:
                print(f"Alignment Error: {e}")
                continue

            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue

            # Apply filters
            filtered_depth = spatial_filter.process(aligned_depth_frame)
            filtered_depth = hole_filling_filter.process(filtered_depth)

            depth_image = np.asanyarray(filtered_depth.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 4. Run YOLO (Stream=True is faster)
            results = model(color_image, stream=True, verbose=False)

            for r in results:
                if r.masks is not None:
                    masks = r.masks.xy 
                    boxes = r.boxes.xyxy
                    classes = r.boxes.cls

                    for i, mask_points in enumerate(masks):
                        pts = np.array(mask_points, dtype=np.int32)
                        
                        # Optimization: Check if mask is empty
                        if pts.size == 0: continue

                        # Mask logic
                        blank_mask = np.zeros(depth_image.shape, dtype=np.uint8)
                        cv2.fillPoly(blank_mask, [pts], 255)
                        
                        object_depths = depth_image[blank_mask == 255]
                        valid_depths = object_depths[object_depths > 0]

                        if len(valid_depths) > 0:
                            depth_meters = np.median(valid_depths) * depth_scale
                            
                            class_name = model.names[int(classes[i])]
                            x1, y1, x2, y2 = map(int, boxes[i])
                            
                            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(color_image, f"{class_name}: {depth_meters:.2f}m", 
                                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow('Byte Vision (Pi Optimized)', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
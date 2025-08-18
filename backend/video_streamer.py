import cv2
import time
import os
import socket
import json
import threading
from flask import Flask, Response
from ultralytics import YOLO

# --- Configuration ---
# Video Streaming Config
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SERVER_PORT = 5001

# UDP Communication Config for movement.py
UDP_IP = "127.0.0.2"
UDP_PORT = 5005

# YOLO Model Config
TARGET_CLASS = 0  # 0 is typically the 'person' class in COCO models

# --- Shared Resources ---
# A lock to ensure thread-safe access to the output frame
frame_lock = threading.Lock()
# The latest annotated frame that the video stream will send
output_frame = None

# --- Initialization ---
# Initialize Flask app for video streaming
app = Flask(__name__)

# Initialize YOLO model
print("Loading YOLO model...")
os.environ['YOLO_VERBOSE'] = 'False'
model = YOLO('models/best.engine')
print("YOLO model loaded.")

# Initialize camera
print("Initializing camera...")
video_capture = cv2.VideoCapture(0)
if not video_capture.isOpened():
    raise RuntimeError("Could not start camera.")
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
print("Camera initialized.")

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"UDP socket created to send data to {UDP_IP}:{UDP_PORT}")


def detection_and_udp_thread():
    """
    This function runs in a separate thread.
    It continuously captures frames, runs inference, sends UDP data,
    and updates the shared output_frame.
    """
    global output_frame
    
    while True:
        # 1. Capture a frame from the camera
        success, frame = video_capture.read()
        if not success:
            time.sleep(0.01)
            continue

        # 2. Run YOLO inference
        results = model(frame, imgsz=640, conf=0.6, verbose=False)
        
        # 3. Get the annotated frame for display
        annotated_frame = results[0].plot()
        
        # 4. Extract detection data
        boxes = results[0].boxes.xyxy.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy()
        
        targets = [box for i, box in enumerate(boxes) if classes[i] == TARGET_CLASS]
        
        # 5. Process and send data via UDP
        if targets:
            largest_target = max(targets, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
            x_center = (largest_target[0] + largest_target[2]) / 2
            y_center = (largest_target[1] + largest_target[3]) / 2
            area = (largest_target[2] - largest_target[0]) * (largest_target[3] - largest_target[1])
            
            data_packet = {
                "x_center": float(x_center),
                "y_center": float(y_center),
                "area": float(area),
                "frame_width": int(frame.shape[1]),
                "frame_height": int(frame.shape[0]),
                "state": "TRACKING"
            }
        else:
            data_packet = {
                "state": "SEARCHING"
            }
        
        # Always send the resulting packet to movement.py
        sock.sendto(json.dumps(data_packet).encode(), (UDP_IP, UDP_PORT))
        
        # 6. Update the shared frame for the video stream
        with frame_lock:
            global output_frame
            output_frame = annotated_frame.copy()

def generate_frames():
    """
    Generator function for the video streaming. It reads the shared
    output_frame and yields it to the Flask web server.
    """
    while True:
        with frame_lock:
            if output_frame is None:
                continue
            
            # Encode the frame as JPEG
            (flag, encoded_image) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        
        # Yield the output frame in the byte format required for streaming
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encoded_image) + b'\r\n')
        
        # Control the streaming frame rate to not overwhelm the network
        time.sleep(0.05) # Stream at ~20 FPS

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Start the detection thread
        print("Starting detection and UDP thread...")
        detector_thread = threading.Thread(target=detection_and_udp_thread)
        detector_thread.daemon = True
        detector_thread.start()
        
        # Start the Flask web server (for video streaming)
        print(f"Starting video streaming server on http://0.0.0.0:{SERVER_PORT}/video_feed")
        app.run(host='0.0.0.0', port=SERVER_PORT, debug=False, threaded=True)
        
    finally:
        # Clean up resources
        print("Releasing resources.")
        video_capture.release()
        sock.close()

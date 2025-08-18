import cv2
import time
import os
import socket
import json
from flask import Flask, Response
from ultralytics import YOLO

# --- Configuration ---
# Video Streaming Config
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SERVER_PORT = 5001

# UDP Communication Config for movement.py
UDP_IP = "127.0.0.2"  # IP for the movement script
UDP_PORT = 5005      # Port for the movement script

# YOLO Model Config
TARGET_CLASS = 0  # 0 is typically the 'person' class in COCO models

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
# Optional: Set manual exposure if needed
# video_capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
# video_capture.set(cv2.CAP_PROP_EXPOSURE, 5)
print("Camera initialized.")

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"UDP socket created to send data to {UDP_IP}:{UDP_PORT}")


def run_inference_and_send_data(frame):
    """
    Runs YOLO inference, ALWAYS sends detection data/status via UDP, 
    and returns the annotated frame.
    """
    # 1. Run YOLO inference
    results = model(frame, imgsz=640, conf=0.6, verbose=False)
    
    # 2. Get the annotated frame to display in the web UI
    annotated_frame = results[0].plot()
    
    # 3. Extract detection data
    boxes = results[0].boxes.xyxy.cpu().numpy()
    classes = results[0].boxes.cls.cpu().numpy()
    
    targets = [box for i, box in enumerate(boxes) if classes[i] == TARGET_CLASS]
    
    # --- MODIFIED LOGIC ---
    # 4. If targets are found, process and send TRACKING data
    if targets:
        largest_target = max(targets, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
        
        x_center = (largest_target[0] + largest_target[2]) / 2
        y_center = (largest_target[1] + largest_target[3]) / 2
        area = (largest_target[2] - largest_target[0]) * (largest_target[3] - largest_target[1])
        
        # Prepare data packet for tracking
        data_packet = {
            "x_center": float(x_center),
            "y_center": float(y_center),
            "area": float(area),
            "frame_width": int(frame.shape[1]),
            "frame_height": int(frame.shape[0])
        }
    
    # 5. <<< NEW: If NO targets are found, send a SEARCHING status >>>
    else:
        # Prepare a simple status packet
        data_packet = {
            "state": "SEARCHING"
        }
    
    # 6. Always send the resulting packet
    sock.sendto(json.dumps(data_packet).encode(), (UDP_IP, UDP_PORT))
    
    return annotated_frame


def generate_frames():
    """Generator function to capture frames, process them, and yield them for streaming."""
    while True:
        success, frame = video_capture.read()
        if not success:
            time.sleep(0.1)
            continue
        
        # Run inference, send UDP data, and get the annotated frame
        processed_frame = run_inference_and_send_data(frame)
        
        # Encode the frame as JPEG for streaming
        (flag, encoded_image) = cv2.imencode(".jpg", processed_frame)
        if not flag:
            continue
            
        # Yield the output frame for the web stream
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encoded_image) + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Start the Flask web server
        print(f"Starting video streaming server on http://0.0.0.0:{SERVER_PORT}/video_feed")
        app.run(host='0.0.0.0', port=SERVER_PORT, debug=False, threaded=True)
    finally:
        # Clean up resources
        print("Releasing resources.")
        video_capture.release()
        sock.close()

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
TARGET_CLASSES = [0, 1]  # ### MODIFIED ###: Look for both class 0 and class 1

# --- Shared Resources ---
frame_lock = threading.Lock()
output_frame = None
control_lock = threading.Lock()
is_sending_udp = True

# --- Initialization ---
app = Flask(__name__)
print("Loading YOLO model...")
os.environ['YOLO_VERBOSE'] = 'False'
model = YOLO('models/kpDetect-v2.1-yolov11n.engine')
print("YOLO model loaded.")

print("Initializing camera...")
video_capture = cv2.VideoCapture(1)
if not video_capture.isOpened():
    raise RuntimeError("Could not start camera.")
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
print("Camera initialized.")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"UDP socket created to send data to {UDP_IP}:{UDP_PORT}")

def control_command_listener():
    """
    Listens for TCP commands ('pause'/'resume') to control the UDP stream.
    This runs in its own thread.
    """
    global is_sending_udp
    # This port MUST match CONTROL_SERVER_PORT in movement.py
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('0.0.0.0', 5006)) # Listen on all available interfaces
        s.listen()
        print(f"Control command listener started on port 5006.")
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Control connection from {addr}")
                data = conn.recv(1024).decode('utf-8')
                if data == 'pause':
                    with control_lock:
                        is_sending_udp = False
                    print("UDP stream PAUSED.")
                elif data == 'resume':
                    with control_lock:
                        is_sending_udp = True
                    print("UDP stream RESUMED.")

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
        
        # ### MODIFIED ###: Find all valid targets and store them with their class ID
        valid_detections = []
        for i, box in enumerate(boxes):
            if classes[i] in TARGET_CLASSES:
                valid_detections.append({
                    "box": box,
                    "class_id": int(classes[i])
                })
        
        # 5. Process and send data via UDP
        if valid_detections:
            # Find the largest target among all valid detections
            largest_detection = max(valid_detections, key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]))
            
            largest_target_box = largest_detection["box"]
            detected_class_id = largest_detection["class_id"]
            
            x_center = (largest_target_box[0] + largest_target_box[2]) / 2
            y_center = (largest_target_box[1] + largest_target_box[3]) / 2
            area = (largest_target_box[2] - largest_target_box[0]) * (largest_target_box[3] - largest_target_box[1])
            
            data_packet = {
                "x_center": float(x_center),
                "y_center": float(y_center),
                "area": float(area),
                "frame_width": int(frame.shape[1]),
                "frame_height": int(frame.shape[0]),
                "state": "TRACKING",
                "class_id": detected_class_id
            }
        else:
            data_packet = {
                "state": "SEARCHING"
            }
        
        # Check if sending is allowed before sending the packet
        with control_lock:
            if is_sending_udp:
                sock.sendto(json.dumps(data_packet).encode(), (UDP_IP, UDP_PORT))

        # 6. Update the shared frame for the video stream
        with frame_lock:
            output_frame = annotated_frame.copy()

def generate_frames():
    """Generator function for video streaming."""
    while True:
        with frame_lock:
            if output_frame is None:
                continue
            
            (flag, encoded_image) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encoded_image) + b'\r\n')
        
        time.sleep(0.05)

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Start the control command listener thread FIRST
        print("Starting control command listener thread...")
        control_thread = threading.Thread(target=control_command_listener)
        control_thread.daemon = True
        control_thread.start()

        # Start the detection thread (as before)
        print("Starting detection and UDP thread...")
        detector_thread = threading.Thread(target=detection_and_udp_thread)
        detector_thread.daemon = True
        detector_thread.start()
        
        # Start the Flask web server (as before)
        print(f"Starting video streaming server on http://0.0.0.0:{SERVER_PORT}/video_feed")
        app.run(host='0.0.0.0', port=SERVER_PORT, debug=False, threaded=True)
        
    finally:
        print("Releasing resources.")
        video_capture.release()
        sock.close()
# run.py
import subprocess
import sys

print("Starting services...")

# Start the GCS server (formerly backend.py)
gcs_process = subprocess.Popen([sys.executable, "backend/gcs_server.py"])
print(f"Started GCS server with PID: {gcs_process.pid}")

# Start the video streamer
video_process = subprocess.Popen([sys.executable, "backend/video_streamer.py"])
print(f"Started video streamer with PID: {video_process.pid}")

# Wait for the processes to complete
gcs_process.wait()
video_process.wait()
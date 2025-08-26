import socket
import json
import time
import math
from pymavlink import mavutil

# --- Drone & Mission Configuration ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 921600
SAFE_ALTITUDE = 3.5  # meters
WAYPOINT_RADIUS = 0.5  # meters
TRACKING_SPEED = 0.5  # m/s
FWD_GAIN = 1.0
ALT_GAIN = 0.5
LANDING_APPROACH_ALT = 0.5 # meters, altitude to trigger final LAND command
LANDING_TIMEOUT = 15 # seconds to search before aborting landing
CENTERING_TIMEOUT = 20 # seconds to search before aborting centering
CENTERING_ALTITUDE = 1.0 # meters, altitude to hold when centering
CENTERING_CONFIRM_TIME = 5.0 # seconds, time to hold position to confirm centering
GAIN_MAX_ALT = 3.5  # Altitude (m) at which the gain is 1.0 (full speed)
GAIN_MIN_ALT = 0.5  # Altitude (m) at which the gain is at its minimum
MAX_HORIZONTAL_GAIN = 1.0 # The gain at or above GAIN_MAX_ALT
MIN_HORIZONTAL_GAIN = 0.2 # The minimum gain at or below GAIN_MIN_ALT

# --- UDP Network Configuration ---
UDP_RECEIVE_IP = "127.0.0.2"
UDP_RECEIVE_PORT = 5005
CONTROL_SERVER_IP = "127.0.0.2"
CONTROL_SERVER_PORT = 5006

# --- MAVLink & ArduPilot Constants ---
GUIDED_MODE = 4
VELOCITY_CONTROL_BITMASK = 0b0000111111000111
POSITION_CONTROL_BITMASK = 0b110111111000

# --- Mission Waypoints ---
# IMPORTANT: Update these with your actual GPS coordinates
WAYPOINTS = [
    (-7.8333808, 110.3843772, 3.5), # Waypoint 1 (Precision Land on Logistics - ID 0)
    (-7.8333350, 110.3843705, 3.5), # Waypoint 2 (Precision Land on Logistics - ID 0)
    (-7.8332890, 110.3843750, 3.5), # Waypoint 3 (Center on Barrel - ID 1)
    (-7.8332528, 110.3843930, 3.5)  # Waypoint 4 (Final Normal Land)
]

# --- Global Socket ---
data_sock = None

# --- Core Functions ---

def send_control_command(command):
    """Sends a 'pause' or 'resume' command to the detection script via TCP."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((CONTROL_SERVER_IP, CONTROL_SERVER_PORT))
            s.sendall(command.encode('utf-8'))
            print(f"Sent control command: '{command}' to detection script.")
    except ConnectionRefusedError:
        print(f"Error: Connection refused. Is the detection script running on port {CONTROL_SERVER_PORT}?")
    except Exception as e:
        print(f"Error sending control command: {e}")

def arm_and_takeoff(master, altitude):
    """Arms the drone and takes off to a specified altitude."""
    print("Setting mode to GUIDED...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        GUIDED_MODE
    )
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()

    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0
        print(f"Current altitude: {current_altitude:.2f}m")
        if current_altitude >= altitude * 0.90:
            print("Target altitude reached.")
            break
        time.sleep(0.1)

def land_normally(master):
    """Commands the drone to perform a standard, non-precision landing."""
    print("Executing normal landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Landed and disarmed.")

def navigate_to_waypoint(master, lat, lon, alt):
    """Commands the drone to fly to a specific GPS waypoint and waits for arrival."""
    print(f"Navigating to waypoint: ({lat}, {lon}) at {alt}m")
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        POSITION_CONTROL_BITMASK, int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg: continue
        current_lat, current_lon = msg.lat / 1e7, msg.lon / 1e7
        dlat = math.radians(lat - current_lat)
        dlon = math.radians(lon - current_lon)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(current_lat)) * math.cos(math.radians(lat)) * math.sin(dlon/2)**2
        distance = 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        print(f"Distance to target: {distance:.1f}m")
        if distance <= WAYPOINT_RADIUS:
            print("Waypoint reached!")
            break
        time.sleep(0.1)
    time.sleep(2)

def calculate_velocities(x_center, y_center, frame_w, frame_h):
    """Calculates horizontal velocities to track the target."""
    x_offset = (x_center - frame_w / 2) / (frame_w / 2)
    y_offset = (y_center - frame_h / 2) / (frame_h / 2)
    right_vel = TRACKING_SPEED * x_offset if abs(x_offset) > 0.1 else 0
    forward_vel = -TRACKING_SPEED * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    return forward_vel, right_vel

def flush_socket_buffer(sock):
    """Clears any old data from the UDP socket buffer."""
    print("Flushing UDP socket buffer...")
    while True:
        try:
            sock.recvfrom(1024)
        except socket.timeout:
            print("Buffer flushed.")
            break

def center_above_target(master, sock, target_class_id):
    """Centers the drone above a target with a specific class ID."""
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Centering above target (ID: {target_class_id}) at {CENTERING_ALTITUDE}m...")
    
    last_known_detection = None
    search_start_time = time.time()
    centered_start_time = None

    while True:
        if time.time() - search_start_time > CENTERING_TIMEOUT:
            print("Centering timeout reached. Aborting and hovering.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return False # Indicate failure

        try:
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            search_start_time = time.time()

            # *** MODIFIED: Check for correct state AND class ID ***
            if detection.get("state") != "TRACKING" or detection.get("class_id") != target_class_id:
                raise socket.timeout()

            last_known_detection = detection
            x, y = detection["x_center"], detection["y_center"]
            w, h = detection["frame_width"], detection["frame_height"]
            
            fwd_vel, right_vel = calculate_velocities(x, y, w, h)
            
            alt_msg = master.recv_match(type='RANGEFINDER', blocking=True, timeout=0.2)
            current_alt = alt_msg.distance if alt_msg else CENTERING_ALTITUDE
            alt_error = CENTERING_ALTITUDE - current_alt
            down_vel = -0.25 * alt_error
            
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))

            center_error_ratio = abs(x - w / 2) / w
            print(f"CENTERING (ID {target_class_id}): Error: {center_error_ratio:.2%}, Alt: {current_alt:.2f}m")

            if center_error_ratio < 0.1 and abs(alt_error) < 0.15:
                if centered_start_time is None:
                    centered_start_time = time.time()
                elif time.time() - centered_start_time > CENTERING_CONFIRM_TIME:
                    print("Target centering confirmed.")
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                    return True
            else:
                centered_start_time = None

        except (socket.timeout, json.JSONDecodeError, KeyError):
            print(f"Searching for target ID {target_class_id}... Hovering.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

def get_dynamic_gain(current_alt):
    """Calculates a dynamic gain scaling factor based on altitude."""
    if current_alt >= GAIN_MAX_ALT: return MAX_HORIZONTAL_GAIN
    if current_alt <= GAIN_MIN_ALT: return MIN_HORIZONTAL_GAIN
    gain = MIN_HORIZONTAL_GAIN + (MAX_HORIZONTAL_GAIN - MIN_HORIZONTAL_GAIN) * \
           ((current_alt - GAIN_MIN_ALT) / (GAIN_MAX_ALT - GAIN_MIN_ALT))
    return gain

def execute_precision_landing(master, sock, target_class_id):
    """Manages precision landing on a target with a specific class ID."""
    flush_socket_buffer(sock)
    send_control_command('resume')
    print(f"Starting precision landing sequence on target (ID: {target_class_id})...")
    
    last_known_detection = None
    search_start_time = time.time()

    while True:
        if time.time() - search_start_time > LANDING_TIMEOUT:
            print("Landing timeout reached. Aborting and hovering.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            return

        try:
            alt_msg = master.recv_match(type='RANGEFINDER', blocking=False, timeout=0.05)
            current_altitude = alt_msg.distance if alt_msg else GAIN_MAX_ALT 

            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())
            search_start_time = time.time()

            # *** MODIFIED: Check for correct state AND class ID ***
            if detection.get("state") != "TRACKING" or detection.get("class_id") != target_class_id:
                raise socket.timeout()

            last_known_detection = detection
            x, y, area = detection["x_center"], detection["y_center"], detection["area"]
            w, h = detection["frame_width"], detection["frame_height"]
            
            fwd_vel, right_vel = calculate_velocities(x, y, w, h)
            
            horizontal_gain = get_dynamic_gain(current_altitude)
            fwd_vel *= horizontal_gain
            right_vel *= horizontal_gain
            
            target_area = 0.2 * (w * h)
            area_error = 1.0 - (area / target_area) if target_area > 0 else 0
            down_vel = TRACKING_SPEED * area_error * ALT_GAIN if abs(area_error) > 0.2 else 0
            
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))

            center_error_ratio = abs(x - w / 2) / w
            print(f"LANDING (ID {target_class_id}): Alt: {current_altitude:.2f}m, Gain: {horizontal_gain:.2f}, Err: {center_error_ratio:.2%}")

            if current_altitude < LANDING_APPROACH_ALT and center_error_ratio < 0.1:
                print("Target centered at low altitude. Switching to LAND mode.")
                land_normally(master)
                time.sleep(5)
                return

        except (socket.timeout, json.JSONDecodeError, KeyError):
            print(f"Searching for target ID {target_class_id}... Hovering.")
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                VELOCITY_CONTROL_BITMASK, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

def main():
    """Main function to connect to the drone and run the new mission."""
    global data_sock
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.bind((UDP_RECEIVE_IP, UDP_RECEIVE_PORT))
    data_sock.settimeout(0.5)

    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    try:
        send_control_command('pause')
        arm_and_takeoff(master, SAFE_ALTITUDE)

        # --- Leg 1: Fly to WP1 and Precision Land on Target 0 ---
        print("\n--- MISSION: Leg 1 - Precision Land at Waypoint 1 (Target ID 0) ---")
        navigate_to_waypoint(master, WAYPOINTS[0][0], WAYPOINTS[0][1], WAYPOINTS[0][2])
        execute_precision_landing(master, sock=data_sock, target_class_id=0)
        
        # --- Leg 2: Takeoff, fly to WP3 and Center on Target 1 ---
        print("\n--- MISSION: Leg 2 - Center over Target at Waypoint 3 (Target ID 1) ---")
        send_control_command('pause')
        arm_and_takeoff(master, SAFE_ALTITUDE)
        navigate_to_waypoint(master, WAYPOINTS[2][0], WAYPOINTS[2][1], WAYPOINTS[2][2])
        center_above_target(master, sock=data_sock, target_class_id=1)

        # --- Leg 3: Fly to WP2 and Precision Land on Target 0 ---
        print("\n--- MISSION: Leg 3 - Precision Land at Waypoint 2 (Target ID 0) ---")
        send_control_command('pause')
        navigate_to_waypoint(master, WAYPOINTS[1][0], WAYPOINTS[1][1], WAYPOINTS[1][2])
        execute_precision_landing(master, sock=data_sock, target_class_id=0)

        # --- Leg 4: Takeoff, fly to WP3 and Center on Target 1 Again ---
        print("\n--- MISSION: Leg 4 - Center over Target at Waypoint 3 Again (Target ID 1) ---")
        send_control_command('pause')
        arm_and_takeoff(master, SAFE_ALTITUDE)
        navigate_to_waypoint(master, WAYPOINTS[2][0], WAYPOINTS[2][1], WAYPOINTS[2][2])
        center_above_target(master, sock=data_sock, target_class_id=1)

        # --- Leg 5: Fly to WP4 and Land ---
        print("\n--- MISSION: Leg 5 - Final Landing at Waypoint 4 ---")
        send_control_command('pause')
        navigate_to_waypoint(master, WAYPOINTS[3][0], WAYPOINTS[3][1], WAYPOINTS[3][2])
        land_normally(master)

        print("\nMission finished successfully!")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Landing immediately...")
        land_normally(master)
    finally:
        if data_sock: data_sock.close()
        print("Resources cleaned up.")

if __name__ == "__main__":
    main()

import socket
import json
import time
import math
import cv2
from pymavlink import mavutil

# Configuration
UDP_IP = "127.0.0.2"
UDP_PORT = 5005
SAFE_ALTITUDE = 3.5
WAYPOINT_RADIUS = 2
TRACKING_SPEED = 0.5
FWD_GAIN = 1.0
ALT_GAIN = 0.5

# Waypoints
WAYPOINTS = [
    (-7.833193, 110.384422, 3.5), # Waypoint 1 (for first precision landing)
    (-7.833216, 110.384489, 3.5)  # Waypoint 2 (for second precision landing)
]

# Initialize UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)  # 100ms timeout

def arm_and_takeoff(master, altitude):
    """Arm and takeoff to specified altitude"""
    print("Setting mode to GUIDED...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4)  # 4 = GUIDED mode for Copter

    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude)
    
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0
        print(f"  ... current altitude: {current_altitude:.2f}m")
        if current_altitude >= altitude * 0.90:
            print("Target altitude reached.")
            break
        time.sleep(0.5)

def disarm(master):
    """Disarm the drone"""
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    master.motors_disarmed_wait()
    print("Disarmed.")

def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates"""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def move_to_waypoint(master, lat, lon, alt):
    """Command drone to move to waypoint"""
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000), int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0)

def navigate_to_waypoint_and_wait(master, lat, lon, alt):
    """Fly to a single waypoint and wait until within WAYPOINT_RADIUS."""
    print(f"Navigating to waypoint: ({lat}, {lon}) at {alt}m")
    move_to_waypoint(master, lat, lon, alt)
    
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            distance = calculate_distance(current_lat, current_lon, lat, lon)
            print(f"Distance to target: {distance:.1f}m")
            if distance <= WAYPOINT_RADIUS:
                print("Waypoint reached!")
                break
        time.sleep(1)
    time.sleep(2)

def calculate_velocities(x_center, y_center, frame_w, frame_h, area):
    """Calculate tracking velocities"""
    x_offset = (x_center - frame_w/2) / (frame_w/2)
    y_offset = (y_center - frame_h/2) / (frame_h/2)
    
    right_vel = TRACKING_SPEED * x_offset if abs(x_offset) > 0.1 else 0
    forward_vel = -TRACKING_SPEED * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    
    target_area = 0.2 * (frame_w * frame_h)
    area_error = 1.0 - (area / target_area) if target_area > 0 else 0
    down_vel = TRACKING_SPEED * area_error * ALT_GAIN if abs(area_error) > 0.2 else 0
    
    return forward_vel, right_vel, down_vel

# def precision_land(master, x_center, y_center, frame_w, frame_h):
#     """Precision landing control"""
#     x_offset = (x_center - frame_w/2) / (frame_w/2)
#     y_offset = (y_center - frame_h/2) / (frame_h/2)
    
#     right_vel = TRACKING_SPEED * x_offset if abs(x_offset) > 0.1 else 0
#     forward_vel = -TRACKING_SPEED * y_offset * FWD_GAIN if abs(y_offset) > 0.1 else 0
    
#     master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
#         0, master.target_system, master.target_component,
#         mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
#         int(0b0000111111000111), 0, 0, 0,
#         forward_vel, right_vel, 0.2,
#         0, 0, 0, 0, 0))

def flush_socket_buffer(sock):
    """Clear any old data from the UDP socket buffer."""
    print("Flushing UDP socket buffer...")
    while True:
        try:
            # Read from the socket with a very short, non-blocking timeout
            sock.recvfrom(1024)
        except socket.timeout:
            # No more data in the buffer
            print("Buffer flushed.")
            break

def execute_precision_landing_sequence(master, sock):
    """
    Handles the complete tracking and landing process at a location.
    Listens for detection data and controls the drone until it has landed and disarmed.
    If the detected object moves out of the camera's sight, it moves back to the last known position.
    """
    flush_socket_buffer(sock)
    print("Starting precision landing sequence...")
    state = "TRACKING"
    landing_complete = False
    last_known_detection = None # Variable to store the last valid detection

    while not landing_complete:
        try:
            # Receive new detection data
            data, _ = sock.recvfrom(1024)
            detection = json.loads(data.decode())

            # <<< NEW LOGIC: Check if the received packet actually contains a target >>>
            # If not, we treat it the same as a timeout.
            if "x_center" not in detection or "y_center" not in detection:
                print("Received status, but no target in frame.")
                raise socket.timeout() # Manually trigger the exception to reuse the "move back" logic

            last_known_detection = detection # <<< Store the latest successful detection if it's valid

            if state == "TRACKING":
                x, y, area = detection["x_center"], detection["y_center"], detection["area"]
                w, h = detection["frame_width"], detection["frame_height"]
                
                fwd_vel, right_vel, down_vel = calculate_velocities(x, y, w, h, area)
                
                master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                    0, master.target_system, master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    int(0b0000111111000111), 0, 0, 0,
                    fwd_vel, right_vel, down_vel, 0, 0, 0, 0, 0))
                
                center_error_ratio = abs(x - w / 2) / w
                area_ratio = area / (w * h)
                print(f"TRACKING: Center Error: {center_error_ratio:.2%}, Area Ratio: {area_ratio:.2%}")

                if center_error_ratio < 0.05 and area_ratio > 0.15:
                    print("Target centered. Switching to PRECISION_LAND.")
                    state = "PRECISION_LAND"
            
            elif state == "PRECISION_LAND":
                # (The rest of the landing logic remains the same)
                print("PRECISION_LAND: Landing...")
                alt_msg = master.recv_match(type='RANGEFINDER', blocking=True, timeout=1)
                if alt_msg and alt_msg.distance < 0.75:
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
                    print("Landed successfully!")
                    disarm(master)
                    landing_complete = True
                    time.sleep(5)
                    continue
    
        except socket.timeout:
            # <<< This block now handles both a true timeout AND a "no target in frame" packet >>>
            print("Target lost from sight.")
            if state in ["TRACKING", "PRECISION_LAND"]:
                if last_known_detection:
                    # If we have a last known position, move towards it
                    print("-> Moving back to last known target position...")
                    x, y, area = last_known_detection["x_center"], last_known_detection["y_center"], last_known_detection["area"]
                    w, h = last_known_detection["frame_width"], last_known_detection["frame_height"]
                    
                    # Calculate horizontal velocity to return to target
                    fwd_vel, right_vel, _ = calculate_velocities(x, y, w, h, area)
                    
                    # Send command to move horizontally (down_vel is 0 for safety)
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component,
                        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        int(0b0000111111000111), 0, 0, 0,
                        fwd_vel, right_vel, 0,  # <<< Note: down_vel is 0 for safety
                        0, 0, 0, 0, 0))
                else:
                    # If we never saw the target, just hover in place
                    print("-> No previous detection data. Hovering.")
                    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        0, master.target_system, master.target_component,
                        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                        int(0b0000111111000111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    
        except (json.JSONDecodeError, KeyError) as e:
            print(f"Error processing detection data: {e}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt
    
    print("Precision landing sequence complete.")

def main():
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550', baud=921600)
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
    
    try:
        # ==================================================================
        # MISSION LEG 1: FLY TO WAYPOINT 1 AND PRECISION LAND
        # ==================================================================
        print("\n--- STARTING MISSION LEG 1 ---")
        # NOTE: Uncomment the following line for a real flight simulation
        arm_and_takeoff(master, SAFE_ALTITUDE)

        wp1_lat, wp1_lon, wp1_alt = WAYPOINTS[0]
        navigate_to_waypoint_and_wait(master, wp1_lat, wp1_lon, wp1_alt)
        execute_precision_landing_sequence(master, sock)
        
        # ==================================================================
        # MISSION LEG 2: TAKEOFF AND FLY TO WAYPOINT 2 FOR ANOTHER PRECISION LAND
        # ==================================================================
        print("\n--- STARTING MISSION LEG 2 ---")
        # NOTE: Uncomment the following line for a real flight simulation
        arm_and_takeoff(master, SAFE_ALTITUDE)
        
        wp2_lat, wp2_lon, wp2_alt = WAYPOINTS[1]
        navigate_to_waypoint_and_wait(master, wp2_lat, wp2_lon, wp2_alt)
        execute_precision_landing_sequence(master, sock)

        print("\nMission finished successfully!")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Landing immediately...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

if __name__ == "__main__":
    main()
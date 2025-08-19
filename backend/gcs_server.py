import asyncio
import json
import websockets
from pymavlink import mavutil
import paramiko # <--- 1. IMPORT PARAMIKO

# --- SSH Configuration ---
DRONE_IP = '192.168.10.118' # IP DRONE
DRONE_USER = 'kingphoenix' # USERNAME DRONE

# --- MAVLink Connection ---
# This part remains the same
connection_string = 'udp:0.0.0.0:14552'
master = mavutil.mavlink_connection(connection_string)
print(f"Waiting for heartbeat on {connection_string}...")
master.wait_heartbeat()
print("Heartbeat received! MAVLink connection established.")

# --- REQUEST DATA STREAMS ---
# This part remains the same
print("Requesting data streams...")
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 500000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 500000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 1000000, 0, 0, 0, 0, 0)
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 1000000, 0, 0, 0, 0, 0)
print("Data streams requested.")

# Global variables remain the same
connected_clients = set()
vehicle_state = {}

# <--- 3. ADD THE SSH EXECUTION FUNCTION ---
def execute_ssh_command(command):
    """Connects to the drone via SSH and executes a command."""
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        print(f"Connecting to {DRONE_USER}@{DRONE_IP} via SSH...")
        ssh.connect(DRONE_IP, username=DRONE_USER) # Assumes SSH key setup
        print(f"Executing remote command: {command}")
        # This executes the command and does not wait for it to complete
        stdin, stdout, stderr = ssh.exec_command(command)
        print("Mission start command sent successfully.")
    except Exception as e:
        print(f"SSH Execution Failed: {e}")
    finally:
        # Ensure the connection is closed
        if 'ssh' in locals() and ssh.get_transport().is_active():
            ssh.close()
            print("SSH connection closed.")

# The mavlink_loop function is completely unchanged
async def mavlink_loop():
    """Continuously read MAVLink messages and forward them."""
    while True:
        msg = master.recv_match(
            type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'HEARTBEAT',
                  'STATUSTEXT', 'SYS_STATUS', 'RANGEFINDER'],
            blocking=False
        )
        if msg:
            data = None
            msg_type = msg.get_type()

            if msg_type == 'STATUSTEXT':
                log_data = {'type': 'log', 'severity': msg.severity, 'text': msg.text.strip()}
                if connected_clients:
                    websockets.broadcast(connected_clients, json.dumps(log_data))
            
            elif msg_type == 'HEARTBEAT':
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                data = {'flight_mode': master.flightmode, 'system_status': msg.system_status, 'armed': bool(is_armed)}
                vehicle_state.update(data)

            elif msg_type == 'ATTITUDE':
                data = {'pitch': msg.pitch * 180.0 / 3.14, 'roll': msg.roll * 180.0 / 3.14, 'yaw': msg.yaw * 180.0 / 3.14}
                vehicle_state.update(data)

            elif msg_type == 'GLOBAL_POSITION_INT':
                data = {'lat': msg.lat / 1e7, 'lon': msg.lon / 1e7, 'alt_msl': msg.alt / 1000.0, 'alt_rel': msg.relative_alt / 1000.0, 'heading': msg.hdg / 100.0}
                vehicle_state.update(data)

            elif msg_type == 'VFR_HUD':
                data = {'airspeed': msg.airspeed, 'groundspeed': msg.groundspeed, 'throttle': msg.throttle, 'climb': msg.climb}
                vehicle_state.update(data)

            elif msg_type == 'SYS_STATUS':
                data = {'voltage': msg.voltage_battery / 1000.0, 'current': msg.current_battery / 100.0, 'level': msg.battery_remaining}
                vehicle_state.update(data)
                
            elif msg_type == 'RANGEFINDER':
                data = {'distance': msg.distance}
                vehicle_state.update(data)

            if data and connected_clients:
                state_update = {'type': 'state', 'data': vehicle_state}
                websockets.broadcast(connected_clients, json.dumps(state_update))
        await asyncio.sleep(0.01)

# <--- 4. MODIFY THE HANDLER TO LISTEN FOR COMMANDS ---
async def handler(websocket):
    """Handle new WebSocket connections and listen for incoming commands."""
    print(f"Client connected from {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        # Send the initial state upon connection
        if vehicle_state:
            initial_state = {'type': 'state', 'data': vehicle_state}
            await websocket.send(json.dumps(initial_state))
        
        # Listen for incoming messages from the web client
        async for message in websocket:
            try:
                command = json.loads(message)
                if command.get('action') == 'start_mission':
                    print("Received 'start_mission' command from web client.")
                    # IMPORTANT: Use the FULL path to your script on the drone
                    mission_command = 'python3 /home/kingphoenix/Web-GCS/backend/movement.py' # <-- CONFIGURE THIS PATH
                    execute_ssh_command(mission_command)
            except json.JSONDecodeError:
                print(f"Received invalid JSON from client: {message}")
            except Exception as e:
                print(f"Error processing command: {e}")
    finally:
        print(f"Client disconnected from {websocket.remote_address}")
        connected_clients.remove(websocket)

# <--- 5. MODIFY MAIN TO RUN BOTH TASKS CONCURRENTLY ---
async def main():
    """Start the MAVLink loop and the WebSocket server to run in parallel."""
    # Start the WebSocket server
    server = await websockets.serve(handler, "0.0.0.0", 8765)
    print("WebSocket server started on ws://0.0.0.0:8765")
    
    # Create a task for the mavlink_loop to run in the background
    mavlink_task = asyncio.create_task(mavlink_loop())
    
    # Wait for both the server and the mavlink loop to complete
    # The server runs forever, so this will keep the program alive
    await asyncio.gather(server.wait_closed(), mavlink_task)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down.")
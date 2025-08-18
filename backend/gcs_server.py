import asyncio
import json
import websockets
from pymavlink import mavutil

# --- MAVLink Connection ---
# Listen for incoming UDP packets from the companion computer or SITL
connection_string = 'udp:0.0.0.0:14552' # udp:0.0.0.0:14550 for all IP and tcp:127.0.0.1:5760 for SITL
master = mavutil.mavlink_connection(connection_string)
print(f"Waiting for heartbeat on {connection_string}...")
master.wait_heartbeat()
# ... inside backend.py

master.wait_heartbeat()
print("Heartbeat received! MAVLink connection established.")

# --- REQUEST DATA STREAMS ---
# Tell the vehicle to start sending the data we want
print("Requesting data streams...")
# For ATTITUDE
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 500000, # 500000 microseconds = 0.5 seconds = 2 Hz
    0, 0, 0, 0, 0)

# For GLOBAL_POSITION_INT
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000, # 2 Hz
    0, 0, 0, 0, 0)

# For VFR_HUD
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 500000, # 2 Hz
    0, 0, 0, 0, 0)
    
# For SYS_STATUS (Battery)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000, # 1000000 microseconds = 1 second = 1 Hz
    0, 0, 0, 0, 0)

# For STATUSTEXT (Log Messages)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 1000000, # 1 Hz
    0, 0, 0, 0, 0)



print("Data streams requested.")

# A set to store all connected WebSocket clients
connected_clients = set()
# ... the rest of your script continues here
print("Heartbeat received! MAVLink connection established.")

# A set to store all connected WebSocket clients
connected_clients = set()

# A dictionary to store the latest state of the vehicle
vehicle_state = {}

# In backend.py, replace the mavlink_loop function with this:

# In backend.py, replace your mavlink_loop with this updated version

# Replace the mavlink_loop function in backend.py
async def mavlink_loop():
    """Continuously read MAVLink messages and forward them."""
    while True:
        # MISSION_CURRENT has been removed from this list
        msg = master.recv_match(
            type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'HEARTBEAT', 
                  'STATUSTEXT', 'SYS_STATUS'],
            blocking=False 
        )
        
        if msg:
            data = None
            msg_type = msg.get_type()
            
            if msg_type == 'STATUSTEXT':
                log_data = {
                    'type': 'log',
                    'severity': msg.severity,
                    'text': msg.text.strip()
                }
                if connected_clients:
                    websockets.broadcast(connected_clients, json.dumps(log_data))

            elif msg_type == 'HEARTBEAT':
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                data = { 
                    'flight_mode': master.flightmode, 
                    'system_status': msg.system_status,
                    'armed': bool(is_armed) # Add the new armed status
                }
                vehicle_state.update(data)

            elif msg_type == 'ATTITUDE':
                data = { 'pitch': msg.pitch * 180.0 / 3.14159265, 'roll': msg.roll * 180.0 / 3.14159265, 'yaw': msg.yaw * 180.0 / 3.14159265 }
                vehicle_state.update(data)
            
            elif msg_type == 'GLOBAL_POSITION_INT':
                data = { 'lat': msg.lat / 1e7, 'lon': msg.lon / 1e7, 'alt_msl': msg.alt / 1000.0, 'alt_rel': msg.relative_alt / 1000.0, 'heading': msg.hdg / 100.0 }
                vehicle_state.update(data)
            
            elif msg_type == 'VFR_HUD':
                data = { 'airspeed': msg.airspeed, 'groundspeed': msg.groundspeed, 'throttle': msg.throttle, 'climb': msg.climb }
                vehicle_state.update(data)
                
            elif msg_type == 'HEARTBEAT':
                data = { 'flight_mode': master.flightmode, 'system_status': msg.system_status }
                vehicle_state.update(data)
            
            elif msg_type == 'SYS_STATUS':
                data = { 'voltage': msg.voltage_battery / 1000.0, 'current': msg.current_battery / 100.0, 'level': msg.battery_remaining }
                vehicle_state.update(data)

            if data and connected_clients:
                websockets.broadcast(connected_clients, json.dumps(vehicle_state))

        await asyncio.sleep(0.01)

async def handler(websocket):
    """Handle new WebSocket connections."""
    print(f"Client connected from {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        # Send the last known state to the newly connected client
        if vehicle_state:
            await websocket.send(json.dumps(vehicle_state))
        await websocket.wait_closed()
    finally:
        print(f"Client disconnected from {websocket.remote_address}")
        connected_clients.remove(websocket)

async def main():
    """Start the MAVLink loop and the WebSocket server."""
    server = await websockets.serve(handler, "0.0.0.0", 8765)
    print("WebSocket server started on ws://0.0.0.0:8765")
    
    await mavlink_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down.")


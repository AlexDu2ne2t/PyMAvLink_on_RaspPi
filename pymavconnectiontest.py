import time
from pymavlink import mavutil

# Establish a connection to the flight controller (Serial or UDP)
# If you're using serial (e.g., USB connected), replace '/dev/ttyUSB0' with your serial port
# For UDP, use 'udp:127.0.0.1:14550' or similar based on your setup

connection_string = "/dev/serial0"  # For serial connection
# connection_string = "udp:127.0.0.1:14550"  # If you're using UDP

# Open the MAVLink connection
master = mavutil.mavlink_connection(connection_string, baud=115200)

print(f"Connecting to flight controller at {connection_string}...")

# Wait for the connection to establish
master.wait_heartbeat()

print("Connection established!")

# Print system status to verify successful connection
try:
    while True:
        # Request the system status (to ensure communication is happening)
        master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 1, 1)
        
        # Listen for messages from the flight controller
        msg = master.recv_match(type='RC_CHANNELS')
        if msg:
            print(f"Received message: {msg}")
        
        # Keep the connection alive
        time.sleep(1)

except KeyboardInterrupt:
    print("Connection verification terminated by user.")
except Exception as e:
    print(f"Error during connection verification: {e}")

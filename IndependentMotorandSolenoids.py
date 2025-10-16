import time
from pymavlink import mavutil
from gpiozero import OutputDevice   

# Define GPIO pins
pump_pin = 17
solenoid_pin = 27

# Initialize pump and solenoids
pump = OutputDevice(pump_pin)
solenoid = OutputDevice(solenoid_pin)

# Setup MAVLink connection
connection_string = "/dev/serial0"
master = mavutil.mavlink_connection(connection_string, baud=115200)

print(f"Connecting to flight controller at {connection_string}...")
master.wait_heartbeat()
print("Connection established!")

# Request RC_CHANNELS stream once at 10Hz
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
    10, 1
)

# Track previous states
prev_rc11_high = None
prev_rc12_high = None

def rc_channel_11_high(msg):
    return msg.chan11_raw is not None and msg.chan11_raw > 1500

def rc_channel_11_low(msg):
    return msg.chan11_raw is not None and msg.chan11_raw <= 1500

def rc_channel_12_high(msg):
    return msg.chan12_raw is not None and msg.chan12_raw > 1500

def rc_channel_12_low(msg):
    return msg.chan12_raw is not None and msg.chan12_raw <= 1500

try:
    while True:
        # Flush old messages, get latest RC_CHANNELS
        latest_msg = None
        while True:
            msg = master.recv_match(type='RC_CHANNELS', blocking=False)
            if msg:
                latest_msg = msg
            else:
                break  # No more in buffer
        
        if latest_msg:
            print(f"Channel 11 raw: {latest_msg.chan11_raw}")
            rc11_high = rc_channel_11_high(latest_msg)
            rc12_high = rc_channel_12_high(latest_msg)

            # Only act on Channel 11 if state changed
            if rc11_high != prev_rc11_high:
                if rc11_high:
                    print("RC_CHANNEL_11 is HIGH — Starting pump")
                    pump.on()
                else:
                    print("RC_CHANNEL_11 is LOW — Stopping pump")
                    pump.off()
                prev_rc11_high = rc11_high

            # Only act on Channel 12 if state changed
            if rc12_high != prev_rc12_high:
                if rc12_high:
                    print("RC_CHANNEL_12 is HIGH — Activating solenoid")
                    solenoid.on()
                else:
                    print("RC_CHANNEL_12 is LOW — Deactivating solenoid")
                    solenoid.off()
                prev_rc12_high = rc12_high

        else:
            print("No RC_CHANNELS message received.")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Interrupted by user. Stopping pump.")
    pump.off()
    solenoid.off()
except Exception as e:
    print(f"Error: {e}")
    pump.off()
    solenoid.off()

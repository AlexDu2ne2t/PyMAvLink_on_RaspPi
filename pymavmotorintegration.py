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

# pump control state
pump_active = False

def rc_channel_11_high(msg):
    return msg.chan11_raw is not None and msg.chan11_raw > 1500

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
        #This logic means that, if no message has been picked up, no attempt to read it is made.
        if latest_msg:
            print(f"Channel 11 raw: {latest_msg.chan11_raw}")
            rc11_high = rc_channel_11_high(latest_msg)

            if rc11_high and not pump_active:
                print("RC_CHANNEL_11 is HIGH — Starting pump loop")
                pump_active = True

                while True:
                    print("Pump ON")
                    pump.on()
                    solenoid.off()
                    time.sleep(3)

                    print("Pump OFF")
                    solenoid.on()
                    pump.off()
                    time.sleep(3)

                    # Check RC11 again
                    latest_check = None
                    while True:
                        msg = master.recv_match(type='RC_CHANNELS', blocking=False)
                        if msg:
                            latest_check = msg
                        else:
                            break

                    if not latest_check or not rc_channel_11_high(latest_check):
                        print("RC_CHANNEL_11 is LOW — Stopping pump")
                        break

                pump.off()
                solenoid.on()
                pump_active = False

            elif not rc11_high:
                print("RC_CHANNEL_11 is LOW — Pump remains off")
                pump.off()
                pump_active = False

        else:
            print("No RC_CHANNELS message received.")

        time.sleep(0.2)

except KeyboardInterrupt:
    print("Interrupted by user. Stopping pump.")
    pump.off()
    solenoid.on()
except Exception as e:
    print(f"Error: {e}")
    pump.off()
    solenoid.on()

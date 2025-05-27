from gpiozero import OutputDevice
from time import sleep

# Define GPIO pins (change as needed)
pump_pin = 17        # GPIO17 (physical pin 11)
solenoid_pin = 27    # GPIO27 (physical pin 13)

# Set up devices
pump = OutputDevice(pump_pin)
solenoid = OutputDevice(solenoid_pin)

print("Starting pump and solenoid cycle...")

try:
    while True:
        print("Pump ON, Solenoid ON")
        pump.on()
        solenoid.off()
        sleep(3)

        print("Pump ON, Solenoid OFF")
        solenoid.on()
        # Pump stays on
        sleep(3)

except KeyboardInterrupt:
    print("\nInterrupt received. Turning off devices...")

finally:
    pump.off()
    solenoid.off()
    print("Devices turned off. Exiting safely.")

import time
from pymavlink import mavutil

# Set the connection parameters (adjust accordingly)
connection_string = '/dev/ttyTHS1'  # Serial port of the autopilot
baud_rate = 57600  # Baud rate of the autopilot

# Create a MAVLink connection
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# Wait for the heartbeat message to be received
master.wait_heartbeat()
print("Drone connected!")

# Wait some time
time.sleep(2)

# Arm the drone
master.arducopter_arm()

# Check if the drone is armed
if master.target_system == 1 and master.target_component == 1:
    print("Drone is armed!")
else:
    print("Failed to arm the drone.")
    exit()

# Wait 3 seconds before disarming the drone
time.sleep(3)

# Disarm the drone
master.arducopter_disarm()

# Check if the drone is disarmed
if not master.target_system == 1 or not master.target_component == 1:
    print("Drone is disarmed!")
else:
    print("Failed to disarm the drone.")
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

time.sleep(2)

# Arm the drone
master.arducopter_arm()

time.sleep(3)

# Disarm the drone
master.arducopter_disarm()
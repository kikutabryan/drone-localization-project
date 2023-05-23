import cv2
import numpy as np
import time
from pymavlink import mavutil

# Define the size of the marker in meters and the spacing between them
marker_size = 0.027
marker_spacing = 0.0054

# Define the aruco dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Load the camera matrix and dist coeffs from numpy array files
camera_matrix = np.load('camera_matrix.npy')
dist_coeffs = np.load('dist_coeffs.npy')

# Video capture object
cap = None

# Create the aruco board object with 4x4 grid and 16 markers
markers_x = 4
markers_y = 4
board = cv2.aruco.GridBoard_create(markers_x, markers_y, marker_size, marker_spacing, aruco_dict)

# Set the x y z coordinates
x = 0
y = 0
z = 0

# Rate limiter set up
frequency = 30
period = 1 / frequency
t0 = time.perf_counter()
time_counter = t0
target_time = t0

# Connection information
connection_address = '/dev/ttyTHS1'
baud_rate = 57600

# Create connection
master = mavutil.mavlink_connection(connection_address, baud=baud_rate)
master.wait_heartbeat()
print('Hearbeat Received!')

def main():
    while True:
        # Open the video capture from source 0
        if cap is None or not cap.isOpened():
            try:
                cap = cv2.VideoCapture(0)
                if cap.isOpened():
                    print('Video capture was opened successfully.')
                else:
                    print('Failed to open video capture. Retrying in 1 second...')
                    time.sleep(1) # Wait for 1 second before retrying

            except cv2.error as e:
                print('Error:', str(e))
                print('Retrying in 1 second...')
                time.sleep(1) # Wait for 1 second before retrying

        else:
            try:
                # Read a frame from the video capture
                ret, frame = cap.read()
                if not ret:
                    break
                
                # Convert the frame to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Detect the aruco markers in the frame
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                
                # If at least one marker is detected
                if ids is not None:
                    # Draw the detected markers on the frame
                    frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    
                    # Estimate the pose of the board relative to the camera
                    ret, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, None, None, False)
                    
                    # If the pose estimation is successful
                    if ret > 0:
                        # Draw the axis of the board on the frame
                        frame = cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size)

                    # Determine rvec and tvec from the board's frame of reference
                    R, jacobian = cv2.Rodrigues(rvec)
                    R = np.matrix(R).T
                    inv_tvec = np.dot(R, np.matrix(-tvec))
                    inv_rvec, jacobian = cv2.Rodrigues(R)

                    # The obtained rvec and tvec are from the camera's frame of reference to the frame of reference of the origin of the marker board
                    rvec = inv_rvec
                    tvec = inv_tvec

                    # Get the XYZ coordinates from the array
                    x = tvec[0, 0]
                    y = tvec[1, 0]
                    z = tvec[2, 0]

                    print('X: ', x, 'Y: ', y, 'Z: ', z)

                # Create a message with a vision position estimate
                msg = master.mav.vision_position_estimate_encode(
                    usec = int(time.time() * 1000000), # Current time in microseconds
                    x = x, # X position in meters
                    y = y, # Y position in meters
                    z = z, # Z position in meters
                    roll = 0, # Set roll angle to zero
                    pitch = 0, # Set pitch angle to zero
                    yaw = 0 # Set yaw angle to zero
                )
                
                # Delay the code to run at set frequency
                now = time.perf_counter()
                target_time += period
                if now < target_time:
                    time.sleep(target_time - now)

                # Send the message
                master.mav.send(msg)
                print(msg)
                
                # Show the frame in a window
                cv2.imshow('Aruco Marker Board', frame)
                
                # Wait for a key press for 1 ms
                key = cv2.waitKey(1)
                
                # If the key is 'q', break the loop
                if key == ord('q'):
                    break

            finally:
                # Release the video capture and destroy all windows
                cap.release()
                cv2.destroyAllWindows()

# Run the main function
if __name__ == '__main__':
    main()
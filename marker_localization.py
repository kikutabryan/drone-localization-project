import cv2
import numpy as np

# Define the size of the marker in meters and the spacing between them
marker_size = 0.027
marker_spacing = 0.0054

# Define the aruco dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Load the camera matrix and dist coeffs from numpy array files
camera_matrix = np.load('camera_matrix.npy')
dist_coeffs = np.load('dist_coeffs.npy')

# Create the aruco board object with 4x4 grid and 16 markers
board = cv2.aruco.GridBoard_create(4, 4, marker_size, marker_spacing, aruco_dict)

# Open the video capture from source 0
cap = cv2.VideoCapture(0)

# Loop until the user presses 'q' or the video ends
while True:
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

        print('x: ', x, 'y: ', y, 'z: ', z)
    
    # Show the frame in a window
    cv2.imshow('Aruco Marker Board', frame)
    
    # Wait for a key press for 1 ms
    key = cv2.waitKey(1)
    
    # If the key is 'q', break the loop
    if key == ord('q'):
        break

# Release the video capture and destroy all windows
cap.release()
cv2.destroyAllWindows()
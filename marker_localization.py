import cv2
import numpy as np
import time
from pymavlink import mavutil

# Constants for camera source
SOURCE_DEFAULT_CAMERA = 0
SOURCE_CSI_CAMERA = 1
CAMERA = SOURCE_CSI_CAMERA

# Constants for display output
DISPLAY_NONE = 0
DISPLAY_WINDOW = 1
DISPLAY_VIDEO_WRITER = 2
DISPLAY = DISPLAY_VIDEO_WRITER

# Constants for marker size
MARKER_SIZE = 0.1
MARKER_SPACING = 0.1

# Constants for amount of markers
MARKERS_X = 2
MARKERS_Y = 2

# Constants for camera offsets
X_OFFSET = 0
Y_OFFSET = 0
Z_OFFSET = 0

def main():
    # Select the camera source
    source = CAMERA

    # Display video
    display = DISPLAY

    # Set the source address based on the selected source
    if source == SOURCE_DEFAULT_CAMERA:
        address = 0  # Default camera
    elif source == SOURCE_CSI_CAMERA:
        address = (
            # CSI camera
            "nvarguscamerasrc sensor-id={sensor_id} ! "
            "video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
            "nvvidconv flip-method={flip_method} ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
        ).format(
            sensor_id=0,
            capture_width=1280,
            capture_height=720,
            framerate=60,
            flip_method=3
        )

    # Define the size of the marker in meters and the spacing between them
    marker_size = MARKER_SIZE
    marker_spacing = MARKER_SPACING

    # Define the aruco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
    aruco_params = cv2.aruco.DetectorParameters_create()

    # Load the camera matrix and distortion coefficients from numpy array files
    camera_matrix = np.load('camera_matrix.npy')
    dist_coeffs = np.load('dist_coeffs.npy')

    # Create the aruco board object with a 4x4 grid and 16 markers
    markers_x = MARKERS_X
    markers_y = MARKERS_Y
    board = cv2.aruco.GridBoard_create(markers_y, markers_x, marker_size, marker_spacing, aruco_dict)

    # Rate limiter setup
    frequency = 30
    period = 1 / frequency
    t0 = time.perf_counter()
    time_counter = t0
    target_time = t0

    # Connection information
    # connection_address = '/dev/ttyTHS1'
    # baud_rate = 57600

    # Create connection
    # master = mavutil.mavlink_connection(connection_address, baud=baud_rate)
    # master.wait_heartbeat()
    # print('Heartbeat Received!')

    # Video capture object
    cap = None

    # Videowriter object
    out = None

    # Pipeline for sending video
    pipeline = (
        "appsrc ! "
        "videoconvert ! "
        "x264enc tune=zerolatency speed-preset=superfast ! "
        "h264parse ! "
        "rtph264pay ! "
        "udpsink host=10.0.0.161 port=5600"
    )

    try:
        while True:
            # Open the video capture from the specified address
            if cap is None or not cap.isOpened():
                try:
                    cap = cv2.VideoCapture(address)
                    if cap.isOpened():
                        print('Video capture was opened successfully.')
                        if display == DISPLAY_VIDEO_WRITER:
                            # Get the video dimensions
                            frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            # Create a VideoWriter object with the GStreamer pipeline
                            out = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 30, (frame_width, frame_height), True)
                    else:
                        print('Failed to open video capture. Retrying in 1 second...')
                        time.sleep(1)  # Wait for 1 second before retrying

                except cv2.error as e:
                    print('Error:', str(e))
                    print('Retrying in 1 second...')
                    time.sleep(1)  # Wait for 1 second before retrying

            else:
                # Read a frame from the video capture
                ret, frame = cap.read()
                if not ret:
                    pass

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
                        x = tvec[0, 0] - X_OFFSET
                        y = tvec[1, 0] - Y_OFFSET
                        z = -tvec[2, 0] - Z_OFFSET

                        print('X:', format(x, ".2f"), ' Y:', format(y, ".2f"), ' Z:', format(z, ".2f"))

                        # Create a message with a vision position estimate
                        # msg = master.mav.vision_position_estimate_encode(
                        #     usec=int(time.time() * 1000000),  # Current time in microseconds
                        #     x=x,  # X position in meters
                        #     y=y,  # Y position in meters
                        #     z=z,  # Z position in meters
                        #     roll=0,  # Set roll angle to zero
                        #     pitch=0,  # Set pitch angle to zero
                        #     yaw=0  # Set yaw angle to zero
                        # )

                        # Delay the code to run at the set frequency
                        now = time.perf_counter()
                        target_time += period
                        if now < target_time:
                            time.sleep(target_time - now)

                        # Send the message
                        # master.mav.send(msg)
                        # print(msg)

                # Display the video capture frame
                if display == DISPLAY_WINDOW:
                    cv2.imshow('Aruco Marker Board', frame)
                    cv2.waitKey(1)

                # Write the frame to the VideoWriter
                elif display == DISPLAY_VIDEO_WRITER:
                    out.write(frame)

    except KeyboardInterrupt:
        print("Program interrupted by user.")

    finally:
        print("Cleaning up...")

        # Release the video capture and destroy all windows
        cap.release()
        if display == DISPLAY_WINDOW:
            cv2.destroyAllWindows()
        elif display == DISPLAY_VIDEO_WRITER:
            out.release()

# Run the main function
if __name__ == '__main__':
    main()

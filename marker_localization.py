import cv2
import numpy as np
import time

def main():
    # Define the size of the marker in meters and the spacing between them
    marker_size = 0.053
    marker_spacing = 0.0106

    # Define the aruco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
    aruco_params = cv2.aruco.DetectorParameters_create()

    # Load the camera matrix and dist coeffs from numpy array files
    camera_matrix = np.load('camera_matrix.npy')
    dist_coeffs = np.load('dist_coeffs.npy')

    # Video dimensions
    width = 1280
    height = 720

    # Video source
    # source = 0 # default camera
    source = ( # CSI camera
        "nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        "nvvidconv flip-method={flip_method} ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
    ).format(
        sensor_id=0,
        capture_width=width,
        capture_height=height,
        framerate=60,
        flip_method=3
    )

    # Video capture object
    cap = None

    # Pipeline for sending video
    pipeline = (
        "appsrc ! "
        "videoconvert ! "
        "x264enc tune=zerolatency speed-preset=superfast ! "
        "h264parse ! "
        "rtph264pay ! "
        "udpsink host=10.0.0.161 port=5600"
    )

    # Create a VideoWriter object with the GStreamer pipeline
    out = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 60, (width, height), True)

    # Create the aruco board object with 4x4 grid and 16 markers
    markers_x = 2
    markers_y = 2
    board = cv2.aruco.GridBoard_create(markers_y, markers_x, marker_size, marker_spacing, aruco_dict)

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

    try:
        while True:
            # Open the video capture from source 0
            if cap is None or not cap.isOpened():
                try:
                    cap = cv2.VideoCapture(source)
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
                # Read a frame from the video capture
                ret, frame = cap.read()
                if not ret:
                    break

                # # Convert the frame to grayscale
                # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # # Detect the aruco markers in the frame
                # corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

                # # If at least one marker is detected
                # if ids is not None:
                #     # Draw the detected markers on the frame
                #     frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                #     # Estimate the pose of the board relative to the camera
                #     ret, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, None, None, False)

                #     # If the pose estimation is successful
                #     if ret > 0:
                #         # Draw the axis of the board on the frame
                #         frame = cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size)

                #         # Determine rvec and tvec from the board's frame of reference
                #         R, jacobian = cv2.Rodrigues(rvec)
                #         R = np.matrix(R).T
                #         inv_tvec = np.dot(R, np.matrix(-tvec))
                #         inv_rvec, jacobian = cv2.Rodrigues(R)

                #         # The obtained rvec and tvec are from the camera's frame of reference to the frame of reference of the origin of the marker board
                #         rvec = inv_rvec
                #         tvec = inv_tvec

                #         # Get the XYZ coordinates from the array
                #         x = tvec[0, 0]
                #         y = tvec[1, 0]
                #         z = -tvec[2, 0]

                #         print('X:', format(x, ".2f"), ' Y:', format(y, ".2f"), ' Z:', format(z, ".2f"))

                #         # Delay the code to run at set frequency
                #         now = time.perf_counter()
                #         target_time += period
                #         if now < target_time: 
                #             time.sleep(target_time - now)

                # Write the frame to the VideoWriter
                out.write(frame)

    except KeyboardInterrupt:
        print("Program interrupted by user.")

    finally:
        print("Cleaning up...")
        # Release the video capture and destroy all windows
        cap.release()  

# Run the main function
if __name__ == '__main__':
    main()

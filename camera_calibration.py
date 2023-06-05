import cv2
import numpy as np
import time

def calibrate_camera(source, pattern_size, square_size):
    # Define termination criteria for calibration
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points (0,0,0), (1,0,0), (2,0,0), ..., (7,5,0)
    object_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    object_points[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    object_points *= square_size

    # Arrays to store object points and image points from all images
    obj_points = []  # 3D points in real-world space
    img_points = []  # 2D points in image plane

    # Video capture object
    cap = None

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
            # Read frame from video capture
            ret, frame = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

            if ret:
                obj_points.append(object_points)
                cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                img_points.append(corners)

                # Draw and display the corners
                cv2.drawChessboardCorners(frame, pattern_size, corners, ret)
                cv2.imshow('Chessboard', frame)

            cv2.imshow('Video Capture', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Release the video capture and close windows
    cap.release()
    cv2.destroyAllWindows()

    # Perform camera calibration
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert the last captured frame to grayscale
    ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

    # Save camera matrix and distortion coefficients to files
    np.save('camera_matrix.npy', camera_matrix)
    np.save('dist_coeffs.npy', dist_coeffs)

    return camera_matrix, dist_coeffs

if __name__ == '__main__':
    # source = 0  # Default camera
    source = ( # CSI camera
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

    pattern_size = (6, 7)  # Number of inner corners of the calibration pattern
    square_size = 0.0254  # Size of each square in meters (assuming the calibration pattern is printed on a square grid)

    camera_matrix, dist_coeffs = calibrate_camera(source, pattern_size, square_size)

    print('Camera Matrix:')
    print(camera_matrix)
    print('Distortion Coefficients:')
    print(dist_coeffs)

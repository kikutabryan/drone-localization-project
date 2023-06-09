import cv2
from cv2 import aruco
import numpy as np

def create_aruco_marker_board(markers_y, markers_x, marker_length, marker_separation, output_file):
    # Define the parameters for the marker board
    board = aruco.GridBoard_create(markers_y, markers_x, marker_length, marker_separation, aruco.getPredefinedDictionary(aruco.DICT_6X6_1000))

    # Generate the marker board image
    size = (marker_length + marker_separation) * markers_y
    board_image = board.draw((size, size))

    # Rotate the marker board image by 90 degrees counterclockwise
    board_image = np.rot90(board_image)

    # Save the marker board image to a file
    cv2.imwrite(output_file, board_image)

    print(f"Marker board image saved as '{output_file}'")

# Define the parameters for the marker board
# 10 mm = 37.7953 PX
markers_x = 2  # Number of markers along the X-axis
markers_y = 2  # Number of markers along the Y-axis
marker_length = 189  # Length of each marker (in pixels)
marker_separation = 38  # Separation between markers (in pixels)
output_file = "marker_board.png"  # Output file name

# Create the ArUco marker board
create_aruco_marker_board(markers_y, markers_x, marker_length, marker_separation, output_file)

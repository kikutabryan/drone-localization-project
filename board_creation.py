import cv2
from cv2 import aruco

def create_aruco_marker_board(markers_x, markers_y, marker_length, marker_separation, output_file):
    # Define the parameters for the marker board
    board = aruco.GridBoard_create(markers_x, markers_y, marker_length, marker_separation, aruco.getPredefinedDictionary(aruco.DICT_6X6_250))

    # Generate the marker board image
    size = (marker_length + marker_separation) * markers_x
    board_image = board.draw((size, size))

    # Save the marker board image to a file
    cv2.imwrite(output_file, board_image)

    print(f"Marker board image saved as '{output_file}'")

# Define the parameters for the marker board
markers_x = 4  # Number of markers along the X-axis
markers_y = 4  # Number of markers along the Y-axis
marker_length = 100  # Length of each marker (in pixels)
marker_separation = 20  # Separation between markers (in pixels)
output_file = "marker_board.png"  # Output file name

# Create the ArUco marker board
create_aruco_marker_board(markers_x, markers_y, marker_length, marker_separation, output_file)

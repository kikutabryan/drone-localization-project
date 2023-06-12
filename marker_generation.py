import cv2
from cv2 import aruco
import numpy as np

def mm_to_pixels(mm, dpi, scale):
    inches = mm / 25.4
    pixels = inches * dpi
    pixels = int(pixels * scale)
    return pixels

def generate_aruco_marker(marker_id, marker_size, x_label, y_label, output_path):
    # Get the ArUco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)

    # Generate the ArUco marker
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size, marker_image, 1)

    # Rotate the marker image by 90 degrees counterclockwise
    marker_image = np.rot90(marker_image)

    # Add a whitespace border around the marker
    border_size = 40
    border_color = 255
    marker_image = cv2.copyMakeBorder(marker_image, border_size, border_size, border_size, border_size, cv2.BORDER_CONSTANT, value=border_color)

    # Add a black border around the white space
    line_width = 2
    marker_image = cv2.copyMakeBorder(marker_image, line_width, line_width, line_width, line_width, cv2.BORDER_CONSTANT, value=(0,0,0))

    # Set the font properties for the labels
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_thickness = 1

    # Calculate the size of the x-axis and y-axis labels
    text_size_x, _ = cv2.getTextSize(x_label, font, font_scale, font_thickness)
    text_size_y, _ = cv2.getTextSize(y_label, font, font_scale, font_thickness)
    title_size, _ = cv2.getTextSize(str(marker_id), font, font_scale, font_thickness)

    # Calculate the position of the x-axis label
    x_label_position = (marker_size + 2 * border_size - text_size_x[0]) // 2, marker_size + border_size + (border_size + text_size_x[1]) // 2

    # Calculate the position of the y-axis label
    y_label_position = (border_size - text_size_y[0]) // 2, (marker_size + 2 * border_size + text_size_y[1]) // 2

    # Calculate the position of the id title label
    title_id_position = (marker_size + 2 * border_size - title_size[0]) // 2, (border_size + title_size[1]) // 2

    # Add the x-axis and y-axis labels to the marker image with border
    cv2.putText(marker_image, x_label, x_label_position, font, font_scale, 0, font_thickness, cv2.LINE_AA)
    cv2.putText(marker_image, y_label, y_label_position, font, font_scale, 0, font_thickness, cv2.LINE_AA)
    cv2.putText(marker_image, str(marker_id), title_id_position, font, font_scale, 0, font_thickness, cv2.LINE_AA)

    # Create the output file for the marker
    output_file = output_path + f"aruco_marker_{marker_id}.png"

    # Save the marker image with labels as a PNG file
    cv2.imwrite(output_file, marker_image)


def main():
    start_id = 0
    marker_count = 5
    marker_size = mm_to_pixels(100, 96, 1.034875298)
    output_path = "aruco_markers/"
    x_label = "Y"
    y_label = "X"

    for marker_id in range(start_id, marker_count):
        # Generate the ArUco marker with labels
        generate_aruco_marker(marker_id, marker_size, x_label, y_label, output_path)

# Run the main function
if __name__ == '__main__':
    main()
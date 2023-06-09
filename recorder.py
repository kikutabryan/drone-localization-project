import cv2
import time

def main():
    # Select the camera source
    source = 1
    
    # Display video
    display = False

    # Set the source address based on the selected source
    if source == 0:
        address = 0  # Default camera
    elif source == 1:
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

    output_file = 'video.mp4'  # Output file name
    codec = cv2.VideoWriter_fourcc(*'mp4v')  # Codec (four character code)
    fps = 30.0  # Frames per second

    # Video capture object
    cap = None
    out = None

    try:
        while True:
            # Open the video capture from the specified address
            if cap is None or not cap.isOpened():
                try:
                    cap = cv2.VideoCapture(address)
                    if cap.isOpened():
                        print('Video capture was opened successfully.')
                        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        out = cv2.VideoWriter(output_file, codec, fps, (frame_width, frame_height)) # Create the video writer object
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

                out.write(frame)  # Write the frame to the output video file

                # Display the video capture frame
                if display:
                    cv2.imshow('Recording', frame)
                    cv2.waitKey(1)

    except KeyboardInterrupt:
        print("Program interrupted by user.")

    finally:
        print("Cleaning up...")

        # Release the video capture and destroy all windows
        cap.release()
        out.release()  # Release the video writer object
        if display:
            cv2.destroyAllWindows()  # Close all OpenCV windows

# Run the main function
if __name__ == '__main__':
    main()
import cv2
import time

def main():
    source = 0  # Default camera
    # source = ( # CSI camera
    #     "nvarguscamerasrc sensor-id={sensor_id} ! "
    #     "video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
    #     "nvvidconv flip-method={flip_method} ! "
    #     "videoconvert ! "
    #     "video/x-raw, format=(string)BGR ! appsink"
    # ).format(
    #     sensor_id=0,
    #     capture_width=1280,
    #     capture_height=720,
    #     framerate=60,
    #     flip_method=3
    # )

    output_file = 'video.mp4'  # Output file name
    codec = cv2.VideoWriter_fourcc(*'mp4v')  # Codec (four character code)
    fps = 30.0  # Frames per second

    cap = None
    

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # Get the camera's frame width
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  # Get the camera's frame height

    out = cv2.VideoWriter(output_file, codec, fps, (frame_width, frame_height))  # Create the video writer object
    
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
                    pass

                out.write(frame)  # Write the frame to the output video file

                cv2.imshow('Recording', frame)  # Display the resulting frame

                cv2.waitKey(1)
    
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    
    finally:
        print("Cleaning up...")
        # Release the video capture
        cap.release()

    cap.release()  # Release the video capture object
    out.release()  # Release the video writer object
    cv2.destroyAllWindows()  # Close all OpenCV windows

# Run the main function
if __name__ == '__main__':
    main()
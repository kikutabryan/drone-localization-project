import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 30)
        self.rate = self.create_timer(1, self.timer_callback)

        # Define the size of the marker in meters and the spacing between them
        self.marker_size = 0.053
        self.marker_spacing = 0.0106

        # Define the aruco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Load the camera matrix and dist coeffs from numpy array files
        self.camera_matrix = np.load('camera_matrix.npy')
        self.dist_coeffs = np.load('dist_coeffs.npy')

        # Video source
        self.source = 0 # default camera

        # Video capture object
        self.cap = None

        # Create the aruco board object with 4x4 grid and 16 markers
        self.markers_x = 2
        self.markers_y = 2
        self.board = cv2.aruco.GridBoard_create(
            self.markers_y, self.markers_x, self.marker_size, self.marker_spacing, self.aruco_dict
        )

    def timer_callback(self):
        # this is an empty callback function
        pass

    def publish_position(self):
        # Set the x y z coordinates
        x = 0
        y = 0
        z = 0

        try:
            while True:
                # Open the video capture from source 0
                if self.cap is None or not self.cap.isOpened():
                    try:
                        self.cap = cv2.VideoCapture(self.source)
                        if self.cap.isOpened():
                            self.get_logger().info('Video capture was opened successfully.')
                        else:
                            self.get_logger().info('Failed to open video capture. Retrying in 1 second...')
                            self.rate.sleep(1) # Wait for 1 second before retrying

                    except cv2.error as e:
                        self.get_logger().error('Error: ' + str(e))
                        self.get_logger().info('Retrying in 1 second...')
                        self.rate.sleep(1) # Wait for 1 second before retrying

                else:
                    # Read a frame from the video capture
                    ret, frame = self.cap.read()
                    if not ret:
                        break

                    # Convert the frame to grayscale
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                    # Detect the aruco markers in the frame
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        gray, self.aruco_dict, parameters=self.aruco_params
                    )

                    # If at least one marker is detected
                    if ids is not None:
                        # Estimate the pose of the board relative to the camera
                        ret, rvec, tvec = cv2.aruco.estimatePoseBoard(
                            corners, ids, self.board, self.camera_matrix, self.dist_coeffs,
                            None, None, False
                        )

                        # If the pose estimation is successful
                        if ret > 0:
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
                            z = -tvec[2, 0]

                            # Publish the position as a ROS2 message
                            pose_msg = PoseStamped()
                            pose_msg.header.frame_id = 'map'  # Set the appropriate frame ID
                            pose_msg.pose.position.x = x
                            pose_msg.pose.position.y = y
                            pose_msg.pose.position.z = z
                            self.publisher_.publish(pose_msg)

        except KeyboardInterrupt:
            self.get_logger().info("Program interrupted by user.")

        finally:
            self.get_logger().info("Cleaning up...")
            # Release the video capture and destroy all windows
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    position_publisher = PositionPublisher()

    position_publisher.publish_position()

    rclpy.spin(position_publisher)

    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

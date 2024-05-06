import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import cv2
import numpy as np

def occupancy_grid_to_cv2_image(occupancy_grid_msg):
    # Extract data from the OccupancyGrid message
    width = occupancy_grid_msg.info.width
    height = occupancy_grid_msg.info.height
    data = occupancy_grid_msg.data

    # Reshape the data into a 2D numpy array
    occupancy_grid = np.array(data).reshape((height, width))

    # Convert occupancy grid data to grayscale image
    # 0 represents free space, 100 represents occupied space, -1 represents unknown space
    # You may need to adjust this mapping depending on your use case
    occupancy_grid_image = np.zeros((height, width, 3), dtype=np.uint8)
    occupancy_grid_image[occupancy_grid == 0] = [255, 255, 255]  # White for free space
    occupancy_grid_image[occupancy_grid == 100] = [0, 0, 0]       # Black for occupied space
    occupancy_grid_image[occupancy_grid == -1] = [128, 128, 128]  # Gray for unknown space

    return occupancy_grid_image

class OccupancyViewer(Node):
    def __init__(self):
        super().__init__('occupancy_viewer_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/merge_map',  # Change this to the correct topic name
            self.occupancy_grid_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Change this to the correct Odometry topic name
            self.odometry_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.odom_subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def occupancy_grid_callback(self, msg):
        # Convert occupancy grid message to an OpenCV image
        self.occupancy_grid_image = occupancy_grid_to_cv2_image(msg)

        self.update_display()

    def odometry_callback(self, msg):
        # Extract robot's position from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Draw a red dot at the robot's position on the occupancy grid image
        cv2.circle(self.occupancy_grid_image, (int(x), int(y)), 5, (0, 0, 255), -1)

        self.update_display()

    def update_display(self):
        # Display the image
        cv2.imshow('Occupancy Grid', self.occupancy_grid_image)
        cv2.waitKey(1)  # Refresh display


def main(args=None):
    rclpy.init(args=args)
    occupancy_viewer_node = OccupancyViewer()
    rclpy.spin(occupancy_viewer_node)
    # Destroy OpenCV windows upon shutdown
    cv2.destroyAllWindows()
    occupancy_grid_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

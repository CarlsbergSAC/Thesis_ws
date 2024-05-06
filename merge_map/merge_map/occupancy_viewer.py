import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
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
    occupancy_grid_image[occupancy_grid < 50] = [255, 255, 255]  # White for free space
    occupancy_grid_image[occupancy_grid >= 50] = [0, 0, 0]       # Black for occupied space
    occupancy_grid_image[occupancy_grid == -1] = [128, 128, 128]  # Gray for unknown space

    return occupancy_grid_image

class OccupancyViewer(Node):
    def __init__(self):
        super().__init__('occupancy_viewer_node')

        # get input parameters
        self.declare_parameter("number_robots")
        self.number_robots = int(self.get_parameter('number_robots').value)

        # declare occupancy grid subscription and value
        self.occupancy_grid = None
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/merge_map',  # Change this to the correct topic name
            self.occupancy_grid_callback,
            10)

        # robot positions
        self.robot_positions = {}
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/robot_0/odom',  # Change this to the correct Odometry topic name
            self.odometry_callback,
            10)


    def occupancy_grid_callback(self, msg):
        # Convert occupancy grid message to an OpenCV image
        self.occupancy_grid = msg
        
        self.update_display()

    def odometry_callback(self, msg):
        # Extract robot's position from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.robot_positions[0] = (int(x), int(y))
        

    def update_display(self):
        # Display the image
         # Convert occupancy grid message to an OpenCV image
        if self.occupancy_grid is not None:
            cv_image = occupancy_grid_to_cv2_image(self.occupancy_grid)

            for pos in self.robot_positions:
                # convert x,y to pixel location
                x,y = self.convert_robot_pos(self.robot_positions[pos])
                self.get_logger().info(f'x: {x}, y: {y}')
                cv2.circle(cv_image, (x,y), 5, (0, 0, 255), -1)


            cv_image = cv2.resize(cv_image, (800, 800))
            cv2.imshow('Occupancy Grid', cv_image)
            cv2.waitKey(1)  # Refresh display

    def convert_robot_pos(self, pos):

        x_world, y_world = pos
        # Convert to grid indices
        x_grid = int((x_world - self.occupancy_grid.info.origin.position.x) / self.occupancy_grid.info.resolution)
        y_grid = int((y_world - self.occupancy_grid.info.origin.position.y) / self.occupancy_grid.info.resolution)
        # Ensure position is within grid bounds
        x_grid = max(0, min(x_grid, self.occupancy_grid.info.width - 1))
        y_grid = max(0, min(y_grid, self.occupancy_grid.info.height - 1))

        return x_grid, y_grid


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

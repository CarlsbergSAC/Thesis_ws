import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from threading import Lock
import threading

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

        for i in range(self.number_robots):
            subscription = self.create_subscription(
                Odometry,
                f'/robot_{i}/odom',  # Adjust topic name pattern as per your setup
                lambda msg, i=i: self.odometry_callback(msg, i),  # Pass i to callback using lambda
                10)

        # for odom and map data
        self.lock_pos = threading.Lock()  # Mutex lock for thread safety
        self.lock_grid = threading.Lock()

        # map image publishing thread
        self.process_maps_thread = threading.Thread(target=self.update_display)
        self.process_maps_thread.daemon = True  # Make sure the thread exits when the main thread exits
        self.process_maps_thread.start()


    def occupancy_grid_callback(self, msg):
        # Convert occupancy grid message to an OpenCV image
        with self.lock_grid:
            self.occupancy_grid = msg
        

    def odometry_callback(self, msg, robot_index):
        # Extract robot's position from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self.lock_pos:
            self.robot_positions[robot_index] = (x, y)
        

    def update_display(self):
        # Display the image
         # Convert occupancy grid message to an OpenCV image
        while rclpy.ok():

            with self.lock_pos:
                robot_pos = self.robot_positions
            with self.lock_grid:
                grid = self.occupancy_grid

            if grid is not None:
                cv_image = occupancy_grid_to_cv2_image(grid)

                for pos in robot_pos:
                    # convert x,y to pixel location
                    x,y = self.convert_robot_pos(robot_pos[pos])
          
                    cv2.circle(cv_image, (x,y), 5, (0, 0, 255), -1)


                cv_image = cv2.resize(cv_image, (800, 800))
                cv2.imshow('Occupancy Grid', cv_image)
                cv2.waitKey(1)  # Refresh display

                #self.get_logger().info(f'x: {x}, y: {y}')

            time.sleep(0.5)

    def convert_robot_pos(self, pos):
        
        with self.lock_grid:
            grid = self.occupancy_grid

        x_world, y_world = pos
        # Convert to grid indices
        x_grid = int((x_world - grid.info.origin.position.x) / grid.info.resolution)
        y_grid = int((y_world - grid.info.origin.position.y) / grid.info.resolution)
        # Ensure position is within grid bounds
        x_grid = max(0, min(x_grid, grid.info.width - 1))
        y_grid = max(0, min(y_grid, grid.info.height - 1))

        return x_grid, y_grid


def main(args=None):
    rclpy.init(args=args)
    occupancy_viewer_node = OccupancyViewer()

    try:
        rclpy.spin(occupancy_viewer_node)
    
    except KeyboardInterrupt:
        pass
    
    # Destroy OpenCV windows upon shutdown
    cv2.destroyAllWindows()
    occupancy_grid_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

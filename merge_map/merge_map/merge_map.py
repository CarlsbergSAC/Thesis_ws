# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import logging
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import sys
from queue import Queue
from threading import Lock
import threading
import time


def merge_maps(map1, map2):
    merged_map = OccupancyGrid()
    merged_map.header = map1.header
    merged_map.header.frame_id = 'merge_map'
    min_x = min(map1.info.origin.position.x, map2.info.origin.position.x)
    min_y = min(map1.info.origin.position.y, map2.info.origin.position.y)
    max_x = max(map1.info.origin.position.x + (map1.info.width * map1.info.resolution),
                map2.info.origin.position.x + (map2.info.width * map2.info.resolution))
    max_y = max(map1.info.origin.position.y + (map1.info.height * map1.info.resolution),
                map2.info.origin.position.y + (map2.info.height * map2.info.resolution))
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
    for y in range(map1.info.height):
        for x in range(map1.info.width):
            i = x + y * map1.info.width
            merged_x = int(np.floor((map1.info.origin.position.x + x * map1.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map1.info.origin.position.y + y * map1.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            merged_map.data[merged_i] = map1.data[i]
    for y in range(map2.info.height):
        for x in range(map2.info.width):
            i = x + y * map2.info.width
            merged_x = int(np.floor((map2.info.origin.position.x + x * map2.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map2.info.origin.position.y + y * map2.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            if merged_map.data[merged_i] == -1:
                merged_map.data[merged_i] = map2.data[i]
    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.get_logger().info('init')

        # get input parameters
        self.declare_parameter("number_robots")
        self.number_robots = int(self.get_parameter('number_robots').value)

        self.get_logger().info(str(self.number_robots))

        # publisher for merge_map
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)

        # make subscription to all robots map
        self.map_subscription = []
        for i in range(self.number_robots):
            self.map_subscription.append(self.create_subscription(OccupancyGrid, '/robot_'+str(i)+'/map', self.map_callback, 10))

        self.merge_map = None
        
        # publish map tf
        self.publisher_tf = self.create_publisher(TFMessage, '/tf', 10)

        # Queue and lock for map processing
        self.map_queue = Queue()
        self.map_processing_lock = Lock()

        # Start tf publishing thread
        self.tf_thread = threading.Thread(target=self.publish_map_tf)
        self.tf_thread.daemon = True  # Make sure the thread exits when the main thread exits
        self.tf_thread.start()

        # Start map processing thread
        self.process_maps_thread = threading.Thread(target=self.process_maps)
        self.process_maps_thread.daemon = True  # Make sure the thread exits when the main thread exits
        self.process_maps_thread.start()

    def map_callback(self, msg):
        map = msg
        self.map_queue.put(map)
        self.get_logger().debug('map_callback_end')

    def process_maps(self):
        #self.publish_map_tf()   
        while rclpy.ok():
            if not self.map_queue.empty():
                map = self.map_queue.get()
            else:
                time.sleep(0.1)  # Wait for new maps with a 100ms timeout
                continue

            if self.merge_map is not None:
                self.merge_map = merge_maps(self.merge_map, map)
                self.publisher.publish(self.merge_map)
            else:
                self.merge_map = map
                self.publisher.publish(self.merge_map)
            

            self.get_logger().debug('map_published')

    def publish_map_tf(self):
        while rclpy.ok():
            tf_message = TFMessage()

            # Create a non-static transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'world'
            transform.child_frame_id = 'merge_map'
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0

            tf_message.transforms.append(transform)

        
            self.publisher_tf.publish(tf_message)

            # if self.merge_map is not None:
            #     self.publisher.publish(self.merge_map)

            #self.get_logger().info('tf2_pub')
            time.sleep(0.1) 


def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()

    try:
        rclpy.spin(merge_map_node)
                # Spin the node in its own thread
        # node_thread = threading.Thread(target=lambda: rclpy.spin(merge_map_node))
        # node_thread.start()

        # # Wait for the node thread to finish
        # node_thread.join()
    except KeyboardInterrupt:
        pass
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

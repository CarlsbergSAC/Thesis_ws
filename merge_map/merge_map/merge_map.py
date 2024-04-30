# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import signal
import logging
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


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
    
        self.number_robots  = self.get_parameter()

        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.subscription1 = self.create_subscription(OccupancyGrid, '/map1', self.map1_callback, 10)
        self.subscription2 = self.create_subscription(OccupancyGrid, '/map2', self.map2_callback, 10)
        self.map1 = None
        self.map2 = None
        
        # publish map tf
        self.publisher_tf = self.create_publisher(TFMessage, '/tf', 10)

    def map1_callback(self, msg):
        self.map1 = msg
        #self.get_logger().info('map1_callback')
        if self.map2 is not None:
            msg = merge_maps(self.map1, self.map2)
            self.publisher.publish(msg)
        else:
            self.publisher.publish(self.map1)
            #self.map2=self.map1
        self.publish_map_tf()
    
    def map2_callback(self, msg):
        self.map2 = msg
        #self.get_logger().info('map2_callback')
        # if self.map1 is not None:
        #     msg = merge_maps(self.map1, self.map2)
        #     self.publisher.publish(msg)

    def publish_map_tf(self):
        tf_message = TFMessage()
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'
        static_transform.child_frame_id = 'merge_map'
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        tf_message.transforms.append(static_transform)
        self.publisher_tf.publish(tf_message)
        self.get_logger().info('tf2_pub')



def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()

    try:
        rclpy.spin(merge_map_node)
        #rclpy.spin(tf2_publisher)
    except KeyboardInterrupt:
        pass
    merge_map_node.destroy_node()
    tf2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

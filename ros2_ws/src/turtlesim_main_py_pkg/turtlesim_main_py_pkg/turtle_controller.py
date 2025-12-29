#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import rclpy
from rclpy.node import Node

class MyOOPNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.counter_ = 0
        self.get_logger().info('MyOOPNode 1 has been initialized!')
        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_) )
        self.counter_ += 1



def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of MyOOPNode
    node = MyOOPNode("py_oop_node_1")
    
    # Log an info message to indicate the node has started
    node.get_logger().info('Hello 1, ROS 2 from MyOOPNode!')
    
    try:
        # Keep the node running, processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # This block always runs, even if an exception occurs
        node.get_logger().info('Shutting down MyOOPNode...')
        # Destroy the node explicitly
        node.destroy_node() 
        # Shutdown the ROS 2 client library
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# End of my_oop_node.py
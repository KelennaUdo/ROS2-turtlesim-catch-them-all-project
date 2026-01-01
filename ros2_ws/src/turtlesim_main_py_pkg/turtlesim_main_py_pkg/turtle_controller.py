#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
from math import atan2, pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info('TurtleController has been initialized!')
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
        )
        self.log_period_ = self.declare_parameter('log_period', 1.0).value
        self.last_log_time_ = self.get_clock().now()
        
    target_pose = Pose()
    target_pose.x = 0.0
    target_pose.y = 0.0

    def P_controller_to_calculate_vel_cmd(self, current_pose: Pose, target_pose: Pose) -> Twist:
        # Calculate errors
        dx = target_pose.x - current_pose.x
        dy = target_pose.y - current_pose.y
        distance_error = (dx**2 + dy**2)**0.5
        angle_to_target = atan2(dy, dx)
        angle_error = self._normalize_angle(angle_to_target - current_pose.theta)

        # Proportional control gains
        K_linear = 1.0
        K_angular = 6.0

        # Calculate control commands
        linear_velocity = K_linear * distance_error
        angular_velocity = K_angular * angle_error

        # Create and return the Twist message
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_velocity
        return cmd
    
    # Helper method to normalize angles to the range [-pi, pi]
    def _normalize_angle(self, angle: float) -> float:
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def pose_callback(self, msg: Pose) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time_).nanoseconds >= int(self.log_period_ * 1e9):
            self.get_logger().info(
                f'Pose x={msg.x:.2f} y={msg.y:.2f} theta={msg.theta:.2f}'
            )
            self.last_log_time_ = now
        # Publish a zero-velocity command on each pose update.
        cmd = self.P_controller_to_calculate_vel_cmd(msg, self.target_pose)
        self.cmd_vel_pub_.publish(cmd)





def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of TurtleController
    node = TurtleController("turle_contoller")
    
    # Log an info message to indicate the node has started
    node.get_logger().info('Hello ROS 2 from TurtleController!')
    
    try:
        # Keep the node running, processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # This block always runs, even if an exception occurs
        node.get_logger().info('Shutting down TurtleController...')
        # Destroy the node explicitly
        node.destroy_node() 
        # Shutdown the ROS 2 client library
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# End of turtle_controller.py

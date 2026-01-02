#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
from math import atan2, pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_turtlesim_interfaces.msg import TurtleInfo
from my_turtlesim_interfaces.msg import TurtleInfoArray
from my_turtlesim_interfaces.srv import CatchTurtle

class TurtleController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info('TurtleController has been initialized!')
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Subscription to the master turtle's (turtle1) pose
        self.pose_sub_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
        )
        # subscription to the alive turtles topic
        self.alive_turtles_sub_ = self.create_subscription(
            TurtleInfoArray,
            '/alive_turtles',
            self.alive_turtles_callback,
            10,
        )
        # client for the catch_turtle service
        self.catch_turtle_client_ = self.create_client(
            CatchTurtle,
            '/catch_turtle'
        )

        # logging period parameter
        self.log_period_ = self.declare_parameter('log_period', 1.0).value
        self.last_log_time_ = self.get_clock().now()

        # target turtle pose and default initialization
        self.target_pose = Pose()
        self.target_pose.x = 5.0
        self.target_pose.y = 5.0
        self.target_pose.theta = 0.0

        # target turtle info and catch state
        self.target_turtle = TurtleInfo()
        self.catch_in_flight_ = False

        # current pose of the master turtle
        self.current_pose = Pose()



    # convert TurtleInfo to Pose
    def __get_pose__(self, turtle_info: TurtleInfo) -> Pose:
        pose = Pose()
        pose.x = turtle_info.x
        pose.y = turtle_info.y
        pose.theta = turtle_info.theta
        return pose
    

    
    # get the closest turtle from a list of turtles
    def get_closest_turtle(self, turtles: list) -> TurtleInfo:
        min_distance = float('inf')
        closest_turtle = None
        for turtle in turtles:
            dx = turtle.x - self.current_pose.x
            dy = turtle.y - self.current_pose.y
            distance = (dx**2 + dy**2)**0.5
            # Check if this turtle is closer than the current closest
            if distance < min_distance:
                min_distance = distance
                closest_turtle = turtle
        return closest_turtle



    def P_controller_to_calculate_vel_cmd(self, current_pose: Pose, target_pose: Pose) -> Twist:
        # Calculate errors
        dx = target_pose.x - current_pose.x
        dy = target_pose.y - current_pose.y

        distance_error = (dx**2 + dy**2)**0.5
    
        ###################################################
        # Call catch_turtle when close enough to the target.
        if distance_error < 0.2 and not self.catch_in_flight_:
            if self.catch_turtle_client_.service_is_ready():
                request = CatchTurtle.Request()
                request.name = self.target_turtle.name
                future = self.catch_turtle_client_.call_async(request)
                future.add_done_callback(self.handle_catch_response)
                self.catch_in_flight_ = True
            else:
                self.get_logger().warn('CatchTurtle service not available')
        ###################################################

        angle_to_target = atan2(dy, dx)
        angle_error = self._normalize_angle(angle_to_target - current_pose.theta)

        # Proportional control gains
        K_linear = 2.5
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
        self.current_pose = msg
        now = self.get_clock().now()
        if (now - self.last_log_time_).nanoseconds >= int(self.log_period_ * 1e9):
            self.get_logger().info(
                f'Pose x={self.current_pose.x:.2f} y={self.current_pose.y:.2f} theta={self.current_pose.theta:.2f}'
            )
            self.last_log_time_ = now
        # Publish a zero-velocity command on each pose update.
        cmd = self.P_controller_to_calculate_vel_cmd(self.current_pose, self.target_pose)
        self.cmd_vel_pub_.publish(cmd)

    def alive_turtles_callback(self, msg: TurtleInfoArray,) -> None:
        if len(msg.turtles) == 0:
            self.get_logger().info('No alive turtles detected.')
        else:            
            self.target_turtle = self.get_closest_turtle(msg.turtles)
            self.target_pose = self.__get_pose__(self.target_turtle)

    def handle_catch_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'CatchTurtle service call failed: {exc}')
            self.catch_in_flight_ = False
            return

        if response.success:
            self.get_logger().info(f'Successfully caught {self.target_turtle.name}')

        else:
            self.get_logger().warn(f'Failed to catch {self.target_turtle.name}')
        self.catch_in_flight_ = False


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

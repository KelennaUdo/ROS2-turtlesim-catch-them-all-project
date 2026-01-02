#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import random
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_turtlesim_interfaces.msg import TurtleInfo
from my_turtlesim_interfaces.msg import TurtleInfoArray
from my_turtlesim_interfaces.srv import CatchTurtle


class TurtleSpawner(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.get_logger().info('TurtleSpawner has been initialized!')

        # Client for the turtlesim spawn service.
        self.spawn_client_ = self.create_client(
            Spawn, 
            '/spawn'
        )

        # Publisher for the turtle info array.
        self.turtle_info_pub_ = self.create_publisher(
            TurtleInfoArray,
            '/alive_turtles',
            10
        )

        # server of the /catch_turtle service 
        self.catch_turtle_server_ = self.create_service(
            CatchTurtle,
            '/catch_turtle',
            self.catch_turtle_callback
        )
        # client for the kill service to remove turtles
        self.kill_client_ = self.create_client(
            Kill,
            '/kill'
        )

        # Initialize the list of alive turtles
        self.alive_turtles_ = TurtleInfoArray()

        # Timer period (seconds) controls how often a new turtle is spawned.
        self.spawn_period_ = self.declare_parameter('spawn_period', 1.0).value
        self.spawn_count_ = 0
        self._service_warned = False
        self.create_timer(self.spawn_period_, self.timer_callback)

        # function to update alive turtles list and publish it
    def update_alive_turtles_and_publish(self, turtle_info: TurtleInfo, is_being_removed: bool) -> None:
        if is_being_removed:
            for turtle in self.alive_turtles_.turtles:
                if turtle.name == turtle_info.name:
                    self.alive_turtles_.turtles.remove(turtle)
                    break
        else:
            self.alive_turtles_.turtles.append(turtle_info)
        self.turtle_info_pub_.publish(self.alive_turtles_)

    def timer_callback(self) -> None:
        # Avoid sending requests until the service becomes available.
        if not self.spawn_client_.service_is_ready():
            if not self._service_warned:
                self.get_logger().warn('Waiting for /spawn service...')
                self._service_warned = True
            return

        self._service_warned = False
        request = Spawn.Request()
        # Generate a simple repeating pattern of spawn positions in-bounds.
        request.x = random.uniform(1.0, 9.0)
        request.y = random.uniform(1.0, 9.0)
        request.theta = 0.0
        # Name turtles sequentially so they are easy to identify.
        request.name = f'turtle{self.spawn_count_ + 2}'

        # Update and publish the list of alive turtles.
        turtle_info = TurtleInfo()
        turtle_info.name = request.name
        turtle_info.x = request.x
        turtle_info.y = request.y
        turtle_info.theta = request.theta
        self.update_alive_turtles_and_publish(turtle_info, is_being_removed=False)

        # Call the service asynchronously to keep the timer callback fast.
        future = self.spawn_client_.call_async(request)
        future.add_done_callback(self.handle_spawn_response)

        self.spawn_count_ += 1
    

    def handle_spawn_response(self, future) -> None:
        # Log the outcome of the spawn request.
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to spawn turtle: {exc}')
            
            # if the spawn failed, remove the turtle from the list
            turtle_info = TurtleInfo()
            turtle_info.name = future.result().name
            self.update_alive_turtles_and_publish(turtle_info, is_being_removed=True)
            return

        self.get_logger().info(f'Spawned turtle: {response.name}')

    def catch_turtle_callback(self, request: CatchTurtle.Request, response: CatchTurtle.Response) -> CatchTurtle.Response:
        turtle_name_to_catch = request.name
        turtle_found = False

        for turtle in self.alive_turtles_.turtles:
            if turtle.name == turtle_name_to_catch:
                turtle_found = True
                # Call the kill service to remove the turtle
                kill_request = Kill.Request()
                kill_request.name = turtle_name_to_catch
                if not self.kill_client_.service_is_ready():
                    self.get_logger().warn('Waiting for /kill service...')
                    turtle_found = False
                    break
                kill_future = self.kill_client_.call_async(kill_request)
                kill_future.add_done_callback(
                    partial(self.handle_kill_response, turtle_name_to_catch)
                )
                break

        if not turtle_found:
            self.get_logger().info(f'Turtle {turtle_name_to_catch} not found among alive turtles.')


        response.success = turtle_found
        return response

    def handle_kill_response(self, turtle_name: str, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to kill turtle: {exc}')
            return

        # If successful, update and publish the list of alive turtles.
        turtle_info = TurtleInfo()
        turtle_info.name = turtle_name
        self.update_alive_turtles_and_publish(turtle_info, is_being_removed=True)
        self.get_logger().info(f'Turtle {turtle_name} has been killed.')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner('turtle_spawner')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down TurtleSpawner...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

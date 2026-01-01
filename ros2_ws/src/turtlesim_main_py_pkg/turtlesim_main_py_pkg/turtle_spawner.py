#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random


class TurtleSpawner(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.get_logger().info('TurtleSpawner has been initialized!')

        # Client for the turtlesim spawn service.
        self.spawn_client_ = self.create_client(Spawn, '/spawn')
        # Timer period (seconds) controls how often a new turtle is spawned.
        self.spawn_period_ = (
            self.declare_parameter('spawn_period', 1.0).value
        )
        self.spawn_count_ = 0
        self._service_warned = False
        self.create_timer(self.spawn_period_, self.timer_callback)

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
            return

        self.get_logger().info(f'Spawned turtle: {response.name}')


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

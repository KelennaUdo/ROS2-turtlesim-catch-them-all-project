# ROS 2 Turtlesim — Catch Them All

A Robot Operating System 2 (ROS 2) learning project using Turtlesim. One turtle (`turtle1`) autonomously chases and “catches” turtles that spawn at random positions, using ROS 2 topics and services.

## Project overview

This project is based on a common ROS 2 learning challenge:

- **turtlesim_node** (from the Turtlesim package) runs the simulation.
- **turtle_spawner** spawns turtles, keeps a list of “alive” turtles, and exposes a service to catch a turtle.
- **turtle_controller** drives `turtle1` toward a target turtle using a proportional (P) controller, and calls the catch service when it reaches the target.

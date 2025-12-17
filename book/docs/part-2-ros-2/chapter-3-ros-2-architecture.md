---
sidebar_position: 3
---

# Chapter 3: ROS 2 Architecture

This chapter dives into the heart of modern robotics development: the Robot Operating System 2 (ROS 2). We'll explore its fundamental architecture, key concepts, and how it enables complex robotic systems to communicate and coordinate.

## The ROS 2 Graph

At the core of ROS 2 is the "ROS graph," a network of interconnected processes (nodes) that exchange information. This distributed architecture promotes modularity and reusability, allowing developers to create complex robotic behaviors by combining smaller, independent components.

## Nodes, Topics, Services, and Actions

ROS 2 provides several communication mechanisms for inter-node communication:

*   **Nodes**: Executable processes that perform computation (e.g., a camera driver node, a navigation node).
*   **Topics**: A publish/subscribe messaging system for continuous data streams (e.g., sensor data, motor commands).
*   **Services**: A request/reply mechanism for discrete, synchronous communication (e.g., requesting a robot to perform a specific action and waiting for a response).
*   **Actions**: A long-running goal-oriented communication mechanism with feedback, status, and result (e.g., commanding a robot to navigate to a far-off destination).

## The ROS 2 Command-Line Interface

ROS 2 comes with a powerful set of command-line tools for inspecting, debugging, and interacting with a running ROS 2 system. We'll cover essential commands like `ros2 run`, `ros2 topic list`, `ros2 topic echo`, `ros2 node list`, and `ros2 service call`.

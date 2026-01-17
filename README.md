# Autonomous Navigation of TurtleBot3 using Bi-RRT and Pure Pursuit in ROS/Gazebo
This repository implements a complete autonomous navigation pipeline for TurtleBot3 in a ROS–Gazebo simulation.
A modified Bi-directional RRT (Bi-RRT) global planner is used to generate collision-free paths between goal points, while a Pure Pursuit controller tracks the resulting trajectories and avoids static obstacles in the environment.

## Objective
The robot must autonomously navigate through a cluttered environment and reach a sequence of target positions under time constraints:
- x₀ = [0, 0, 0]
- p₁ = [-1.5, -2.9] (≤ 60 s, optimal ≈ 44 s)
- p₂ = [-3.0, 6.0] (≤ 110 s, optimal ≈ 73 s)
- p₃ = [5.25, 5.5] (≤ 75 s, optimal ≈ 51 s)

Coordinates are expressed in meters in the Gazebo world frame.

## Motivation
Sampling-based planners such as RRT are well-suited for high-dimensional and cluttered environments.
A Bi-directional RRT (Bi-RRT) approach improves convergence speed by growing two trees—one from the start and one from the goal—and attempting to connect them.

The Pure Pursuit controller was selected for local trajectory tracking because:
- It is computationally lightweight
- It is well-suited for differential-drive robots
- It provides smooth curvature-based control for path following
- This combination enables fast global planning with reliable low-level execution.

## System Architecture
The navigation system is organized as a modular ROS pipeline:
- **Mission Scheduler (Goal Manager Node)**
  - Sends one target waypoint at a time to the planner
  - Ensures sequental execution of p₁ → p₂ → p₃ and the intermediate waypoints between them
- **Global Planner (Bi-RRT Node)**
  - Computes collision-free paths between successive goals
- **Local Planner Node**
  - Refines the global path and selects tracking points
- **Controller Node (Pure Pursuit)**
  - Tracks the local trajectory
  - Outputs velocity commands to the robot
This structure separates task sequencing, planning, and control, mirroring real-world autonomy pipelines.

Ensures sequential execution of 

Global Planner (Modified Bi-RRT Node)

Computes collision-free paths between successive goals

Generates intermediate waypoints

Local Planner Node

Refines the global path and selects tracking points

Controller Node (Pure Pursuit)

Tracks the local trajectory

Outputs velocity commands to the robot

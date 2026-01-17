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
<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/f4cc9c91-d825-4c9f-bf58-6d0c9e6b66b9" />


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

## Navigation Algorithm
Two trees are grown:
- One from the current robot pose
- One from goal position

At each iteration:
1. Sample a random point
2. Extend both trees toward the sample
3. Attempt to connect them
4. Validate collision-free segments

Once connected, the resulting path is extracted and post-processed by the local planner

## Local Planning and Control (Pure Pursuit)
Given a path:
- A lookahead point at a "Lookahead distance" $`L_d`$ is selected
- Curvature is computed as:

<p align="center">
$k={2sin(α)\over L_d}$
</p>

$`α`$ is the heading error and the control commands are:
<p align="center">
$v=v_ref_ , ω=v*k$
</p>

## Dependencies

## Execution

## Results

<img width="918" height="641" alt="image" src="https://github.com/user-attachments/assets/ffaa2063-1a4b-4fd1-bd23-33abd7af1739" />

<img width="2313" height="1288" alt="image" src="https://github.com/user-attachments/assets/ac39b70d-9f7f-41b2-aaa3-904364052561" />

<img width="2292" height="1288" alt="image" src="https://github.com/user-attachments/assets/29041fce-0289-4d8b-9930-c56e6d273455" />


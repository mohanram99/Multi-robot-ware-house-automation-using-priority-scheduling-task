# Multi-robot-ware-house-automation-using-priority-scheduling-task
Multi-robot warehouse automation with priority scheduling, PID navigation, drone-based shelf inspection and zone-reservation collision management — ROS2 Jazzy + Gazebo Harmonic
# Warehouse Multi-Robot Automation System

A simulation framework for intelligent warehouse logistics built with ROS2 Jazzy 
and Gazebo Harmonic. The system deploys one autonomous mobile robot (AMR) and one 
quadrotor drone in a structured warehouse environment to demonstrate coordinated 
multi-agent task execution with priority-based scheduling.

## What this system does

The drone performs an autonomous aerial inspection loop over shelf rows, detecting 
ArUco fiducial markers with its downward-facing camera and publishing shelf 
occupancy status in real time. The centralized scheduler receives these detections 
and dynamically builds a task queue scored by three criteria: task priority 
(1–3), AMR distance to the target shelf, and elapsed waiting time (aging). 
The highest-scoring task is dispatched to the AMR, which navigates to the shelf 
using grid-based A* global planning and PID-based local trajectory tracking, 
with a LiDAR safety stop active throughout execution. A zone-reservation 
mechanism prevents simultaneous aerial and ground access to the same shelf zone. 
Task outcomes are logged to CSV for quantitative evaluation.

## Key technical contributions

- Multi-criteria task scorer combining priority, distance, and aging — 
  prevents starvation and improves fleet throughput over naive priority queues
- Drone-driven dynamic re-prioritisation — vision detections inject tasks 
  at runtime, immediately surfacing in the scored dispatch cycle
- Two-layer safety architecture — zone reservation for agent conflict 
  prevention, LiDAR reactive stop for environment collision avoidance
- Decoupled UI for manual task injection and live queue monitoring
- Quantitative evaluation: task completion time vs priority level, 
  queue drain time, prescan vs no-prescan throughput comparison

## Stack

ROS2 Jazzy Jalisco · Gazebo Harmonic · Python 3 (rclpy) · OpenCV 4 · 
Nav2 · SLAM Toolbox · RViz2


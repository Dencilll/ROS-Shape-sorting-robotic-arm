# ROS-Based Robotic Arm Shape Sorting System

## Project Overview

This project implements a 3-DOF robotic arm simulation using ROS and RViz. The arm sorts objects based on their shapes into accepted and rejected areas.

## Objectives

- Design a robotic arm using URDF.
- Visualize the robotic arm in RViz.
- Implement forward and inverse kinematics.
- Control the arm joints using ROS.
- Simulate pick-and-place operations.
- Sort objects into accepted and rejected areas.

## Technologies Used

- ROS Noetic
- Python
- URDF
- RViz
- ROS visualization markers

## Project Structure

```text
shape_sorting_robot/
├── launch/
│   └── Sorting_arm.launch
├── scripts/
│   └── sorting_controller.py
├── urdf/
│   └── sorting_arm.urdf
├── CMakeLists.txt
├── package.xml
└── README.md

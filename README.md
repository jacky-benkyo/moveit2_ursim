# ROS 2 MoveIt 2: Industrial Robot Control Architecture

This repository demonstrates a robust, hardware-abstracted motion planning architecture using **ROS 2 (Humble)** and **MoveIt 2**. It showcases how the exact same C++ client application can control a fully simulated industrial robotic arm (via Official URSim Docker) and seamlessly migrate to a different robot brand (Yaskawa Motoman) using fake hardware.

## 🏗️ System Architecture
The system adopts a 3-Terminal decoupled architecture:
1. **Body (Hardware/Simulator):** The physical robot or Dockerized URSim.
2. **Nervous System (Driver):** `ros2_control` hardware interfaces (`ur_robot_driver` / `motoman_robot_driver`).
3. **Brain (MoveIt 2 & Client):** Path planning, collision checking, and our custom C++ execution node.

## 🚀 Setup & Usage (Universal Robots - UR5e)

### Prerequisites
* ROS 2 Humble & MoveIt 2
* Universal Robots ROS 2 Driver
* Docker (User must be in the `docker` group)

### 1. Start the URSim Simulator (Terminal 1)
Boot up the official Universal Robots Docker image:
```bash
ros2 run ur_robot_driver start_ursim.sh -m ur5e

### 2. Launch the ROS 2 Driver (Terminal 2)

Connect ros2_control to the Dockerized simulator:
Bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.5

###  3. Launch MoveIt 2 and Execute Trajectory (Terminal 3 & 4)

Start the MoveIt 2 planning context and RViz:
Bash

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
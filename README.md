# ROS 2 MoveIt 2 Universal Robots (UR5e) Simulation

This repository provides a complete ROS 2 Humble environment for controlling a Universal Robots UR5e in simulation (URSim) using MoveIt 2. It includes advanced demonstrations of collision avoidance.robot brand (Yaskawa Motoman) using fake hardware.

## 🏗️ System Architecture
The system adopts a 3-Terminal decoupled architecture:
1. **Body (Hardware/Simulator):** The physical robot or Dockerized URSim.
2. **Nervous System (Driver):** `ros2_control` hardware interfaces (`ur_robot_driver`).
3. **Brain (MoveIt 2 & Client):** Path planning, collision checking, and our custom C++ execution node.

---
##  Quick Start for Windows Users (Recommended)

The easiest way to run this project is using VS Code Dev Containers. This method handles all dependencies (ROS 2, MoveIt 2, Drivers) automatically inside a Docker container.
Prerequisites

    Docker Desktop: Download here.

    VS Code: Download here.

    Dev Containers Extension: Search for "Dev Containers" in the VS Code extensions marketplace and install it.

Setup Instructions

### 1. open a terminal / powershell terminal and clone the Repository:
```bash

    git clone https://github.com/your-username/your-repo-name.git
```

### 2. Open in VS Code: 
Open the folder in VS Code.

### 3. Reopen in Container: 
A notification will appear in the bottom right corner. Click "Reopen in Container".

        Note: The first build may take several minutes.

### 4. Terminal Ready: 
Once the build is complete, your VS Code terminal is already inside a Linux environment with ROS 2 pre-installed.    
---

## ⚙️ Installation & Workspace Setup in Ubuntu Environment

### 1. Clone Required Repositories
Create a ROS 2 workspace and clone this application along with the official Universal Robots driver.
```bash
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src

# Clone the Universal Robots ROS 2 Driver
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

# Clone this repository
git clone https://github.com/jacky-benkyo/moveit2_ursim.git my_moveit_app
```

### 2. Install Dependencies

Use rosdep to install all required low-level hardware interfaces and controllers:
```bash

cd ~/ws_moveit2
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

Build the targeted packages (including previously ignored controllers and drivers):
```bash

colcon build --symlink-install --packages-up-to ur_robot_driver ur_calibration ur_controllers my_moveit_app
```

###  4. Docker Group Permission Adjustment (Crucial)

To allow the ROS 2 script to spawn the URSim Docker container without sudo, add your user to the Docker group:
```bash
sudo usermod -aG docker $USER
newgrp docker
```
---

## 🚀 Setup & Usage (Universal Robots - UR5e)
Regardless of whether you use the Dev Container or a native Linux install, follow these steps to execute the simulation.
###  Step 1: Start the URSim Simulator (Terminal 1)

Boot up the official Universal Robots Docker image:
```bash

source ~/ws_moveit2/install/setup.bash
ros2 run ur_robot_driver start_ursim.sh -m ur5e
```
    PolyScope Configuration: Open a browser at http://192.168.56.101:6080/vnc.html. Turn the power ON, release brakes (START), and run the External Control URCap program by pressing Play (▶️).


### Step 2: Launch the ROS 2 Driver (Terminal 2)

Run Dockerized UR simulator with robot model UR5e:
```bash
ros2 run ur_robot_driver start_ursim.sh -m ur5e
```
Connect ros2_control to the Dockerized simulator:
```bash
source ~/ws_moveit2/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=false
```

###  Step 3: Launch MoveIt 2 (Terminal 3)

Start the MoveIt 2 planning context and RViz (Terminal 3):
```bash

source ~/ws_moveit2/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

###  Step 4: Execute Trajectory (Terminal 4)

Execute the Cartesian path (Terminal 4):
```bash

source ~/ws_moveit2/install/setup.bash
ros2 launch my_moveit_app my_test_run.launch.py
```

---

## 🧱Advanced Feature: Collision Avoidance 

This repository also includes an advanced demonstration of MoveIt 2's OMPL planner dynamically avoiding obstacles. The C++ node injects a virtual collision object (a table) into the `PlanningSceneInterface` and calculates a collision-free trajectory.

### ⚠️ Important Prerequisite (URSim PolyScope)
Before executing the trajectory, you **must** ensure the robot is actively listening for commands from the ROS 2 driver:
1. Open the URSim PolyScope VNC in your browser (`http://192.168.56.101:6080/vnc.html`).
2. Navigate to **Program > URCaps > External Control**.
3. Press the **Play (▶️)** button at the bottom of the screen. (The driver terminal should log `Robot ready to receive control commands`).

### Execute the Collision Node (Terminal 4)
Run the custom C++ node to spawn the virtual table and execute the evasive maneuver:

```bash
source ~/ws_moveit2/install/setup.bash
ros2 launch my_moveit_app collision_test_run.launch.py
```
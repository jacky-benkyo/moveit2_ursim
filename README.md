# ROS 2 MoveIt 2: Industrial Robot Control Architecture

This repository demonstrates a robust, hardware-abstracted motion planning architecture using **ROS 2 (Humble)** and **MoveIt 2**. It showcases how the exact same C++ client application can control a fully simulated industrial robotic arm (via Official URSim Docker) and seamlessly migrate to a different robot brand (Yaskawa Motoman) using fake hardware.

## 🏗️ System Architecture
The system adopts a 3-Terminal decoupled architecture:
1. **Body (Hardware/Simulator):** The physical robot or Dockerized URSim.
2. **Nervous System (Driver):** `ros2_control` hardware interfaces (`ur_robot_driver` / `motoman_robot_driver`).
3. **Brain (MoveIt 2 & Client):** Path planning, collision checking, and our custom C++ execution node.

---

## ⚙️ Installation & Workspace Setup

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


## 🚀 Setup & Usage (Universal Robots - UR5e)
###  Step 1: Start the URSim Simulator (Terminal 1)

Boot up the official Universal Robots Docker image:
```bash

source ~/ws_moveit2/install/setup.bash
ros2 run ur_robot_driver start_ursim.sh -m ur5e

    PolyScope Configuration: Open a browser at http://192.168.56.101:6080/vnc.html. Turn the power ON, release brakes (START), and run the External Control URCap program by pressing Play (▶️).
```

### Step 2: Launch the ROS 2 Driver (Terminal 2)

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
# roboracer_dissertation

# 🏎️ Roboracer: Dynamic Wall-Following Robot using ROS 2 & Gazebo Sim

This project implements a **dynamic wall-following robot** using a **gap-following algorithm** with **ROS 2 Humble** and **Gazebo Sim 6.17.0**. It is specifically designed to handle **dynamic tracks** like those used in university racing competitions — **no hardcoded values**, everything adapts to the environment.

---

## 📁 Project Structure

```
roboracer_description/
├── launch/
│ ├── gazebo.launch.py # Launches robot and world in Gazebo
│ ├── wall_follow.launch.py # Launches the wall-following node
```

```
roboracer_py/
├── GapFollowingAlgorithm.py # Python node that implements dynamic gap following
roboracer_description/
├── launch/
│ ├── gazebo.launch.py # Launches robot and custom world in Gazebo 
│ ├── wall_follow.launch.py # Launches the wall-following node
├── worlds/
│ └── levine_loop_world.world # Custom dynamic world for navigation
├── urdf/
│ └── f1tenth_chasis.urdf # Ackermann-style robot model
```

```
roboracer_py/
├── gap_followin.py # Python node for gap-following logic Inside in 
```

## 🚀 How to Run

### ✅ Prerequisites

- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) installed
- [Gazebo Sim 6.17.0](https://gazebosim.org/docs/all-releases) installed
- Colcon workspace created and sourced

---

### 🔨 Build Instructions

```
cd ~/ros2_ws
colcon build --packages-select roboracer_description roboracer_py
source install/setup.bash
```

🧭 Launch the Simulation (Gazebo + Robot)

```
ros2 launch roboracer_description gazebo.launch.py
```

This launches:

The robot in a dynamic world.

All required sensor plugins (LiDAR, camera, etc.).

🧠 Launch Wall-Following Algorithm
In a new terminal:

```
ros2 launch roboracer_description wall_follow.launch.py
```

This launches:

The gapFollowing.py node from roboracer_py

Real-time dynamic path following using LiDAR data

⚙️ Features
✅ Fully dynamic path planning (no hardcoded turns or distances)

✅ Works in any simulated environment

✅ ROS 2 & Gazebo Sim integration

✅ Clean modular structure for packages and nodes

🌍 Topics Used

/scan – LiDAR input for gap analysis

/drive – Output command (AckermannDriveStamped)

/ego_racecar/odom – Odometry (optional for speed estimation)

🖼️ Example Visualisation
Example of wall-following behaviour in RViz + Gazebo:

If you wana change then inside in roboracer_description their is launch file wall_follow.launch.py just chnage it with your node thats it 

ro

<!-- Replace with your own GIF/image -->

📦 Dependencies
rclpy

sensor_msgs

ackermann_msgs

numpy, math, etc. (Python libraries for logic)

🧑‍💻 Author
Ahmad Yar
University of Surrey – Dynamic Track Navigation Project

ROS 2 | Gazebo Sim | Python Robotics

📌 Notes
Check that your URDF or SDF file is correctly publishing the required TFs.

Ensure the LiDAR is active and publishing to /scan before launching wall_follow.launch.py.

The node will compute the best path dynamically using gap-following logic.


## 🐳 Docker Launch

The project is divided into two main modules:
1.  **F1TENTH Gym ROS:** A simulator for testing autonomous racing algorithms for a 1/10th scale F1 car.
2.  **Roboracer Project:** A Gazebo-based simulator for testing computer vision packages like YOLO and visual odometry.

The entire simulation environment is containerized using Docker and can be launched with a single script.

### Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed and running.
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for GPU acceleration (required for Gazebo).

### 🚀 How to Run

The project is launched using the `run.sh` script located in the root directory.

```bash
./run.sh
```

This script performs the following steps:
1.  **Configures X11 forwarding** to allow GUI applications (like Gazebo and RViz) to run from within the Docker containers.
2.  **Builds and starts the `f1tenth_gym_ros` container.** This module has its own `docker-compose.yml` which mounts the `f1tenth_gym_ros` directory into the container, allowing you to modify the code on the fly.
3.  **Builds and starts the `roboracer_project` container.** This module also has its own `docker-compose.yml` (and a variant for NVIDIA GPUs) that mounts the project's workspace into the container.

After running the script, two containers will be active, each with its own simulation environment.

### 📦 Running Commands in the F1TENTH Container

To run commands inside the `f1tenth_gym_ros` container, you first need to open a shell in it:

```bash
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
```

Once inside the container, you can launch the simulation and your algorithms.

1.  **Source the ROS 2 Workspace:**
    Before running any ROS 2 commands, you need to source the workspace:
    ```bash
    colcon build && source install/setup.bash
    ```

2.  **Launch the Simulator:**
    Use `ros2 launch` to start the F1TENTH simulator.

    ```bash
    ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    ```

3.  **Run PP Algorithm:**
    In a **new terminal**, open another shell in the container and run your algorithm.

    ```bash
    docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
    source install/setup.bash
    ros2 launch pure_pursuit sim_pure_pursuit_launch.py 
    ```




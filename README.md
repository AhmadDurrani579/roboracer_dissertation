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

The entire simulation environment is containerized using Docker and can be launched with dedicated scripts for each module.

### Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed and running.
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for GPU acceleration (required for Gazebo).

### 🚀 How to Run

The project is launched using dedicated scripts for each module, located in the root directory. Use the `--build` flag to force a rebuild of the Docker images.

-   **To launch the F1TENTH Gym ROS simulator:**
    ```bash
    # To run with existing images
    ./run_f1tenth.sh

    # To rebuild images and then run
    ./run_f1tenth.sh --build
    ```

-   **To launch the Roboracer Gazebo simulator:**
    ```bash
    # To run with existing images
    ./run_roboracer.sh

    # To rebuild images and then run
    ./run_roboracer.sh --build
    ```

These scripts perform the following steps:
1.  **Configures X11 forwarding** to allow GUI applications (like Gazebo and RViz) to run from within the Docker containers.
2.  **Builds and starts the corresponding container** (`f1tenth_gym_ros` or `roboracer_project`). Each module has its own `docker-compose.yml` which mounts the necessary directories into the container.

After running the desired script, a container will be active with its simulation environment.

### 📦 Running Commands in the F1TENTH Container

When you run `./run_f1tenth.sh`, you will be automatically connected to the F1TENTH container with an interactive shell. Once inside the container, you can launch the simulation and your algorithms.

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
    In a **new terminal**, start another F1TENTH container and run your algorithm:

    ```bash
    ./run_f1tenth.sh
    # Once inside the container:
    source install/setup.bash
    ros2 launch pure_pursuit sim_pure_pursuit_launch.py 
    ```

### 📦 Running Commands in the Roboracer Container

When you run `./run_roboracer.sh`, you will be automatically connected to the Roboracer container with an interactive shell. This container is used for Gazebo simulations and testing computer vision packages like YOLO and visual odometry.

1.  **Launch the Gazebo Simulation:**
    First, launch the Gazebo simulation:
    ```bash
    colcon build && source install/setup.bash
    ros2 launch roboracer_description gazebo.launch.py
    ```

2.  **Running Individual Computer Vision Nodes:**
    The following nodes can be run in separate terminals after launching the Gazebo simulation. For each command, start a new roboracer container in a new terminal:

    - **Visual Odometry:**
      ```bash
      ./run_roboracer.sh
      # Once inside the container:
      source install/setup.bash
      ros2 run roboracer_visual_odom visual_odom
      ```

    - **RGB-D Pointcloud Publisher:**
      ```bash
      ./run_roboracer.sh
      # Once inside the container:
      source install/setup.bash
      ros2 run roboracer_visual_odom rgbd_pointcloud_publisher
      ```

    - **YOLOv8 Detection:**
      ```bash
      ./run_roboracer.sh
      # Once inside the container:
      source install/setup.bash
      ros2 run roboracer_yolov8_detector yolov8_detection_node
      ```

    - **3D Detection:**
      ```bash
      ./run_roboracer.sh
      # Once inside the container:
      source install/setup.bash
      ros2 run roboracer_yolov8_detector detection_3d_node
      ```




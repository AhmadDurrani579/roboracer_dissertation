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





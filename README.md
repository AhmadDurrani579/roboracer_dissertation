# 🏎️ Roboracer: Autonomous Navigation using ROS 2, LiDAR & Real Hardware

This project implements a **fully autonomous racing robot** using **ROS 2 Humble**, **LiDAR-based perception**, and **real-world hardware deployment**.

The system is designed for **dynamic environments (no hardcoding)** and integrates:
- Simulation (Gazebo)
- Real-world deployment (VESC + Jetson)
- Perception (LiDAR + Vision)
- Control (Ackermann steering)

---

# 🎥 Demo Videos

## ⚡ Real Hardware (VESC + ROS 2)
👉 https://youtube.com/shorts/0dPR5gpo4pw  

- VESC motor control integrated with ROS 2  
- Ackermann steering validated  
- Real-time robot movement  

---

## 🧠 3D Reconstruction (Perception Pipeline)
👉 https://youtu.be/0DcLDs3WChk  

- RGB-D / LiDAR-based reconstruction  
- Real-time 3D environment understanding  

---

## 🔍 Feature Extraction (Computer Vision)
👉 https://youtu.be/ZjUJ5WAdLZQ  

- Feature detection pipeline  
- Used for visual odometry / perception  

---

## 🚗 Gap Following Algorithm (Autonomous Navigation)
👉 https://youtu.be/n4tCURybJj0  

- Dynamic path planning using LiDAR  
- No hardcoded trajectories  
- Adapts to any environment  

---

## 🏗️ Full Hardware Deployment
👉 https://youtube.com/shorts/ynngSGGuumY  

- Jetson + LiDAR + VESC integration  
- Real-world autonomous navigation  

---

# 🖼️ Project Visuals

## 🧪 Simulation (Gazebo + RViz)
![Simulation](media/gazebo.png)

## 🔧 Real Robot Setup
![Robot](media/robot.jpg)

---

# 🛠️ Hardware Setup

## 🔌 Components Used

- 🧠 Jetson Nano (ROS 2 computation)
- ⚡ VESC (motor controller)
- 📡 LiDAR sensor (environment perception)
- 🎮 Joystick (manual control & testing)
- 🚗 Ackermann steering vehicle (F1TENTH platform)

---

# ⚙️ Key Features

- ✅ Fully dynamic navigation (no hardcoded values)
- ✅ Real-time LiDAR processing
- ✅ AckermannDriveStamped control
- ✅ Simulation + real-world deployment
- ✅ Sensor fusion ready architecture
- ✅ Docker-based reproducible environment

---

# 🧠 System Architecture

Pipeline:

LiDAR (/scan)  
→ Gap Following Algorithm  
→ Decision Making  
→ AckermannDriveStamped (/drive)  
→ VESC Control  
→ Robot Movement  

---

# 📁 Project Structure

roboracer_description/
├── launch/
│ ├── gazebo.launch.py
│ ├── wall_follow.launch.py
├── worlds/
│ └── levine_loop_world.world
├── urdf/
│ └── f1tenth_chasis.urdf

roboracer_py/
├── gap_followin.py
├── GapFollowingAlgorithm.py

---

# 🚀 How to Run

## 🔨 Build

cd ~/ros2_ws  
colcon build --packages-select roboracer_description roboracer_py  
source install/setup.bash  

---

## 🧭 Launch Simulation

ros2 launch roboracer_description gazebo.launch.py  

---

## 🧠 Run Navigation

ros2 launch roboracer_description wall_follow.launch.py  

---

# 🌍 ROS Topics

- /scan → LiDAR input  
- /drive → Ackermann control  
- /ego_racecar/odom → Odometry  

---

# 🐳 Docker Support

./run_roboracer.sh  

Supports:
- Gazebo simulation  
- YOLO detection  
- Visual odometry  

---

# 📦 Dependencies

- rclpy  
- sensor_msgs  
- ackermann_msgs  
- numpy  
- math  

---

# 🧑‍💻 Author

Ahmad Yar  
University of Surrey  
ROS 2 | Robotics | Autonomous Systems  

---

# 📌 Notes

- Ensure LiDAR is publishing to /scan  
- Verify TF tree is correct  
- Works in both simulation and real-world setups  

---

# 🚀 Why This Project Matters

- Real-world robotics system (not just simulation)
- End-to-end pipeline (perception → planning → control)
- Designed for dynamic environments (warehouse-ready concept)
- Demonstrates strong ROS 2 + hardware integration skills

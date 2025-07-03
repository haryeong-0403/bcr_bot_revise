### BCR Bot Simulation on ROS2 Humble + Gazebo Fortress(Docker)

This project provides a fully customized development and simulation environment for the BCR Bot using ROS2 Humble and Gazebo Fortress, containerized in Docker for portability and reproducibility.
BCR Bot has been revised with multiple cameras (RGB-D + Stereo), topic remapping, and full Ignition-Gazebo integration.
Instead of using the default small_warehouse world provided by the bcr_bot package, we modified the original tugbog_in_warehouse world by removing the Tugbot model, allowing us to use the same warehouse environment independently.

## 🚀 Features

✅ ROS2 Humble Desktop Full: All core ROS2 packages

✅ Gazebo Fortress: Physics-based simulation

✅ Custom BCR Bot:

    Front / Back / Left / Right Kinect RGB-D Cameras

    Front / Back / Left / Right Stereo Cameras

    Integrated 2D LiDAR

✅ Sensor Topic Remapping: Prefixed topics for all cameras (front_, back_, etc.)

✅ Ignition Gazebo + ROS2 bridge

✅ Launchable world environments (e.g., warehouse, empty)

✅ GUI + RViz2 support

## 📋 Prerequisites

# Docker Installation

```bash
sudo apt update
sudo apt install -y docker.io docker-compose
sudo usermod -aG docker $USER
sudo systemctl start docker
sudo systemctl enable docker
```

# GUI Support(for Gazebo)

```bash
xhost +local:docker
```

## 🏗️ Build & Run 

1. Clone Project

```bash
git clone https://github.com/<your-username>/ros_humble_bcr_bot.git
cd ros_humble_bcr_bot
```

2. Build Docker Image

```bash
docker compose build
```

3. Run Docker Container

```bash
docker compose up -d ros2-dev
```

4. Access Container

```bash
docker compose exec ros2-dev bash
```

## 🎯 Using the BCR Bot

# Inside Container

```bash
colcon build --symlink-install # Free colcon build
source install/setup.bash
```

# Launch BCR Bot with Sensors

```bash
ros2 launch bcr_bot ign.launch.py # In bcr_bot.xacro, change the arg 
```

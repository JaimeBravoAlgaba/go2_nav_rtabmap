# go2_nav_rtabmap

ROS 2 bringup package for **Go2 navigation using RTAB-Map and Nav2**, including tools for odometry alignment and camera TF configuration.

This repository provides launch files and utilities to run **RGB-D SLAM with RTAB-Map** and **navigation with Nav2** on the Unitree Go2 robot using an Intel RealSense camera.

---

# System Overview

This stack combines:

* **Unitree Go2 robot drivers** (robot base + odometry)
* **Intel RealSense camera** (RGB-D perception)
* **RTAB-Map** (visual SLAM and mapping)
* **Nav2** (global planning, local planning and navigation)
* **Custom odometry alignment tool**

Typical TF tree:

```
map
 └── rtabmap/odom
      └── odom
           └── base_link
                └── camera_link
```

Where:

* `odom → base_link` is provided by the robot
* `base_link → camera_link` is a static transform
* `rtabmap/odom` is produced by RTAB-Map
* `align_odoms` aligns `rtabmap/odom` with the robot odometry using `/initialpose`

---

# Requirements

Tested with:

* **ROS 2 Humble / Iron / Jazzy**
* **Nav2**
* **RTAB-Map**
* **Intel RealSense ROS**

Required packages:

```
nav2_bringup
rtabmap_launch
rviz2
realsense2_camera
```

---

# Installation

Clone the repository inside your ROS 2 workspace:

```bash
cd ~/go2_ws/src
git clone https://github.com/YOUR_USERNAME/go2_nav_rtabmap.git
```

Build the workspace:

```bash
cd ~/go2_ws
colcon build --packages-select go2_nav_rtabmap --symlink-install
source install/setup.bash
```

---

# Running on the Go2 Robot

Before launching the navigation stack from this repository, the **robot drivers** and the **RGB-D camera** must be running.

---

## 1. Start the Go2 ROS 2 drivers

Clone and build the official Unitree Go2 ROS 2 repository:

https://github.com/Unitree-Go2-Robot/go2_robot

Then start the robot bringup:

```bash
source go2_ws/install/setup.bash
ros2 launch go2_bringup go2.launch.py
```

This launch starts the robot drivers and publishes:

* `odom → base_link` TF from the robot odometry
* robot state topics
* velocity command interfaces
* joint states

This odometry is used by Nav2 and aligned with RTAB-Map.

---

## 2. Start the Intel RealSense camera

Launch the RealSense camera node with aligned depth and point cloud enabled:

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera \
  -r __node:=camera \
  -p rgb_camera.profile:=424x240x30 \
  -p depth_module.profile:=424x240x30 \
  -p pointcloud.enable:=true \
  -p align_depth.enable:=true \
  -p color_qos:=SENSOR_DATA \
  -p depth_qos:=SENSOR_DATA \
  -p color_info_qos:=SENSOR_DATA \
  -p depth_info_qos:=SENSOR_DATA \
  -p pointcloud.pointcloud_qos:=SENSOR_DATA
```

This provides the RGB-D streams used by RTAB-Map:

```
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
/camera/depth/color/points
```

---

# Running RTAB-Map and Navigation

The stack is launched in two stages to avoid startup race conditions between RTAB-Map and Nav2.

---

## 1. Start RTAB-Map SLAM

```bash
ros2 launch go2_nav_rtabmap rtabmap_bringup.launch.py
```

This launch starts:

* `rgbd_odometry`
* `rtabmap`
* `rviz2`
* `align_odoms` (custom odometry alignment node)

RTAB-Map builds the map and estimates odometry.

---

## 2. Start Nav2 Navigation

Open a new terminal and run:

```bash
ros2 launch go2_nav_rtabmap nav2_bringup.launch.py
```

This starts:

* planner server
* controller server
* behavior server
* BT navigator
* costmaps

Nav2 will then be able to plan and execute navigation goals.

---

# Setting the Initial Pose

To align the robot odometry with the RTAB-Map frame:

1. In **RViz**, set the **Fixed Frame** to:

```
rtabmap/odom
```

2. Use **2D Pose Estimate**.

The `align_odoms` node will update the transform between:

```
rtabmap/odom → odom
```

This aligns the robot localization with the SLAM map.

---

# Camera Transform

The camera position relative to the robot base must be defined with a static transform.

Example:

```bash
ros2 run tf2_ros static_transform_publisher \
0.35 0 0.1 0 0.57 0 base_link camera_link
```

This defines the physical offset between the robot base and the RealSense camera.

In practice this transform should be placed in the launch file or in the robot URDF.

---

# Repository Structure

```
go2_nav_rtabmap/
│
├── launch/
│   ├── rtabmap_bringup.launch.py
│   ├── nav2_bringup.launch.py
│
├── config/
│   └── nav2_rtabmap.yaml
│
├── rviz/
│   └── go2_custom.rviz
│
├── scripts/
│   └── align_odoms.py
│
└── README.md
```

---

# Features

* RTAB-Map RGB-D SLAM integration
* Nav2 navigation stack configuration
* Automatic odometry alignment using `/initialpose`
* RealSense camera integration
* Go2 robot compatibility

---

# License

MIT License

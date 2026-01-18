# 🛰️ AMR 2026: Tactical LiDAR-RTK SLAM

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-orange?style=for-the-badge)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python)](https://www.python.org/)

A professional, high-performance **3D SLAM Framework** for Autonomous Mobile Robots (AMR). This system integrates 3D LiDAR point clouds with high-precision RTK GPS data, featuring an **intelligent auto-calibration engine** and a **cyber-tactical mission control** for real-time satellite tracking.

---

##  Key Features

*   **🔬 Auto-Calibrated Sensor Fusion**: Automatically estimates the LiDAR-to-Vehicle mounting offset by analyzing motion vectors, eliminating the need for manual extrinsic calibration.
*   **🌳 Semantic 3D Mapping**: Real-time geometric classification engine that automatically segments and colors trees (green), poles (yellow), structures (blue), and terrain (grey).
*   **📡 Tactical Mission Control (HUD)**: A dedicated Python-based command center with real-time telemetry (Speed, Heading, Coordinates) and dynamic ESRI satellite imagery.
*   **🔄 Dual-Phase SLAM**:
    *   **Phase 1 (Mapping)**: High-density 3D map construction with trajectory recording.
    *   **Phase 2 (Localization)**: Static map navigation with automated loop closure detection.
*   **🛡️ Robust Preprocessing**: Advanced filtering stack using Hampel and DBSCAN to eliminate LiDAR noise and "ghost rings" from the ground.

---

## 🛠️ Tech Stack

| Component | Technology |
| :--- | :--- |
| **Framework** | ROS 2 Humble |
| **3D Processing** | Open3D, NumPy, Scipy |
| **UI / Visualization** | RViz 2, Matplotlib (Custom Tactic UI), PIL |
| **Geo-Spatial** | ESRI World Imagery API, WGS84 Projection |
| **Language** | Python 3.10, C++ (Backend) |

---

## 🚀 Quick Start

### 1. Installation
Ensure you have ROS 2 Humble installed. Then install the Python dependencies:
```bash
pip install numpy open3d scipy matplotlib requests Pillow
```

### 2. Build the Project
```bash
cd "/home/alfonso/Documents/SEMESTRE TEC/amr_2026_research_m&l"
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the Mission
This will automatically open the **3D HD Mapping (RViz)** and the **Tactical Mission Control (Python)**:
```bash
ros2 launch lidar_rtk_slam rtk_direct_slam.launch.py
```

---

## 📊 System Architecture

1.  **RTK Preprocessor**: Converts LLA coordinates to local ENU tangent plane with outlier removal.
2.  **LiDAR Preprocessor**: Handles voxel downsampling and ROI ghost removal.
3.  **Map Builder**: Computes auto-calibration, classifies geometry, and saves the final `.ply` map.
4.  **Mission Control**: Tactical HUD that projects ENU trajectories back to satellite-aligned LLA coordinates.

---

## 📁 Repository Structure
```text
.
├── src/lidar_rtk_slam/
│   ├── lidar_rtk_slam/         # Core Python Nodes
│   │   ├── rtk_direct_map_builder.py   # SLAM Engine
│   │   └── satellite_navigation_node.py # Tactical HUD
│   ├── launch/                 # Launch Orchestrators
│   └── rviz/                   # Pro Visual Configs
├── data/                       # ROS 2 Bags & Datasets
└── maps/                       # Generated High-Precision Maps
```

---

## 👤 Author
**Project AMR 2026 - Research**
*Special thanks to the Google DeepMind Antigravity Team for the collaborative engineering.*

---

*“Precision in mapping, clarity in mission control.”* 🏎️🛰️🦾

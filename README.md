# AMR 2026: Advanced LiDAR-RTK SLAM Framework

A professional-grade SLAM (Simultaneous Localization and Mapping) framework developed for high-precision autonomous navigation in outdoor environments. This system leverages 3D LiDAR data and RTK-GPS fusion, specifically optimized for the ROS 2 Humble distribution.

---

## Technical Overview

The framework implements a dual-phase SLAM architecture designed to handle large-scale environments with minimal manual intervention. The core innovation lies in its ability to synchronize disparate sensor data through automated extrinsic calibration and semantic geometric classification.

### Core Algorithms & Implementation

#### 1. Intelligent Auto-Calibration Engine
Manual extrinsic calibration between a LiDAR and an RTK-GPS is error-prone and time-consuming. This framework implements a kinematic-based auto-calibration routine:
*   **Motion Vector Correlation**: The system listens to the first 15 keyframes of motion. It computes the vehicle's heading $(\theta_{RTK})$ using the differential ENU position.
*   **LiDAR Alignment**: Simultaneously, the SLAM engine analyzes the point cloud distribution to find the principal axis of the environment. 
*   **Optimization**: It computes the angular offset $(\Delta\theta)$ required to align the LiDAR coordinate system with the Earth-referenced RTK frame, ensuring that "Forward" in the point cloud matches "Forward" in the geodetic frame.

#### 2. Semantic Geometric Classification
The system performs real-time point cloud parsing to distinguish between static infrastructure and organic obstacles:
*   **Segmentation**: Points are clustered using **DBSCAN** ($eps=0.40, min\_points=12$) after a floor-removal pass.
*   **Geometric Heuristics**: 
    *   **Vertical Man-made**: Objects with high height-to-width ratios ($>2.2$) and consistent narrow widths ($<0.6m$) are classified as **Poles/Lamps** (Yellow).
    *   **Organic Clusters**: Dense, irregular clusters are classified as **Trees** (Green).
    *   **Infrastructural Planes**: Large planar segments ($width > 4.0m$) are marked as **Buildings/Structures** (Blue).

#### 3. High-Fidelity Geodetic Processing
To ensure sub-decimeter accuracy in navigation, the following geodetic corrections are applied:
*   **LLA to ENU Projection**: Uses the WGS84 ellipsoid model to project global Latitude/Longitude to a local East-North-Up Cartesian plane.
*   **Hampel Outlier Removal**: A sliding-window filter ($N=9$) detects and suppresses GPS multi-path errors or signal jumps.
*   **Lever-Arm Correction**: Compensates for the physical 3D displacement between the GPS antenna and the vehicle's center of mass, preventing trajectory warping during turns.

---

## System Architecture & Data Flow

### 1. Pre-processing Layer
*   **Lidar Node**: Handles cylindrical ROI filtering to remove the vehicle's ego-chassis reflections (radius < 4m) and voxelizes the cloud to 0.15m.
*   **RTK Node**: Synchronizes NMEA fix data and generates a stabilized TF between the `map` and `rtk_antenna` frames.

### 2. SLAM Engine (Dual-Phase)
*   **Phase I: HD Mapping (V1)**: Points are transformed into the global frame using the calibrated offset and integrated into a spatial voxel grid. 
*   **Loop Closure Detection**: The system monitors the **Euclidean distance** back to the origin. Once a minimum distance is met (e.g., 80m) and the robot returns to the start radius (15m), the map is "frozen."
*   **Phase II: Static localization (V2)**: The robot localizes against the previously built HD map, providing a stable pose for path planning without accumulating map drift.

### 3. Tactical Mission Control (HUD)
A standalone telemetry interface built with Matplotlib and Python, providing a "Satellite View" overlay:
*   **Dynamic Tile Fetching**: Integrates ESRI World Imagery at Level 18 zoom.
*   **Projected Paths**: Inverse-projects the SLAM ENU trajectory back to global Latitude/Longitude to align exactly with satellite maps.
*   **Telemetry HUD**: Real-time display of Speed (km/h), Heading, and operational phase status.

---

## Visualization Guide

### RViz 2 (3D HD Map)
Focuses on the high-fidelity 3D reconstruction. You will see:
*   **Global Map**: Semantic-colored point cloud (Buildings, Trees, Poles).
*   **Robot Model**: Current 3D pose and orientation.
*   **Phase Indicator**: Visual feedback of mapping progression.

### Mission Control Window (2D Tactical)
Focuses on geospatial context:
*   **Red Cursor**: Real-time vehicle position and heading.
*   **Orange Trail**: Mapping phase trajectory (V1).
*   **Cyan Trail**: Localization phase trajectory (V2).

---

## Technical Specifications & Requirements

| Metric | Specification |
| :--- | :--- |
| **Middle-ware** | ROS 2 Humble Hawksbill |
| **Input Topics** | `/rtk/fix` (sensor_msgs/NavSatFix), `/velodyne_points` (sensor_msgs/PointCloud2) |
| **Output Topics** | `/slam/map`, `/slam/path`, `/rtk/odom_enu` |
| **Dependencies** | Open3D (0.15+), NumPy, Scipy, Matplotlib, Pillow, Requests |
| **Accuracy** | Approx. 5-10cm (RTK-dependent) |

---

## Installation & Deployment

### 1. Environment Setup
```bash
pip install numpy open3d scipy matplotlib requests Pillow
```

### 2. Workspace Construction
```bash
# Navigate to your ROS 2 workspace
cd amr_2026_research_m&l
colcon build --symlink-install --packages-select lidar_rtk_slam
source install/setup.bash
```

### 3. Launch Orchestration
```bash
# This launch file starts all preprocessing, SLAM, and UI nodes simultaneously
ros2 launch lidar_rtk_slam rtk_direct_slam.launch.py
```

---

## Development & Maintenance
**Project**: AMR 2026 - Research Division  
**Lead Engineer**: Alfonso  
**Focus**: Precision Geospatial Localization & Persistent 3D Environment Reconstruction

---
*Generated by the AMR 2026 Research Team.*

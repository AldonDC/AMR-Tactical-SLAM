# AMR 2026: Advanced LiDAR-RTK SLAM Framework

A professional-grade SLAM (Simultaneous Localization and Mapping) framework developed for high-precision autonomous navigation in outdoor environments. This system leverages 3D LiDAR data and RTK-GPS fusion, specifically optimized for the ROS 2 Humble distribution.

---

## Technical Overview

The framework implements a dual-phase SLAM architecture designed to handle large-scale environments with minimal manual intervention. The core innovation lies in its ability to synchronize disparate sensor data through automated extrinsic calibration and semantic geometric classification.

### Core Algorithms & Implementation

#### 1. Intelligent Auto-Calibration Engine
The system eliminates the traditional "hand-eye" calibration requirement. By analyzing the temporal relationship between the **RTK-GPS velocity vectors** and the **LiDAR point cloud distribution**, the algorithm identifies the optimal mounting offset.
*   **Mechanism**: The engine correlates the vehicle's direction of travel (derived from motion) with the maximum depth of the LiDAR field-of-view to compute a precise rotational offset.

#### 2. Semantic Geometric Classification
Unlike standard SLAM which treats all points equally, this framework performs real-time classification:
*   **Feature Extraction**: Using a combination of **Voxel-Downsampling** and **DBSCAN (Density-Based Spatial Clustering)**, the system isolates high-density clusters.
*   **Classification Logic**: Specifically designed heuristics analyze height-to-width ratios to distinguish between permanent structures (buildings), vertical landmarks (poles/lamps), and organic features (trees).

#### 3. Robust Data Preprocessing Stack
To ensure map fidelity in "noisy" outdoor environments, the system applies:
*   **Hampel Filter**: Employed on the GPS stream to detect and suppress LLA coordinate outliers.
*   **Lever-Arm Correction**: Compensates for the physical distance between the GPS antenna and the vehicle's center of rotation.
*   **Cylindrical ROI Filtering**: Removes vehicle-chassis reflections and LiDAR "ghost rings" using dynamic distance thresholds.

---

## System Architecture

### Phase I: HD Mapping (V1)
In this phase, the system builds a global 3D sparse-map. It utilizes **Keyframe-based Insertion** to maintain performance, only adding cloud data when the vehicle has exceeded a set distance threshold (e.g., 3 meters), ensuring the map remains dense but efficient.

### Phase II: Tactical Localization (V2)
Upon detecting a loop closure, the system transitions to a localization-only mode. It projects the vehicle's state onto the established static map, providing high-frequency pose updates without further map corruption.

### Tactical Mission Control (HUD)
A dedicated telemetry interface developed in Python/Matplotlib. This Head-Up Display (HUD) provides:
*   **Visual Projection**: Real-time transformation of local ENU coordinates to global LLA for overlay on ESRI high-resolution satellite imagery.
*   **Live Telemetry**: Direct monitoring of velocity (km/h), heading (degrees), and geodetic coordinates.
*   **Phase Monitoring**: Dynamic visual badges indicating the current operational state of the SLAM engine.

---

## Technical Specifications

*   **Middleware**: ROS 2 Humble (LTS)
*   **Point Cloud Engine**: Open3D / sensor_msgs_py
*   **Geospatial Projection**: WGS84 Local Tangent Plane (ENU)
*   **UI Framework**: custom Python/Matplotlib HUD & RViz 2
*   **Data Structures**: PCD/PLY format with embedded semantic color coding

---

## Installation & Deployment

### Prerequisites
```bash
pip install numpy open3d scipy matplotlib requests Pillow
```

### Execution
1.  **Clone and Build**:
    ```bash
    cd amr_2026_research_m&l
    colcon build --symlink-install
    source install/setup.bash
    ```
2.  **Launch Primary Stack**:
    ```bash
    ros2 launch lidar_rtk_slam rtk_direct_slam.launch.py
    ```

---

## Development Info
**Project**: AMR 2026 - Research Division  
**Focus**: Precision Geospatial Localization & 3D Environment Reconstruction

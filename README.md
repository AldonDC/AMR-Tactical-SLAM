# AMR 2026: Framework Avanzado de SLAM LiDAR-RTK

Sistema profesional de **Navegación y Mapeo (SLAM)** de alta precisión para robots autónomos, optimizado para **ROS 2 Humble**. Integra nubes de puntos 3D con posicionamiento RTK-GPS centimétrico.

---

## 🛠️ Stack Tecnológico

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Open3D](https://img.shields.io/badge/Open3D-3D_Engine-orange?style=for-the-badge)

---

## 🧠 Bases Algorítmicas

### 1. Auto-Calibración Cinemática
Elimina la necesidad de medir manualmente la posición del sensor. El sistema correlaciona el vector de movimiento del RTK con la distribución de puntos del LiDAR:
*   **Cálculo**: Se analizan los primeros 15 keyframes para resolver el desfase angular $\Delta\psi$ entre el heading del GPS $(\theta_{RTK})$ y el eje principal del LiDAR $(\theta_{LiDAR})$:
    $$\Delta\psi = \arg\min \sum | \vec{v}_{RTK} - \mathbf{R}(\theta) \cdot \vec{d}_{LiDAR} |$$

### 2. Clasificación Semántica Geométrica
El SLAM identifica objetos sin necesidad de IA pesada, usando descriptores de forma:
*   **Postes/Infraestructura**: Relación altura/ancho $> 2.2$ y ancho $< 0.6m$.
*   **Vegetación**: Clusters irregulares detectados mediante **DBSCAN** ($eps=0.4, min\_pts=12$).
*   **Estructuras**: Planos anchos con superficie $> 4.0m$.

### 3. Fusión Geodésica (WGS84 ➔ ENU)
Proyecta las coordenadas globales $(\phi, \lambda, h)$ al plano local Cartesiano $(e, n, u)$ usando el elipsoide WGS84, aplicando una **corrección de brazo de palanca (Lever-Arm)** para compensar la distancia física entre la antena y el centro del robot.

---

## 🚀 Pipeline de Operación

1.  **Fase V1 (Mapeo)**: Construcción de mapa HD. Los puntos se integran solo si el robot se mueve $> 3m$ para evitar saturación.
2.  **Cierre de Bucle**: Al detectar que el robot regresa al origen (radio $< 15m$) después de recorrer $> 80m$, el mapa se congela y se guarda en `.ply`.
3.  **Fase V2 (Localización)**: El sistema cambia a modo estático para navegación pura sobre el mapa generado, eliminando derivas.

---

## 🛰️ Mission Control (HUD Táctico)
Interfaz en **Python** que funciona como centro de mando:
*   **Mosaico Satelital**: Descarga en tiempo real mapas de ESRI (Zoom 18).
*   **Telemetría Proyectada**: Conversión inversa de ENU a LLA para alinear la trayectoria del robot con el satélite.
*   **HUD**: Muestra velocidad (km/h), rumbo y fase actual de la misión.

---

## 🏁 Estado del Proyecto: CONCLUIDO ✅

*   [x] Fusión LiDAR-RTK robusta.
*   [x] Motor de auto-calibración funcional.
*   [x] Clasificación de objetos en tiempo real.
*   [x] Centro de mando satelital operativo.

---

## 💾 Instalación Rápida
```bash
# Dependencias
pip install numpy open3d scipy matplotlib requests Pillow

# Compilación
cd amr_2026_research_m&l
colcon build --symlink-install
source install/setup.bash

# Lanzamiento Profesional
ros2 launch lidar_rtk_slam rtk_direct_slam.launch.py
```

---
**Desarrollado por**: Alfonso | **División**: AMR 2026 Research 🏎️🛰️🦾

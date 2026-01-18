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
Elimina la necesidad de medir manualmente la posición del sensor. El sistema correlaciona el vector de movimiento del RTK con la distribución de puntos del LiDAR para calcular el desfase angular $\Delta\psi$:

$$
\Delta\psi = \arg\min_{\theta} \sum_{i=1}^{n} \left\| \vec{v}_{RTK,i} - \mathbf{R}(\theta) \cdot \vec{d}_{LiDAR,i} \right\|
$$

Donde $\theta_{RTK}$ es el rumbo del GPS y $\theta_{LiDAR}$ es el eje principal detectado en la nube de puntos.

### 2. Clasificación Semántica Geométrica
El SLAM identifica objetos sin necesidad de IA pesada, usando descriptores de forma y el algoritmo **DBSCAN** ($eps=0.4, min\_pts=12$):

*   **Postes**: Relación $\frac{altura}{ancho} > 2.2$ y $ancho < 0.6m$.
*   **Vegetación**: Clusters de alta densidad irregular (Árboles/Arbustos).
*   **Estructuras**: Superficies planas con $longitud > 4.0m$.

### 3. Fusión Geodésica (WGS84 ➔ ENU)
Proyecta coordenadas geodésicas $(\phi, \lambda, h)$ al plano local Cartesiano $(x, y, z)$ mediante una transformación de plano tangente local (ENU), aplicando corrección de **Lever-Arm** para compensar el desplazamiento físico entre antena y centro del robot.

---

## 🚀 Pipeline de Operación

1.  **Fase V1 (Mapeo)**: Construcción de mapa HD. Integración de puntos activada por umbral de movimiento ($> 3m$).
2.  **Cierre de Bucle**: Al detectar regreso al origen (radio $< 15m$) tras recorrer $> 80m$, el mapa se exporta a formato `.ply`.
3.  **Fase V2 (Localización)**: Navegación de estado sólido sobre el mapa estático con alta frecuencia de actualización de pose.

---

## 🛰️ Mission Control (HUD Táctico)
Centro de mando desarrollado en **Python** para monitorización táctica:
*   **Mosaico Satelital**: Integración dinámica con ESRI World Imagery (Zoom 18).
*   **Trayectorias Inversas**: Proyección de datos ENU de vuelta a coordenadas globales para alineación satelital precisa.
*   **Telemetría**: Visualización en tiempo real de velocidad ($km/h$), rumbo y estado de fase.

---

## 🏁 Estado del Proyecto: CONCLUIDO ✅

*   [x] Fusión LiDAR-RTK robusta.
*   [x] Motor de auto-calibración cinemática.
*   [x] Clasificación de objetos en tiempo real.
*   [x] Centro de mando satelital de alta resolución.

---

## 💾 Instalación Rápida
```bash
# Dependencias
pip install numpy open3d scipy matplotlib requests Pillow

# Compilación
cd amr_2026_research_m&l
colcon build --symlink-install
source install/setup.bash

# Lanzamiento
ros2 launch lidar_rtk_slam rtk_direct_slam.launch.py
```

---
**Desarrollado por**: Alfonso | **División**: AMR 2026 Research 🏎️🛰️🦾

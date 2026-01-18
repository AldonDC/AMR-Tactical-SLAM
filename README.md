# AMR 2026: Framework Avanzado de SLAM LiDAR-RTK

Sistema profesional de **Navegación y Mapeo (SLAM)** de alta precisión para robots autónomos, optimizado para **ROS 2 Humble**. Integra nubes de puntos 3D con posicionamiento RTK-GPS centimétrico.

---

## 🛠️ Stack Tecnológico

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Open3D](https://img.shields.io/badge/Open3D-3D_Engine-orange?style=for-the-badge)

---

## 🛰️ Innovación y Control de Misión

### Experiencia de Visualización Dual
Este framework redefine la monitorización de robots autónomos mediante una arquitectura de visualización dividida:
*   **Mapeo HD 3D (RViz)**: Reconstrucción estructural del entorno en tiempo real, permitiendo inspeccionar la densidad de la nube de puntos y la clasificación de objetos.
*   **Tactical HUD (Python)**: Centro de mando satelital que proyecta telemetría avanzada (velocidad, rumbo, coordenadas) sobre cartografía de alta resolución, ideal para misiones de campo.

### Calibración Autónoma "Zero-Effort"
El sistema elimina la complejidad de la calibración manual. Gracias a un motor de inteligencia cinemática, el robot detecta automáticamente la posición y el ángulo del sensor LiDAR analizando sus primeros metros de movimiento. Esto garantiza una alineación perfecta entre el mapa y el GPS sin intervención humana.

---

## 🚀 Pipeline de Operación

1.  **Fase V1 (Mapeo)**: El sistema construye activamente el mapa HD mientras el vehículo explora, aplicando filtros robustos para eliminar ruido y "anillos fantasma".
2.  **Detección de Cierre de Bucle**: Al regresar al punto de inicio (tras recorrer una distancia mínima de 80m), el sistema congela automáticamente el mapa y lo exporta a formato `.ply`.
3.  **Fase V2 (Localización)**: El motor cambia a un estado de navegación sólida sobre el mapa estático, proporcionando una pose ultra-estable para tareas de planificación y control.

---

## 🎯 Características Tácticas Clave

*   **Clasificación Geométrica**: Identificación automática de infraestructura (postes, paredes) y vegetación (árboles) mediante descriptores de forma.
*   **Fusión Geodésica de Alta Precisión**: Proyección inteligente de coordenadas globales (WGS84) a ejes cartesianos locales, compensando físicamente la posición de la antena RTK.
*   **Dashboard de Telemetría**: Visualización de métricas críticas de misión en una interfaz independiente, optimizada para equipos de monitoreo remoto.

---

## 🏁 Estado del Proyecto: CONCLUIDO ✅

*   [x] Fusión LiDAR-RTK robusta y estable.
*   [x] Motor de auto-calibración cinemática.
*   [x] Clasificación de objetos en tiempo real.
*   [x] Centro de mando satelital operativo y fluido.

---

## 💾 Instalación y Uso Rápido
```bash
# 1. Instalar dependencias
pip install numpy open3d scipy matplotlib requests Pillow

# 2. Compilar Workspace
cd amr_2026_research_m&l
colcon build --symlink-install
source install/setup.bash

# 3. Lanzar Centro de Misión
ros2 launch lidar_rtk_slam rtk_direct_slam.launch.py
```

---
**Desarrollado por**: Alfonso | **División**: AMR 2026 Research 🏎️🛰️🦾

# AMR 2026: Framework Avanzado de SLAM LiDAR-RTK

Sistema profesional de **Navegación y Mapeo (SLAM)** de alta precisión para robots autónomos, optimizado para **ROS 2 Humble**. Integra nubes de puntos 3D con posicionamiento RTK-GPS centimétrico.

---

## 🛠️ Stack Tecnológico

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Open3D](https://img.shields.io/badge/Open3D-3D_Engine-orange?style=for-the-badge)

---

## Demostración del Sistema

### 🗺️ Mapeo HD 3D y Clasificación Semántica
En el entorno de RViz se puede observar cómo el algoritmo detecta y clasifica objetos en tiempo real mientras construye la nube de puntos global.
<img width="1752" height="847" alt="image" src="https://github.com/user-attachments/assets/3bda2b56-c43a-48f0-a9ed-47fbc5356bd2" />


### 🛰️ Centro de Misión Táctico (HUD)
Nuestra interfaz personalizada permite un seguimiento satelital preciso, mostrando la trayectoria proyectada sobre mapas de alta resolución de ESRI.
| Vista de Satélite | Telemetría en Tiempo Real |
| :---: | :---: |
| ![Satellite View Placeholder](https://via.placeholder.com/400x300.png?text=Mission+Control+View) | ![Telemetry HUD Placeholder](https://via.placeholder.com/400x300.png?text=HUD+Telemetry+Display) |
> *Nota: Las líneas naranja y cian representan las fases de Mapeo (V1) y Localización (V2) respectivamente.*

---

## Control del sistema

### Experiencia de Visualización Dual
Este framework redefine la monitorización de robots autónomos mediante una arquitectura de visualización dividida:
*   **Mapeo HD 3D (RViz)**: Reconstrucción estructural del entorno en tiempo real.
*   **Tactical HUD (Python)**: Centro de mando satelital con telemetría integrada.

### Calibración Autónoma "Zero-Effort"
El sistema elimina la complejidad de la calibración manual. Gracias a un motor de inteligencia cinemática, el robot detecta automáticamente la posición y el ángulo del sensor LiDAR analizando sus primeros metros de movimiento.

---

## 🚀 Pipeline de Operación

1.  **Fase V1 (Mapeo)**: El sistema construye activamente el mapa HD, aplicando filtros robustos para eliminar ruido y "anillos fantasma".
2.  **Detección de Cierre de Bucle**: Al regresar al origen (tras recorrer $> 80m$), el sistema congela y exporta el mapa automáticamente a `.ply`.
3.  **Fase V2 (Localización)**: El motor cambia a un estado de navegación sólida sobre el mapa estático para tareas de planificación y control.

---

##  Galería de Resultados Finales
Colección de mapas 3D generados con éxito durante las misiones de prueba:
*   **Mapa Global del Complejo**: `[INSERTAR IMAGEN: Resultado final del .ply en Open3D]`
*   **Destaque de Clasificación**: `[INSERTAR IMAGEN: Zoom a árboles y postes detectados]`

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

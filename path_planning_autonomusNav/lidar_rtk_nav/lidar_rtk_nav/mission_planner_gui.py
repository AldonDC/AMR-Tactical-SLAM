#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from tkinter import ttk, messagebox
import open3d as o3d
import numpy as np
from PIL import Image, ImageTk
import threading

class MissionPlannerGUI(Node):
    """
    Interfaz Gráfica Profesional para Planificación de Misiones.
    Permite agregar waypoints haciendo clic en el mapa 2D.
    """
    def __init__(self):
        super().__init__('mission_planner_gui')
        
        self.waypoints = []  # Lista de waypoints [(x, y), ...]
        self.map_bounds = None  # [x_min, x_max, y_min, y_max]
        self.map_image = None
        self.canvas_scale = 1.0
        self.map_loaded = False
        
        # Publisher
        self.pub_path = self.create_publisher(Path, '/navigation/path', 10)
        
        self.get_logger().info('🎯 Mission Planner GUI Inicializado')
        
        # Crear GUI primero (instantáneo)
        self.create_gui()
        
        # Cargar mapa en thread separado
        self.status_label.config(text='⏳ Cargando mapa...')
        map_thread = threading.Thread(target=self.load_map_async, daemon=True)
        map_thread.start()
    
    def load_map_async(self):
        """Carga el mapa en segundo plano."""
        self.load_map('/home/alfonso/Documents/SEMESTRE TEC/pruebas_ros2_mapping&Localization/auto_calibrated_map.ply')
        
        # Actualizar canvas cuando termine
        if self.map_image:
            self.root.after(0, self.update_map_display)
    
    def update_map_display(self):
        """Actualiza el canvas con el mapa cargado."""
        if self.map_image:
            self.tk_image = ImageTk.PhotoImage(self.map_image)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
            self.map_loaded = True
            self.status_label.config(text='✅ Mapa cargado - Haz clic para agregar waypoints')
    
    def load_map(self, map_file):
        """Carga el mapa .ply y genera vista 2D."""
        try:
            self.get_logger().info(f'📂 Cargando mapa: {map_file}')
            pcd = o3d.io.read_point_cloud(map_file)
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            
            # Submuestrear para velocidad (usar 1 de cada 5 puntos)
            step = 3
            points = points[::step]
            colors = colors[::step] if len(colors) > 0 else colors
            
            # Obtener límites del mapa
            x_min, y_min = points[:, 0].min(), points[:, 1].min()
            x_max, y_max = points[:, 0].max(), points[:, 1].max()
            self.map_bounds = [x_min, x_max, y_min, y_max]
            
            # Crear imagen 2D (vista desde arriba) - resolución más baja para velocidad
            resolution = 0.15  # metros por pixel (antes 0.1)
            width = int((x_max - x_min) / resolution)
            height = int((y_max - y_min) / resolution)
            
            img = np.zeros((height, width, 3), dtype=np.uint8)
            
            for i, pt in enumerate(points):
                px = int((pt[0] - x_min) / resolution)
                py = int((y_max - pt[1]) / resolution)  # Invertir Y
                if 0 <= px < width and 0 <= py < height:
                    color = (colors[i] * 255).astype(np.uint8) if len(colors) > 0 else [200, 200, 200]
                    img[py, px] = color
            
            # Escalar para visualización
            max_dim = 800
            scale = min(max_dim / width, max_dim / height)
            new_w, new_h = int(width * scale), int(height * scale)
            self.canvas_scale = scale
            
            img_pil = Image.fromarray(img).resize((new_w, new_h), Image.LANCZOS)
            self.map_image = img_pil
            
            self.get_logger().info(f'✅ Mapa cargado: {width}x{height} px → {new_w}x{new_h} px')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error al cargar mapa: {str(e)}')
    
    def create_gui(self):
        """Crea la interfaz gráfica."""
        self.root = tk.Tk()
        self.root.title('🎯 MISSION PLANNER - Tactical Interface')
        self.root.geometry('1200x700')
        self.root.configure(bg='#1e1e1e')
        
        # Frame principal
        main_frame = tk.Frame(self.root, bg='#1e1e1e')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Panel izquierdo: Mapa
        left_panel = tk.Frame(main_frame, bg='#2d2d2d', relief=tk.SUNKEN, bd=2)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        tk.Label(left_panel, text='🗺️ MAPA TÁCTICO', bg='#2d2d2d', fg='cyan', 
                 font=('Arial', 14, 'bold')).pack(pady=5)
        
        self.canvas = tk.Canvas(left_panel, bg='black', cursor='crosshair')
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        if self.map_image:
            self.tk_image = ImageTk.PhotoImage(self.map_image)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        
        self.canvas.bind('<Button-1>', self.on_canvas_click)
        
        # Panel derecho: Control
        right_panel = tk.Frame(main_frame, bg='#2d2d2d', relief=tk.SUNKEN, bd=2, width=300)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))
        right_panel.pack_propagate(False)
        
        tk.Label(right_panel, text='🎯 CONTROL DE MISIÓN', bg='#2d2d2d', fg='orange',
                 font=('Arial', 12, 'bold')).pack(pady=10)
        
        # Lista de waypoints
        tk.Label(right_panel, text='Waypoints:', bg='#2d2d2d', fg='white',
                 font=('Arial', 10)).pack(pady=5)
        
        self.waypoint_listbox = tk.Listbox(right_panel, bg='#1e1e1e', fg='lime',
                                           font=('Courier', 10), height=15)
        self.waypoint_listbox.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Botones
        btn_frame = tk.Frame(right_panel, bg='#2d2d2d')
        btn_frame.pack(pady=10)
        
        tk.Button(btn_frame, text='🚀 ENVIAR MISIÓN', bg='#00ff00', fg='black',
                  font=('Arial', 10, 'bold'), command=self.send_mission).pack(fill=tk.X, pady=5)
        
        tk.Button(btn_frame, text='🗑️ LIMPIAR', bg='#ff4444', fg='white',
                  font=('Arial', 10, 'bold'), command=self.clear_waypoints).pack(fill=tk.X, pady=5)
        
        tk.Button(btn_frame, text='💾 GUARDAR', bg='#4444ff', fg='white',
                  font=('Arial', 10, 'bold'), command=self.save_mission).pack(fill=tk.X, pady=5)
        
        # Status
        self.status_label = tk.Label(right_panel, text='⏸️ Esperando waypoints...', 
                                      bg='#2d2d2d', fg='yellow', font=('Arial', 9))
        self.status_label.pack(pady=10)
    
    def on_canvas_click(self, event):
        """Maneja clic en el canvas para agregar waypoint."""
        if not self.map_bounds: return
        
        # Convertir coordenadas de canvas a mapa
        canvas_x, canvas_y = event.x, event.y
        
        x_min, x_max, y_min, y_max = self.map_bounds
        resolution = 0.1
        
        # Escala inversa
        map_x = x_min + (canvas_x / self.canvas_scale) * resolution
        map_y = y_max - (canvas_y / self.canvas_scale) * resolution
        
        self.waypoints.append((map_x, map_y))
        
        # Actualizar lista
        self.waypoint_listbox.insert(tk.END, f'{len(self.waypoints)}. X={map_x:.2f}, Y={map_y:.2f}')
        
        # Dibujar punto en canvas
        self.canvas.create_oval(canvas_x-5, canvas_y-5, canvas_x+5, canvas_y+5,
                                fill='lime', outline='white', width=2)
        
        # Conectar con línea si hay más de un punto
        if len(self.waypoints) > 1:
            prev_x, prev_y = self.waypoints[-2]
            prev_canvas_x = (prev_x - x_min) / resolution * self.canvas_scale
            prev_canvas_y = (y_max - prev_y) / resolution * self.canvas_scale
            self.canvas.create_line(prev_canvas_x, prev_canvas_y, canvas_x, canvas_y,
                                    fill='cyan', width=2, arrow=tk.LAST)
        
        self.status_label.config(text=f'✅ {len(self.waypoints)} waypoints agregados')
    
    def send_mission(self):
        """Publica la misión a ROS."""
        if not self.waypoints:
            messagebox.showwarning('⚠️ Sin waypoints', 'Agrega al menos un punto.')
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
        
        self.pub_path.publish(path_msg)
        
        self.status_label.config(text=f'🚀 MISIÓN ENVIADA ({len(self.waypoints)} pts)')
        self.get_logger().info(f'🚀 Misión publicada con {len(self.waypoints)} waypoints')
    
    def clear_waypoints(self):
        """Limpia todos los waypoints."""
        self.waypoints.clear()
        self.waypoint_listbox.delete(0, tk.END)
        
        # Redibujar mapa
        self.canvas.delete('all')
        if self.map_image:
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        
        self.status_label.config(text='🗑️ Waypoints eliminados')
    
    def save_mission(self):
        """Guarda la misión en archivo JSON."""
        import json
        if not self.waypoints:
            messagebox.showwarning('⚠️', 'No hay waypoints para guardar')
            return
        
        with open('mission_waypoints.json', 'w') as f:
            json.dump({'waypoints': [{'x': x, 'y': y} for x, y in self.waypoints]}, f, indent=2)
        
        messagebox.showinfo('💾', 'Misión guardada en mission_waypoints.json')
    
    def run(self):
        """Ejecuta la GUI."""
        self.root.mainloop()

def main():
    rclpy.init()
    node = MissionPlannerGUI()
    
    # ROS en thread separado
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # GUI en thread principal
    node.run()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

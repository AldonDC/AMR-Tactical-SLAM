#!/usr/bin/env python3
"""
WEB DASHBOARD - Sistema Autónomo AMR
Interfaz web profesional con diseño estructurado
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from flask import Flask, render_template_string, jsonify, request
import threading
import math

app = Flask(__name__)

state = {
    'robot': {'lat': 0, 'lon': 0, 'x': 0, 'y': 0, 'yaw': 0},
    'waypoints': [],
    'trajectory_v1': [],
    'trajectory_v2': [],
    'metrics': {
        'status': 'STANDBY',
        'steering': 0,
        'angular_vel': 0,
        'linear_vel': 0,
        'dist_wp': 0,
        'current_wp': 0,
        'total_wp': 0
    },
    'origin': None
}

ros_node = None

def xy_to_latlon(x, y, origin):
    """Convierte coordenadas ENU (x,y) a Lat/Lon"""
    if origin is None:
        return None, None
    R = 6378137.0  # Radio de la Tierra en metros
    origin_lat_rad = math.radians(origin['lat'])
    
    # Conversión simple equirectangular
    lat = origin['lat'] + math.degrees(y / R)
    lon = origin['lon'] + math.degrees(x / (R * math.cos(origin_lat_rad)))
    return lat, lon


def latlon_to_xy(lat, lon, origin):
    if origin is None:
        return 0, 0
    R = 6378137.0
    origin_lat = math.radians(origin['lat'])
    x = R * math.radians(lon - origin['lon']) * math.cos(origin_lat)
    y = R * math.radians(lat - origin['lat'])
    return x, y

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Sistema Autónomo AMR</title>
    <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@500;700&family=Inter:wght@400;500;600&display=swap" rel="stylesheet">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        :root {
            --bg-primary: #0d1117;
            --bg-secondary: #161b22;
            --bg-tertiary: #21262d;
            --accent: #00ff88;
            --accent2: #00aaff;
            --orange: #ff8800;
            --red: #ff4757;
            --text: #f0f6fc;
            --text-muted: #8b949e;
            --border: #30363d;
        }
        
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Inter', sans-serif;
            background: var(--bg-primary);
            color: var(--text);
            height: 100vh;
            overflow: hidden;
        }
        
        /* ===== LAYOUT PRINCIPAL ===== */
        .dashboard {
            display: grid;
            grid-template-columns: 320px 1fr 320px;
            grid-template-rows: 60px 1fr;
            height: 100vh;
            gap: 12px;
            padding: 12px;
        }
        
        /* ===== HEADER ===== */
        header {
            grid-column: 1 / -1;
            background: var(--bg-secondary);
            border: 1px solid var(--border);
            border-radius: 12px;
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 0 24px;
        }
        
        .brand {
            display: flex;
            align-items: center;
            gap: 12px;
        }
        
        .brand-icon {
            width: 40px;
            height: 40px;
            background: linear-gradient(135deg, var(--accent), var(--accent2));
            border-radius: 10px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 1.3rem;
        }
        
        .brand-text {
            font-family: 'Orbitron', sans-serif;
            font-size: 1.1rem;
            font-weight: 700;
            letter-spacing: 1px;
        }
        
        .brand-text span { color: var(--accent); }
        
        .header-stats {
            display: flex;
            align-items: center;
            gap: 24px;
        }
        
        .stat-item {
            text-align: center;
        }
        
        .stat-label {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .stat-value {
            font-family: 'Orbitron', sans-serif;
            font-size: 0.9rem;
            color: var(--accent);
        }
        
        .status-badge {
            padding: 8px 20px;
            border-radius: 20px;
            font-size: 0.75rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .status-standby { background: var(--bg-tertiary); color: var(--text-muted); }
        .status-navigating { background: var(--accent); color: #000; }
        
        /* ===== PANEL IZQUIERDO ===== */
        .left-panel {
            display: flex;
            flex-direction: column;
            gap: 12px;
        }
        
        .card {
            background: var(--bg-secondary);
            border: 1px solid var(--border);
            border-radius: 12px;
            padding: 16px;
        }
        
        .card-title {
            font-size: 0.7rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: var(--text-muted);
            margin-bottom: 16px;
            padding-bottom: 8px;
            border-bottom: 1px solid var(--border);
        }
        
        /* Steering */
        .steering-display {
            text-align: center;
        }
        
        .wheel-container {
            width: 100px;
            height: 100px;
            margin: 0 auto 16px;
        }
        
        .wheel-svg {
            width: 100%;
            height: 100%;
            transition: transform 0.15s ease;
        }
        
        .steering-value {
            font-family: 'Orbitron', sans-serif;
            font-size: 2rem;
            font-weight: 700;
            color: var(--accent);
        }
        
        .steering-direction {
            font-size: 0.85rem;
            color: var(--text-muted);
            margin-top: 4px;
        }
        
        /* Metrics Grid */
        .metrics-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
        }
        
        .metric-box {
            background: var(--bg-tertiary);
            border-radius: 8px;
            padding: 12px;
            text-align: center;
        }
        
        .metric-label {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-transform: uppercase;
            margin-bottom: 6px;
        }
        
        .metric-value {
            font-family: 'Orbitron', sans-serif;
            font-size: 1.2rem;
            font-weight: 600;
            color: var(--accent);
        }
        
        /* ===== MAPA CENTRAL ===== */
        .map-section {
            background: var(--bg-secondary);
            border: 1px solid var(--border);
            border-radius: 12px;
            overflow: hidden;
            position: relative;
        }
        
        #map { height: 100%; width: 100%; }
        
        .map-legend {
            position: absolute;
            top: 12px;
            right: 12px;
            background: rgba(13, 17, 23, 0.95);
            border: 1px solid var(--border);
            padding: 12px 16px;
            border-radius: 10px;
            z-index: 1000;
            font-size: 0.75rem;
        }
        
        .legend-title {
            font-weight: 600;
            margin-bottom: 8px;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 1px;
            font-size: 0.65rem;
        }
        
        .legend-item {
            display: flex;
            align-items: center;
            gap: 10px;
            margin-bottom: 6px;
        }
        
        .legend-line {
            width: 24px;
            height: 4px;
            border-radius: 2px;
        }
        
        .legend-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
        }
        
        /* ===== PANEL DERECHO ===== */
        .right-panel {
            display: flex;
            flex-direction: column;
            gap: 12px;
        }
        
        /* Waypoints */
        .waypoint-list {
            flex: 1;
            max-height: 180px;
            overflow-y: auto;
        }
        
        .waypoint-empty {
            color: var(--text-muted);
            text-align: center;
            padding: 20px;
            font-size: 0.85rem;
        }
        
        .waypoint-item {
            display: flex;
            align-items: center;
            gap: 10px;
            padding: 10px 12px;
            background: var(--bg-tertiary);
            border-radius: 8px;
            margin-bottom: 8px;
        }
        
        .wp-number {
            width: 24px;
            height: 24px;
            background: var(--accent);
            color: #000;
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: 700;
            font-size: 0.75rem;
        }
        
        .wp-coords {
            font-family: monospace;
            font-size: 0.8rem;
            color: var(--text-muted);
        }
        
        /* Buttons */
        .btn-group {
            display: flex;
            flex-direction: column;
            gap: 10px;
        }
        
        .btn {
            padding: 14px 20px;
            border: none;
            border-radius: 10px;
            font-family: inherit;
            font-size: 0.9rem;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #00c853, var(--accent));
            color: #000;
        }
        
        .btn-danger {
            background: linear-gradient(135deg, #d32f2f, var(--red));
            color: #fff;
        }
        
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 15px rgba(0, 255, 136, 0.3);
        }
        
        /* Graphs */
        .graphs-section {
            flex: 1;
            display: grid;
            grid-template-rows: 1fr 1fr;
            gap: 12px;
        }
        
        .graph-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
        }
        
        .graph-card {
            background: var(--bg-secondary);
            border: 1px solid var(--border);
            border-radius: 10px;
            padding: 12px;
            display: flex;
            flex-direction: column;
        }
        
        .graph-title {
            font-size: 0.65rem;
            font-weight: 600;
            text-transform: uppercase;
            color: var(--text-muted);
            margin-bottom: 8px;
        }
        
        .graph-container {
            flex: 1;
            min-height: 80px;
        }
        
        /* Scrollbar */
        ::-webkit-scrollbar { width: 6px; }
        ::-webkit-scrollbar-track { background: transparent; }
        ::-webkit-scrollbar-thumb { background: var(--border); border-radius: 3px; }
    </style>
</head>
<body>
    <div class="dashboard">
        <!-- HEADER -->
        <header>
            <div class="brand">
                <div class="brand-icon">🤖</div>
                <div class="brand-text">SISTEMA AUTÓNOMO <span>AMR</span></div>
            </div>
            <div class="header-stats">
                <div class="stat-item">
                    <div class="stat-label">Latitud</div>
                    <div class="stat-value" id="h-lat">---</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Longitud</div>
                    <div class="stat-value" id="h-lon">---</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Waypoint</div>
                    <div class="stat-value" id="h-wp">0/0</div>
                </div>
            </div>
            <div id="status" class="status-badge status-standby">STANDBY</div>
        </header>
        
        <!-- PANEL IZQUIERDO -->
        <div class="left-panel">
            <div class="card">
                <div class="card-title">🎯 Control de Dirección</div>
                <div class="steering-display">
                    <div class="wheel-container">
                        <svg class="wheel-svg" id="wheel" viewBox="0 0 100 100">
                            <circle cx="50" cy="50" r="42" fill="none" stroke="#30363d" stroke-width="5"/>
                            <circle cx="50" cy="50" r="6" fill="#30363d"/>
                            <line x1="50" y1="8" x2="50" y2="38" stroke="#8b949e" stroke-width="5" stroke-linecap="round"/>
                            <line x1="8" y1="50" x2="38" y2="50" stroke="#8b949e" stroke-width="5" stroke-linecap="round"/>
                            <line x1="92" y1="50" x2="62" y2="50" stroke="#8b949e" stroke-width="5" stroke-linecap="round"/>
                        </svg>
                    </div>
                    <div class="steering-value" id="steer-val">0.0°</div>
                    <div class="steering-direction" id="steer-dir">RECTO</div>
                </div>
            </div>
            
            <div class="card">
                <div class="card-title">📊 Métricas en Tiempo Real</div>
                <div class="metrics-grid">
                    <div class="metric-box">
                        <div class="metric-label">Vel. Lineal</div>
                        <div class="metric-value" id="m-vlin">0.00</div>
                    </div>
                    <div class="metric-box">
                        <div class="metric-label">Vel. Angular</div>
                        <div class="metric-value" id="m-vang">0.00</div>
                    </div>
                    <div class="metric-box">
                        <div class="metric-label">Distancia WP</div>
                        <div class="metric-value" id="m-dist">0.0m</div>
                    </div>
                    <div class="metric-box">
                        <div class="metric-label">Progreso</div>
                        <div class="metric-value" id="m-prog">0/0</div>
                    </div>
                </div>
            </div>
            
            <div class="graphs-section">
                <div class="graph-row">
                    <div class="graph-card">
                        <div class="graph-title">Ángulo de Giro</div>
                        <div class="graph-container"><canvas id="g1"></canvas></div>
                    </div>
                    <div class="graph-card">
                        <div class="graph-title">Vel. Angular</div>
                        <div class="graph-container"><canvas id="g2"></canvas></div>
                    </div>
                </div>
                <div class="graph-row">
                    <div class="graph-card">
                        <div class="graph-title">Distancia WP</div>
                        <div class="graph-container"><canvas id="g3"></canvas></div>
                    </div>
                    <div class="graph-card">
                        <div class="graph-title">Vel. Lineal</div>
                        <div class="graph-container"><canvas id="g4"></canvas></div>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- MAPA CENTRAL -->
        <div class="map-section">
            <div id="map"></div>
            <div class="map-legend">
                <div class="legend-title">Leyenda</div>
                <div class="legend-item">
                    <div class="legend-line" style="background: #ff8800;"></div>
                    <span>V1: Trayectoria Mapeo</span>
                </div>
                <div class="legend-item">
                    <div class="legend-line" style="background: #00ddff;"></div>
                    <span>V2: Trayectoria RTK</span>
                </div>
                <div class="legend-item">
                    <div class="legend-dot" style="background: #00ff00;"></div>
                    <span>Waypoints</span>
                </div>
                <div class="legend-item">
                    <div class="legend-dot" style="background: #ff0000;"></div>
                    <span>Robot AMR</span>
                </div>
            </div>
        </div>
        
        <!-- PANEL DERECHO -->
        <div class="right-panel">
            <div class="card" style="flex: 1; display: flex; flex-direction: column;">
                <div class="card-title">📍 Waypoints de Misión</div>
                <div class="waypoint-list" id="wp-list">
                    <div class="waypoint-empty">Click en el mapa para añadir waypoints</div>
                </div>
                <div class="btn-group">
                    <button class="btn btn-primary" onclick="startMission()">
                        🚀 INICIAR NAVEGACIÓN
                    </button>
                    <button class="btn btn-danger" onclick="stopMission()">
                        ⏹️ DETENER / LIMPIAR
                    </button>
                </div>
            </div>
        </div>
    </div>

    <script>
        // ===== MAP =====
        const map = L.map('map', { maxZoom: 22 }).setView([0, 0], 2);
        L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            maxZoom: 22,
            maxNativeZoom: 19
        }).addTo(map);

        
        const robotIcon = L.divIcon({
            html: '<div style="width:16px;height:16px;background:#ff0000;border-radius:4px;border:3px solid white;box-shadow:0 2px 8px rgba(0,0,0,0.5);"></div>',
            iconSize: [16, 16], iconAnchor: [8, 8]
        });
        const robotMarker = L.marker([0, 0], { icon: robotIcon }).addTo(map);
        
        const pathV1 = L.polyline([], { color: '#ff8800', weight: 3 }).addTo(map);
        const pathV2 = L.polyline([], { color: '#00ddff', weight: 3 }).addTo(map);
        const wpLayer = L.layerGroup().addTo(map);
        
        let originSet = false;
        
        map.on('click', async (e) => {
            const res = await fetch('/api/waypoint', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({lat: e.latlng.lat, lon: e.latlng.lng})
            });
            if ((await res.json()).status === 'ok') updateWaypoints();
        });
        
        // ===== CHARTS =====
        const createChart = (id, color) => new Chart(document.getElementById(id), {
            type: 'line',
            data: { labels: [], datasets: [{ data: [], borderColor: color, borderWidth: 2, tension: 0.3, fill: false, pointRadius: 0 }] },
            options: {
                responsive: true, maintainAspectRatio: false, animation: false,
                scales: { x: { display: false }, y: { grid: { color: '#30363d' }, ticks: { color: '#8b949e', font: { size: 8 } } } },
                plugins: { legend: { display: false } }
            }
        });
        
        const charts = {
            g1: createChart('g1', '#ffc107'),
            g2: createChart('g2', '#ff6b6b'),
            g3: createChart('g3', '#00ff88'),
            g4: createChart('g4', '#00aaff')
        };
        
        const updateChart = (chart, val) => {
            chart.data.labels.push('');
            chart.data.datasets[0].data.push(val);
            if (chart.data.labels.length > 50) { chart.data.labels.shift(); chart.data.datasets[0].data.shift(); }
            chart.update('none');
        };
        
        // ===== POLLING =====
        let lastWpCount = -1;
        
        async function poll() {
            try {
                const s = (await (await fetch('/api/state')).json());
                
                // Status
                document.getElementById('status').textContent = s.metrics.status;
                document.getElementById('status').className = 'status-badge status-' + s.metrics.status.toLowerCase();
                
                // Header stats
                if (s.robot.lat !== 0) {
                    document.getElementById('h-lat').textContent = s.robot.lat.toFixed(6);
                    document.getElementById('h-lon').textContent = s.robot.lon.toFixed(6);
                    robotMarker.setLatLng([s.robot.lat, s.robot.lon]);
                    if (!originSet && s.origin) { map.setView([s.robot.lat, s.robot.lon], 18); originSet = true; }
                }
                document.getElementById('h-wp').textContent = `${s.metrics.current_wp}/${s.metrics.total_wp}`;
                
                // Trajectories
                if (s.trajectory_v1.length > 0) pathV1.setLatLngs(s.trajectory_v1.map(p => [p.lat, p.lon]));
                if (s.trajectory_v2.length > 0) pathV2.setLatLngs(s.trajectory_v2.map(p => [p.lat, p.lon]));
                
                // Steering
                const steer = s.metrics.steering;
                document.getElementById('wheel').style.transform = `rotate(${-steer}deg)`;
                document.getElementById('steer-val').textContent = steer.toFixed(1) + '°';
                document.getElementById('steer-dir').textContent = steer > 10 ? '← IZQUIERDA' : (steer < -10 ? 'DERECHA →' : '↑ RECTO');
                
                // Metrics
                document.getElementById('m-vlin').textContent = s.metrics.linear_vel.toFixed(2);
                document.getElementById('m-vang').textContent = s.metrics.angular_vel.toFixed(2);
                document.getElementById('m-dist').textContent = s.metrics.dist_wp.toFixed(1) + 'm';
                document.getElementById('m-prog').textContent = `${s.metrics.current_wp}/${s.metrics.total_wp}`;
                
                // Charts
                updateChart(charts.g1, steer);
                updateChart(charts.g2, s.metrics.angular_vel);
                updateChart(charts.g3, s.metrics.dist_wp);
                updateChart(charts.g4, s.metrics.linear_vel);
                
                // Waypoints
                if (s.waypoints.length !== lastWpCount) {
                    lastWpCount = s.waypoints.length;
                    wpLayer.clearLayers();
                    const list = document.getElementById('wp-list');
                    if (s.waypoints.length === 0) {
                        list.innerHTML = '<div class="waypoint-empty">Click en el mapa para añadir waypoints</div>';
                    } else {
                        list.innerHTML = s.waypoints.map((w, i) => {
                            L.circleMarker([w.lat, w.lon], { radius: 8, color: '#00ff00', fillOpacity: 0.8 }).addTo(wpLayer);
                            return `<div class="waypoint-item"><div class="wp-number">${i+1}</div><span class="wp-coords">${w.lat.toFixed(6)}, ${w.lon.toFixed(6)}</span></div>`;
                        }).join('');
                    }
                }
            } catch (e) { console.error(e); }
        }
        
        async function startMission() { await fetch('/api/start', { method: 'POST' }); }
        async function stopMission() { await fetch('/api/stop', { method: 'POST' }); lastWpCount = -1; }
        
        setInterval(poll, 150);
        async function updateWaypoints() { lastWpCount = -1; }
    </script>
</body>
</html>
'''

# ===== ROS2 NODE =====
class WebDashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.origin = None
        
        self.create_subscription(NavSatFix, '/rtk/fix', self.fix_cb, 10)
        self.create_subscription(Odometry, '/rtk/odom_enu', self.odom_cb, 10)
        self.create_subscription(Odometry, '/sim/odom', self.sim_odom_cb, 10)
        self.create_subscription(Path, '/slam/path_mapping', self.v1_cb, 10)
        self.create_subscription(Path, '/slam/path_localization', self.v2_cb, 10)
        self.create_subscription(Float32MultiArray, '/autopilot/metrics', self.metrics_cb, 10)
        
        self.pub_nav = self.create_publisher(Path, '/navigation/path', 10)
        self.create_timer(5.0, self.log_status)
        self.get_logger().info('🌐 Dashboard AMR: http://localhost:5000')
    
    def log_status(self):
        self.get_logger().info(f'📊 V1:{len(state["trajectory_v1"])} V2:{len(state["trajectory_v2"])}')
    
    def fix_cb(self, msg):
        global state
        if self.origin is None:
            self.origin = {'lat': msg.latitude, 'lon': msg.longitude}
            state['origin'] = self.origin
            self.get_logger().info(f'🌍 Origen: {msg.latitude:.6f}, {msg.longitude:.6f}')
        state['robot']['lat'] = msg.latitude
        state['robot']['lon'] = msg.longitude
    
    def odom_cb(self, msg):
        global state
        state['robot']['x'] = msg.pose.pose.position.x
        state['robot']['y'] = msg.pose.pose.position.y
        qz, qw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        state['robot']['yaw'] = math.atan2(2*(qw*qz), 1-2*(qz*qz))
    
    def sim_odom_cb(self, msg):
        global state
        state['robot']['x'] = msg.pose.pose.position.x
        state['robot']['y'] = msg.pose.pose.position.y
        if state['origin']:
            lat, lon = xy_to_latlon(state['robot']['x'], state['robot']['y'], state['origin'])
            if lat: state['robot']['lat'], state['robot']['lon'] = lat, lon
    
    def v1_cb(self, msg):
        global state
        if not msg.poses or not state['origin']: return
        traj = []
        for p in msg.poses:
            lat, lon = xy_to_latlon(p.pose.position.x, p.pose.position.y, state['origin'])
            if lat: traj.append({'lat': lat, 'lon': lon})
        state['trajectory_v1'] = traj
    
    def v2_cb(self, msg):
        global state
        if not msg.poses or not state['origin']: return
        traj = []
        for p in msg.poses:
            lat, lon = xy_to_latlon(p.pose.position.x, p.pose.position.y, state['origin'])
            if lat: traj.append({'lat': lat, 'lon': lon})
        state['trajectory_v2'] = traj
    
    def metrics_cb(self, msg):
        global state
        if len(msg.data) >= 5:
            state['metrics'].update({
                'dist_wp': msg.data[1],
                'current_wp': int(msg.data[2]),
                'total_wp': int(msg.data[3]),
                'steering': msg.data[4],
                'status': 'NAVIGATING' if int(msg.data[3]) > 0 else 'STANDBY'
            })

# ===== FLASK =====
@app.route('/')
def index(): return render_template_string(HTML_TEMPLATE)

@app.route('/api/state')
def get_state(): return jsonify(state)

@app.route('/api/start', methods=['POST'])
def start():
    global ros_node
    if ros_node and state['waypoints']:
        path = Path()
        path.header.frame_id = 'map'
        for wp in state['waypoints']:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = wp['x'], wp['y']
            path.poses.append(pose)
        ros_node.pub_nav.publish(path)
    return jsonify({'status': 'ok'})

@app.route('/api/stop', methods=['POST'])
def stop():
    global ros_node
    if ros_node: ros_node.pub_nav.publish(Path())
    state['waypoints'] = []
    return jsonify({'status': 'ok'})

@app.route('/api/waypoint', methods=['POST'])
def add_wp():
    data = request.json
    if not state['origin']: return jsonify({'status': 'error'}), 400
    x, y = latlon_to_xy(data['lat'], data['lon'], state['origin'])
    state['waypoints'].append({'lat': data['lat'], 'lon': data['lon'], 'x': x, 'y': y})
    return jsonify({'status': 'ok', 'count': len(state['waypoints'])})

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)

def main():
    global ros_node
    rclpy.init()
    ros_node = WebDashboardNode()
    threading.Thread(target=run_flask, daemon=True).start()
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

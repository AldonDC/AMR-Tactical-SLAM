#!/bin/bash
# ============================================================================
# SLAM Demo Launcher Script
# ============================================================================
# Usage: ./run_slam_demo.sh
# ============================================================================

set -e

# Colors for output
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║           🚗 LiDAR-RTK SLAM Professional Demo                     ║"
echo "║                    Waymo-Style Visualization                      ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Source ROS 2
echo -e "${GREEN}[1/4] Sourcing ROS 2 Humble...${NC}"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Check if bag exists
if [ ! -d "$SCRIPT_DIR/slam_demo_bag" ]; then
    echo -e "${YELLOW}Warning: slam_demo_bag not found!${NC}"
    echo "Please run the conversion first:"
    echo "  python3 src/lidar_rtk_tools/scripts/mat_to_rosbag2.py <your_mat_file>.mat slam_demo_bag"
    exit 1
fi

# Launch SLAM
echo -e "${GREEN}[2/4] Launching SLAM pipeline with professional visualization...${NC}"
echo -e "${CYAN}[INFO] Starting SLAM nodes and RViz2...${NC}"
ros2 launch lidar_rtk_slam slam_professional.launch.py &
SLAM_PID=$!

# Wait for nodes to start
echo -e "${GREEN}[3/4] Waiting for nodes to initialize...${NC}"
sleep 5

# Play bag
echo -e "${GREEN}[4/4] Playing rosbag2 with sensor data...${NC}"
echo -e "${CYAN}[INFO] Press Ctrl+C to stop playback${NC}"
echo ""
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}  🎬 PLAYBACK STARTED - Watch the mapping in RViz2!${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════════════════════${NC}"
echo ""

# Play at normal speed with clock
ros2 bag play "$SCRIPT_DIR/slam_demo_bag" --clock --rate 1.0

# Cleanup
echo ""
echo -e "${GREEN}Playback complete. Keeping SLAM running for inspection...${NC}"
echo -e "${CYAN}Press Enter to shutdown...${NC}"
read

kill $SLAM_PID 2>/dev/null || true

echo -e "${GREEN}Done!${NC}"

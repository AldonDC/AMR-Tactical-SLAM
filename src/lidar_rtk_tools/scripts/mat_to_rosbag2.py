#!/usr/bin/env python3
"""
MATLAB .mat to rosbag2 Converter - Optimized Version

Converts MATLAB datasets (LiDAR + RTK) to ROS 2 rosbag2 format.
Uses fast byte conversion to avoid slow validation.

Author: Alfonso
Date: 2025
"""

import argparse
import sys
import os
import numpy as np
import struct

try:
    import h5py
    HAS_H5PY = True
except ImportError:
    HAS_H5PY = False
    print("Error: h5py required. Install with: pip install h5py")
    sys.exit(1)

try:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField, NavSatFix, NavSatStatus
except ImportError:
    print("Error: ROS 2 packages required. Source your ROS 2 workspace first.")
    sys.exit(1)


def detect_lidar_frames_h5(h5_file):
    """Detect LiDAR frames from HDF5 MATLAB file."""
    frames = []
    timestamps = []
    
    print(f"Keys in file: {list(h5_file.keys())}")
    
    if '#refs#' in h5_file:
        refs_group = h5_file['#refs#']
        print(f"Scanning #refs# for pointCloud objects...")
        
        location_keys = []
        for key in refs_group.keys():
            item = refs_group[key]
            if hasattr(item, 'keys') and 'Location' in item.keys():
                location_keys.append(key)
        
        location_keys.sort()
        print(f"Found {len(location_keys)} pointCloud objects with Location field")
        
        for i, key in enumerate(location_keys):
            try:
                loc_data = refs_group[key]['Location'][()]
                
                if loc_data.ndim == 3:
                    pts = loc_data.reshape(3, -1).T
                elif loc_data.ndim == 2:
                    pts = loc_data.T if loc_data.shape[0] == 3 else loc_data
                else:
                    continue
                
                valid_mask = ~np.isnan(pts).any(axis=1)
                pts = pts[valid_mask].astype(np.float32)
                
                if len(pts) > 100:
                    frames.append(pts)
                    timestamps.append(float(i) * 0.1)
                    
            except Exception as e:
                if i < 3:
                    print(f"  Skipping {key}: {e}")
        
        print(f"Successfully loaded {len(frames)} LiDAR frames")
    
    return frames, timestamps


def detect_rtk_data_h5(h5_file):
    """Detect RTK/GPS data from HDF5 MATLAB file."""
    lat, lon, alt = None, None, None
    
    print(f"Looking for RTK data...")
    
    if 'lat' in h5_file and 'lon' in h5_file:
        lat = h5_file['lat'][()].flatten()
        lon = h5_file['lon'][()].flatten()
        alt = h5_file['alt'][()].flatten() if 'alt' in h5_file else np.zeros_like(lat)
        
        # Create timestamps (20 Hz)
        timestamps = np.arange(len(lat)) * 0.05
        
        print(f"Detected {len(lat)} RTK samples")
        return True, lat, lon, alt, timestamps
    
    return False, None, None, None, None


def create_pointcloud2_fast(points, stamp_sec, stamp_nanosec, frame_id="velodyne"):
    """Create PointCloud2 message with fast byte array creation."""
    msg = PointCloud2()
    
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(stamp_nanosec)
    msg.header.frame_id = frame_id
    
    # Clean data
    points = np.nan_to_num(points[:, :3], nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
    
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    
    msg.height = 1
    msg.width = len(points)
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    
    # Fast conversion: use array module to avoid validation
    msg.data = array.array('B', points.tobytes()).tolist()
    
    return msg


def create_navsatfix(lat, lon, alt, stamp_sec, stamp_nanosec):
    """Create NavSatFix message."""
    msg = NavSatFix()
    
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(stamp_nanosec)
    msg.header.frame_id = "rtk_antenna"
    
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    
    msg.latitude = float(lat)
    msg.longitude = float(lon)
    msg.altitude = float(alt)
    
    msg.position_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    
    return msg


# Import array module
import array


def convert_mat_to_rosbag2(mat_file, output_bag):
    """Main conversion function."""
    print(f"Loading MATLAB file: {mat_file}")
    
    h5_file = h5py.File(mat_file, 'r')
    
    lidar_frames, lidar_timestamps = detect_lidar_frames_h5(h5_file)
    has_rtk, rtk_lat, rtk_lon, rtk_alt, rtk_timestamps = detect_rtk_data_h5(h5_file)
    
    if len(lidar_frames) == 0 and not has_rtk:
        print("Error: No data found")
        h5_file.close()
        return False
    
    # Remove old bag
    if os.path.exists(output_bag):
        import shutil
        shutil.rmtree(output_bag)
    
    # Setup writer
    storage_options = rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )
    
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    
    # Create topics
    if len(lidar_frames) > 0:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name='/velodyne_points', type='sensor_msgs/msg/PointCloud2', serialization_format='cdr'
        ))
    
    if has_rtk:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name='/rtk/fix', type='sensor_msgs/msg/NavSatFix', serialization_format='cdr'
        ))
    
    # Write LiDAR
    if len(lidar_frames) > 0:
        print(f"Writing {len(lidar_frames)} LiDAR frames...")
        for i, (frame, ts) in enumerate(zip(lidar_frames, lidar_timestamps)):
            stamp_sec = int(ts)
            stamp_nanosec = int((ts - stamp_sec) * 1e9)
            
            msg = create_pointcloud2_fast(frame, stamp_sec, stamp_nanosec)
            writer.write('/velodyne_points', serialize_message(msg), int(ts * 1e9))
            
            if (i + 1) % 200 == 0:
                print(f"  {i + 1}/{len(lidar_frames)} frames ({frame.shape[0]} pts)")
        
        print(f"  ✅ All {len(lidar_frames)} LiDAR frames written")
    
    # Write RTK
    if has_rtk:
        print(f"Writing {len(rtk_lat)} RTK samples...")
        for i in range(len(rtk_lat)):
            ts = rtk_timestamps[i]
            stamp_sec = int(ts)
            stamp_nanosec = int((ts - stamp_sec) * 1e9)
            
            msg = create_navsatfix(rtk_lat[i], rtk_lon[i], rtk_alt[i], stamp_sec, stamp_nanosec)
            writer.write('/rtk/fix', serialize_message(msg), int(ts * 1e9))
        
        print(f"  ✅ All {len(rtk_lat)} RTK samples written")
    
    h5_file.close()
    del writer
    
    print(f"\n✅ Conversion complete: {output_bag}")
    return True


def main():
    parser = argparse.ArgumentParser(description='Convert MATLAB .mat to ROS 2 rosbag2')
    parser.add_argument('input', help='Input .mat file')
    parser.add_argument('output', help='Output rosbag2 directory')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input):
        print(f"Error: File not found: {args.input}")
        sys.exit(1)
    
    success = convert_mat_to_rosbag2(args.input, args.output)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

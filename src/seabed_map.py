#!/usr/bin/env python3
"""
seabed_map.py

Usage:
    python3 seabed_map.py \
        --bag bags/blue3_2025-03-19-13-58-48.bag \
        --resolution 0.5 \
        --output seabed_map.png
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import utm
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from bps_oculus.core import unpack_data_entry, polar_to_cart

# --- RawData message definition (WHOI-inspired) ---
MSG_TEXT = """\
std_msgs/Header header
int8 DATA_OUT=0
int8 DATA_IN=1
int8 direction
uint8[] data
"""

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--bag',       type=Path, required=True,
                   help='Path to input .bag file')
    p.add_argument('--resolution', type=float, default=0.5,
                   help='Grid cell size in meters')
    p.add_argument('--output',    type=Path, default=Path('seabed_map.png'),
                   help='Output PNG file for the seabed map')
    p.add_argument('--max-frames', type=int, default=None,
                   help='Maximum number of sonar frames to process')
    p.add_argument('--skip-frames', type=int, default=1,
                   help='Process only every Nth sonar frame (downsample)')
    return p.parse_args()

def main():
    args = parse_args()
    print(f"Arguments: bag={args.bag}, resolution={args.resolution}, output={args.output}", flush=True)

    # 1) Set up typestore with RawData
    add_types = get_types_from_msg(MSG_TEXT, 'apl_msgs/msg/RawData')
    typestore = get_typestore(Stores.ROS1_NOETIC)
    typestore.register(add_types)

    # 2) Prepare to read bag, and holders for global data
    poses = {
        'easting': None,
        'northing': None,
        'heading': None,
        'depth': None
    }
    all_xw, all_yw, all_int = [], [], []
    total_raw_frames = 0
    processed_frames = 0
    print(f"Starting to read bag: {args.bag}", flush=True)

    with Reader(str(args.bag)) as reader:
        for connection, timestamp, raw in reader.messages():
            topic = connection.topic

            # --- update vehicle pose ---
            if topic == '/bluerov2/mavros/global_position/global':
                msg = typestore.deserialize_ros1(raw, connection.msgtype)
                e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
                poses['easting'], poses['northing'] = e, n

            elif topic == '/bluerov2/mavros/global_position/compass_hdg':
                msg = typestore.deserialize_ros1(raw, connection.msgtype)
                poses['heading'] = float(msg.data)

            elif topic == '/bluerov2/mavros/global_position/rel_alt':
                msg = typestore.deserialize_ros1(raw, connection.msgtype)
                poses['depth'] = float(msg.data)

            # --- sonar frame: unpack, cart-convert, transform, collect ---
            elif topic == '/oculus/raw_data':
                # skip until we have a full pose
                if None in poses.values():
                    continue
                total_raw_frames += 1
                # downsample by skipping frames
                if args.skip_frames > 1 and (total_raw_frames % args.skip_frames) != 0:
                    continue
                processed_frames += 1
                # stop if we've hit max frames
                if args.max_frames is not None and processed_frames > args.max_frames:
                    print(f"Reached max frames limit ({args.max_frames}). Stopping processing.", flush=True)
                    break
                if processed_frames % 50 == 0:
                    print(f"Processed {processed_frames} sonar frames so far...", flush=True)

                # 1) unpack & cart
                msg           = typestore.deserialize_ros1(raw, connection.msgtype)
                ping, polar, _ = unpack_data_entry(msg.data.tobytes())
                if ping is None:
                    continue
                cart = polar_to_cart(polar)

                # 2) flatten local coords + intensities
                xs = cart.x_table.ravel()
                ys = cart.y_table.ravel()
                inten = cart.cart_image.ravel()

                # 3) rotate by heading (yaw) and translate to UTM
                θ = np.deg2rad(poses['heading'])
                X =  xs * np.cos(θ) - ys * np.sin(θ) + poses['easting']
                Y =  xs * np.sin(θ) + ys * np.cos(θ) + poses['northing']

                all_xw.append(X)
                all_yw.append(Y)
                all_int.append(inten)

    # nothing read?
    if not all_xw:
        raise RuntimeError("No sonar data collected – check topics & typestore")
    
    # concatenate
    all_xw  = np.hstack(all_xw)
    all_yw  = np.hstack(all_yw)
    all_int = np.hstack(all_int)

    print(f"Finished reading. Total sonar frames processed: {processed_frames}. Building 2D map...", flush=True)
    # 4) build 2D intensity map via histogram
    xmin, xmax = all_xw.min(), all_xw.max()
    ymin, ymax = all_yw.min(), all_yw.max()
    nx = int(np.ceil((xmax - xmin) / args.resolution))
    ny = int(np.ceil((ymax - ymin) / args.resolution))

    # weighted sum of intensities
    heatmap, xedges, yedges = np.histogram2d(
        all_xw, all_yw,
        bins=[nx, ny],
        range=[[xmin, xmax], [ymin, ymax]],
        weights=all_int
    )
    # count per cell for normalization
    counts, _, _ = np.histogram2d(
        all_xw, all_yw,
        bins=[nx, ny],
        range=[[xmin, xmax], [ymin, ymax]]
    )
    # avoid division by zero
    mask = counts > 0
    heatmap[mask] /= counts[mask]

    # 5) plot & save
    plt.figure(figsize=(8, 6))
    # extent ensures axes are labeled in meters
    extent = [xmin, xmax, ymin, ymax]
    plt.imshow(
        heatmap.T,                          # transpose so Y goes up
        origin='lower',
        extent=extent,
        aspect='equal'
    )
    plt.colorbar(label='Mean backscatter intensity')
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    plt.title('2D Seabed Backscatter Map')
    plt.tight_layout()
    plt.savefig(str(args.output), dpi=300)
    print(f"Map saved to {args.output}")

if __name__ == '__main__':
    main()
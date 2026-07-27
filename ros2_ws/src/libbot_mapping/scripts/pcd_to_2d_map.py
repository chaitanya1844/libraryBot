#!/usr/bin/env python3
"""Converts a FAST-LIO2 PCD point-cloud map into a 2D occupancy grid
(.pgm + .yaml) in the same format nav2_map_server / slam_toolbox produce,
so Nav2's costmaps can use a FAST-LIO2 map exactly like a slam_toolbox one.

Simple slice-and-threshold projection, not true ray-traced occupancy:
points within a horizontal z-band (meant to catch walls/shelves, not the
floor or ceiling) are projected onto the XY plane at a fixed resolution;
any cell a point lands in is marked occupied, everything else is marked
free. This is the same simplification most standalone pcd2pgm tools use
and is good enough for Nav2 planning around static furniture, but unlike
a real SLAM map it doesn't know the difference between "confirmed empty"
and "never actually seen".

    python3 pcd_to_2d_map.py map.pcd output_name --resolution 0.05 \\
        --z-min 0.05 --z-max 0.9

z-min/z-max are relative to the PCD's own z=0, which is wherever
FAST-LIO2 happened to initialize (the lidar's height at that moment, see
fast_lio_map_odom_bridge.py) -- not the floor. Check the point cloud's
actual z-range first (this script prints it) and adjust z-min/z-max to
land on wall/shelf height, not the floor or ceiling, if the defaults cut
out real obstacles or let the floor itself show up as clutter.
"""
import argparse
import os

import numpy as np
import open3d as o3d
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd_path', help='Input FAST-LIO2 PCD map file')
    parser.add_argument('output_name', help='Output name (no extension) -- writes <name>.pgm/.yaml')
    parser.add_argument('--output-dir', default=os.path.join(
        os.path.dirname(__file__), '..', 'maps'))
    parser.add_argument('--resolution', type=float, default=0.05, help='meters/pixel')
    parser.add_argument('--z-min', type=float, default=0.05,
                         help='meters above the PCD origin to start including points (skip the floor)')
    parser.add_argument('--z-max', type=float, default=0.9,
                         help='meters above the PCD origin to stop including points (skip the ceiling)')
    parser.add_argument('--occupied-min-points', type=int, default=2,
                         help='points a cell needs before it counts as occupied (denoising)')
    parser.add_argument('--padding-m', type=float, default=1.0,
                         help='free-space margin added around the point cloud\'s own extent')
    args = parser.parse_args()

    cloud = o3d.io.read_point_cloud(args.pcd_path)
    pts = np.asarray(cloud.points)
    if pts.shape[0] == 0:
        raise SystemExit(f'{args.pcd_path} has no points -- was FAST-LIO2 stopped with '
                          'SIGINT (Ctrl-C), not kill -9? PCDWriter only saves on clean shutdown.')

    print(f'Loaded {len(pts)} points. z range: [{pts[:, 2].min():.2f}, {pts[:, 2].max():.2f}]')

    band = pts[(pts[:, 2] >= args.z_min) & (pts[:, 2] <= args.z_max)]
    print(f'{len(band)} points in the [{args.z_min}, {args.z_max}] z-band used for the map')
    if len(band) == 0:
        raise SystemExit('No points survived the z-band filter -- check --z-min/--z-max '
                          'against the printed z range above.')

    x_min = band[:, 0].min() - args.padding_m
    y_min = band[:, 1].min() - args.padding_m
    x_max = band[:, 0].max() + args.padding_m
    y_max = band[:, 1].max() + args.padding_m

    width = int(np.ceil((x_max - x_min) / args.resolution))
    height = int(np.ceil((y_max - y_min) / args.resolution))

    col = ((band[:, 0] - x_min) / args.resolution).astype(np.int32)
    row = ((band[:, 1] - y_min) / args.resolution).astype(np.int32)
    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (row, col), 1)

    # pgm convention: 254 = free, 0 = occupied, row 0 is the image's
    # *bottom* in ROS map_server's y-up world -- pgm itself stores
    # top-to-bottom, so flip vertically on write.
    grid = np.full((height, width), 254, dtype=np.uint8)
    grid[counts >= args.occupied_min_points] = 0
    grid = np.flipud(grid)

    os.makedirs(args.output_dir, exist_ok=True)
    pgm_path = os.path.join(args.output_dir, args.output_name + '.pgm')
    yaml_path = os.path.join(args.output_dir, args.output_name + '.yaml')

    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        f.write(grid.tobytes())

    with open(yaml_path, 'w') as f:
        yaml.safe_dump({
            'image': args.output_name + '.pgm',
            'mode': 'trinary',
            'resolution': args.resolution,
            'origin': [float(x_min), float(y_min), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }, f)

    print(f'Wrote {pgm_path} ({width}x{height}) and {yaml_path}')


if __name__ == '__main__':
    main()

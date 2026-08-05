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

z-min/z-max are meters above the FLOOR, not the PCD's own z=0 -- that
origin is wherever FAST-LIO2 happened to initialize (the lidar's height
at that moment, see fastlio_map_odom_bridge.py), which is a different,
essentially arbitrary height every mapping session (different spawn
pose, different start-of-mapping pose) and was previously off by however
tall the robot's sensor mast is (measured ~0.75m on this robot -- a
whole map's worth of "z offset" between what z-min/z-max=[0.05, 0.9]
actually selected and the wall/shelf band the docstring meant, silently
different per map depending only on where mapping happened to start).
The floor is instead auto-detected as the single largest near-horizontal
cluster of points in the cloud's lower half (an indoor floor is normally
*by far* the biggest flat surface any lidar sees -- confirmed on this
robot's own maps: 3-5x the point count of the next largest band) and
z-min/z-max are applied relative to that. Override with --floor-z if a
particular map's real floor isn't the largest cluster (e.g. a scene with
more shelf-top than floor coverage).
"""
import argparse
import os

import numpy as np
import open3d as o3d
import yaml
from scipy import ndimage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd_path', help='Input FAST-LIO2 PCD map file')
    parser.add_argument('output_name', help='Output name (no extension) -- writes <name>.pgm/.yaml')
    parser.add_argument('--output-dir', default=os.path.join(
        os.path.dirname(__file__), '..', 'maps'))
    parser.add_argument('--pcd-output-dir', default=os.path.join(
        os.path.dirname(__file__), '..', 'pcd_maps'),
        help='Where to copy the source PCD, permanently renamed to match '
             '<output_name>.pcd -- without this, every saved 2D map name '
             'points back at the single, ever-overwritten pcd_maps/latest.pcd, '
             'so picking an older map while a newer PCD has since been saved '
             'silently pairs the wrong 3D data with it (localization then '
             'confidently aligns against geometry from a different session).')
    parser.add_argument('--resolution', type=float, default=0.05, help='meters/pixel')
    parser.add_argument('--z-min', type=float, default=0.05,
                         help='meters above the detected floor to start including points (skip the floor itself)')
    parser.add_argument('--z-max', type=float, default=0.9,
                         help='meters above the detected floor to stop including points (skip the ceiling)')
    parser.add_argument('--floor-z', type=float, default=None,
                         help='override auto-detected floor height (PCD\'s own z units) -- use if the '
                              'largest flat surface in this particular map isn\'t actually the floor')
    parser.add_argument('--occupied-min-points', type=int, default=2,
                         help='points a cell needs before it counts as occupied (denoising)')
    parser.add_argument('--min-blob-cells', type=int, default=4,
                         help='occupied cells must belong to an 8-connected blob at least this '
                              'big to survive -- real walls/shelves are long connected runs of '
                              'occupied cells, so this drops small speckle (a handful of isolated '
                              'or near-isolated cells, e.g. from ICP jitter during mapping) without '
                              'touching actual structure. Set to 1 to disable.')
    parser.add_argument('--padding-m', type=float, default=1.0,
                         help='free-space margin added around the point cloud\'s own extent')
    args = parser.parse_args()

    cloud = o3d.io.read_point_cloud(args.pcd_path)
    pts = np.asarray(cloud.points)
    if pts.shape[0] == 0:
        raise SystemExit(f'{args.pcd_path} has no points -- was FAST-LIO2 stopped with '
                          'SIGINT (Ctrl-C), not kill -9? PCDWriter only saves on clean shutdown.')

    z_lo, z_hi = pts[:, 2].min(), pts[:, 2].max()
    print(f'Loaded {len(pts)} points. z range: [{z_lo:.2f}, {z_hi:.2f}]')

    if args.floor_z is not None:
        floor_z = args.floor_z
        print(f'Using --floor-z override: {floor_z:.2f}')
    else:
        # 3cm bins over the lower half of the cloud's own z range -- the
        # floor is normally the single largest flat surface a lidar sees
        # indoors, and restricting the search to the lower half avoids a
        # large ceiling occasionally outscoring it.
        lower_half = pts[pts[:, 2] <= z_lo + 0.5 * (z_hi - z_lo)]
        bins = max(int((lower_half[:, 2].max() - lower_half[:, 2].min()) / 0.03), 1)
        hist, edges = np.histogram(lower_half[:, 2], bins=bins)
        peak = np.argmax(hist)
        floor_mask = (lower_half[:, 2] >= edges[peak]) & (lower_half[:, 2] < edges[peak + 1])
        floor_z = float(lower_half[floor_mask, 2].mean())
        print(f'Auto-detected floor at z={floor_z:.2f} ({hist[peak]} points in that band, '
              f'vs {len(lower_half)} points total in the lower half) -- override with --floor-z '
              'if this looks wrong')

    band = pts[(pts[:, 2] >= floor_z + args.z_min) & (pts[:, 2] <= floor_z + args.z_max)]
    print(f'{len(band)} points in the [{args.z_min}, {args.z_max}] m-above-floor z-band used for the map')
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
    occupied = counts >= args.occupied_min_points

    # Drops small speckle that occupied-min-points alone doesn't catch:
    # a spurious point hit repeatedly (e.g. ICP jitter during a slow pass,
    # or a transient misregistration) clears occupied_min_points just
    # fine since it's the same handful of cells lit up over and over, but
    # it's still only a few isolated/near-isolated cells, not a wall --
    # real walls/shelves are long connected runs. Label 8-connected
    # occupied blobs and drop any smaller than min_blob_cells. This is
    # exactly the failure mode that pinned a robot's own spawn cell as
    # unplannable once: a few-cell noise blob landed almost on top of it,
    # inflation marked the start pose lethal, and every plan request
    # failed before the robot could move at all.
    if args.min_blob_cells > 1 and occupied.any():
        labels, num = ndimage.label(occupied, structure=np.ones((3, 3)))
        sizes = ndimage.sum(occupied, labels, index=np.arange(1, num + 1))
        small = np.where(sizes < args.min_blob_cells)[0] + 1
        if len(small) > 0:
            occupied[np.isin(labels, small)] = False
            print(f'Dropped {len(small)} speckle blob(s) smaller than '
                  f'{args.min_blob_cells} cells ({int(sum(sizes[small - 1]))} cells total)')

    # pgm convention: 254 = free, 0 = occupied, row 0 is the image's
    # *bottom* in ROS map_server's y-up world -- pgm itself stores
    # top-to-bottom, so flip vertically on write.
    grid = np.full((height, width), 254, dtype=np.uint8)
    grid[occupied] = 0
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

    os.makedirs(args.pcd_output_dir, exist_ok=True)
    named_pcd_path = os.path.join(args.pcd_output_dir, args.output_name + '.pcd')
    # Re-zeroed on Z (shifted by -floor_z), not a verbatim copy: the 2D
    # map above always renders at z=0 in the map frame (nav2_map_server
    # hardcodes that -- the yaml's 3rd "origin" number is yaw, not z,
    # there's no way to give the grid its own height), and
    # fastlio_map_odom_bridge separately anchors live map-frame Z to
    # roughly "current ground" during localization. Saving this PCD with
    # its own raw z=0 (wherever FAST-LIO2 happened to initialize, not the
    # floor) left it floating below both of those by the same ~0.75m the
    # z-band fix above already found -- the static reference cloud (and
    # anything ICP-matched against it) visibly sitting under the flat 2D
    # map/floor plane instead of on it. Shifting by floor_z here costs
    # ICP nothing: it only matches shapes, and this file's absolute Z
    # was already untrusted/unused for the live map->odom Z (see that
    # file's own docstring).
    if os.path.abspath(named_pcd_path) == os.path.abspath(args.pcd_path):
        pts[:, 2] -= floor_z
        cloud.points = o3d.utility.Vector3dVector(pts)
        o3d.io.write_point_cloud(named_pcd_path, cloud)
    else:
        cloud.points = o3d.utility.Vector3dVector(pts - np.array([0.0, 0.0, floor_z]))
        o3d.io.write_point_cloud(named_pcd_path, cloud)
    print(f'Wrote z-zeroed source PCD (floor now at z=0) to {named_pcd_path} '
          '(permanently paired with this map name)')


if __name__ == '__main__':
    main()

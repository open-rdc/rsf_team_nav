#!/usr/bin/env python3
"""芝生マスク (make_grass_mask.py の出力) に, 指定した区間だけ通路の中央が最も低くなる勾配を足す (global costmap 用).

keepout フィルタは inflation の後にかかるので, global の inflation_radius を広げても芝生の縁からは勾配が付かない.
芝生の手前の帯 (0.6m) だけだと帯の内側はどこもコスト 0 で, global path は入口の位置しだいで芝生の縁 (壁際) に沿う.
WP first..last+1 の折れ線に最も近いセル (その区間の通路) に, 芝生から width [m] まで芝生に近いほど高い値 (接するセルで peak) を
元の値との大きい方で入れる. 他の区間 (road_edge_keep など) の値は変えない.

usage:
  add_center_ramp.py <in.pgm> <waypoints.yaml> <out.pgm> [--first 9] [--last 18] [--width 2.0] [--peak 50]
                     [--route-first 0] [--route-last 30]
"""
import argparse
import os
import sys

import numpy as np
import yaml
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_grass_mask import EXTENT, RESOLUTION, route_distance  # noqa: E402


def read_pgm(path):
    """P5 の pgm を (H, W) uint8 で読む. 行 0 が y の小さい側"""
    data = open(path, 'rb').read()
    fields = data.split(maxsplit=4)
    width, height = int(fields[1]), int(fields[2])
    return np.flipud(np.frombuffer(fields[4], dtype=np.uint8)[:width * height].reshape(height, width)).copy()


def write_pgm(image, path):
    height, width = image.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(np.flipud(image).astype(np.uint8).tobytes())


def section_cells(route, first, last, route_first, shape):
    """route (WP route_first からの折れ線) の各セルで最も近い区間が WP first..last+1 のものなら True"""
    dists = np.stack([route_distance(route[k:k + 2], shape=shape) for k in range(len(route) - 1)])
    nearest = np.argmin(dists, axis=0) + route_first
    return (nearest >= first) & (nearest <= last)


def center_ramp(occupancy, section, width=2.0, peak=50, resolution=RESOLUTION):
    """section 内の芝生でないセルに, 芝生から width [m] まで芝生に近いほど高い値 (接するセルで peak) を元の値と大きい方で入れる"""
    grass = occupancy >= 100
    dist = ndimage.distance_transform_edt(~grass) * resolution
    ramp = np.rint(np.clip((width + resolution - dist) / width, 0.0, 1.0) * peak).astype(np.uint8)
    out = occupancy.copy()
    target = section & ~grass
    out[target] = np.maximum(out[target], ramp[target])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('in_pgm')
    ap.add_argument('waypoints')
    ap.add_argument('out_pgm')
    ap.add_argument('--first', type=int, default=9)
    ap.add_argument('--last', type=int, default=18)
    ap.add_argument('--width', type=float, default=2.0)
    ap.add_argument('--peak', type=int, default=50)
    ap.add_argument('--route-first', type=int, default=0)
    ap.add_argument('--route-last', type=int, default=30)
    args = ap.parse_args()
    waypoints = yaml.safe_load(open(args.waypoints))['waypoints']
    route = [(w['x'], w['y']) for w in waypoints[args.route_first:args.route_last + 1]]
    occupancy = read_pgm(args.in_pgm)
    expected = (int(round((EXTENT[3] - EXTENT[2]) / RESOLUTION)), int(round((EXTENT[1] - EXTENT[0]) / RESOLUTION)))
    if occupancy.shape != expected:
        sys.exit(f'{args.in_pgm} is {occupancy.shape}, expected {expected}')
    section = section_cells(route, args.first, args.last, args.route_first, occupancy.shape)
    write_pgm(center_ramp(occupancy, section, args.width, args.peak), args.out_pgm)
    print(f'WP{args.first}-{args.last + 1}: ramp {args.width}m (peak {args.peak}) -> {args.out_pgm}')


if __name__ == '__main__':
    main()

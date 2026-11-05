#!/usr/bin/env python3
"""点群地図の地面反射強度から, 経路沿いの芝生を keepout にするマスク (raw の pgm) を作る.

芝生は舗装より LiDAR の反射強度が強い. 地面 (各セルの最低点から 0.12m 以内) の点の平均反射強度を
0.2m 格子にし, 5x5 のメディアン + オープニングをかけたあと threshold を超えるセルを芝生とする.
経路 (WP first..last の折れ線) から protect [m] 以内は通路として常に空け, reach [m] より遠くは対象外.

usage:
  make_grass_mask.py <pcd> <waypoints.yaml> <out_prefix> [--protect 1.1] [--reach 8.0]
                     [--first 0] [--last 30] [--threshold 22] [--band 0.6] [--peak 50]
out_prefix.pgm と out_prefix.yaml を書き出す (map_server の raw モード, 100 が通行不可,
芝生の手前 band [m] は芝生に近いほど高い値 (最大 peak) で, global path を通路の中央へ寄せる).
"""
import argparse
import os

import numpy as np
import yaml
from scipy import ndimage

EXTENT = (-80.0, 110.0, -10.0, 120.0)  # x0, x1, y0, y1 [m]. 確認走行区間を含む範囲
RESOLUTION = 0.2


def ground_intensity(points, extent=EXTENT, resolution=RESOLUTION, ground_band=0.12):
    """points: (N, 4) x y z intensity. 各セルの地面の点の平均反射強度 (H, W). 点の無いセルは nan"""
    x0, x1, y0, y1 = extent
    inside = (points[:, 0] > x0) & (points[:, 0] < x1) & (points[:, 1] > y0) & (points[:, 1] < y1)
    p = points[inside]
    width, height = int(round((x1 - x0) / resolution)), int(round((y1 - y0) / resolution))
    ix = ((p[:, 0] - x0) / resolution).astype(int)
    iy = ((p[:, 1] - y0) / resolution).astype(int)
    k = iy * width + ix
    zmin = np.full(width * height, np.inf)
    np.minimum.at(zmin, k, p[:, 2])
    ground = p[:, 2] < zmin[k] + ground_band
    total = np.zeros(width * height)
    count = np.zeros(width * height)
    np.add.at(total, k[ground], p[ground, 3])
    np.add.at(count, k[ground], 1)
    return np.where(count > 0, total / np.maximum(count, 1), np.nan).reshape(height, width)


def route_distance(route, extent=EXTENT, resolution=RESOLUTION, shape=None):
    """各セル中心から経路の折れ線 route [(x, y), ...] までの距離 (H, W)"""
    x0, _, y0, _ = extent
    height, width = shape
    gx = x0 + (np.arange(width) + 0.5) * resolution
    gy = y0 + (np.arange(height) + 0.5) * resolution
    X, Y = np.meshgrid(gx, gy)
    dist = np.full(X.shape, np.inf)
    for a, b in zip(route[:-1], route[1:]):
        u = np.asarray(b, float) - np.asarray(a, float)
        length = float(np.hypot(*u))
        if length < 1e-9:
            continue
        u /= length
        t = np.clip((X - a[0]) * u[0] + (Y - a[1]) * u[1], 0.0, length)
        dist = np.minimum(dist, np.hypot(X - (a[0] + u[0] * t), Y - (a[1] + u[1] * t)))
    return dist


def grass_mask(intensity, dist, threshold=22.0, protect=1.1, reach=8.0):
    """芝生かつ経路から protect より遠く reach より近いセルを True"""
    med = ndimage.median_filter(np.where(np.isnan(intensity), 0.0, intensity), size=5)
    grass = ndimage.binary_opening(med > threshold, iterations=1)
    return grass & (dist > protect) & (dist < reach)


def grass_occupancy(mask, band=0.6, peak=50, resolution=RESOLUTION):
    """芝生 mask を map_server の raw 値 (0-100) にする. 芝生は 100 (通行不可).
    芝生の手前 band [m] には芝生に近いほど高い値 (接するセルで peak) を付ける. keepout は inflation の後に
    かかるので, これが無いと global path が芝生の縁に沿って引かれ, padding 込みの足跡が芝生にかかったまま進もうとする"""
    dist = ndimage.distance_transform_edt(~mask) * resolution
    ramp = np.clip((band + resolution - dist) / band, 0.0, 1.0) * peak
    return np.where(mask, 100, np.rint(ramp)).astype(np.uint8)


def write_mask(mask, prefix, extent=EXTENT, resolution=RESOLUTION, comment=''):
    """mask (H, W; 行 0 が y の小さい側. bool なら True を 100, uint8 ならその値) を raw モードの pgm と yaml に書き出す"""
    height, width = mask.shape
    image = np.flipud(mask).astype(np.uint8) * 100 if mask.dtype == bool else np.flipud(mask).astype(np.uint8)
    with open(prefix + '.pgm', 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(image.tobytes())
    with open(prefix + '.yaml', 'w') as f:
        f.write(comment)
        f.write(f'image: {os.path.basename(prefix)}.pgm\nmode: raw\nresolution: {resolution}\n'
                f'origin: [{extent[0]}, {extent[2]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def read_pcd_xyzi(path):
    """fields x y z intensity (float32) の binary pcd を (N, 4) で読む"""
    data = open(path, 'rb').read()
    marker = b'DATA binary\n'
    return np.frombuffer(data[data.index(marker) + len(marker):], dtype=np.float32).reshape(-1, 4)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pcd')
    ap.add_argument('waypoints')
    ap.add_argument('out_prefix')
    ap.add_argument('--protect', type=float, default=1.1)
    ap.add_argument('--reach', type=float, default=8.0)
    ap.add_argument('--first', type=int, default=0)
    ap.add_argument('--last', type=int, default=30)
    ap.add_argument('--threshold', type=float, default=22.0)
    ap.add_argument('--band', type=float, default=0.6)
    ap.add_argument('--peak', type=int, default=50)
    args = ap.parse_args()
    waypoints = yaml.safe_load(open(args.waypoints))['waypoints']
    route = [(w['x'], w['y']) for w in waypoints[args.first:args.last + 1]]
    intensity = ground_intensity(read_pcd_xyzi(args.pcd))
    mask = grass_mask(intensity, route_distance(route, shape=intensity.shape),
                      args.threshold, args.protect, args.reach)
    comment = (f'# 芝生 (シミュレータ・地図では通れるが, 完全に入ると失格). 確認走行区間 WP{args.first}-{args.last} の経路から '
               f'{args.protect}-{args.reach}m の芝生を通行不可 (100)\n'
               f'# 点群地図の地面反射強度 (芝生は強い) から scripts/make_grass_mask.py で生成. '
               f'経路の ±{args.protect}m は通路として常に空ける. 芝生の手前 {args.band}m は芝生に近いほど高い値 (最大 {args.peak})\n')
    write_mask(grass_occupancy(mask, args.band, args.peak), args.out_prefix, comment=comment)
    print(f'{mask.sum() * RESOLUTION ** 2:.0f} m2 -> {args.out_prefix}.pgm')


if __name__ == '__main__':
    main()

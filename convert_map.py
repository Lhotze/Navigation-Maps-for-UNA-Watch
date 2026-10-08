#!/usr/bin/env python3
"""
Offline Navigation - map converter, part 1 (display layer).

Turns an OpenStreetMap extract (.osm.pbf, e.g. from download.geofabrik.de) into
small tile files for the watch:

    <out>/<region>/region.json
    <out>/<region>/m/<bx>/<by>.pak      the road tiles, 8 x 8 grid cells per file (see pak.py)

Why many small files: measured on the watch, a seek costs time proportional to
the position inside a file (the FAT cluster chain is walked from the start), so
a few hundred KB per file keeps every access around 1 ms, while one big file
costs ~30 ms per random read.

The grid is absolute (anchored at lat 0 / lon 0), so tiles from different
regions line up and a border tile that appears in two regions is simply the
same cell twice.

Tile file (.m), all values little-endian:
    u8  magic 'M'   u8 version (1)   u16 nWays   u16 nStrings
    nWays x:  u8 class  u8 flags  u16 nameIdx (0xFFFF = none)  u16 nPts
              nPts x (u16 x, u16 y)       # 0..65535 across the tile; y up (north)
    nStrings x: u8 length + UTF-8 bytes
Ways are sorted by class, most important LAST, so drawing in file order
paints major roads over minor ones.

Area layers (filled polygons), same grid, written next to the roads:
    <out>/<region>/a/<bx>/<by>.pak  forest, water, parks, built-up areas   (default)
    <out>/<region>/b/<bx>/<by>.pak  buildings (--buildings; the app no longer uses them)
    u8  magic 'A' or 'B'   u8 version (1)   u16 nPolys
    nPolys x:  u8 type  u8 flags (bit0 = hole: paint with the background)  u16 nPts
               nPts x (u16 x, u16 y)       # ring, closed implicitly
    type: 0 forest, 1 water, 2 park / grass, 3 built-up area (residential, commercial, industrial), 4 building
Polygons are clipped to their tile and sorted in drawing order (park, forest,
water; holes after the fills of their type).
"""

import argparse
import json
import math
import os
import struct
import sys
import time
from collections import defaultdict

import osmium

from pak import BundleWriter, SIDE

# Tile size in micro-degrees. 0.02 deg lat = 2.2 km, 0.03 deg lon = 2.2 km at 49 N.
TILE_DLAT_E6 = 20000
TILE_DLON_E6 = 30000
DLAT = TILE_DLAT_E6 / 1e6
DLON = TILE_DLON_E6 / 1e6

# --- road classes (also used by the watch) ----------------------------------
C_MOTORWAY, C_TRUNK, C_PRIMARY, C_SECONDARY, C_TERTIARY, C_MINOR, C_SERVICE, \
    C_TRACK, C_PATH, C_CYCLE, C_RAIL, C_WATER = range(12)

HIGHWAY_CLASS = {
    'motorway': C_MOTORWAY, 'motorway_link': C_MOTORWAY,
    'trunk': C_TRUNK, 'trunk_link': C_TRUNK,
    'primary': C_PRIMARY, 'primary_link': C_PRIMARY,
    'secondary': C_SECONDARY, 'secondary_link': C_SECONDARY,
    'tertiary': C_TERTIARY, 'tertiary_link': C_TERTIARY,
    'unclassified': C_MINOR, 'residential': C_MINOR, 'living_street': C_MINOR,
    'pedestrian': C_MINOR, 'road': C_MINOR,
    'service': C_SERVICE,
    'track': C_TRACK,
    'path': C_PATH, 'footway': C_PATH, 'steps': C_PATH, 'bridleway': C_PATH,
    'cycleway': C_CYCLE,
}
RAIL = {'rail', 'light_rail'}
WATER = {'river', 'canal'}

F_ONEWAY, F_BRIDGE, F_TUNNEL = 1, 2, 4

SIMPLIFY_M = 2.0  # Douglas-Peucker tolerance


def way_class(tags):
    hw = tags.get('highway')
    if hw is not None:
        cls = HIGHWAY_CLASS.get(hw)
        if cls is None:
            return None
        if tags.get('area') == 'yes':
            return None
        if hw == 'footway' and tags.get('footway') in ('sidewalk', 'crossing', 'traffic_island'):
            return None
        if hw == 'service' and tags.get('service') in ('parking_aisle', 'driveway', 'drive-through'):
            return None
        return cls
    if tags.get('railway') in RAIL:
        return C_RAIL
    if tags.get('waterway') in WATER:
        return C_WATER
    return None


def simplify(pts, eps):
    """Douglas-Peucker on (x, y) in metres. Iterative, returns kept indices."""
    n = len(pts)
    if n <= 2:
        return list(range(n))
    keep = [False] * n
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        ax, ay = pts[a]
        bx, by = pts[b]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        best, bi = -1.0, -1
        for i in range(a + 1, b):
            px, py = pts[i]
            if L2 == 0.0:
                d = (px - ax) ** 2 + (py - ay) ** 2
            else:
                t = ((px - ax) * dx + (py - ay) * dy) / L2
                t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                qx, qy = ax + t * dx - px, ay + t * dy - py
                d = qx * qx + qy * qy
            if d > best:
                best, bi = d, i
        if best > eps * eps:
            keep[bi] = True
            stack.append((a, bi))
            stack.append((bi, b))
    return [i for i in range(n) if keep[i]]


def split_into_cells(lonlat):
    """
    Cut a polyline (list of (lon, lat)) at the grid lines. Returns
    {(ix, iy): [polyline, ...]} where each polyline is a list of (lon, lat).
    """
    cells = defaultdict(list)
    cur_cell = None
    cur = None

    def put(cell, p0, p1):
        nonlocal cur_cell, cur
        if cell != cur_cell or cur is None or cur[-1] != p0:
            cur = [p0]
            cells[cell].append(cur)
            cur_cell = cell
        cur.append(p1)

    for (x0, y0), (x1, y1) in zip(lonlat, lonlat[1:]):
        # parameters where the segment crosses a vertical / horizontal grid line
        ts = [0.0, 1.0]
        dx, dy = x1 - x0, y1 - y0
        if dx != 0.0:
            a, b = sorted((x0 / DLON, x1 / DLON))
            for k in range(math.floor(a) + 1, math.ceil(b)):
                t = (k * DLON - x0) / dx
                if 0.0 < t < 1.0:
                    ts.append(t)
        if dy != 0.0:
            a, b = sorted((y0 / DLAT, y1 / DLAT))
            for k in range(math.floor(a) + 1, math.ceil(b)):
                t = (k * DLAT - y0) / dy
                if 0.0 < t < 1.0:
                    ts.append(t)
        ts.sort()
        for t0, t1 in zip(ts, ts[1:]):
            if t1 - t0 < 1e-12:
                continue
            p0 = (x0 + dx * t0, y0 + dy * t0)
            p1 = (x0 + dx * t1, y0 + dy * t1)
            tm = (t0 + t1) * 0.5
            mx, my = x0 + dx * tm, y0 + dy * tm
            put((math.floor(mx / DLON), math.floor(my / DLAT)), p0, p1)
    return cells


def quantize(cell, p):
    ix, iy = cell
    x = round((p[0] - ix * DLON) / DLON * 65535)
    y = round((p[1] - iy * DLAT) / DLAT * 65535)
    return min(max(x, 0), 65535), min(max(y, 0), 65535)



# --- area layers --------------------------------------------------------------
T_FOREST, T_WATER, T_PARK, T_BUILTUP, T_BUILDING = 0, 1, 2, 3, 4
AREA_DRAW_ORDER = {T_BUILTUP: 0, T_PARK: 1, T_FOREST: 2, T_WATER: 3, T_BUILDING: 4}
# polygons smaller than this (m^2, bounding box) are left out; simplification in m
AREA_MIN_M2 = {T_FOREST: 3000.0, T_WATER: 400.0, T_PARK: 800.0, T_BUILTUP: 4000.0, T_BUILDING: 0.0}
AREA_SIMPLIFY_M = {T_FOREST: 6.0, T_WATER: 4.0, T_PARK: 4.0, T_BUILTUP: 8.0, T_BUILDING: 0.6}
PARK_LANDUSE = {'grass', 'village_green', 'recreation_ground'}
BUILTUP_LANDUSE = {'residential', 'commercial', 'industrial', 'retail'}
WATER_LANDUSE = {'reservoir', 'basin'}


def area_type(tags):
    b = tags.get('building')
    if b is not None and b != 'no':
        return T_BUILDING
    nat, lu = tags.get('natural'), tags.get('landuse')
    if nat == 'wood' or lu == 'forest':
        return T_FOREST
    if nat == 'water' or lu in WATER_LANDUSE or tags.get('waterway') == 'riverbank':
        return T_WATER
    if tags.get('leisure') == 'park' or lu in PARK_LANDUSE:
        return T_PARK
    if lu in BUILTUP_LANDUSE:
        return T_BUILTUP
    return None


def clip_polygon(poly, x0, y0, x1, y1):
    """Sutherland-Hodgman against the rectangle [x0,x1] x [y0,y1]."""
    def clip(pts, inside, inter):
        out = []
        if not pts:
            return out
        prev = pts[-1]
        for cur in pts:
            if inside(cur):
                if not inside(prev):
                    out.append(inter(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(inter(prev, cur))
            prev = cur
        return out

    def ix(p, q, x):
        t = (x - p[0]) / (q[0] - p[0])
        return (x, p[1] + t * (q[1] - p[1]))

    def iy(p, q, y):
        t = (y - p[1]) / (q[1] - p[1])
        return (p[0] + t * (q[0] - p[0]), y)

    pts = clip(poly, lambda p: p[0] >= x0, lambda p, q: ix(p, q, x0))
    pts = clip(pts, lambda p: p[0] <= x1, lambda p, q: ix(p, q, x1))
    pts = clip(pts, lambda p: p[1] >= y0, lambda p, q: iy(p, q, y0))
    pts = clip(pts, lambda p: p[1] <= y1, lambda p, q: iy(p, q, y1))
    return pts


def ring_to_polygon(ring, typ):
    """Node ring -> simplified list of (lon, lat), or None if too small."""
    try:
        ll = [(n.lon, n.lat) for n in ring if n.location.valid()]
    except osmium.InvalidLocationError:
        return None
    if len(ll) > 1 and ll[0] == ll[-1]:
        ll.pop()
    if len(ll) < 3:
        return None
    if outside_bbox([p[0] for p in ll], [p[1] for p in ll]):
        return None
    lat0 = ll[0][1]
    kx = 111320.0 * math.cos(math.radians(lat0))
    ky = 110574.0
    m = [(x * kx, y * ky) for x, y in ll]
    xs = [p[0] for p in m]
    ys = [p[1] for p in m]
    if (max(xs) - min(xs)) * (max(ys) - min(ys)) < AREA_MIN_M2[typ]:
        return None
    # simplify as a closed ring (repeat the first point as the last)
    idx = simplify(m + [m[0]], AREA_SIMPLIFY_M[typ])
    idx = [i for i in idx if i < len(ll)]
    if len(idx) < 3:
        return None
    return [ll[i] for i in idx]


def add_polygon(tiles, poly, typ, hole):
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    ix0, ix1 = math.floor(min(xs) / DLON), math.floor(max(xs) / DLON)
    iy0, iy1 = math.floor(min(ys) / DLAT), math.floor(max(ys) / DLAT)
    for ix in range(ix0, ix1 + 1):
        for iy in range(iy0, iy1 + 1):
            if ix0 == ix1 and iy0 == iy1:
                part = poly
            else:
                part = clip_polygon(poly, ix * DLON, iy * DLAT, (ix + 1) * DLON, (iy + 1) * DLAT)
            if len(part) < 3:
                continue
            q = [quantize((ix, iy), p) for p in part]
            tiles[(ix, iy)].append((typ, 1 if hole else 0, q))


def convert_areas(pbf, outdir, name, buildings):
    areas = defaultdict(list)       # roads-style layer 'a'
    blds = defaultdict(list)        # layer 'b'
    t0 = time.time()
    n = 0
    fp = (osmium.FileProcessor(pbf)
          .with_locations()
          .with_areas()
          .with_filter(osmium.filter.KeyFilter('landuse', 'natural', 'leisure', 'building', 'waterway')))
    for o in fp:
        if not o.is_area():
            continue
        typ = area_type(o.tags)
        if typ is None or (typ == T_BUILDING and not buildings):
            continue
        n += 1
        target = blds if typ == T_BUILDING else areas
        try:
            for outer in o.outer_rings():
                poly = ring_to_polygon(outer, typ)
                if poly is None:
                    continue
                add_polygon(target, poly, typ, False)
                if typ != T_BUILDING:
                    for inner in o.inner_rings(outer):
                        hp = ring_to_polygon(inner, typ)
                        if hp is not None:
                            add_polygon(target, hp, typ, True)
        except osmium.InvalidLocationError:
            continue
        if n % 200000 == 0:
            print(f'  {n} areas, {time.time() - t0:.0f}s', flush=True)
    print(f'areas read: {n} ({time.time() - t0:.0f}s)')

    root = os.path.join(outdir, name)
    for layer, ext, magic, tiles in (('a', 'a', 'A', areas), ('b', 'b', 'B', blds)):
        if layer == 'b' and not buildings:
            continue
        sizes = []
        bw = BundleWriter(os.path.join(root, layer))
        for (ix, iy), polys in tiles.items():
            polys.sort(key=lambda p: (AREA_DRAW_ORDER[p[0]], p[1]))
            body = bytearray()
            for typ, hole, q in polys:
                body += struct.pack('<BBH', typ, hole, len(q))
                body += b''.join(struct.pack('<HH', x, y) for x, y in q)
            data = struct.pack('<BBH', ord(magic), 1, len(polys)) + bytes(body)
            bw.add(ix, iy, data)
            sizes.append(len(data))
        bw.close()
        if sizes:
            sizes.sort()
            print(f'layer {layer}: {len(sizes)} tiles, {sum(sizes) / 1e6:.1f} MB; '
                  f'median {sizes[len(sizes) // 2] / 1024:.0f} KB, '
                  f'p95 {sizes[int(len(sizes) * 0.95)] / 1024:.0f} KB, max {sizes[-1] / 1024:.0f} KB')


BBOX = None      # (minlat, minlon, maxlat, maxlon): leave out everything outside, set by --bbox


def outside_bbox(lons, lats):
    """True if the whole object lies outside the --bbox area."""
    if BBOX is None:
        return False
    return (max(lats) < BBOX[0] or min(lats) > BBOX[2] or max(lons) < BBOX[1] or min(lons) > BBOX[3])


def convert(pbf, outdir, name):
    tiles = defaultdict(list)   # (ix, iy) -> [(cls, flags, name, [(x,y),...]), ...]
    n_ways = n_kept = n_pts_in = n_pts_out = 0
    t0 = time.time()

    fp = (osmium.FileProcessor(pbf)
          .with_locations()
          .with_filter(osmium.filter.KeyFilter('highway', 'railway', 'waterway')))
    minlat = minlon = 1e9
    maxlat = maxlon = -1e9

    for o in fp:
        if not o.is_way():
            continue
        n_ways += 1
        tags = o.tags
        cls = way_class(tags)
        if cls is None:
            continue
        try:
            ll = [(n.lon, n.lat) for n in o.nodes if n.location.valid()]
        except osmium.InvalidLocationError:
            continue
        if len(ll) < 2:
            continue
        if outside_bbox([p[0] for p in ll], [p[1] for p in ll]):
            continue
        n_pts_in += len(ll)

        # simplify in metres around the way's own latitude
        lat0 = ll[0][1]
        kx = 111320.0 * math.cos(math.radians(lat0))
        ky = 110574.0
        m = [(x * kx, y * ky) for x, y in ll]
        idx = simplify(m, SIMPLIFY_M)
        ll = [ll[i] for i in idx]

        flags = 0
        if tags.get('oneway') in ('yes', '1', 'true') or tags.get('junction') == 'roundabout' \
                or tags.get('highway') in ('motorway', 'motorway_link'):
            flags |= F_ONEWAY
        if tags.get('bridge') not in (None, 'no'):
            flags |= F_BRIDGE
        if tags.get('tunnel') not in (None, 'no'):
            flags |= F_TUNNEL
        nm = tags.get('name') or tags.get('ref')
        if nm is not None:
            nm = nm.encode('utf-8')[:60].decode('utf-8', 'ignore')

        for cell, polys in split_into_cells(ll).items():
            for poly in polys:
                if len(poly) < 2:
                    continue
                q = [quantize(cell, p) for p in poly]
                tiles[cell].append((cls, flags, nm, q))
                n_pts_out += len(q)
        n_kept += 1
        for x, y in ll:
            minlat, maxlat = min(minlat, y), max(maxlat, y)
            minlon, maxlon = min(minlon, x), max(maxlon, x)

        if n_ways % 100000 == 0:
            print(f'  {n_ways} ways, {len(tiles)} tiles, {time.time() - t0:.0f}s', flush=True)

    print(f'read {n_ways} ways, kept {n_kept}; points {n_pts_in} -> {n_pts_out} '
          f'({time.time() - t0:.0f}s)')

    # write tiles
    root = os.path.join(outdir, name)
    os.makedirs(os.path.join(root, 'm'), exist_ok=True)
    sizes = []
    mw = BundleWriter(os.path.join(root, 'm'))
    for (ix, iy), ways in tiles.items():
        ways.sort(key=lambda w: -w[0])      # minor first, major last
        # classes are numbered most important first, so sort descending = minor first
        strings, sidx = [], {}
        body = bytearray()
        for cls, flags, nm, q in ways:
            if nm is None:
                ni = 0xFFFF
            else:
                ni = sidx.get(nm)
                if ni is None:
                    ni = sidx[nm] = len(strings)
                    strings.append(nm.encode('utf-8'))
            body += struct.pack('<BBHH', cls, flags, ni, len(q))
            body += b''.join(struct.pack('<HH', x, y) for x, y in q)
        head = struct.pack('<BBHH', ord('M'), 1, len(ways), len(strings))
        tail = b''.join(bytes([len(s)]) + s for s in strings)
        data = head + bytes(body) + tail

        mw.add(ix, iy, data)
        sizes.append(len(data))
    mw.close()

    meta = {
        'format': 2,
        'bundleSide': SIDE,
        'name': name,
        'tileDLatE6': TILE_DLAT_E6,
        'tileDLonE6': TILE_DLON_E6,
        'bbox': [round(minlat, 5), round(minlon, 5), round(maxlat, 5), round(maxlon, 5)],
        'tiles': len(sizes),
        'bytes': sum(sizes),
        'source': os.path.basename(pbf),
    }
    with open(os.path.join(root, 'region.json'), 'w') as f:
        json.dump(meta, f, indent=1)

    sizes.sort()
    print(f'{len(sizes)} tiles, {sum(sizes) / 1e6:.1f} MB total; '
          f'median {sizes[len(sizes) // 2] / 1024:.0f} KB, '
          f'p95 {sizes[int(len(sizes) * 0.95)] / 1024:.0f} KB, max {sizes[-1] / 1024:.0f} KB')
    return meta


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pbf')
    ap.add_argument('out', help='output directory (the region folder is created inside)')
    ap.add_argument('--name', default=None, help='region folder name (default: pbf file name)')
    ap.add_argument('--no-areas', action='store_true', help='skip forest / water / park polygons')
    ap.add_argument('--buildings', action='store_true',
                    help='also write the building layer (about ten times the size of the roads)')
    ap.add_argument('--areas-only', action='store_true', help='only the area layers, keep the road tiles as they are')
    ap.add_argument('--bbox', default=None, help='minlat,minlon,maxlat,maxlon - only this area (a small test region)')
    a = ap.parse_args()
    if a.bbox:
        BBOX = tuple(float(v) for v in a.bbox.split(','))
    nm = a.name or os.path.basename(a.pbf).split('.')[0]
    if not a.areas_only:
        convert(a.pbf, a.out, nm)
    if not a.no_areas or a.buildings:
        convert_areas(a.pbf, a.out, nm, a.buildings)

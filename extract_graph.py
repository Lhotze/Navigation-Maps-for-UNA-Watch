#!/usr/bin/env python3
"""
Offline Navigation - routing, step 1: road graph from an OpenStreetMap extract.

Reads a .osm.pbf and writes a plain graph (binary files) that the contraction
tool (build_ch) turns into the routing tiles for the watch:

    <out>/graph/nodes.bin    u32 n, then n x (i32 lat, i32 lon)    degrees * 1e7
    <out>/graph/edges.bin    u32 n, then n x Edge (40 bytes, see EDGE_DTYPE)
    <out>/graph/shapes.bin   i32 pairs (lat, lon), the geometry of every edge
    <out>/graph/names.txt    one road name per line (edge.name is the line number, 0 = none)
    <out>/graph/info.json

There is no car profile: roads only cars may use (motorways, trunk roads) are left out.

Nodes are the junctions and ends of roads. An edge is the stretch of one OSM
way between two nodes; it keeps its shape points. Edges longer than
MAX_EDGE_M are cut into pieces by extra nodes, so that on the watch the road
nearest to a position is always found in the tiles around it. Each edge carries the cost
(in 0.1 s) of using it in each direction for two profiles - bicycle and
pedestrian - or INF (0xFFFFFFFF) where the profile may not use it:

    cost[0] bike fwd   cost[1] bike bwd   cost[2] foot fwd   cost[3] foot bwd
"fwd" runs from node a to node b along the way's own direction.

The costs are what makes the routes behave: bicycle = prefers
cycle paths and quiet roads and avoids big roads, pedestrian = avoids big roads
and uses paths. Turn restrictions are NOT modelled (one-ways are).
"""

import argparse
import json
import math
import os
import re
import sys
import time
from array import array

import numpy as np
import osmium

import convert_map
from convert_map import HIGHWAY_CLASS, C_PATH, C_CYCLE

INF = 0xFFFFFFFF
MAX_EDGE_M = 500.0     # longer edges are split
PIECE_M = 400.0       # about this long

EDGE_DTYPE = np.dtype([
    ('a', '<u4'), ('b', '<u4'),
    ('length', '<f4'),            # metres
    ('shape', '<u4'),             # first point in shapes.bin (index of the pair)
    ('nshape', '<u2'),            # number of points, including both ends
    ('cls', 'u1'),                # road class as in the map tiles
    ('flags', 'u1'),
    ('name', '<u4'),
    ('cost', '<u4', (4,)),
])
assert EDGE_DTYPE.itemsize == 40

F_ROUNDABOUT, F_BRIDGE, F_TUNNEL = 1, 2, 4

CAR_CLASSES = {'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link',
               'secondary', 'secondary_link', 'tertiary', 'tertiary_link', 'unclassified',
               'residential', 'living_street', 'service', 'road'}
BIG_ROADS = {'motorway', 'motorway_link', 'trunk', 'trunk_link'}
# ways that can be part of any profile's network
ROUTABLE = set(HIGHWAY_CLASS) | {'bridleway'}

CAR_DEFAULT_KMH = {
    'motorway': 105, 'motorway_link': 60, 'trunk': 85, 'trunk_link': 50,
    'primary': 70, 'primary_link': 45, 'secondary': 62, 'secondary_link': 40,
    'tertiary': 50, 'tertiary_link': 35, 'unclassified': 45, 'residential': 30,
    'living_street': 8, 'service': 18, 'road': 30, 'track': 15,
}
CAR_FRACTION = 0.8          # real traffic is slower than the limit
CAR_EDGE_PENALTY_S = 2.0    # junction / turn cost per edge
YES = ('yes', 'true', '1', 'designated', 'permissive')
DENY = ('no', 'private')


def parse_maxspeed(v):
    if not v:
        return None
    v = v.strip().lower()
    if v in ('none', 'signals'):
        return 130.0
    if v == 'walk':
        return 7.0
    m = re.match(r'(\d+(?:\.\d+)?)\s*(mph)?', v)
    if not m:
        return None
    x = float(m.group(1))
    return x * 1.609 if m.group(2) else x


def first(tags, keys):
    for k in keys:
        v = tags.get(k)
        if v is not None:
            return v
    return None


def oneway_dirs(tags, hw):
    """(forward allowed, backward allowed) for motor vehicles."""
    ow = tags.get('oneway')
    if ow in ('yes', 'true', '1'):
        return True, False
    if ow == '-1' or ow == 'reverse':
        return False, True
    if ow == 'no':
        return True, True
    if tags.get('junction') in ('roundabout', 'circular') or hw in ('motorway', 'motorway_link'):
        return True, False
    return True, True


def bike_contraflow(tags):
    if tags.get('oneway:bicycle') == 'no':
        return True
    for k in ('cycleway', 'cycleway:left', 'cycleway:right', 'cycleway:both'):
        v = tags.get(k, '')
        if v.startswith('opposite'):
            return True
    return tags.get('bicycle') in ('yes', 'designated') and tags.get('oneway') in ('yes', '-1') \
        and tags.get('oneway:bicycle') != 'yes' and tags.get('highway') in ('cycleway', 'path', 'track')


def has_cycle_infra(tags):
    for k in ('cycleway', 'cycleway:left', 'cycleway:right', 'cycleway:both'):
        if tags.get(k) in ('lane', 'track', 'shared_busway', 'share_busway', 'opposite_lane', 'opposite_track'):
            return True
    return tags.get('bicycle') == 'designated'


def surface_factor(tags, mode):
    s = tags.get('surface')
    sm = tags.get('smoothness')
    f = 1.0
    if s in ('sett', 'cobblestone', 'unhewn_cobblestone', 'cobblestone:flattened', 'paving_stones:lanes'):
        f = 0.65 if mode == 'bike' else 0.9
    elif s in ('compacted', 'fine_gravel', 'pebblestone'):
        f = 0.85 if mode == 'bike' else 0.95
    elif s in ('gravel', 'ground', 'dirt', 'earth', 'sand', 'grass', 'mud', 'unpaved', 'woodchips', 'grass_paver'):
        f = 0.55 if mode == 'bike' else 0.8
    if sm in ('bad', 'very_bad', 'horrible', 'very_horrible', 'impassable'):
        f *= 0.7 if mode == 'bike' else 0.85
    return f


def bike_speed(tags, hw):
    """km/h as an effective figure (it carries the preferences), None if forbidden."""
    acc = first(tags, ('bicycle', 'vehicle', 'access'))
    bic = tags.get('bicycle')
    if acc in DENY and bic not in YES:
        return None
    if bic == 'dismount':
        return 4.0
    if hw in BIG_ROADS:
        if bic in YES:
            return 10.0 * surface_factor(tags, 'bike')
        return None
    if hw == 'cycleway':
        v = 20.0
    elif hw in ('residential', 'unclassified', 'road'):
        v = 17.0
    elif hw == 'living_street':
        v = 13.0
    elif hw == 'service':
        v = 14.0
    elif hw == 'pedestrian':
        v = 8.0 if bic in YES else 4.5
    elif hw == 'track':
        tt = tags.get('tracktype', '')
        v = {'grade1': 17.0, 'grade2': 14.0, 'grade3': 11.0, 'grade4': 7.0, 'grade5': 6.0}.get(tt, 12.0)
    elif hw in ('path', 'bridleway'):
        v = 15.0 if bic == 'designated' else (12.0 if bic in YES else 10.0)
    elif hw == 'footway':
        v = 12.0 if bic in YES else 4.5      # pushing the bike
    elif hw == 'steps':
        v = 1.5
    elif hw in ('tertiary', 'tertiary_link'):
        v = 15.0
    elif hw in ('secondary', 'secondary_link'):
        v = 10.5
    elif hw in ('primary', 'primary_link'):
        v = 7.5
    else:
        v = 12.0
    if hw in ('tertiary', 'tertiary_link', 'secondary', 'secondary_link', 'primary', 'primary_link') \
            and has_cycle_infra(tags):
        v = min(v * 1.6, 17.0)
    return v * surface_factor(tags, 'bike')


def foot_speed(tags, hw):
    acc = first(tags, ('foot', 'access'))
    foot = tags.get('foot')
    if acc in DENY and foot not in YES:
        return None
    if hw in BIG_ROADS:
        return None
    if hw == 'cycleway':
        if foot == 'no':
            return None
        v = 4.2
    elif hw in ('footway', 'pedestrian', 'path', 'living_street', 'residential', 'unclassified', 'road',
                'bridleway'):
        v = 5.0
    elif hw == 'steps':
        v = 3.0
    elif hw == 'service':
        v = 4.6
    elif hw == 'track':
        v = 4.6
    elif hw in ('tertiary', 'tertiary_link'):
        v = 4.4
    elif hw in ('secondary', 'secondary_link'):
        v = 3.4
    elif hw in ('primary', 'primary_link'):
        v = 2.4
    else:
        v = 4.5
    return v * surface_factor(tags, 'foot')


def deci(length_m, kmh, extra_s=0.0):
    if kmh is None or kmh <= 0:
        return INF
    return int(min(0xFFFFFFFE, round((length_m / (kmh / 3.6) + extra_s) * 10)))


def edge_costs(tags, hw, length_m):
    """The four costs of an edge, see the module docstring."""
    fwd_ok, bwd_ok = oneway_dirs(tags, hw)
    out = [INF] * 4

    v = bike_speed(tags, hw)
    if v is not None:
        c = deci(length_m, v, 0.5)
        both = bike_contraflow(tags)
        out[0] = c if (fwd_ok or both) else INF
        out[1] = c if (bwd_ok or both) else INF

    v = foot_speed(tags, hw)
    if v is not None:
        c = deci(length_m, v)
        out[2] = out[3] = c
    return out


def seg_length(lat, lon):
    """Length in metres of a polyline given as arrays of degrees."""
    kx = 111320.0 * math.cos(math.radians(float(lat[0])))
    dx = np.diff(lon) * kx
    dy = np.diff(lat) * 110574.0
    return float(np.sum(np.hypot(dx, dy)))


def extract(pbf, outdir):
    t0 = time.time()
    os.makedirs(outdir, exist_ok=True)

    # --- pass 1: which nodes are junctions / ends ---------------------------------
    refs = array('q')
    ends = array('q')
    nways = 0
    fp = osmium.FileProcessor(pbf, osmium.osm.WAY).with_filter(osmium.filter.KeyFilter('highway'))
    for w in fp:
        hw = w.tags.get('highway')
        if hw not in ROUTABLE or w.tags.get('area') == 'yes':
            continue
        r = [n.ref for n in w.nodes]
        if len(r) < 2:
            continue
        nways += 1
        refs.extend(r)
        ends.append(r[0])
        ends.append(r[-1])
    refs_np = np.frombuffer(refs, dtype=np.int64)
    uniq, counts = np.unique(refs_np, return_counts=True)
    junction = uniq[counts >= 2]
    rn = np.union1d(junction, np.unique(np.frombuffer(ends, dtype=np.int64)))
    del refs, refs_np, uniq, counts
    print(f'pass 1: {nways} ways, {len(rn)} routing nodes ({time.time() - t0:.0f}s)', flush=True)

    nlat = np.zeros(len(rn), dtype=np.int32)
    nlon = np.zeros(len(rn), dtype=np.int32)
    have = np.zeros(len(rn), dtype=bool)

    # --- pass 2: edges -------------------------------------------------------------
    ea, eb = array('I'), array('I')
    elen = array('f')
    eshape, enshape = array('I'), array('H')
    ecls, eflags = array('B'), array('B')
    ename = array('I')
    ecost = array('I')
    shapes = array('i')
    names = ['']
    name_id = {'': 0}
    nshape_pts = 0
    extra_lat, extra_lon = [], []      # nodes added inside long edges

    fp = (osmium.FileProcessor(pbf, osmium.osm.NODE | osmium.osm.WAY)
          .with_locations()
          .with_filter(osmium.filter.KeyFilter('highway')))
    done = 0
    for w in fp:
        if not w.is_way():       # nodes with a highway tag (signals, crossings) come through the filter too
            continue
        tags = w.tags
        hw = tags.get('highway')
        if hw not in ROUTABLE or tags.get('area') == 'yes':
            continue
        try:
            pts = [(n.ref, n.lat, n.lon) for n in w.nodes if n.location.valid()]
        except osmium.InvalidLocationError:
            continue
        if len(pts) < 2:
            continue
        if convert_map.outside_bbox([p[2] for p in pts], [p[1] for p in pts]):
            continue
        ref = np.array([p[0] for p in pts], dtype=np.int64)
        lat = np.array([p[1] for p in pts])
        lon = np.array([p[2] for p in pts])
        idx = np.searchsorted(rn, ref)
        idx[idx >= len(rn)] = len(rn) - 1
        is_r = rn[idx] == ref
        brk = np.nonzero(is_r)[0]
        if len(brk) < 2:
            continue

        # a node may be recorded once; all copies have the same position
        sel = brk
        nid = idx[sel]
        new = ~have[nid]
        if new.any():
            nlat[nid[new]] = np.round(lat[sel][new] * 1e7).astype(np.int32)
            nlon[nid[new]] = np.round(lon[sel][new] * 1e7).astype(np.int32)
            have[nid[new]] = True

        cls = HIGHWAY_CLASS.get(hw, C_PATH)
        flags = 0
        if tags.get('junction') in ('roundabout', 'circular'):
            flags |= F_ROUNDABOUT
        if tags.get('bridge') not in (None, 'no'):
            flags |= F_BRIDGE
        if tags.get('tunnel') not in (None, 'no'):
            flags |= F_TUNNEL
        nm = tags.get('name') or tags.get('ref') or ''
        nid_name = name_id.get(nm)
        if nid_name is None:
            nid_name = name_id[nm] = len(names)
            names.append(nm)

        # cumulative length along the way, to cut long stretches
        kx = 111320.0 * math.cos(math.radians(float(lat[0])))
        seg = np.hypot(np.diff(lon) * kx, np.diff(lat) * 110574.0)
        cum = np.concatenate([[0.0], np.cumsum(seg)])

        for s, e in zip(brk[:-1], brk[1:]):
            total = float(cum[e] - cum[s])
            if total <= 0.0 and idx[s] == idx[e]:
                continue
            cuts = [s]
            if total > MAX_EDGE_M and e - s >= 2:
                pieces = int(math.ceil(total / PIECE_M))
                for k in range(1, pieces):
                    want = cum[s] + total * k / pieces
                    i = int(np.searchsorted(cum[s:e + 1], want - cum[s])) + s
                    i = min(max(i, cuts[-1] + 1), e - 1)
                    if i > cuts[-1] and i < e:
                        cuts.append(i)
            cuts.append(e)

            # node ids of the cut points (extra nodes get ids after the routing nodes)
            ids = []
            for ci, p in enumerate(cuts):
                if ci == 0:
                    ids.append(int(idx[s]))
                elif ci == len(cuts) - 1:
                    ids.append(int(idx[e]))
                else:
                    ids.append(len(rn) + len(extra_lat))
                    extra_lat.append(int(round(lat[p] * 1e7)))
                    extra_lon.append(int(round(lon[p] * 1e7)))

            for (p, q), ia, ib in zip(zip(cuts[:-1], cuts[1:]), ids[:-1], ids[1:]):
                la, lo = lat[p:q + 1], lon[p:q + 1]
                length = float(cum[q] - cum[p])
                costs = edge_costs(tags, hw, length)
                if all(c == INF for c in costs):
                    continue
                ea.append(ia)
                eb.append(ib)
                elen.append(length)
                eshape.append(nshape_pts)
                enshape.append(len(la))
                ecls.append(cls)
                eflags.append(flags)
                ename.append(nid_name)
                ecost.extend(costs)
                for y, x in zip(la, lo):
                    shapes.append(int(round(y * 1e7)))
                    shapes.append(int(round(x * 1e7)))
                nshape_pts += len(la)
        done += 1
        if done % 100000 == 0:
            print(f'  {done} ways, {len(ea)} edges, {time.time() - t0:.0f}s', flush=True)

    nlat = np.concatenate([nlat, np.array(extra_lat, dtype=np.int32)])
    nlon = np.concatenate([nlon, np.array(extra_lon, dtype=np.int32)])
    n_edges = len(ea)
    edges = np.zeros(n_edges, dtype=EDGE_DTYPE)
    edges['a'] = np.frombuffer(ea, dtype=np.uint32)
    edges['b'] = np.frombuffer(eb, dtype=np.uint32)
    edges['length'] = np.frombuffer(elen, dtype=np.float32)
    edges['shape'] = np.frombuffer(eshape, dtype=np.uint32)
    edges['nshape'] = np.frombuffer(enshape, dtype=np.uint16)
    edges['cls'] = np.frombuffer(ecls, dtype=np.uint8)
    edges['flags'] = np.frombuffer(eflags, dtype=np.uint8)
    edges['name'] = np.frombuffer(ename, dtype=np.uint32)
    edges['cost'] = np.frombuffer(ecost, dtype=np.uint32).reshape(n_edges, 4)

    # keep only nodes that have an edge, renumber
    used = np.zeros(len(nlat), dtype=bool)
    used[edges['a']] = True
    used[edges['b']] = True
    newid = np.cumsum(used) - 1
    edges['a'] = newid[edges['a']]
    edges['b'] = newid[edges['b']]
    nlat, nlon = nlat[used], nlon[used]
    print(f'pass 2: {n_edges} edges, {len(nlat)} nodes, {nshape_pts} shape points '
          f'({time.time() - t0:.0f}s)', flush=True)

    g = os.path.join(outdir, 'graph')
    os.makedirs(g, exist_ok=True)
    with open(os.path.join(g, 'nodes.bin'), 'wb') as f:
        f.write(np.uint32(len(nlat)).tobytes())
        f.write(np.stack([nlat, nlon], axis=1).astype('<i4').tobytes())
    with open(os.path.join(g, 'edges.bin'), 'wb') as f:
        f.write(np.uint32(n_edges).tobytes())
        f.write(edges.tobytes())
    with open(os.path.join(g, 'shapes.bin'), 'wb') as f:
        f.write(np.frombuffer(shapes, dtype=np.int32).tobytes())
    with open(os.path.join(g, 'names.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(n.replace('\n', ' ') for n in names))
    bike = int(np.sum((edges['cost'][:, 0] != INF) | (edges['cost'][:, 1] != INF)))
    foot = int(np.sum((edges['cost'][:, 2] != INF) | (edges['cost'][:, 3] != INF)))
    info = {'nodes': int(len(nlat)), 'edges': n_edges, 'shapePoints': int(nshape_pts),
            'bikeEdges': bike, 'footEdges': foot, 'source': os.path.basename(pbf)}
    with open(os.path.join(g, 'info.json'), 'w') as f:
        json.dump(info, f, indent=1)
    print(info)
    print(f'done in {time.time() - t0:.0f}s')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pbf')
    ap.add_argument('out', help='output directory (a graph/ folder is created inside)')
    ap.add_argument('--bbox', default=None, help='minlat,minlon,maxlat,maxlon - only this area (a small test region)')
    a = ap.parse_args()
    if a.bbox:
        convert_map.BBOX = tuple(float(v) for v in a.bbox.split(','))
    extract(a.pbf, a.out)

#!/usr/bin/env python3
"""
Offline Navigation - routing, step 3: pack graph + hierarchies into tile files.

  pack_routing.py <graph dir> <out region dir> [--profiles bike,foot] [--ch-dir DIR]

Input : the files of extract_graph.py (nodes, edges, shapes, names) and one
        ch_<profile>.bin per profile from build_ch.
Output: <region>/route/index.bin          which tile has which nodes / edges
        <region>/route/g/<bx>/<by>.pak    geometry of the tiles (all profiles), 8 x 8 tiles per bundle (see pak.py)
        <region>/route/c0|c1/<bx>/<by>.pak contraction hierarchy arcs, per profile (0 bicycle, 1 on foot)

The tile grid is the one of the map tiles (0.02 deg x 0.03 deg). Nodes and edges
are numbered tile by tile, so the id alone tells which file holds it: a binary
search over index.bin (a few KB in memory) does it. An edge belongs to the tile
of its first node; since no edge is longer than 500 m, the road nearest to any
position is always in the 3x3 tiles around it.

index.bin  (little endian)
    'RIX2', u32 tiles, u32 nodes, u32 edges, u32 samples,
    samples x { i32 ix, i32 iy, u32 firstNode, u32 firstEdge }     every 64th tile; the watch keeps these only
    tiles x { i32 ix, i32 iy, u32 firstNode, u32 nodeCount, u32 firstEdge, u32 edgeCount }
    sorted by (ix, iy) - which is also the order of node ids and edge ids.

g tile
    u8 'G', u8 1, u16 nNodes, u16 nEdges, u16 nStrings, u32 stringsOffset
    nodes[nNodes]  : i32 lat, i32 lon, u8 degree, 3 x pad    (degrees * 1e7; degree = roads meeting here)
    edges[nEdges]  : 42 bytes
        u32 a, u32 b            global node ids (a is in this tile)
        i32 aLat, i32 aLon      position of a
        i32 bLat, i32 bLon      position of b (it may lie in a neighbour tile)
        u16 lengthDm            length in decimetres
        u8  cls, u8 flags       road class as in the map tiles; flags as extract_graph
        u8  nInner              shape points strictly between a and b
        u8  mainMask            bit 0 bike, 1 foot: the edge belongs to the big connected network of that
                                profile (a position must be snapped to such an edge, not to an isolated piece)
        u16 nameIdx             index into the string table, 0xFFFF = none
        u16 innerOffset         ... in units of 4 bytes, from the start of the inner-point area
        u16 cost[4]             travel cost of the whole edge in 0.1 s: bike fwd, bike bwd,
                                foot fwd, foot bwd; 0xFFFF = not usable
    inner points   : per edge nInner x (i16 dLat, i16 dLon), each relative to the previous point
                     (the first relative to a), in units of 1e-6 degrees
    strings        : u16 offsets[nStrings] (from the first string), then nStrings x (u8 length, UTF-8 bytes)
  The parts are laid out one after the other: nodes, edges, inner points, strings. The watch never
  loads a whole tile (a city tile can be 500 KB): it reads the edges in blocks, and fetches the
  inner points / name of the few edges it needs.

c tile
    u8 'C', u8 1, u16 nNodes, u16 0
    u32 offsets[nNodes + 1]            byte offsets of the node records from the start of the tile
    per node: u16 nOut, u16 nIn, then nOut + nIn arcs { u32 to, u32 weight, u32 via }
    out arcs lead to higher-ranked nodes ('to'), in arcs come from higher-ranked nodes ('to' = source).
    via: high bit set = original edge (bit 0 direction 0 = a->b, bits 1..30 = global edge id);
         otherwise the global id of the middle node of a shortcut.
"""

import argparse
import json
import os
import struct
import sys
import time

import numpy as np

from pak import BundleWriter, SIDE

from extract_graph import EDGE_DTYPE, INF

DLAT7 = 200000       # 0.02 deg in 1e-7 deg
DLON7 = 300000       # 0.03 deg
ORIG = 0x80000000
PROFILES = ['bike', 'foot']


def load_graph(gdir):
    with open(os.path.join(gdir, 'nodes.bin'), 'rb') as f:
        n = int(np.frombuffer(f.read(4), dtype='<u4')[0])
        nodes = np.frombuffer(f.read(n * 8), dtype='<i4').reshape(n, 2)
    with open(os.path.join(gdir, 'edges.bin'), 'rb') as f:
        m = int(np.frombuffer(f.read(4), dtype='<u4')[0])
        edges = np.frombuffer(f.read(m * EDGE_DTYPE.itemsize), dtype=EDGE_DTYPE)
    shapes = np.fromfile(os.path.join(gdir, 'shapes.bin'), dtype='<i4').reshape(-1, 2)
    with open(os.path.join(gdir, 'names.txt'), encoding='utf-8') as f:
        names = f.read().split('\n')
    return nodes, edges, shapes, names


def load_ch(path):
    with open(path, 'rb') as f:
        assert f.read(4) == b'CHG1'
        n, nout, nin = np.frombuffer(f.read(12), dtype='<u4')
        n, nout, nin = int(n), int(nout), int(nin)
        rank = np.frombuffer(f.read(4 * n), dtype='<u4')
        outOff = np.frombuffer(f.read(4 * (n + 1)), dtype='<u4')
        inOff = np.frombuffer(f.read(4 * (n + 1)), dtype='<u4')
        outUp = np.frombuffer(f.read(12 * nout), dtype='<u4').reshape(nout, 3)
        inUp = np.frombuffer(f.read(12 * nin), dtype='<u4').reshape(nin, 3)
    return rank, outOff, inOff, outUp, inUp


def regroup(off, arcs, order):
    """CSR rows re-ordered: new row i = old row order[i]."""
    start = off[order].astype(np.int64)
    ln = (off[order + 1].astype(np.int64) - start)
    new_off = np.concatenate([[0], np.cumsum(ln)])
    total = int(new_off[-1])
    idx = np.repeat(start - new_off[:-1], ln) + np.arange(total, dtype=np.int64)
    return new_off, arcs[idx]


def inner_points(shapes, e_shape, e_n, a_lat, a_lon):
    """Inner shape points of an edge as int16 deltas (1e-6 deg); long jumps get filler points."""
    pts = shapes[e_shape + 1:e_shape + e_n - 1]       # strictly between the ends
    out = []
    pl, po = a_lat, a_lon
    for la, lo in pts:
        dl = (int(la) - pl) / 10.0
        do = (int(lo) - po) / 10.0
        # keep each step inside int16 by inserting midpoints
        steps = int(max(abs(dl), abs(do)) // 30000) + 1
        for k in range(1, steps + 1):
            tla = pl + (int(la) - pl) * k // steps
            tlo = po + (int(lo) - po) * k // steps
            out.append((tla, tlo))
        pl, po = int(la), int(lo)
    # to deltas against the previous (rounded) point
    res = []
    cl, co = a_lat, a_lon
    for la, lo in out:
        dl = int(round((la - cl) / 10.0))
        do = int(round((lo - co) / 10.0))
        res.append((dl, do))
        cl += dl * 10
        co += do * 10
    return res


def pack(gdir, region, profiles, chdir):
    t0 = time.time()
    nodes, edges, shapes, names = load_graph(gdir)
    n = len(nodes)
    m = len(edges)
    print(f'graph: {n} nodes, {m} edges ({time.time() - t0:.0f}s)', flush=True)

    ix = np.floor_divide(nodes[:, 1].astype(np.int64), DLON7)
    iy = np.floor_divide(nodes[:, 0].astype(np.int64), DLAT7)
    # tiles in (ix, iy) order; new node ids follow
    order = np.lexsort((np.arange(n), iy, ix))          # order[new] = old
    newid = np.empty(n, dtype=np.int64)
    newid[order] = np.arange(n)

    key = ix * 100000 + iy
    skey = key[order]
    change = np.concatenate([[True], skey[1:] != skey[:-1]])
    tile_first = np.nonzero(change)[0]
    tile_count = np.diff(np.concatenate([tile_first, [n]]))
    tile_ix = ix[order][tile_first]
    tile_iy = iy[order][tile_first]
    nt = len(tile_first)
    tile_of_node_new = np.cumsum(change) - 1             # per new node: tile number

    # edges: tile of node a
    ea_tile = tile_of_node_new[newid[edges['a']]]
    eorder = np.lexsort((np.arange(m), ea_tile))         # eorder[new] = old
    enew = np.empty(m, dtype=np.int64)
    enew[eorder] = np.arange(m)
    etile_sorted = ea_tile[eorder]
    e_first = np.searchsorted(etile_sorted, np.arange(nt), side='left')
    e_count = np.searchsorted(etile_sorted, np.arange(nt), side='right') - e_first

    # --- connected networks per profile: positions may only snap to the big one
    main_mask = np.zeros(m, dtype=np.uint8)
    for pi in range(2):
        usable = (edges['cost'][:, 2 * pi] != INF) | (edges['cost'][:, 2 * pi + 1] != INF)
        parent = list(range(n))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        for a_, b_ in zip(edges['a'][usable].tolist(), edges['b'][usable].tolist()):
            ra, rb = find(a_), find(b_)
            if ra != rb:
                parent[ra] = rb
        root = np.array([find(i) for i in range(n)])
        sizes = np.bincount(root, minlength=n)
        big = max(300, int(sizes.max() * 0.02))
        ismain = sizes[root[edges['a']]] >= big
        main_mask |= np.where(usable & ismain, 1 << pi, 0).astype(np.uint8)
        print(f'profile {PROFILES[pi]}: largest network {int(sizes.max())} nodes, '
              f'{int(np.sum(sizes >= big))} networks kept ({time.time() - t0:.0f}s)', flush=True)

    route = os.path.join(region, 'route')
    os.makedirs(route, exist_ok=True)
    with open(os.path.join(route, 'index.bin'), 'wb') as f:
        samples = list(range(0, nt, 64))
        f.write(b'RIX2')
        f.write(struct.pack('<IIII', nt, n, m, len(samples)))
        for t in samples:      # every 64th tile: what the watch keeps in memory
            f.write(struct.pack('<iiII', int(tile_ix[t]), int(tile_iy[t]), int(tile_first[t]), int(e_first[t])))
        for t in range(nt):
            f.write(struct.pack('<iiIIII', int(tile_ix[t]), int(tile_iy[t]), int(tile_first[t]),
                                int(tile_count[t]), int(e_first[t]), int(e_count[t])))

    # --- geometry tiles
    sizes = []
    gw = BundleWriter(os.path.join(route, 'g'))
    gw = BundleWriter(os.path.join(route, 'g'))
    nodes_new = nodes[order]
    deg = np.zeros(n, dtype=np.int64)
    np.add.at(deg, edges['a'].astype(np.int64), 1)
    np.add.at(deg, edges['b'].astype(np.int64), 1)
    deg_new = np.minimum(deg[order], 255).astype(np.uint8)
    for t in range(nt):
        f0, fc = int(tile_first[t]), int(tile_count[t])
        g0, gc = int(e_first[t]), int(e_count[t])
        tn = nodes_new[f0:f0 + fc]
        rec = np.zeros(fc, dtype=[('lat', '<i4'), ('lon', '<i4'), ('deg', 'u1'), ('pad', 'u1', 3)])
        rec['lat'], rec['lon'], rec['deg'] = tn[:, 0], tn[:, 1], deg_new[f0:f0 + fc]
        body_nodes = rec.tobytes()
        strings, sidx = [], {}
        edge_bytes = bytearray()
        inner = bytearray()
        for k in range(gc):
            oe = edges[eorder[g0 + k]]
            a_new, b_new = int(newid[oe['a']]), int(newid[oe['b']])
            a_lat, a_lon = int(nodes[oe['a']][0]), int(nodes[oe['a']][1])
            b_lat, b_lon = int(nodes[oe['b']][0]), int(nodes[oe['b']][1])
            pts = inner_points(shapes, int(oe['shape']), int(oe['nshape']), a_lat, a_lon)
            if len(pts) > 255:                       # absurdly detailed edge: thin it out
                step = len(pts) // 255 + 1
                pts = pts[step - 1::step]
            costs = [min(int(c), 0xFFFF) if int(c) != INF else 0xFFFF for c in oe['cost']]
            costs = [0xFFFE if (c == 0xFFFF and int(oe['cost'][i]) != INF) else c for i, c in enumerate(costs)]
            nm_i = int(oe['name'])
            nm = names[nm_i] if nm_i else ''
            if nm:
                si = sidx.get(nm)
                if si is None:
                    si = sidx[nm] = len(strings)
                    strings.append(nm.encode('utf-8')[:60])
            else:
                si = 0xFFFF
            off4 = len(inner) // 4
            for dl, do in pts:
                inner += struct.pack('<hh', dl, do)
            edge_bytes += struct.pack('<IIiiiiHBBBBHH4H', a_new, b_new, a_lat, a_lon, b_lat, b_lon,
                                      min(65535, int(round(float(oe['length']) * 10))),
                                      int(oe['cls']), int(oe['flags']), len(pts), int(main_mask[eorder[g0 + k]]), si,
                                      min(off4, 65535), *costs)
        soffs, pos2 = [], 0
        for st_ in strings:
            soffs.append(pos2)
            pos2 += 1 + len(st_)
        tail = struct.pack(f'<{len(strings)}H', *soffs) + b''.join(bytes([len(x)]) + x for x in strings)
        str_off = 12 + len(body_nodes) + len(edge_bytes) + len(inner)
        head = struct.pack('<BBHHHI', ord('G'), 1, fc, gc, len(strings), str_off)
        data = head + body_nodes + bytes(edge_bytes) + bytes(inner) + tail
        gw.add(int(tile_ix[t]), int(tile_iy[t]), data)
        gw.flush_before(int(tile_ix[t]) // SIDE)         # tiles come in ix order: earlier columns are complete
        sizes.append(len(data))
        if t % 400 == 0:
            print(f'  g {t}/{nt} ({time.time() - t0:.0f}s)', flush=True)
    gw.close()
    sizes.sort()
    print(f'geometry: {nt} tiles, {sum(sizes) / 1e6:.1f} MB, median {sizes[len(sizes) // 2] / 1024:.0f} KB, '
          f'max {sizes[-1] / 1024:.0f} KB', flush=True)
    del edge_bytes, inner

    # --- hierarchy tiles per profile
    for pi, pname in enumerate(PROFILES):
        if pname not in profiles:
            continue
        rank, outOff, inOff, outUp, inUp = load_ch(os.path.join(chdir, f'ch_{pname}.bin'))
        assert len(rank) == n
        o_off, o_arcs = regroup(outOff, outUp, order)
        i_off, i_arcs = regroup(inOff, inUp, order)

        def remap(arcs):
            a = arcs.astype(np.int64)
            a[:, 0] = newid[a[:, 0]]
            via = a[:, 2]
            isorig = (via & ORIG) != 0
            eid = (via & 0x7FFFFFFF) >> 1
            dirb = via & 1
            new_via = np.where(isorig, ORIG | (enew[np.where(isorig, eid, 0)] << 1) | dirb, 0)
            nodevia = np.where(~isorig, newid[np.where(~isorig, via, 0)], 0)
            a[:, 2] = np.where(isorig, new_via, nodevia)
            return a.astype('<u4')

        o_arcs = remap(o_arcs)
        i_arcs = remap(i_arcs)
        csizes = []
        cw = BundleWriter(os.path.join(route, f'c{pi}'))
        for t in range(nt):
            f0, fc = int(tile_first[t]), int(tile_count[t])
            recs = []
            for v in range(f0, f0 + fc):
                oa = o_arcs[o_off[v]:o_off[v + 1]]
                ia = i_arcs[i_off[v]:i_off[v + 1]]
                recs.append(struct.pack('<HH', len(oa), len(ia)) + oa.tobytes() + ia.tobytes())
            head_len = 6 + 4 * (fc + 1)
            offs, pos = [], head_len
            for r in recs:
                offs.append(pos)
                pos += len(r)
            offs.append(pos)
            data = struct.pack('<BBHH', ord('C'), 1, fc, 0) + struct.pack(f'<{fc + 1}I', *offs) + b''.join(recs)
            cw.add(int(tile_ix[t]), int(tile_iy[t]), data)
            cw.flush_before(int(tile_ix[t]) // SIDE)
            csizes.append(len(data))
        cw.close()
        csizes.sort()
        print(f'{pname}: {nt} tiles, {sum(csizes) / 1e6:.1f} MB, median {csizes[len(csizes) // 2] / 1024:.0f} KB, '
              f'max {csizes[-1] / 1024:.0f} KB ({time.time() - t0:.0f}s)', flush=True)

    info = {'nodes': n, 'edges': m, 'tiles': nt, 'profiles': profiles, 'bundleSide': SIDE,
            'tileDLat7': DLAT7, 'tileDLon7': DLON7}
    with open(os.path.join(route, 'route.json'), 'w') as f:
        json.dump(info, f, indent=1)
    # keep the node/edge permutation for the test tools
    np.save(os.path.join(gdir, 'node_new_of_old.npy'), newid)
    np.save(os.path.join(gdir, 'edge_new_of_old.npy'), enew)
    print(f'done ({time.time() - t0:.0f}s)')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('graph')
    ap.add_argument('region')
    ap.add_argument('--profiles', default='bike,foot')
    ap.add_argument('--ch-dir', default=None, help='where ch_<profile>.bin are (default: the graph dir parent)')
    a = ap.parse_args()
    chd = a.ch_dir or os.path.dirname(a.graph.rstrip('/'))
    pack(a.graph, a.region, a.profiles.split(','), chd)

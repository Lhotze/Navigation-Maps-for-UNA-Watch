#!/usr/bin/env python3
"""
Offline Navigation - address search index.

  build_addr.py <pbf> <region dir> [--bbox minlat,minlon,maxlat,maxlon]

Municipalities are OSM admin_level 8 (and level 6 for cities that are a district of their own).

Writes <region>/addr/ for the search "place -> street -> house number":

  places.bin   municipalities (OSM admin_level 8), sorted by folded name
  streets.bin  named streets, grouped by municipality, sorted by folded name inside it
  numbers.bin  house numbers with their position, grouped by street
  names.bin    the display strings (UTF-8, with the umlauts) everything points to
  info.json

Folded names are what the watch matches the typed letters against: upper case,
umlauts reduced to their base letter (A O U), sharp s -> SS, accents dropped,
hyphens and dots -> space/nothing. The wheel on the watch then only needs
A-Z, 0-9 and the space.

All files are little endian; every record has a fixed size so that a record
is found by seeking, and a sorted range by binary search.

  places.bin   'APL1', u32 count, then 48-byte records:
      char fold[24], u32 nameOff, i32 lat, i32 lon (deg * 1e7),
      u32 firstStreet, u32 streetCount, u32 reserved
  streets.bin  'AST1', u32 count, then 48-byte records:
      char fold[24], u32 nameOff, u32 subOff (0xFFFFFFFF = none), i32 lat, i32 lon,
      u32 firstNumber, u32 numberCount
  numbers.bin  'ANM1', u32 count, then 16-byte records:
      char number[8] (zero padded), i32 lat, i32 lon
  names.bin    u8 length + bytes, at the offsets given above
"""

import argparse
import json
import math
import os
import struct
import sys
import time
from collections import defaultdict

import numpy as np
import osmium

NONE = 0xFFFFFFFF
FOLD_LEN = 24


def fold(s):
    """Upper case, umlauts to base letters, ss for the sharp s, only A-Z 0-9 and single spaces."""
    out = []
    for ch in s:
        c = ch
        if c in 'äÄ': c = 'A'
        elif c in 'öÖ': c = 'O'
        elif c in 'üÜ': c = 'U'
        elif c == 'ß': c = 'SS'
        elif c in 'éèêëÉÈÊË': c = 'E'
        elif c in 'àáâãåÀÁÂÃÅ': c = 'A'
        elif c in 'íìîïÍÌÎÏ': c = 'I'
        elif c in 'óòôõÓÒÔÕ': c = 'O'
        elif c in 'úùûÚÙÛ': c = 'U'
        elif c in 'çÇ': c = 'C'
        elif c in 'ñÑ': c = 'N'
        c = c.upper()
        for x in c:
            if 'A' <= x <= 'Z' or '0' <= x <= '9':
                out.append(x)
            elif x in ' -_/':
                out.append(' ')
            # dots, commas, apostrophes and everything else are dropped
    return ' '.join(''.join(out).split())[:FOLD_LEN]


class Names:
    def __init__(self):
        self.buf = bytearray()
        self.index = {}

    def add(self, s):
        if s in self.index:
            return self.index[s]
        b = s.encode('utf-8')[:120]
        off = len(self.buf)
        self.buf.append(len(b))
        self.buf += b
        self.index[s] = off
        return off


def in_bbox(lat, lon, bb):
    return bb is None or (bb[0] <= lat <= bb[2] and bb[1] <= lon <= bb[3])


def points_in_polygon(px, py, vx, vy):
    """Even-odd test of many points against one polygon (arrays of lon/lat)."""
    inside = np.zeros(len(px), dtype=bool)
    n = len(vx)
    j = n - 1
    for i in range(0, n, 256):
        i1 = min(n, i + 256)
        xi, yi = vx[i:i1], vy[i:i1]
        xj, yj = vx[np.arange(i, i1) - 1], vy[np.arange(i, i1) - 1]
        for k in range(len(xi)):
            cond = ((yi[k] > py) != (yj[k] > py))
            if not cond.any():
                continue
            xint = (xj[k] - xi[k]) * (py - yi[k]) / (yj[k] - yi[k] + 1e-300) + xi[k]
            inside ^= cond & (px < xint)
    return inside


def ring_coords(ring):
    xs, ys = [], []
    for n in ring:
        if n.location.valid():
            xs.append(n.lon)
            ys.append(n.lat)
    return np.array(xs), np.array(ys)


def build(pbf, region, bb):
    t0 = time.time()
    names = Names()

    # --- 1. municipalities (admin_level 8) -------------------------------------------
    munis = []          # dict(name, polys=[(xs, ys)], bbox)
    fp = (osmium.FileProcessor(pbf).with_locations().with_areas()
          .with_filter(osmium.filter.KeyFilter('boundary')))
    for o in fp:
        if not o.is_area():
            continue
        t = o.tags
        if t.get('boundary') != 'administrative' or t.get('admin_level') not in ('6', '8') or not t.get('name'):
            continue
        polys = []
        try:
            for outer in o.outer_rings():
                xs, ys = ring_coords(outer)
                if len(xs) >= 3:
                    polys.append((xs, ys))
        except osmium.InvalidLocationError:
            continue
        if not polys:
            continue
        allx = np.concatenate([p[0] for p in polys]); ally = np.concatenate([p[1] for p in polys])
        box = (ally.min(), allx.min(), ally.max(), allx.max())
        if bb is not None and (box[2] < bb[0] or box[0] > bb[2] or box[3] < bb[1] or box[1] > bb[3]):
            continue
        # label point: centre of the largest ring's bounding box (good enough to centre a map on)
        big = max(polys, key=lambda p: (p[0].max() - p[0].min()) * (p[1].max() - p[1].min()))
        munis.append({'name': t['name'], 'polys': polys, 'bbox': box, 'level': int(t['admin_level']),
                      'lat': float(big[1].mean()), 'lon': float(big[0].mean())})
    # Cities that are a district of their own (Heidelberg, Karlsruhe, ...) have no level 8 boundary:
    # keep a level 6 area only if it contains no level 8 municipality.
    l8 = [m for m in munis if m['level'] == 8]
    keep = []
    for m in munis:
        if m['level'] == 8:
            keep.append(m)
            continue
        px = np.array([x['lon'] for x in l8]); py = np.array([x['lat'] for x in l8])
        inside = np.zeros(len(px), dtype=bool)
        for xs, ys in m['polys']:
            inside |= points_in_polygon(px, py, xs, ys)
        if not inside.any():
            keep.append(m)
    munis = keep
    print(f'municipalities: {len(munis)} ({time.time() - t0:.0f}s)', flush=True)

    # --- 2. place nodes (for the "which part of town" label) ---------------------------
    place_rank = {'city': 5, 'town': 4, 'village': 3, 'suburb': 3, 'hamlet': 2, 'quarter': 2,
                  'neighbourhood': 1, 'borough': 3}
    pl_lat, pl_lon, pl_name, pl_type = [], [], [], []
    fp = osmium.FileProcessor(pbf, osmium.osm.NODE).with_filter(osmium.filter.KeyFilter('place'))
    for o in fp:
        if not o.is_node():
            continue
        t = o.tags
        if t.get('place') in place_rank and t.get('name') and o.location.valid():
            if in_bbox(o.location.lat, o.location.lon, bb):
                pl_lat.append(o.location.lat); pl_lon.append(o.location.lon); pl_name.append(t['name']); pl_type.append(t['place'])
    pl_lat = np.array(pl_lat); pl_lon = np.array(pl_lon)
    print(f'place nodes: {len(pl_name)} ({time.time() - t0:.0f}s)', flush=True)

    # --- 3. streets --------------------------------------------------------------------
    STREET_HW = {'motorway_link', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified', 'residential',
                 'living_street', 'pedestrian', 'service', 'road', 'primary_link', 'secondary_link',
                 'tertiary_link', 'trunk_link'}
    ways = []           # (name, midLat, midLon, samples ndarray (n,2) lat/lon)
    fp = (osmium.FileProcessor(pbf, osmium.osm.NODE | osmium.osm.WAY).with_locations()
          .with_filter(osmium.filter.KeyFilter('highway')))
    for w in fp:
        if not w.is_way():
            continue
        t = w.tags
        if t.get('highway') not in STREET_HW:
            continue
        nm = t.get('name')
        if not nm:
            continue
        if t.get('highway') == 'service' and t.get('service') in ('parking_aisle', 'driveway', 'drive-through'):
            continue
        try:
            pts = [(n.lat, n.lon) for n in w.nodes if n.location.valid()]
        except osmium.InvalidLocationError:
            continue
        if len(pts) < 2:
            continue
        mid = pts[len(pts) // 2]
        if not in_bbox(mid[0], mid[1], bb):
            continue
        step = max(1, len(pts) // 12)
        ways.append((nm, mid[0], mid[1], np.array(pts[::step] + [pts[-1]])))
    print(f'named street ways: {len(ways)} ({time.time() - t0:.0f}s)', flush=True)

    # which municipality is each street in?
    wlat = np.array([w[1] for w in ways]); wlon = np.array([w[2] for w in ways])
    wmuni = np.full(len(ways), -1, dtype=np.int32)
    for mi, m in enumerate(munis):
        b = m['bbox']
        cand = np.nonzero((wlat >= b[0]) & (wlat <= b[2]) & (wlon >= b[1]) & (wlon <= b[3]) & (wmuni < 0))[0]
        if len(cand) == 0:
            continue
        ins = np.zeros(len(cand), dtype=bool)
        for xs, ys in m['polys']:
            ins |= points_in_polygon(wlon[cand], wlat[cand], xs, ys)
        wmuni[cand[ins]] = mi
    print(f'streets placed in municipalities: {int(np.sum(wmuni >= 0))} ({time.time() - t0:.0f}s)', flush=True)

    # 3b. Where the boundaries only give districts (admin_level 6 - the council areas of the UK, for
    #     example) the towns and villages are place nodes: give each street of such a district to the
    #     town or village it belongs to (nearest, a city reaching further than a village). Streets
    #     near no place stay with the district.
    radius = {'city': 12000.0, 'town': 5000.0, 'village': 2000.0}
    pl_idx = [k for k in range(len(pl_name)) if pl_type[k] in radius]
    if pl_idx and any(m['level'] == 6 for m in munis):
        pidx = np.array(pl_idx)
        plat_a, plon_a = pl_lat[pidx], pl_lon[pidx]
        prad = np.array([radius[pl_type[k]] for k in pl_idx])
        virtual = {}                                   # (district, place) -> index into munis
        moved = 0
        for mi in range(len(munis)):
            m = munis[mi]
            if m['level'] != 6:
                continue
            inside = np.zeros(len(pidx), dtype=bool)
            for xs, ys in m['polys']:
                inside |= points_in_polygon(plon_a, plat_a, xs, ys)
            cands = np.nonzero(inside)[0]
            if len(cands) == 0:
                continue
            ws = np.nonzero(wmuni == mi)[0]
            for wi in ws:
                dy = (plat_a[cands] - wlat[wi]) * 110574.0
                dx = (plon_a[cands] - wlon[wi]) * 111320.0 * math.cos(math.radians(wlat[wi]))
                d = np.hypot(dx, dy)
                score = d / prad[cands]
                score[d > prad[cands]] = np.inf
                j = int(np.argmin(score))
                if not np.isfinite(score[j]):
                    continue
                pk = pl_idx[int(cands[j])]
                key = (mi, pk)
                if key not in virtual:
                    virtual[key] = len(munis)
                    munis.append({'name': pl_name[pk], 'polys': [], 'bbox': m['bbox'], 'level': 0,
                                  'lat': float(pl_lat[pk]), 'lon': float(pl_lon[pk])})
                wmuni[wi] = virtual[key]
                moved += 1
        print(f'streets given to towns and villages: {moved} ({time.time() - t0:.0f}s)', flush=True)

    # nearest place node for the label
    def nearest_place(lat, lon):
        if len(pl_lat) == 0:
            return -1
        d = (pl_lat - lat) ** 2 + ((pl_lon - lon) * math.cos(math.radians(lat))) ** 2
        k = int(np.argmin(d))
        return k if d[k] < (0.08 ** 2) else -1

    entries = {}        # (muni, fold, subName) -> dict
    for i, (nm, la, lo, samples) in enumerate(ways):
        mi = int(wmuni[i])
        if mi < 0:
            continue
        f = fold(nm)
        if not f:
            continue
        pk = nearest_place(la, lo)
        sub = pl_name[pk] if pk >= 0 else None
        if sub is not None and fold(sub) == fold(munis[mi]['name']):
            sub = None
        key = (mi, f, sub)
        e = entries.get(key)
        if e is None:
            e = entries[key] = {'name': nm, 'lat': la, 'lon': lo, 'samples': [], 'numbers': {}}
        e['samples'].append(samples)

    # --- 4. house numbers ----------------------------------------------------------------
    by_fold = defaultdict(list)
    for key, e in entries.items():
        e['samples'] = np.concatenate(e['samples'])
        by_fold[key[1]].append(key)
    fp = (osmium.FileProcessor(pbf, osmium.osm.NODE | osmium.osm.WAY).with_locations()
          .with_filter(osmium.filter.KeyFilter('addr:housenumber')))
    n_addr = n_ok = 0
    for o in fp:
        t = o.tags
        hn, st = t.get('addr:housenumber'), t.get('addr:street')
        if not hn or not st:
            continue
        if o.is_node():
            if not o.location.valid():
                continue
            la, lo = o.location.lat, o.location.lon
        elif o.is_way():
            try:
                pts = [(n.lat, n.lon) for n in o.nodes if n.location.valid()]
            except osmium.InvalidLocationError:
                continue
            if not pts:
                continue
            la = sum(p[0] for p in pts) / len(pts); lo = sum(p[1] for p in pts) / len(pts)
        else:
            continue
        if not in_bbox(la, lo, bb):
            continue
        n_addr += 1
        cands = by_fold.get(fold(st))
        if not cands:
            continue
        best, bd = None, 1e18
        for key in cands:
            s = entries[key]['samples']
            d = ((s[:, 0] - la) ** 2 + ((s[:, 1] - lo) * math.cos(math.radians(la))) ** 2).min()
            if d < bd:
                bd, best = d, key
        if best is None or bd > (0.006 ** 2):          # farther than ~650 m: not this street
            continue
        # superscripts (23²) and other non-ASCII characters are dropped: the wheel only has A-Z 0-9
        num = ''.join(c for c in hn.strip().replace(' ', '') if ord(c) < 128)[:7]
        if num and num.upper() not in entries[best]['numbers']:
            entries[best]['numbers'][num.upper()] = (num, la, lo)
            n_ok += 1
    print(f'house numbers: {n_addr} read, {n_ok} kept ({time.time() - t0:.0f}s)', flush=True)

    # --- 5. write ----------------------------------------------------------------------
    outd = os.path.join(region, 'addr')
    os.makedirs(outd, exist_ok=True)

    def numkey(item):
        n = item[1][0]
        digits = ''.join(c for c in n if '0' <= c <= '9')      # not c.isdigit(): that accepts "²"
        return (int(digits) if digits else 0, n)

    # municipality order = folded name; streets grouped by it
    mi_sorted = sorted(range(len(munis)), key=lambda i: fold(munis[i]['name']))
    by_muni = defaultdict(list)
    for key, e in entries.items():
        by_muni[key[0]].append((key, e))

    place_rec, street_rec, number_rec = [], [], []
    for mi in mi_sorted:
        m = munis[mi]
        if not by_muni.get(mi):
            continue                         # a district whose streets all went to its towns
        lst = sorted(by_muni.get(mi, []), key=lambda ke: (ke[0][1], ke[0][2] or ''))
        first = len(street_rec)
        for key, e in lst:
            nums = sorted(e['numbers'].items(), key=numkey)
            fn = len(number_rec)
            for _, (num, la, lo) in nums:
                number_rec.append(struct.pack('<8sii', num.encode('ascii', 'ignore')[:8],
                                              int(round(la * 1e7)), int(round(lo * 1e7))))
            sub = names.add(key[2]) if key[2] else NONE
            street_rec.append(struct.pack('<24sIIiiII', key[1].encode('ascii'), names.add(e['name']), sub,
                                          int(round(e['lat'] * 1e7)), int(round(e['lon'] * 1e7)),
                                          fn, len(nums)))
        place_rec.append(struct.pack('<24sIiiIII', fold(m['name']).encode('ascii'), names.add(m['name']),
                                     int(round(m['lat'] * 1e7)), int(round(m['lon'] * 1e7)),
                                     first, len(street_rec) - first, 0))
    with open(os.path.join(outd, 'places.bin'), 'wb') as f:
        f.write(b'APL1' + struct.pack('<I', len(place_rec)) + b''.join(place_rec))
    with open(os.path.join(outd, 'streets.bin'), 'wb') as f:
        f.write(b'AST1' + struct.pack('<I', len(street_rec)) + b''.join(street_rec))
    with open(os.path.join(outd, 'numbers.bin'), 'wb') as f:
        f.write(b'ANM1' + struct.pack('<I', len(number_rec)) + b''.join(number_rec))
    with open(os.path.join(outd, 'names.bin'), 'wb') as f:
        f.write(bytes(names.buf))
    info = {'places': len(place_rec), 'streets': len(street_rec), 'numbers': len(number_rec)}
    with open(os.path.join(outd, 'info.json'), 'w') as f:
        json.dump(info, f, indent=1)
    sizes = {n: os.path.getsize(os.path.join(outd, n)) for n in ('places.bin', 'streets.bin', 'numbers.bin', 'names.bin')}
    print(info, {k: f'{v / 1e6:.2f} MB' for k, v in sizes.items()}, f'({time.time() - t0:.0f}s)')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pbf')
    ap.add_argument('region')
    ap.add_argument('--bbox', default=None)
    a = ap.parse_args()
    bb = tuple(float(v) for v in a.bbox.split(',')) if a.bbox else None
    build(a.pbf, a.region, bb)

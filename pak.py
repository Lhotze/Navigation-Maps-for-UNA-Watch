"""
Bundles of tile files.

Writing every tile as a file of its own gives tens of thousands of files for a
large region, which the watch's USB drive copes badly with. The tiles of one
layer are therefore packed side x side into a bundle (SIDE = 2: four tiles, which
keeps a file below about 1.2 MB - the watch has trouble with bigger ones):

    <layer>/<bx>/<by>.pak
        'PAK1', u16 side, u16 0
        side*side x { u32 offset, u32 length }    slot = (iy mod side) * side + (ix mod side); length 0 = no such tile
        the tiles one after the other

where bx = floor(ix / side), by = floor(iy / side). The side is in region.json
(bundleSide). A tile is read with one seek to its offset; the header is read once
per bundle and kept.
"""

import os
import struct

SIDE = 2


class BundleWriter:
    """Collects the tiles of one layer and writes the bundles; flush_before() lets
    a caller that produces tiles in ix order write finished bundles early."""

    def __init__(self, layer_dir, side=None):
        self.side = side or SIDE
        self.dir = layer_dir
        self.bundles = {}            # (bx, by) -> {slot: bytes}
        self.files = 0
        self.bytes = 0

    def add(self, ix, iy, data):
        key = (ix // self.side, iy // self.side)
        self.bundles.setdefault(key, {})[(iy % self.side) * self.side + (ix % self.side)] = data

    def _write(self, key, slots):
        bx, by = key
        d = os.path.join(self.dir, str(bx))
        os.makedirs(d, exist_ok=True)
        head = bytearray(b'PAK1' + struct.pack('<HH', self.side, 0))
        body = bytearray()
        pos = 8 + 8 * self.side * self.side
        table = []
        for s in range(self.side * self.side):
            data = slots.get(s)
            if data:
                table.append((pos + len(body), len(data)))
                body += data
            else:
                table.append((0, 0))
        for off, ln in table:
            head += struct.pack('<II', off, ln)
        with open(os.path.join(d, f'{by}.pak'), 'wb') as f:
            f.write(head)
            f.write(body)
        self.files += 1
        self.bytes += len(head) + len(body)

    def flush_before(self, bx_limit):
        """Write and drop all bundles whose column bx is below bx_limit."""
        for key in [k for k in self.bundles if k[0] < bx_limit]:
            self._write(key, self.bundles.pop(key))

    def close(self):
        for key in list(self.bundles):
            self._write(key, self.bundles.pop(key))

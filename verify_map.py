#!/usr/bin/env python3
"""Compares a map folder on the PC with the copy on the watch and lists what is wrong.

    python verify_map.py maps/map /run/media/you/UNA/Apps/<app>/maps/map

Reports files that are missing, have another size, or have other content (a file
that was copied badly can have the right size and only zeros inside). With --fix it
writes the damaged files to the watch again.
"""
import argparse
import hashlib
import os
import shutil
import sys


def digest(path):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', help='the map folder on the PC')
    ap.add_argument('copy', help='the map folder on the watch')
    ap.add_argument('--fix', action='store_true', help='copy damaged or missing files again')
    a = ap.parse_args()
    src, dst = a.source.rstrip('/'), a.copy.rstrip('/')
    if not os.path.isdir(src):
        sys.exit(f'{src} is not a folder')
    if not os.path.isdir(dst):
        sys.exit(f'{dst} is not a folder - is the watch connected?')

    missing, size_bad, content_bad, ok = [], [], [], 0
    for root, _, files in os.walk(src):
        for name in sorted(files):
            p = os.path.join(root, name)
            rel = os.path.relpath(p, src)
            q = os.path.join(dst, rel)
            if not os.path.isfile(q):
                missing.append(rel)
            elif os.path.getsize(q) != os.path.getsize(p):
                size_bad.append(rel)
            elif digest(q) != digest(p):
                content_bad.append(rel)
            else:
                ok += 1
    extra = 0
    for root, _, files in os.walk(dst):
        for name in files:
            if not os.path.isfile(os.path.join(src, os.path.relpath(os.path.join(root, name), dst))):
                extra += 1

    print(f'{ok} files fine, {len(missing)} missing, {len(size_bad)} with wrong size, '
          f'{len(content_bad)} with wrong content, {extra} extra files on the watch')
    for title, lst in (('missing', missing), ('wrong size', size_bad), ('wrong content', content_bad)):
        for rel in lst[:20]:
            print(f'  {title}: {rel}')
        if len(lst) > 20:
            print(f'  ... and {len(lst) - 20} more {title}')
    bad = missing + size_bad + content_bad
    if a.fix and bad:
        for rel in bad:
            q = os.path.join(dst, rel)
            os.makedirs(os.path.dirname(q), exist_ok=True)
            shutil.copyfile(os.path.join(src, rel), q)
        print(f'copied {len(bad)} files again - eject the watch properly and run this once more')
    elif bad:
        print('run again with --fix to copy these files again')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()

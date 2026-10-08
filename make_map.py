#!/usr/bin/env python3
"""Offline Navigation - build the map package for the watch from OpenStreetMap data.

    1. Put one or more .osm.pbf files next to this script (download them as you need:
       countries, regions, cities, across borders - anything from geofabrik.de or bbbike.org).
    2. python make_map.py
    3. Copy  maps/map  to  maps/map  in the app folder of the watch.

All .pbf files in the folder are merged into ONE map (overlaps are fine). Instead you can
name sources explicitly: a local file, a URL, or a Geofabrik path
    python make_map.py europe/germany/rheinland-pfalz europe/austria/vorarlberg

See README.md for where to get extracts and what to expect.

Steps (run as far as possible in parallel):
    convert_map.py   road + forest/water/built-up tiles         -> m/, a/
    extract_graph.py routing graph (bicycle + on foot)          -> work/graph
    build_ch         contraction hierarchies (one per profile)  -> work/ch_*.bin
    build_addr.py    address index (place -> street -> number)  -> addr/
    pack_routing.py  routing tiles and index                    -> route/
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))


def ensure_python_packages():
    """Needs osmium and numpy. If they are missing, creates .venv next to this script,
    installs them there and runs the script again with that Python."""
    try:
        import osmium  # noqa: F401
        import numpy  # noqa: F401
        return
    except ImportError:
        pass
    venv = os.path.join(HERE, '.venv')
    py = os.path.join(venv, 'Scripts' if os.name == 'nt' else 'bin', 'python' + ('.exe' if os.name == 'nt' else ''))
    if os.path.abspath(sys.executable) == os.path.abspath(py):
        sys.exit('osmium / numpy could not be installed into .venv - see the messages above, '
                 'or run:  pip install -r requirements.txt')
    print('The Python packages osmium and numpy are missing; installing them into ' + venv, flush=True)
    try:
        if not os.path.isfile(py):
            subprocess.check_call([sys.executable, '-m', 'venv', venv])
        subprocess.check_call([py, '-m', 'pip', 'install', '-r', os.path.join(HERE, 'requirements.txt')])
    except (subprocess.CalledProcessError, OSError) as e:
        sys.exit(f'automatic install failed ({e}). Install by hand:\n'
                 f'  python -m venv .venv && .venv/bin/pip install -r requirements.txt && .venv/bin/python make_map.py')
    if os.name == 'nt':                  # execv does not replace the process on Windows
        sys.exit(subprocess.call([py] + sys.argv))
    os.execv(py, [py] + sys.argv)


ensure_python_packages()
PY = sys.executable


def log(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


def fetch(source, dl_dir):
    """Returns (local pbf path, default region name)."""
    if os.path.isfile(source):
        name = re.sub(r'(-latest)?\.osm\.pbf$|\.pbf$', '', os.path.basename(source))
        return os.path.abspath(source), name
    if re.match(r'^https?://', source):
        url = source
    else:                                        # Geofabrik path like europe/germany/bremen
        url = f'https://download.geofabrik.de/{source.strip("/")}-latest.osm.pbf'
    name = re.sub(r'(-latest)?\.osm\.pbf$|\.pbf$', '', os.path.basename(url))
    os.makedirs(dl_dir, exist_ok=True)
    dst = os.path.join(dl_dir, os.path.basename(url))
    if os.path.isfile(dst):
        log(f'using the download from earlier: {dst}')
        return dst, name
    log(f'downloading {url}')
    last = [0.0]

    def hook(blocks, bs, total):
        if time.time() - last[0] > 5 and total > 0:
            last[0] = time.time()
            print(f'   {blocks * bs / 1e6:.0f} / {total / 1e6:.0f} MB', flush=True)
    try:
        urllib.request.urlretrieve(url, dst + '.part', hook)
    except Exception as e:
        sys.exit(f'download failed: {e}')
    os.replace(dst + '.part', dst)
    return dst, name


def merge_pbf(files, dst):
    """Merges several extracts into one file; objects present in more than one are kept once."""
    import osmium
    log(f'merging {len(files)} files into one map')
    w = osmium.SimpleWriter(dst)
    m = osmium.MergeInputReader()
    for f in files:
        m.add_file(f)
    m.apply(w)
    w.close()
    log(f'merged: {os.path.getsize(dst) / 1e6:.0f} MB')


def find_pbf_files(folder):
    out = []
    for f in sorted(os.listdir(folder)):
        if f.lower().endswith('.pbf') and os.path.isfile(os.path.join(folder, f)):
            out.append(os.path.join(folder, f))
    return out


def ensure_build_ch():
    exe = os.path.join(HERE, 'build_ch' + ('.exe' if os.name == 'nt' else ''))
    src = os.path.join(HERE, 'build_ch.cpp')
    if os.path.isfile(exe) and os.path.getmtime(exe) >= os.path.getmtime(src):
        return exe
    cxx = shutil.which('g++') or shutil.which('clang++') or shutil.which('c++')
    if cxx:
        log(f'compiling build_ch with {os.path.basename(cxx)}')
        cmd = [cxx, '-O2', '-std=c++17', '-o', exe, src]
    elif shutil.which('cl'):                         # Visual Studio: run from its "Developer Command Prompt"
        log('compiling build_ch with cl (Visual Studio)')
        cmd = ['cl', '/nologo', '/O2', '/std:c++17', '/EHsc', '/Fe:' + exe, '/Fo:' + os.path.join(HERE, 'build_ch.obj'), src]
    else:
        how = {'nt': 'Install MSYS2/MinGW (g++) or the Visual Studio Build Tools and run this from the '
                     '"Developer Command Prompt", ',
               'darwin': 'Run "xcode-select --install" for the compiler, '}.get(
                   'nt' if os.name == 'nt' else sys.platform, 'Install g++ or clang, ')
        sys.exit('no C++ compiler found (needed once for build_ch.cpp). ' + how +
                 'or use the Dockerfile - see README.md')
    r = subprocess.run(cmd, cwd=HERE)
    if r.returncode:
        sys.exit('compiling build_ch.cpp failed')
    return exe


ACTIVE = []


class Job:
    def __init__(self, label, cmd, logdir):
        self.label = label
        self.cmd = cmd
        self.t0 = time.time()
        self.logpath = os.path.join(logdir, label.replace(' ', '_') + '.log')
        self.logf = open(self.logpath, 'w')
        env = dict(os.environ, PYTHONUTF8='1', PYTHONIOENCODING='utf-8')      # names print the same on every OS
        self.p = subprocess.Popen(cmd, stdout=self.logf, stderr=subprocess.STDOUT, cwd=HERE, env=env)
        ACTIVE.append(self)

    def poll(self):
        return self.p.poll()

    def finish(self):
        rc = self.p.wait()
        self.logf.close()
        if self in ACTIVE:
            ACTIVE.remove(self)
        if rc:
            for other in ACTIVE:                 # do not leave the other steps running
                other.p.kill()
            tail = ''.join(open(self.logpath, errors='replace').readlines()[-15:])
            sys.exit(f'\n"{self.label}" failed (exit {rc}). Last lines of {self.logpath}:\n{tail}')
        log(f'{self.label} done ({time.time() - self.t0:.0f} s)')


def wait_all(jobs):
    for j in jobs:
        j.finish()


def run_stage(specs, logdir, sequential):
    """specs: list of (label, cmd). Runs them in parallel unless sequential."""
    jobs = []
    for label, cmd in specs:
        log(f'start: {label}')
        j = Job(label, cmd, logdir)
        if sequential:
            j.finish()
        else:
            jobs.append(j)
    wait_all(jobs)


def dir_size(path):
    total = n = 0
    for root, _, files in os.walk(path):
        for f in files:
            total += os.path.getsize(os.path.join(root, f))
            n += 1
    return total, n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', nargs='*',
                    help='local .osm.pbf, URL or Geofabrik path (europe/germany/bremen); '
                         'none given: all .pbf files in the folder of this script (or --dir)')
    ap.add_argument('--dir', default=HERE, help='folder searched for .pbf files when no source is given')
    ap.add_argument('-o', '--out', default=os.path.join(HERE, 'maps'), help='output directory (default: ./maps)')
    ap.add_argument('--name', help='folder name of the map (default: map)')
    ap.add_argument('--bbox', help='minlat,minlon,maxlat,maxlon - only this part of the file')
    ap.add_argument('--work', help='directory for intermediate files (default: <out>/.work_<name>)')
    ap.add_argument('--no-addr', action='store_true', help='skip the address index (no address search)')
    ap.add_argument('--sequential', action='store_true',
                    help='one step at a time: slower, but needs much less memory')
    ap.add_argument('--keep-work', action='store_true', help='keep the intermediate files')
    a = ap.parse_args()

    out_root = os.path.abspath(a.out)
    name = re.sub(r'[^a-z0-9_-]+', '_', (a.name or 'map').lower()).strip('_') or 'map'
    region = os.path.join(out_root, name)
    work = os.path.abspath(a.work or os.path.join(out_root, f'.work_{name}'))
    logdir = os.path.join(work, 'logs')
    os.makedirs(logdir, exist_ok=True)

    # the input files
    if a.source:
        files = [fetch(src, os.path.join(out_root, '.downloads'))[0] for src in a.source]
    else:
        files = find_pbf_files(os.path.abspath(a.dir))
        if not files:
            sys.exit(f'no .pbf files found in {os.path.abspath(a.dir)}.\n'
                     'Download an extract (e.g. from https://download.geofabrik.de or '
                     'https://extract.bbbike.org) and put it in that folder, then run this again.')
    for f in files:
        log(f'input: {f} ({os.path.getsize(f) / 1e6:.0f} MB)')
    if len(files) == 1:
        pbf = files[0]
    else:
        total = sum(os.path.getsize(f) for f in files) / 1e6
        if total > 1500:
            sys.exit(f'{total:.0f} MB of input is too much to merge in memory (limit about 1500 MB). '
                     'Use smaller extracts.')
        pbf = os.path.join(work, 'merged.osm.pbf')
        merge_pbf(files, pbf)

    others = [d for d in os.listdir(out_root) if os.path.isdir(os.path.join(out_root, d))
              and not d.startswith('.') and d != name] if os.path.isdir(out_root) else []
    if others:
        log(f'NOTE: {out_root} also contains {", ".join(others)} - the app uses only ONE map folder; '
            'copy only the one you want to the watch.')
    if os.path.isdir(region):
        log(f'replacing the earlier {region}')
        shutil.rmtree(region)
    os.makedirs(region)

    mb = os.path.getsize(pbf) / 1e6
    log(f'building {region} from {mb:.0f} MB of map data')
    if mb > 1500:
        log('WARNING: that is a very large file. Memory and time grow with it; a region of up to a few '
            'hundred MB (a country or part of one) is what this is meant for. See README.md.')

    ch = ensure_build_ch()
    t0 = time.time()
    box = ['--bbox', a.bbox] if a.bbox else []

    # stage 1: everything that only needs the .pbf (the region folder exists already)
    specs = [
        ('map tiles', [PY, 'convert_map.py', pbf, out_root, '--name', name] + box),
        ('routing graph', [PY, 'extract_graph.py', pbf, work] + box),
    ]
    if not a.no_addr:
        specs.append(('address index', [PY, 'build_addr.py', pbf, region] + box))
    jobs = []
    for label, cmd in specs:
        log(f'start: {label}')
        j = Job(label, cmd, logdir)
        if a.sequential:
            j.finish()
        else:
            jobs.append(j)
    if jobs:                                   # the hierarchies only need the graph
        g = next(j for j in jobs if j.label == 'routing graph')
        g.finish()
        jobs.remove(g)
    graph = os.path.join(work, 'graph')
    for prof, fname in ((0, 'ch_bike.bin'), (1, 'ch_foot.bin')):
        label = 'hierarchy bike' if prof == 0 else 'hierarchy foot'
        log(f'start: {label}')
        j = Job(label, [ch, graph, str(prof), os.path.join(work, fname), '--test', '100'], logdir)
        if a.sequential:
            j.finish()
        else:
            jobs.append(j)
    wait_all(jobs)

    # stage 2: pack the routing into the region folder
    run_stage([('routing tiles', [PY, 'pack_routing.py', graph, region, '--ch-dir', work])], logdir, True)

    size, files = dir_size(region)
    if not a.keep_work:
        shutil.rmtree(work, ignore_errors=True)
    log(f'finished in {(time.time() - t0) / 60:.1f} min: {region}  ({size / 1e6:.0f} MB, {files} files)')
    print(f'\nCopy the folder "{name}" to  maps/{name}  in the app folder of the watch '
          f'(only one map folder is used).')


if __name__ == '__main__':
    main()

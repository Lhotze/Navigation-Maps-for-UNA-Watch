# Navigation Maps for the UNA Watch

Build tools for the map files of the **Offline Navigation** app for the UNA watch. The app reads its
own compact map format (map tiles, forest/water/built-up areas, routing for bicycle and on foot, and
an address index). You make it yourself, once per region, from free OpenStreetMap data, and copy it
to the watch over USB. Nothing has to be hosted by anyone.

This repository holds only these tools, not the app. The maps are made for Offline Navigation 1.0.0 or newer.

Platforms: developed and tested on Linux. The code avoids Linux-only features and handles Windows
(MinGW or Visual Studio compiler) and macOS (Xcode command line tools), but those two are untested.

**About OsmAnd maps:** OsmAnd's `.obf` files are a different, derived format and cannot be
turned back into routing data. Use the same source OsmAnd uses: OpenStreetMap extracts
(`.osm.pbf`). They are free, updated daily, and exist for every country on earth.

## 1. Get the tool

You need Python 3.9+ and a C++ compiler (g++ or clang; used once). The Python packages
(`osmium`, `numpy`) are installed automatically into a `.venv` folder next to the script
the first time you run it. Or use Docker, nothing else needed (see the top of `Dockerfile`).

## 2. Download the extracts you need

The input is an OpenStreetMap extract in `.osm.pbf` format. Two free services provide them; use
whichever fits. You can use files from both together.

### Option A: Geofabrik (countries and regions, ready-made)

1. Open https://download.geofabrik.de and click your continent, then your country
   (for example Europe > United Kingdom).
2. A country page lists its **sub-regions** in a table (for the United Kingdom: England, Scotland,
   Wales and Northern Ireland). Click a sub-region, for example "Scotland". The table also shows
   the size of each file in the `.osm.pbf` column.
3. On the sub-region page, download the file named `<region>-latest.osm.pbf` (the link in the
   section "Commonly Used Formats"). Example:
   `https://download.geofabrik.de/europe/united-kingdom/scotland-latest.osm.pbf`
4. Put the file in the same folder as `make_map.py`.

Shortcut: you can skip the manual download. Give `make_map.py` the part of the address between
`download.geofabrik.de/` and `-latest.osm.pbf`, and it downloads the file for you:

    python make_map.py europe/united-kingdom/scotland

Take **sub-regions, not whole large countries**: a country's extract can be several GB (all of Great
Britain is 2 GB), more than this tool can merge and the watch can hold. The dated files further down
on the page (like `scotland-260901.osm.pbf`) are older snapshots of the same data; you want the
"latest" one.

### Option B: BBBike (your own rectangle, also across borders)

Use this when the area you need is not one region, for example a trip that crosses a border or just
the surroundings of one city.

1. Open https://extract.bbbike.org.
2. Type a name for your extract in the **Area** field (any name, or search for a place).
3. Pick the format **Protocol Buffer (PBF)** from the format list (it is one of many).
4. Enter your **email address**: the service sends you the download link by email.
5. Choose the area: move and zoom the map to your region and click to create the rectangle, or type
   the corner coordinates (south-west and north-east). Adjust it by dragging its corners. If the
   service says "Area too large", zoom in and make the rectangle smaller.
6. Click the button to start the extract. When it is ready you get an email with a link; download the
   `.osm.pbf` file from it (the name contains your rectangle, for example
   `planet_-3.350,55.890_-3.050,55.990.osm.pbf`).
7. Put the file in the same folder as `make_map.py`.

### Using several files

Download as many files as you need and **put them all in the same folder as `make_map.py`**. The
script merges them into one map, also across borders (for example Scotland plus the north of
England, or two countries plus a BBBike rectangle around a city). Overlapping areas are fine;
objects present in several files are kept once.

Take only what covers where you will travel. The result is roughly 1.2 MB of map per 1 MB of
`.pbf`. The input is merged in memory, so keep the total below about 1.5 GB (needs roughly
10 GB of RAM at that size).

## 3. Build

    python make_map.py

That is all. It writes `maps/map/`. Useful options:

- `--name` folder name of the result (default `map`)
- `--no-addr` skip the address index (no address search, much faster, smaller)
- `--sequential` one step at a time: slower but needs far less memory
- `--bbox minlat,minlon,maxlat,maxlon` use only part of the data
- `--dir <folder>` look for the `.pbf` files in another folder
- Instead of putting files in the folder you can name sources:
  `make_map.py europe/united-kingdom/wales` (Geofabrik path, downloaded), a URL, or a file path.
- `--keep-work` keep intermediate files

Measured on a 16-core PC with 15 GB RAM, with a 270 MB `.pbf`: about 45 minutes, up to ~8 GB RAM,
result about 330 MB.
The slow parts are the two hierarchies (bicycle ~10 min, foot ~40 min) and the house-number matching (~40 min).
Time and RAM grow with the size of the input; a city or district takes a few minutes.
Merging two neighbouring regions (425 MB together) took 30 s.

## 4. Copy to the watch

Connect the watch by USB and copy the folder `maps/map` to `maps/map` in the
app folder of "Offline Navigation". Delete an older map there first.

## Check the copy on the watch

Copying thousands of files over USB can leave a file with the right size but wrong
content (the app then reports "route none" or missing tiles). Compare the copy with
the original while the watch is connected as a drive, and let the tool repair it:

    python verify_map.py maps/map /path/to/watch/<app folder>/maps/map
    python verify_map.py maps/map /path/to/watch/<app folder>/maps/map --fix

Eject the watch properly afterwards (unmount it before pulling the cable).

## What is in the map

Roads, paths and tracks usable by bicycle and on foot (routing for both), forests, water,
parks and built-up areas (grey hatching), street names, and an address index
(place -> street -> house number). No buildings, no car routing.

## Updating

Replace the `.pbf` files with fresh ones and run the command again; the old result is replaced.

## License

MIT, see `LICENSE`. Map data is (c) OpenStreetMap contributors under the ODbL, see `NOTICE.md`.

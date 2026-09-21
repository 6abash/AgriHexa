# Navigation — Satellite Terrain Path Planner

A ground-station Python tool that lets an operator plan an autonomous
mission for AgriHexa directly on a satellite/aerial image, then drives
the robot through it live over Wi-Fi.

## Pipeline

1. **Image ingestion** — loads a satellite screenshot (e.g. cropped
   from Google Maps) of the target field. If none is found, a synthetic
   terrain photo is generated so the planner can still be demoed.
2. **Terrain costing** — resizes the image to a grid and derives a
   per-cell traversal cost from two cues: an HSV green mask (vegetation
   = easier ground) and Sobel gradient magnitude (roughness = a proxy
   for slope/incline, penalized more heavily). Cells above a threshold
   are marked impassable, so A* is steered around steep terrain and
   obstacles, not just toward the shortest line.
3. **Waypoint selection** — `matplotlib.ginput` lets the operator click
   as many waypoints as they like directly on the satellite image.
4. **Path planning** — 8-connected A* (`a_star_search`) runs between
   each consecutive waypoint pair, using terrain cost (not just
   distance) as the edge weight, so the resulting path favors flat,
   vegetated ground and minimizes climbs/descents.
5. **Mission execution** — the grid path is translated into the
   robot's move vocabulary (`F/B/L/R/FR/FL/BR/BL/S`) and streamed step
   by step over a TCP socket to the ESP32 (see [`/firmware`](../firmware)),
   waiting for an `ACK` after each step before sending the next. A live
   dual-panel animation (satellite view + cost map) tracks the robot's
   position as it moves.

## Running

```bash
cd navigation
pip install -r requirements.txt
python satellite_path_planner.py
```

Set `mock_hardware=True` in the `execute_mission_and_animate` call at
the bottom of the script to dry-run the planner and animation without
an open Wi-Fi socket to real hardware. Update `esp32_ip` to match the
static IP configured in the ESP32 firmware.

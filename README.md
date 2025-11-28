# stihl_visualization
ROS2 package providing lightweight visualization utilities for the Stihl mobile platform.

This package contains ROS2 nodes and utilities for visualizing a robot mesh on a common map frame,
publishing a polygon that demarks an operational area, rendering detected weeds as markers, and
publishing prerecorded points as a `nav_msgs/Path` message for inspection in Foxglove Studio or RViz.

**Contents**
- Nodes:
		- `visualization` — reads a polygon (lat/lon) from `config/stihl.yaml`, projects coordinates to a local tangent plane in meters, publishes a polygon `Marker`, and places a robot mesh using live GNSS fixes (`sensor_msgs/NavSatFix`).
		- `heatmap_plotter` — maintains a simple grid/heatmap of weed detections and publishes an `OccupancyGrid` and text `MarkerArray` for non-empty cells.
- Config:
		- `config/stihl.yaml` — polygon coordinates (latitude, longitude) used by `polygon_visualization`.
		- other example configs in `config/` used by supporting nodes (e.g. heatmap settings).

## Installation

Prerequisites:
- ROS2 (tested with Humble; follow official installation docs for your distro)
- Python dependencies used by package nodes (install via `rosdep` below)

Typical setup steps (from workspace root):

```bash
# source ROS2
source /opt/ros/humble/setup.bash

cd ~/stihl_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source install/setup.bash
```

If you add or change Python dependencies, ensure they are installed in the environment used for ROS2 (system Python, venv, or colcon's install).

## Usage

Start the visualizations using the provided launch file (example):

```bash
source install/setup.bash
ros2 launch stihl_visualization visualization.launch.py
```

You can also run the console nodes directly (installed as console scripts by `setup.py`):

```bash
ros2 run stihl_visualization visualization
ros2 run stihl_visualization heatmap_plotter
```

`movement_test.py` is a convenience/demo script included in the package source — it may be executed directly for quick local debugging (it is not registered as a console script by default).

## Nodes, Topics and Messages

- `visualization` node
		- Subscribes: `/ublox_gps_node/fix` — `sensor_msgs/NavSatFix` (optional; used to place the robot mesh at live GNSS-derived local positions)
		- Publishes:
				- `/polygon_marker` — `visualization_msgs/Marker` (LINE_STRIP) in frame `map`
				- `/robot_marker` — `visualization_msgs/Marker` (MESH_RESOURCE) for the robot mesh
				- `/ground_marker` — `visualization_msgs/Marker` (CUBE) for a ground plane

- `heatmap_plotter` node
		- Subscribes: GPS and detection topics (configured via node config)
		- Publishes: an `nav_msgs/OccupancyGrid` heatmap and `visualization_msgs/MarkerArray` for text markers

All visualization outputs use the `map` frame for consistent overlay in Foxglove Studio or RViz.

## Coordinate conversions


- Geodetic → local planar meters (used by `visualization`):
	- Latitude/longitude pairs in `config/stihl.yaml` are projected to a local tangent plane using an Azimuthal Equidistant (AEQD) projection centered on the first coordinate in the YAML file. The result is in meters (local ENU-like coordinates) so the polygon lines and robot mesh align in the `map` frame.

## Configuration

- `config/stihl.yaml` — must contain `polygon_coordinates` as a list of `[lat, lon]` pairs. The first pair is used as the projection origin.

Other config files in `config/` provide example parameters for heatmap sizing and behavior; review them to tune grid extent, resolution and topics.

## Developer notes / API

- `stihl_visualization/visualization.py` — `VisualizationNode` implementation. Reads `config/stihl.yaml`, computes a local projection using `pyproj`, publishes polygon `Marker` and robot mesh placements from live GNSS fixes.
- `stihl_visualization/heatmap_plotter.py` — `HeatmapPlotter` implementation. Maintains a grid/heatmap of weed detections and publishes `nav_msgs/OccupancyGrid` plus marker overlays.
- `meshes/` — place mesh assets here (e.g., `stihl.stl`). URIs use the package scheme, e.g. `package://stihl_visualization/meshes/stihl.stl`.

## Troubleshooting and tips

- If markers or meshes are not visible in Foxglove Studio or RViz, ensure the workspace is properly built and sourced so `package://` URIs resolve:

```bash
source install/setup.bash
```

- Missing YAML: `polygon_visualization` will raise an error if `config/stihl.yaml` is absent or lacks `polygon_coordinates` — ensure the file exists and contains at least two coordinates.
- Mesh scale: markers set mesh scale factors small by default; tune `marker.scale` values to fit the scene visually.

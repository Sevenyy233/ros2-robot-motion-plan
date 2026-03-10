# Nav 2.5D Navigation System

This package implements a 2.5D navigation system for non-flat terrain using ROS 2 Humble and Nav2.

## Architecture

The system is decoupled into two phases:
1.  **Mapping**: Uses RTAB-Map to generate a 2D occupancy grid and 3D map.
2.  **Navigation**: Uses Nav2 with AMCL and Map Server to navigate on the static map, while using the **Gradient Layer** to avoid steep slopes in real-time.

## Installation

```bash
# Build
colcon build --packages-select nav_25d
source install/setup.bash
```

## 1. Mapping Phase

Launch the simulation and RTAB-Map SLAM:

```bash
ros2 launch nav_25d mapping_25d.launch.py
```

- **Teleoperation**: Open a new terminal to drive the robot:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
- **Mapping**: Drive around the environment to build the map.
- **Save Map**: Once satisfied, save the map (run in `nav_25d/config` directory or specify path):
    ```bash
    cd src/nav_25d/config
    ros2 run nav2_map_server map_saver_cli -f map
    ```
    This will generate `map.yaml` and `map.pgm`.

## 2. Navigation Phase

Terminate the mapping launch. Launch the navigation system with the saved map:

```bash
ros2 launch nav_25d navigation_25d.launch.py map:=/path/to/your/map.yaml
```
*Note: Default map path is `src/nav_25d/config/map.yaml`.*

- **Localization**:
    - The system uses AMCL.
    - In RViz, use **2D Pose Estimate** to set the initial pose if it doesn't match.
- **Navigation**:
    - In RViz, use **2D Goal Pose** to set a destination.
    - The global planner will plan a path.
    - The **Gradient Layer** will add costs to steep slopes (visualized in Costmaps), preventing the robot from traversing them.

## Configuration

- **Simulation**: `launch/simulation.launch.py`
- **Mapping**: `launch/mapping_25d.launch.py`
- **Navigation**: `launch/navigation_25d.launch.py`
- **Nav2 Params**: `params/nav2_params.yaml`
- **RViz**: `rviz/nav_25d.rviz`

### Gradient Layer

- `max_slope_limit`: 0.5 rad (~28 degrees). Slopes steeper than this are lethal.
- `slope_cost_factor`: 100.0. Scaling factor for cost.

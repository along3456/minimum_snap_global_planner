# Minimum Snap Planner

A ROS move_base global planner plugin that combines A* path search with Minimum Snap trajectory optimization for smooth path generation with waypoint support.

## Features

- **A* Path Planning**: Efficient grid-based path search
- **Minimum Snap Optimization**: Generates smooth, dynamically feasible trajectories
- **Waypoint Support**: Plan paths through multiple intermediate waypoints
- **RViz Integration**: Interactive waypoint selection tool
- **Move Base Compatible**: Drop-in replacement for default global planner

## Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
# Already cloned as part of Minimum-Snap project
```

2. Install dependencies:
```bash
sudo apt-get install ros-noetic-move-base ros-noetic-nav-core ros-noetic-costmap-2d
```

3. Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### As a Move Base Plugin

1. Configure move_base to use the Minimum Snap Planner:

```xml
<node pkg="move_base" type="move_base" name="move_base">
  <param name="base_global_planner" value="minimum_snap_planner/WaypointSnapPlanner"/>
  <rosparam file="$(find minimum_snap_planner)/config/planner_params.yaml" command="load"/>
</node>
```

### Using the Waypoint Tool in RViz

1. In RViz, select the "WaypointSnap" tool (`waypoint_rviz_tools/WaypointTool`) from the toolbar (shortcut: 'w')
2. **Left click** on the map to add waypoints
3. **Right click** to clear all waypoints
4. Use "2D Nav Goal" to set the final goal - planning will start automatically

### Planning Flow

1. **Set Waypoints**: Use the Waypoint tool to place intermediate points
2. **Set Goal**: Click "2D Nav Goal" to set the destination
3. **Automatic Planning**: 
   - Robot position (from move_base) → Waypoint 1 → ... → Waypoint N → Goal
   - Each segment uses A* for obstacle avoidance
   - All path points are combined and downsampled
   - Minimum Snap generates a smooth trajectory

## Parameters

Edit `config/planner_params.yaml`:

```yaml
WaypointSnapPlanner:
  downsample_resolution: 0.5      # Path downsampling interval (m)
  max_vel: 1.0                    # Maximum velocity (m/s)
  max_accel: 1.0                  # Maximum acceleration (m/s²)
  polynomial_order: 3             # 2:accel, 3:jerk, 4:snap
  trajectory_time_step: 0.01      # Trajectory sampling rate (s)
  waypoint_topic: "/waypoints"    # Waypoint topic name
```

## Architecture

```
┌─────────────┐
│   RViz      │
│ Waypoint    │
│   Tool      │
└──────┬──────┘
       │ /waypoints
       ↓
┌─────────────────────────────────────┐
│  Minimum Snap Planner               │
│  ┌───────────────────────────────┐  │
│  │ 1. Multi-segment A* Search    │  │
│  │    Start→WP1→WP2→...→Goal     │  │
│  ├───────────────────────────────┤  │
│  │ 2. Path Downsampling          │  │
│  ├───────────────────────────────┤  │
│  │ 3. Minimum Snap Optimization  │  │
│  │    - Smooth trajectory        │  │
│  │    - Dynamic feasibility      │  │
│  └───────────────────────────────┘  │
└──────────┬──────────────────────────┘
           │ plan
           ↓
    ┌─────────────┐
    │  Move Base  │
    └─────────────┘
```

## Topics

### Subscribed
- `/waypoints` (geometry_msgs/PoseArray): Intermediate waypoints
- Costmap from move_base

### Published
- `~global_plan` (nav_msgs/Path): Visualize the generated plan

## Algorithm Details

### A* Search
- Grid-based search with diagonal movement
- Obstacle information from costmap
- Manhattan heuristic with tie-breaking

### Minimum Snap
- Closed-form QP solution
- Minimizes 4th derivative (snap) for smooth motion
- Time allocation based on distance and velocity constraints
- Polynomial trajectory representation

## Troubleshooting

**Problem**: Planner fails to find path
- Check costmap configuration
- Verify waypoints are in free space
- Increase `downsample_resolution` if path is too complex

**Problem**: Trajectory is not smooth
- Decrease `downsample_resolution` for more waypoints
- Increase `polynomial_order` (but computational cost increases)

**Problem**: Waypoints not appearing in RViz
- Check `/waypoints` topic is being published
- Verify RViz is using correct fixed frame
- Restart RViz and reselect the Waypoint tool

## Citation

Based on the Minimum Snap trajectory generation algorithm:

```
Mellinger, D., & Kumar, V. (2011).
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA).
```

## License

BSD License

## Author

Zhang Zhimeng (Original Minimum Snap implementation)
Modified for move_base integration with waypoint support


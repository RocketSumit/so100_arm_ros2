# so100_moveit_config

MoveIt 2 configuration and launch files for the SO100 arm. Use this package to run motion planning, visualization, and the demo.

## Prerequisites

- ROS 2 Humble (or compatible distro)
- MoveIt 2 and dependencies (see `package.xml`)
- **so100_bringup** – robot description and controllers ([so100_bringup](../so100_bringup/README.md))
- For real hardware: **so100_hardware_interface** and robot bringup running

## Build

From your workspace:

```bash
cd /path/to/robo_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select so100_moveit_config
source install/setup.bash
```

Build **so100_bringup** (and **so100_description**) first.

## Get started

### Launch MoveIt demo

In another terminal:

```bash
source install/setup.bash
ros2 launch so100_moveit_config demo.launch.py
```

This starts:

- **move_group** – motion planning
- **RViz** – with the MoveIt Motion Planning plugin

In RViz you can plan and execute motions using the Motion Planning tab (e.g. drag the interactive marker to set a goal, then Plan and Execute).

## Other launch files

| Launch file                   | Description                    |
| ----------------------------- | ------------------------------ |
| `demo.launch.py`              | MoveIt + RViz (main demo)      |
| `move_group.launch.py`        | move_group only (no RViz)      |
| `moveit_rviz.launch.py`       | RViz with MoveIt plugin only   |
| `spawn_controllers.launch.py` | Spawn ros2_control controllers |
| `rsp.launch.py`               | Robot state publisher only     |
| `setup_assistant.launch.py`   | MoveIt Setup Assistant         |

Example: run move_group without RViz:

```bash
ros2 launch so100_moveit_config move_group.launch.py
```

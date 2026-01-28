# SO100 Arm ROS2

This repository contains a Docker-based ROS2 Humble workspace for the SO100 robot arm. The workspace is fully containerized, providing a consistent development setup with all necessary tools and dependencies.

## Prerequisites

- Docker
- Docker Compose

## Getting Started

### 1. Build the Docker Container

Build the ROS2 development container:

```bash
docker-compose build
```

This will create a Docker image with ROS2 Humble and all necessary development tools.

### 2. Start the Container

Start the container in detached mode:

```bash
docker-compose up -d
```

### 3. Enter the Container

Enter the running container:

```bash
docker-compose exec ros2-dev bash
```

Once inside the container, you'll be in the ROS2 workspace at `/home/host_user/ros2_ws/`.

### 4. Install FTServo driver (submodule)

The SO100 arm uses the **ftservo_driver** library (git submodule). Build and install it before building ROS 2 packages:

```bash
cd ~/ros2_ws
git submodule update --init --recursive

cd src/ftservo_driver
mkdir -p build && cd build
cmake ..
make
sudo make install
cd ~/ros2_ws
```

### 5. Build the Workspace

Inside the container, build all packages:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

To build a specific package:

```bash
colcon build --packages-select <package_name>
```

### 6. Install Dependencies

Install ROS2 dependencies using rosdep:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 7. Stop the Container

When done, stop the container:

```bash
docker-compose down
```

## Get Started: Two Modes

After building and sourcing the workspace, you can run the SO100 arm in two ways.

### Mode 1: Bring up only (robot + controllers)

Start the arm and ros2_control controllers only. Use this for direct joint control, testing hardware, or when you do not need motion planning.

**Single terminal:**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch so100_bringup controllers.yaml
```

- Publishes `/joint_states` and loads `arm_controller` and `gripper_controller`.
- For real hardware, connect the arm to the host (e.g. `/dev/ttyACM0`) and ensure the **ftservo_driver** is installed.
- To use mock hardware (no servos), set `use_real_hardware` to `false` in `src/so100_bringup/config/so100_arm.urdf.xacro`, then launch as above.

### Mode 2: With MoveIt (motion planning + RViz)

Start the robot, then start MoveIt with RViz for motion planning and visualization.

**Terminal 1 – robot and controllers:**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch so100_bringup controllers.yaml
```

**Terminal 2 – MoveIt demo:**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch so100_moveit_config demo.launch.py
```

- Use the Motion Planning tab in RViz to plan and execute motions.
- See [so100_moveit_config](src/so100_moveit_config/README.md) for more launch options.

## Directory Structure

```markdown
.
├── docker/
│ ├── Dockerfile
│ ├── .env
│ └── ros_entrypoint.sh
├── docker-compose.yml
├── README.md
└── src/
├── ftservo_driver/ # FTServo library (git submodule)
├── so100_description/ # URDF and meshes
├── so100_hardware_interface/# ros2_control hardware interface
├── so100_bringup/ # Launch and config for robot + controllers
└── so100_moveit_config/ # MoveIt 2 config and launch
```

## Pre-commit Hooks

This repository uses pre-commit hooks to ensure code quality and formatting.

### Installation

1. Install pre-commit:

   ```bash
   pip install pre-commit
   ```

2. Install the hooks:

   ```bash
   pre-commit install
   ```

### Usage

Pre-commit hooks will run automatically on each commit. To run manually on all files:

```bash
pre-commit run --all-files
```

The hooks include:

- Trailing whitespace removal
- End-of-file fixer
- Python formatting with Black
- C++ formatting with clang-format
- Markdown formatting with Prettier

## Packages

| Package                      | Description                                                                                                      |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| **ftservo_driver**           | FTServo Linux library for FEETECH BUS servos (git submodule). Must be built and installed before ROS 2 packages. |
| **so100_description**        | SO100 arm URDF, meshes, and robot description.                                                                   |
| **so100_hardware_interface** | ros2_control hardware interface for the SO100 arm (real hardware via FTServo).                                   |
| **so100_bringup**            | Launch and config to bring up the robot and ros2_control controllers.                                            |
| **so100_moveit_config**      | MoveIt 2 configuration and launch files for motion planning and RViz.                                            |

## License

This repository is licensed under the **BSD 3-Clause License**. See [LICENSE](LICENSE) for the full text.

The SO100 packages (`so100_description`, `so100_bringup`, `so100_hardware_interface`, `so100_moveit_config`) are covered by this license. Submodules and other included projects (e.g. **ftservo_driver**) may have their own licenses; see their respective directories.

## Notes

- The container runs with your host user's UID to avoid permission issues
- GUI applications (RViz, etc.) work out of the box with X11 forwarding
- USB devices and video devices are accessible inside the container
- The container runs with host network mode for easy networking
- The `src/` directory is mounted inside the container at `/home/host_user/ros2_ws/src/`

## Actions

- [ ] Add a way to calibrate the servos
- [ ] Add a way to set calibration offsets and directions from config file

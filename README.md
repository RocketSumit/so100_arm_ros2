# SO100 Arm Development Workspace

This repository contains a Docker-based ROS2 Humble development environment for the SO100 robot arm. The environment is fully containerized, providing a consistent development setup with all necessary tools and dependencies.

## Prerequisites

- Docker
- Docker Compose
- NVIDIA Container Toolkit (for GPU support)

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

### 4. Build the Workspace

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

### 5. Install Dependencies

Install ROS2 dependencies using rosdep:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Stop the Container

When done, stop the container:

```bash
docker-compose down
```

## Directory Structure

```markdown
.
├── docker/
│ ├── Dockerfile
│ ├── .env
│ └── ros_entrypoint.sh
├── docker-compose.yml
├── README.md
└── src/ # ROS2 packages
├── so100_description/
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

```markdown
src/
├── so100_description/
```

## Notes

- The container runs with your host user's UID to avoid permission issues
- GUI applications (RViz, etc.) work out of the box with X11 forwarding
- USB devices and video devices are accessible inside the container
- The container runs with host network mode for easy networking
- The `src/` directory is mounted inside the container at `/home/host_user/ros2_ws/src/`

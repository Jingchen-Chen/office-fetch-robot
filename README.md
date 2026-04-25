# Office Fetch Robot

## Description
An autonomous mobile robot for office environments that navigates, detects, and retrieves objects using ROS2 + Gazebo simulation. This project integrates a 2D navigation stack with a vision-based object detection and segmentation module (U-Net) using the official `ipb_ros2_sim` framework.

## System Architecture Overview
The system is divided into two main components:
- **2D Navigation Stack**: Responsible for mapping, localization, and path planning using LIDAR and IMU data.
- **Vision Module (U-Net)**: Processes RGB-D camera feeds to perform semantic segmentation and object detection.
- **Simulation**: Uses `ipb_ros2_sim` with Jackal/Husky robots and ROS2 native Gazebo bridge.

## Repository Structure
```text
office-fetch-robot/
├── docker/                     # Docker configuration (Dockerfile, compose.yaml)
├── docs/                       # Project documentation
├── experiments/                # Training scripts and benchmark results
├── resources/                  # Large assets (meshes, datasets)
├── scripts/                    # Utility scripts
├── src/                        # ROS2 packages
│   ├── ipb_ros2_sim/           # Official simulation framework
│   ├── aruco_localization/     # ArUco marker based localization
│   ├── occupancy_mapping/      # Lidar-based occupancy grid mapping
│   ├── path_planning/          # A* planner and pure pursuit controller
│   ├── vision/                 # U-Net segmentation and baseline detection
│   └── integration/            # Top-level launch and mission control
└── README.md
```

## Quick Start (Docker Workflow)

### 1. Prerequisites
- Docker and Docker Compose installed.
- NVIDIA Container Toolkit (optional, for GPU acceleration).

### 2. Launch the Environment
```bash
cd office-fetch-robot/docker

# Build the image (includes PyTorch, OpenCV, ROS2 Jazzy)
docker compose build

# Start the container
docker compose up -d

# Enter the container
docker compose exec office_fetch_robot zsh
```

### 3. Build and Run
Inside the container:
```bash
# Build the project
cd ~/ros_ws
colcon build --symlink-install
source install/setup.zsh

# Launch the full stack (Simulation + Navigation + Vision)
ros2 launch integration full_stack_launch.py
```

## Topic Configuration (Namespace: `/P1_robot0/`)
| Component | Old Topic | New Topic |
|-----------|-----------|-----------|
| Lidar | `/scan` | `/P1_robot0/lidar_2d` |
| Camera | `/camera/image_raw` | `/P1_robot0/cam_front/image_raw` |
| Odometry | `/odom` | `/P1_robot0/wheel_odom` |
| Control | `/cmd_vel` | `/P1_robot0/cmd_vel` |

## Notes for Team Members
- **No GPU?**: If you don't have an NVIDIA GPU, edit `docker/compose.yaml` and comment out the `deploy` block.
- **Simulation Config**: The simulation defaults to `indoor` (office) config. Modify `full_stack_launch.py` to change `sim_config` if needed.

## Milestones
- **Presentation 1**: CW19 (May 3, 2026) - Project proposal and preliminary design.
- **Presentation 2**: CW27 (Early July 2026) - Mid-term progress and prototype demo.
- **Final Submission**: CW36 (Sept 1, 2026) - Final project report and code.

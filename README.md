# Office Fetch Robot (P14)

## Description
An autonomous mobile robot for office environments that navigates, detects, and retrieves objects using ROS2 + Gazebo simulation. This project aims to integrate a 2D navigation stack with a vision-based object detection and segmentation module to perform retrieval tasks in a dynamic office setting.

## System Architecture Overview
The system is divided into two main components:
- **2D Navigation Stack**: Responsible for mapping, localization, and path planning using LIDAR and IMU data.
- **Vision Module (U-Net)**: Processes RGB-D camera feeds to perform semantic segmentation and object detection for target localization.

## Tech Stack
- **ROS2**: Humble Hawksbill
- **Simulation**: Gazebo
- **Vision**: OpenCV, PyTorch (U-Net)
- **Programming Language**: Python 3, C++ (where performance is critical)

## Repository Structure
```text
office-fetch-robot/
├── build/                      # Colcon build output (ignored)
├── docs/                       # Project documentation
│   ├── architecture.md         # System design details
│   ├── devlog/                 # Daily/weekly development logs
│   ├── meeting-notes/          # Internal team discussion notes
│   └── presentations/          # Slides and presentation materials
├── experiments/                # Training scripts and benchmark results
│   ├── navigation_benchmarks/  # SLAM/Nav2 performance tests
│   └── unet_training/          # U-Net model training scripts
├── install/                    # Colcon install output (ignored)
├── log/                        # Colcon log output (ignored)
├── resources/                  # Large assets (meshes, datasets)
├── scripts/                    # Utility scripts (not ROS nodes)
├── src/                        # ROS2 packages (to be created)
├── .gitignore
└── README.md
```

## Quick Start Instructions
1. **Clone the repository**:
   ```bash
   git clone <repo_url>
   cd office-fetch-robot
   ```
2. **Build with colcon**:
   ```bash
   colcon build --symlink-install
   ```
3. **Source the environment**:
   ```bash
   source install/setup.bash
   ```
4. **Launch the simulation (TBD)**:
   ```bash
   ros2 launch fetch_robot_bringup fetch_robot_gazebo.launch.py
   ```

## Team Workflow
### Branch Naming
- `feature/xxx`: New features or components.
- `fix/xxx`: Bug fixes.
- `docs/xxx`: Documentation updates.
- `chore/xxx`: Maintenance tasks (CI/CD, dependencies).

### PR Review Policy
- At least one approval from another team member is required before merging to `main`.
- All CI checks must pass.

### Commit Convention
- Use descriptive commit messages.
- Format: `<type>(<scope>): <subject>` (e.g., `feat(vision): add U-Net inference node`).

## Milestones
- **Presentation 1**: CW19 (May 3, 2026) - Project proposal and preliminary design.
- **Presentation 2**: CW27 (Early July 2026) - Mid-term progress and prototype demo.
- **Final Submission**: CW36 (Sept 1, 2026) - Final project report and code.

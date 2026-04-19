# System Architecture

## Overall Pipeline
The Office Fetch Robot follows a sequential loop for navigation and retrieval:
1. **Perception**: Camera feeds are processed for object detection and environment mapping.
2. **Mapping**: 2D Occupancy Grid Maps are generated using LIDAR data.
3. **Planning**: Path planning algorithms (Nav2) compute a trajectory to the target object.
4. **Execution**: Controllers (PID/MPC) translate paths into velocity commands for the robot base.

**Pipeline Flow:**
`Camera → ARUCO Detection → Pose Estimation → Occupancy Mapping → Path Planning → Controller → Robot`

## Vision Pipeline
The vision module specializes in identifying the target object within a cluttered office environment.
1. **Image Capture**: Raw RGB feed from the robot's camera.
2. **U-Net Segmentation**: Semantic segmentation mask identifying pixels belonging to the target class (e.g., "cup", "bottle").
3. **Object Detection**: Bounding box extraction from the segmentation mask.
4. **Target Localization**: Projection of 2D coordinates to 3D space using depth data or known camera intrinsics.

## ROS2 Topic Architecture
Key topics utilized in the system:

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw RGB camera feed |
| `/segmentation/mask` | `sensor_msgs/Image` | U-Net segmentation output |
| `/aruco/pose` | `geometry_msgs/PoseStamped` | Estimated pose of target markers |
| `/scan` | `sensor_msgs/LaserScan` | LIDAR data for mapping/obstacle avoidance |
| `/map` | `nav_msgs/OccupancyGrid` | The environment map |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Current navigation target |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for the robot base |

## Module Dependencies Diagram
```text
[ Camera ] ------> [ Vision Module (U-Net) ] ------> [ Target Pose ]
    |                                                     |
    |                                                     v
[ LIDAR  ] ------> [ SLAM / Localization ] --------> [ Path Planner ]
                                                          |
                                                          v
[ Robot Hardware/Gazebo ] <------------------------ [ Controller ]
```


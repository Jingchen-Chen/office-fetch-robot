# Devlog: 2026-04-19 Project Kickoff

## Summary
Official start of the Office Fetch Robot (P14) project. Established the initial repository structure and agreed on the core development workflow.

## Objectives
- [x] Set up the ROS2 project workspace.
- [x] Create initial documentation (README, Architecture).
- [x] Define team collaboration standards (branching, commits).

## Work Done
- Initialized the repository structure with standard ROS2 folders.
- Drafted the system architecture including the vision and navigation pipelines.
- Configured `.gitignore` to handle ROS2, Python, and ML-specific files.
- Created placeholder directories for experiments and meeting notes.

## Challenges & Solutions
- **Challenge**: Deciding on the ROS2 version.
- **Solution**: Chose **Humble Hawksbill** as it is the current Long-Term Support (LTS) version with the best stability for our toolset (Nav2, Gazebo).

## Next Steps
- Initialize ROS2 packages in the `src/` directory.
- Set up the Gazebo simulation environment with a basic office world.
- Begin U-Net model data collection for office object segmentation.

## Notes
- Team meetings scheduled for every Thursday at 16:10 PM.
- Initial focus: Integrating a mobile base in Gazebo and verifying LIDAR-based SLAM.

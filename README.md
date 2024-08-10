# Appras_Bearcar_24

The objective of this repository is to integrate the Isaac ROS Visual SLAM on the Bearcar.

## Requirements

1. Jetson Xavier AGX 
2. At least 128GB SD Card (SSD recommended)
3. Intel Realsense D455 (D435i)
4. URDF of the Robot 
5. Jetpack 5.1.2

## How to use ?

### Isaac ROS Docker Environment

This project based on the Isaac ROS Dev Docker image, which contains predefined dependencies and settings. 
On top of the base image, we add our own layer in order to add additional packages.

The following table shows different alias for the environment:


| Alias                              | Programm Call | Description  |
|----------                          |----------     |----------    |
| go                                 | cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \ ./scripts/run_dev.sh            | Launch the Isaac ROS Docker Container (specified in the .bashrc outside of the container)                                        |
| sr                                 | source /workspaces/isaac_ros-dev/install/setup.bash                          | Source the workspace (workspace will be sourced automatically after start the container initially or attach a new container)     |
| cb                                 | colcon build --symlink-install                                               | Build the workspace                                                                                                              |  
| vslam_go                           | ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py  | Start the Isaac ROS Visual SLAM Node with Realsense node and the frames of Bearcar                                               |
| keypoints                          | ros2 run keypoints_visualizer vslam_sim                                      | Show projected 3D points of the Pointcloud2 Topic onto the 2D image plane                                                        |




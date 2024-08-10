# Appras_Bearcar_24

The objective of this repository is to integrate the Isaac ROS Visual SLAM on the Bearcar.

## Requirements

1. Jetson Xavier AGX 
2. At least 128GB SD Card (SSD recommended)
3. Intel Realsense D455 (D435i)
4. URDF of the Robot 
5. Jetpack 5.1.2

## How to use ?

### Docker-Environment

This project based on the Isaac ROS Dev Docker image, which contains predefined dependencies and settings. 
On top of the base image, we add our own layer in order to add additional packages.
Additionally we added the command 

<hr>

# Usage

## Start the camera node
  
  #### with ros2 run:
    ros2 run realsense2_camera realsense2_camera_node
    # or, with parameters, for example - temporal and spatial filters are enabled:
    ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true
  
  #### with ros2 launch:
    ros2 launch realsense2_camera rs_launch.py
    ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true

<hr>

The following table show different alias for the environment:


| Alias                              | Programm Call            | Description                                                                        |
|----------                          |----------       |----------                                                                          |
| go                                 | run_dev       | random, staged, scenario, parametrized, dynamic_map_random, dynamic_map_staged     |
|                                    |                 |                                                                         |
|                                    |                 |                                                                         |
|                                    |                 |                                                                         |




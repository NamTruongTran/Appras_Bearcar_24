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


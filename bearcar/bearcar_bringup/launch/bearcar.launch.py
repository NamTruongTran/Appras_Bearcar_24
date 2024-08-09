import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('bearcar_description'))
    xacro_file = os.path.join(pkg_path,'urdf','bearcar.urdf.xacro') 
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time]) # Run a terminal command as part of the launch process.
                                                                                                                                     # In this specific case, it runs the xacro command
                                                                                                                                     # Xacro is a language used to generate URDF (Unified Robot Description Format) files. 
                                                                                                                                     # URD files describe the physical and visual properties of a robot 
                                                                                                                                     # -> The Xacro command processes a .xacro file and outputs a URDF
                                                                                                                                     # Runs: xacro /home/nartmangnourt/APP-RAS_ws/src/install/bearcar_description/share/bearcar_description/urdf/robot.urdf.xacro use_ros2_control:=true sim_mode:=false

    rviz_config_dir = os.path.join(
        pkg_path,
        'rviz',
        'bearcar.rviz')

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time} # Dict

    node_robot_state_publisher = Node(                                                     # Create a Node for the robot_state_publisher package. This node publish the state of the robot to the ROS2 network
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
#    node_rviz2 = #Node(                                                     
#        package='rviz2',
#        executable='rviz2',
#        name='rviz2',
#        arguments=['-d', rviz_config_dir],
#        parameters=[{'use_sim_time': use_sim_time}],
#        output='screen'
#    )
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Use ros2_control if true'),

        node_robot_state_publisher,
        node_joint_state_publisher_gui,
#        node_rviz2,
    ])

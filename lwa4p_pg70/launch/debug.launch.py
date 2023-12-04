#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.substitutions import  PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # xacro_file = "lwa4p_pg70.urdf.xacro"
    # #xacro_file = "box_bot.xacro"
    # package_description = "lwa4p_pg70"
    # robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)
    # robot_desc = xacro.process_file(robot_desc_path)
    # xml = robot_desc.toxml()
    # Get Gazebo ROS interface package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Get the location for empty world
    world = os.path.join(
        get_package_share_directory('lwa4p_pg70'),
        'worlds',
        'empty_world.world'
    )

    # Launch Description to run Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Launch Description to run Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Get the package directory 
    pkg_gazebo = get_package_share_directory('lwa4p_pg70')

   

    # Launch Decription to Spawn Robot Model 
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch',
                         'spawn_robot_ros2.launch.py'),
        )
    )

     # RVIZ Configuration
    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("lwa4p_pg70"), "rviz", "display_default.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])
    
    # # Publish Robot Desciption in String form in the topic /robot_description
    # publish_robot_description = Node(
    #     package='lwa4p_pg70',
    #     executable='robot_description_publisher.py',
    #     name='robot_description_publisher',
    #     output='screen',
    #     arguments=['-xml_string', xml,
    #                '-robot_description_topic', '/robot_description'
    #                ]
    # )

    #     # Robot State Publisher
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # robot_state_publisher= Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': xml}],
    #     output="screen"
    # )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    # )
    
    joint_state_gui=Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )
    
    # Launch Description 
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_world,
        rviz_node,
        joint_state_gui,
        # publish_robot_description,
        # robot_state_publisher,
        # joint_state_publisher_node
        
    ])
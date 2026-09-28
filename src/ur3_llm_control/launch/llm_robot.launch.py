#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Master Launch File: llm_robot.launch.py
Khoi dong tron goi 1 lenh duy nhat:
1. Mo phong Ignition Gazebo voi the gioi custom (Ban thao tac, 3 khoi hop mau, 3 vung dich Zone A, B, C).
2. Robot UR3/UR3e gan tay kep 2 ngon co khi (Gripper).
3. MoveIt 2 Motion Planning & Kinematics.
4. RViz 2 hien thi 3D truc quan (Robot, Scene Markers, Zone Labels, Planning).
5. Scene Spawner cap nhat vat can va vat the.
6. LLM Interactive Node tiep nhan cau lenh va dieu khien robot.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            choices=["ur3", "ur3e"],
            description="Dong robot UR muon su dung (ur3 hoac ur3e).",
        ),
        DeclareLaunchArgument(
            "start_sim",
            default_value="true",
            description="Khoi chay toan bo mo phong Gazebo va MoveIt 2 (true/false).",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Mo giao dien RViz 2 tich hop san Marker vat the va zone (true/false).",
        ),
        DeclareLaunchArgument(
            "interactive",
            default_value="true",
            description="Cho phep nhap cau lenh truc tiep tu ban phim console terminal.",
        ),
        DeclareLaunchArgument(
            "command",
            default_value="",
            description="Cau lenh khoi chay thuc hien ngay (vi du: 'Put the red cube in zone B').",
        ),
        DeclareLaunchArgument(
            "start_delay",
            default_value="15.0",
            description="Thoi gian cho (giay) de Gazebo va MoveIt san sang truoc khi node LLM bat dau.",
        ),
    ]

    ur_type = LaunchConfiguration("ur_type")
    start_sim = LaunchConfiguration("start_sim")
    launch_rviz = LaunchConfiguration("launch_rviz")
    interactive = LaunchConfiguration("interactive")
    command = LaunchConfiguration("command")
    start_delay = LaunchConfiguration("start_delay")

    # 1. Duong dan toi file World custom va Gripper Xacro
    world_file = PathJoinSubstitution(
        [FindPackageShare("ur3_llm_control"), "worlds", "table_cubes_zones.sdf"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur3_llm_control"), "rviz", "ur3_llm.rviz"]
    )

    # 2. Khoi chay Gazebo Control (UR Simulation)
    ur_sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "world_file": world_file,
            "description_package": "ur3_llm_control",
            "description_file": "ur_with_gripper.urdf.xacro",
            "launch_rviz": "false",
        }.items(),
        condition=IfCondition(start_sim),
    )

    # 3. Khoi chay MoveIt 2
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": "ur3_llm_control",
            "description_file": "ur_with_gripper.urdf.xacro",
            "launch_rviz": "false",
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(start_sim),
    )

    # 4. RViz 2 hien thi giao dien 3D day du
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_ur3_llm",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
    )

    # 5. Scene Spawner (Phat vat the, vung mau va collision objects)
    scene_spawner_node = Node(
        package="ur3_llm_control",
        executable="scene_spawner",
        name="scene_spawner_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 6. LLM Interactive Node (Chay co do tre de cho MoveIt khoi tao xong)
    llm_node = Node(
        package="ur3_llm_control",
        executable="llm_interactive_node",
        name="llm_interactive_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "interactive": interactive,
                "command": command,
            }
        ],
    )

    delayed_llm_node = TimerAction(
        period=start_delay,
        actions=[llm_node],
    )

    return LaunchDescription(
        declared_arguments
        + [
            ur_sim_control_launch,
            ur_moveit_launch,
            rviz_node,
            scene_spawner_node,
            delayed_llm_node,
        ]
    )

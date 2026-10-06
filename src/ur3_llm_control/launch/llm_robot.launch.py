#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Master Launch File: llm_robot.launch.py
Khoi dong tron goi 1 lenh duy nhat:
1. Mo phong Ignition Gazebo voi the gioi custom (Ban thao tac, 5 khoi hop mau, 3 vung dich Zone A, B, C).
2. Robot UR3/UR3e gan tay kep 2 ngon co khi (Gripper).
3. MoveIt 2 Motion Planning & Kinematics.
4. RViz 2 hien thi 3D truc quan (Robot, Scene Markers, Zone Labels, Planning).
5. Scene Spawner cap nhat vat can va vat the.
6. LLM Interactive Node tiep nhan cau lenh va dieu khien robot.
"""

import os
os.environ["ROS_LOCALHOST_ONLY"] = "1"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
            "gazebo_gui",
            default_value="true",
            description="Mo giao dien do hoa Gazebo GUI (true/false).",
        ),
        DeclareLaunchArgument(
            "start_rviz",
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
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    start_rviz = LaunchConfiguration("start_rviz")
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
            "runtime_config_package": "ur3_llm_control",
            "controllers_file": "ur_controllers.yaml",
            "launch_rviz": "false",
            "gazebo_gui": gazebo_gui,
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
            "moveit_config_package": "ur3_llm_control",
            "moveit_config_file": "ur_with_gripper.srdf.xacro",
            "launch_rviz": "false",
            "launch_servo": "false",
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(start_sim),
    )

    # 4. RViz 2 parameters (robot_description, semantic, kinematics)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur3_llm_control"), "urdf", "ur_with_gripper.urdf.xacro"]
            ),
            " ",
            "name:=ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "safety_limits:=true",
            " ",
            "prefix:=\"\"",
            " ",
            "joint_limit_params:=",
            PathJoinSubstitution(
                [FindPackageShare("ur3_llm_control"), "config", ur_type, "joint_limits.yaml"]
            ),
            " ",
            "kinematics_params:=",
            PathJoinSubstitution(
                [FindPackageShare("ur3_llm_control"), "config", ur_type, "default_kinematics.yaml"]
            ),
            " ",
            "physical_params:=",
            PathJoinSubstitution(
                [FindPackageShare("ur3_llm_control"), "config", ur_type, "physical_parameters.yaml"]
            ),
            " ",
            "visual_params:=",
            PathJoinSubstitution(
                [FindPackageShare("ur3_llm_control"), "config", ur_type, "visual_parameters.yaml"]
            ),
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur3_llm_control"), "srdf", "ur_with_gripper.srdf.xacro"]),
            " ",
            "name:=ur",
            " ",
            "prefix:=\"\"",
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    # RViz 2 hien thi giao dien 3D day du
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_ur3_llm",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": True},
        ],
        condition=IfCondition(start_rviz),
    )

    # 5. Scene Spawner (Phat vat the, vung mau va collision objects)
    scene_spawner_node = Node(
        package="ur3_llm_control",
        executable="scene_spawner",
        name="scene_spawner_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 5.1 Camera & Gripper Bridge: Chuyen hinh anh va topic dieu khien ngon tay gripper
    camera_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_gripper_bridge",
        arguments=[
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/gripper/left_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/gripper/right_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/gripper/left_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            "/gripper/right_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
        ],
        output="screen",
    )

    # 5.2 Camera Perception Node (OpenCV xu ly anh nhan dien vi tri 5 vat the va trang thai cac zone)
    camera_perception_node = Node(
        package="ur3_llm_control",
        executable="camera_perception",
        name="camera_perception_node",
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
                # Input chi do user_console rieng gui qua /user_command. Khong
                # tao them thread input() trong launch terminal de tranh tranh stdin.
                "interactive": False,
                "command": command,
            }
        ],
    )

    delayed_llm_node = TimerAction(
        period=start_delay,
        actions=[llm_node],
    )

    # 7. Tu dong mo cua so Terminal rieng biet de nguoi dung nhap cau lenh truc tiep
    user_console_cmd = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--title=UR3 LLM Command Console",
            "--",
            "ros2",
            "run",
            "ur3_llm_control",
            "user_console",
        ],
        condition=IfCondition(interactive),
    )
    delayed_console = TimerAction(
        period=17.0,
        actions=[user_console_cmd],
    )

    return LaunchDescription(
        declared_arguments
        + [
            ur_sim_control_launch,
            ur_moveit_launch,
            rviz_node,
            scene_spawner_node,
            camera_bridge_node,
            camera_perception_node,
            delayed_llm_node,
            delayed_console,
        ]
    )

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []

    # 1. Các tham số cấu hình chữ vẽ / ảnh
    declared_arguments.append(
        DeclareLaunchArgument(
            "letter",
            default_value="",
            description="Chu cai hoac tu muon ve (vi du: 'A', 'B', 'UR3'). De trong de dung anh letter.png mac dinh.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "image_path",
            default_value="",
            description="Duong dan toi file anh muon ve (de trong se dung file letter.png mac dinh cua package).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "target_width",
            default_value="0.15",
            description="Chieu rong cua net chu / hinh ve (don vi: met, mac dinh 0.15m = 15cm).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "interactive",
            default_value="false",
            description="Bat che do hoi nguoi dung nhap chu tu ban phim console truoc khi ve (true/false).",
        )
    )

    # 2. Các tham số mô phỏng (tùy chọn khởi động kèm Gazebo & MoveIt)
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_sim",
            default_value="false",
            description="Tu dong khoi dong ca mo phong Gazebo va MoveIt (true/false).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e"],
            description="Dong robot UR muon su dung.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Mo giao dien RViz tich hop san Marker ve chu khi start_sim=true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_delay",
            default_value="20.0",
            description="Thoi gian cho (giay) de Gazebo va MoveIt san sang truoc khi node ve chu bat dau chay.",
        )
    )

    # Khởi tạo LaunchConfiguration
    letter = LaunchConfiguration("letter")
    image_path = LaunchConfiguration("image_path")
    target_width = LaunchConfiguration("target_width")
    interactive = LaunchConfiguration("interactive")
    start_sim = LaunchConfiguration("start_sim")
    ur_type = LaunchConfiguration("ur_type")
    start_rviz = LaunchConfiguration("start_rviz")
    start_delay = LaunchConfiguration("start_delay")

    # Xây dựng robot_description, robot_description_semantic và kinematics để nạp cho Node và RViz
    # Giúp loại bỏ hoàn toàn các cảnh báo "No kinematics plugins defined" và "No root/virtual joint specified"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "name:=ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "safety_limits:=true",
            " ",
            "prefix:=\"\"",
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
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=ur",
            " ",
            "prefix:=\"\"",
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    # File cấu hình RViz riêng của package đã cài đặt sẵn display Marker /visualization_marker
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_ur3_draw"), "rviz", "draw_letter.rviz"]
    )

    # Node vẽ chữ
    draw_letter_node = Node(
        package="my_ur3_draw",
        executable="draw_letter_node",
        name="draw_letter_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {
                "use_sim_time": True,
                "letter": letter,
                "image_path": image_path,
                "target_width": target_width,
                "interactive": interactive,
            }
        ],
    )

    # Node RViz2 với cấu hình draw_letter.rviz (đã add sẵn Marker topic /visualization_marker)
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(start_rviz),
        executable="rviz2",
        name="rviz2_draw_letter",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": True},
        ],
    )

    # Nhóm 1: Khi KHÔNG bật start_sim (mặc định khi chạy lẻ node)
    run_direct_action = GroupAction(
        condition=UnlessCondition(start_sim),
        actions=[draw_letter_node],
    )

    # Nhóm 2: Khi BẬT start_sim=true (Cách 1: Khởi động trọn gói 1 lệnh)
    # Tắt launch_rviz trong ur_sim_moveit để RViz tùy biến có sẵn Marker của chúng ta được dùng
    sim_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch", "/ur_sim_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            "use_sim_time": "true",
        }.items(),
    )

    delayed_draw_letter_node = TimerAction(
        period=start_delay,
        actions=[draw_letter_node],
    )

    run_with_sim_action = GroupAction(
        condition=IfCondition(start_sim),
        actions=[
            sim_moveit_launch,
            rviz_node,
            delayed_draw_letter_node,
        ],
    )

    return LaunchDescription(
        declared_arguments + [run_direct_action, run_with_sim_action]
    )

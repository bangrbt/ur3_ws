from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "letter",
            default_value="",
            description="Chu cai hoac tu muon ve (vi du: 'A', 'B', 'UR3'). De trong de dung anh letter.png mac dinh.",
        ),
        DeclareLaunchArgument(
            "image_path",
            default_value="",
            description="Duong dan toi file anh muon ve (de trong se dung file letter.png mac dinh cua package).",
        ),
        DeclareLaunchArgument(
            "target_width",
            default_value="0.15",
            description="Chieu rong cua net chu / hinh ve (don vi: met, mac dinh 0.15m = 15cm).",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e"],
            description="Dong robot UR muon su dung trong mo phong.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Mo giao dien RViz.",
        ),
        DeclareLaunchArgument(
            "start_delay",
            default_value="20.0",
            description="Thoi gian cho (giay) de Gazebo va MoveIt san sang truoc khi node ve chu chay.",
        ),
    ]

    letter = LaunchConfiguration("letter")
    image_path = LaunchConfiguration("image_path")
    target_width = LaunchConfiguration("target_width")
    ur_type = LaunchConfiguration("ur_type")
    start_rviz = LaunchConfiguration("start_rviz")
    start_delay = LaunchConfiguration("start_delay")

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("my_ur3_draw"), "/launch", "/draw_letter.launch.py"]
        ),
        launch_arguments={
            "letter": letter,
            "image_path": image_path,
            "target_width": target_width,
            "start_sim": "true",
            "ur_type": ur_type,
            "start_rviz": start_rviz,
            "start_delay": start_delay,
            "interactive": "false",
        }.items(),
    )

    return LaunchDescription(declared_arguments + [included_launch])


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    nav2_params = LaunchConfiguration("nav2_params")

    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    nav2_params_default = PathJoinSubstitution([
        FindPackageShare("go2_nav_rtabmap"),
        "config",
        "nav2_rtabmap.yaml"
    ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ])
        ),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("nav2_params", default_value=nav2_params_default),

        nav2_launch,
    ])

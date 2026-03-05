from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    # RTAB-Map args
    rtabmap_args = LaunchConfiguration("rtabmap_args")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    frame_id = LaunchConfiguration("frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    odom_topic = LaunchConfiguration("odom_topic")
    approx_sync = LaunchConfiguration("approx_sync")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    approx_sync_max_interval = LaunchConfiguration("approx_sync_max_interval")
    qos = LaunchConfiguration("qos")
    qos_camera_info = LaunchConfiguration("qos_camera_info")
    qos_image = LaunchConfiguration("qos_image")
    rgb_transport = LaunchConfiguration("rgb_transport")
    depth_transport = LaunchConfiguration("depth_transport")
    subscribe_odom = LaunchConfiguration("subscribe_odom")

    rviz_config = PathJoinSubstitution([
        FindPackageShare("go2_nav_rtabmap"),
        "rviz",
        "go2_custom.rviz"
    ])

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rtabmap_launch"),
                "launch",
                "rtabmap.launch.py"
            ])
        ),
        launch_arguments={
            "args": rtabmap_args,
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "frame_id": frame_id,
            "odom_frame_id": odom_frame_id,
            "odom_topic": odom_topic,
            "approx_sync": approx_sync,
            "sync_queue_size": sync_queue_size,
            "approx_sync_max_interval": approx_sync_max_interval,
            "qos": qos,
            "qos_camera_info": qos_camera_info,
            "qos_image": qos_image,
            "rgb_transport": rgb_transport,
            "depth_transport": depth_transport,
            "use_sim_time": use_sim_time,
            "subscribe_odom": subscribe_odom,
        }.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": sim_time_param}],
    )

    align_node = Node(
        package="go2_nav_rtabmap",
        executable="align_odoms",
        name="align_odoms",
        output="screen",
        parameters=[
            {"use_sim_time": sim_time_param},
            {"robot_odom_frame": "odom"},
            {"rtabmap_odom_frame": "rtabmap/odom"},
            {"base_frame": "base_link"},
            {"initialpose_frame": "rtabmap/odom"},
            {"initialpose_topic": "/initialpose"},
            {"publish_rate_hz": 30.0},
        ],
    )
    
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        arguments=[
            "0.35", "0", "0.1",
            "0", "0.57", "0",
            "base_link", "camera_link"
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        DeclareLaunchArgument(
            "rtabmap_args",
            default_value=TextSubstitution(text="--delete_db_on_start --Rtabmap/DetectionRate 0.5"),
        ),
        DeclareLaunchArgument("rgb_topic", default_value=TextSubstitution(text="/camera/color/image_raw")),
        DeclareLaunchArgument("depth_topic", default_value=TextSubstitution(text="/camera/aligned_depth_to_color/image_raw")),
        DeclareLaunchArgument("camera_info_topic", default_value=TextSubstitution(text="/camera/aligned_depth_to_color/camera_info")),
        DeclareLaunchArgument("frame_id", default_value=TextSubstitution(text="base_link")),
        DeclareLaunchArgument("odom_frame_id", default_value=TextSubstitution(text="rtabmap/odom")),
        DeclareLaunchArgument("odom_topic", default_value=TextSubstitution(text="/rtabmap/odom")),
        DeclareLaunchArgument("approx_sync", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("sync_queue_size", default_value=TextSubstitution(text="10")),
        DeclareLaunchArgument("approx_sync_max_interval", default_value=TextSubstitution(text="5.0")),
        DeclareLaunchArgument("qos", default_value=TextSubstitution(text="2")),
        DeclareLaunchArgument("qos_camera_info", default_value=TextSubstitution(text="2")),
        DeclareLaunchArgument("qos_image", default_value=TextSubstitution(text="2")),
        DeclareLaunchArgument("rgb_transport", default_value=TextSubstitution(text="compressed")),
        DeclareLaunchArgument("depth_transport", default_value=TextSubstitution(text="compressedDepth")),
        DeclareLaunchArgument("subscribe_odom", default_value=TextSubstitution(text="false")),

        rtabmap_launch,
        rviz_node,
        align_node,
        camera_tf,
    ])

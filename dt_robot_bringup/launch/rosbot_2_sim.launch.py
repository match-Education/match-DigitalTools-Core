from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import (
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="True",
        description=(
            "Whether to use mecanum drive controller "
            "(otherwise diff drive controller is used)"
        ),
    )

    use_gpu = LaunchConfiguration("use_gpu")
    declare_use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="True",
        description="Whether GPU acceleration is used",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rosbot",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0.0",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # an IR sensor is not implemented yet https://github.com/gazebosim/gz-sensors/issues/19
            "/range/fl@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/range/fr@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/range/rl@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/range/rr@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ],
        remappings=[
            ("/camera/camera_info", "/camera/color/camera_info"),
            ("/camera/image", "/camera/color/image_raw"),
            ("/camera/depth_image", "/camera/depth/image_raw"),
            ("/camera/points", "/camera/depth/points"),
        ],
        output="screen",
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": mecanum,
            "use_sim": "True",
            "use_gpu": use_gpu,
            "simulation_engine": "ignition-gazebo",
        }.items(),
    )

    # The frame of the pointcloud from ignition gazebo 6 isn't provided by <frame_id>.
    # See https://github.com/gazebosim/gz-sensors/issues/239
    depth_cam_frame_fixer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_to_camera",
        output="log",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "1.57",
            "-1.57",
            "0.0",
            "camera_depth_optical_frame",
            "rosbot/base_link/camera_orbbec_astra_camera",
        ],
    )

    return LaunchDescription(
        [
            declare_mecanum_arg,
            gz_spawn_entity,
            ign_bridge,
            bringup_launch,
            depth_cam_frame_fixer,
        ]
    )
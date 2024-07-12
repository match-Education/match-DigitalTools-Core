from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import ReplaceString

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Will be used as namespace for the underlying launch files and nodes
    robot_name = LaunchConfiguration("robot_name")
    declare_namespace_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description="Name of the spawned robot in Gazebo",
    )
    
    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="True",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)"
    )

    use_gpu = LaunchConfiguration("use_gpu")
    declare_use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="True",
        description="Whether GPU acceleration is used",
    )

    activate_obstacle_detection = LaunchConfiguration("activate_obstacle_detection")
    declare_activate_obstacle_detection_arg = DeclareLaunchArgument(
        "activate_obstacle_detection",
        default_value="True",
        description="Activates the node that evaluates the laser scanner and publishes 8 zones.",
    )

    x = LaunchConfiguration("x")
    declare_x_arg = DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="x-position of the mobile robot.",
    )

    y = LaunchConfiguration("y")
    declare_y_arg = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="y-position of the mobile robot.",
    )

    z = LaunchConfiguration("z")
    declare_z_arg = DeclareLaunchArgument(
        "z",
        default_value="0.0",
        description="z-position of the mobile robot.",
    )

    R = LaunchConfiguration("R")
    declare_R_arg = DeclareLaunchArgument(
        "R",
        default_value="0.0",
        description="roll-orientation of the mobile robot.",
    )

    P = LaunchConfiguration("P")
    declare_P_arg = DeclareLaunchArgument(
        "P",
        default_value="0.0",
        description="pitch-orientation of the mobile robot.",
    )

    Y = LaunchConfiguration("Y")
    declare_Y_arg = DeclareLaunchArgument(
        "Y",
        default_value="0.0",
        description="yaw-orientation of the mobile robot.",
    )
    
    # The following include is copied and adapted from the package rosbot_gazebo and spawn.launch.py
    rosbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_gazebo"),
                    "launch",
                    "spawn.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": mecanum,
            "use_sim": "True",
            "use_gpu": use_gpu,
            "simulation_engine": "ignition-gazebo",
            "namespace": robot_name,
            "x": x,
            "y": y,
            "z": z,
            "roll": R,
            "pitch": P,
            "yaw": Y,
        }.items(),
    )

    # Node for easier usage of the laser scan data for collision detection
    # Later the namespace should be added
    obstacle_detection = Node(
        condition=IfCondition(PythonExpression([activate_obstacle_detection])),
        package="dt_sensor_helper",
        executable="simplify_laser_scan_class_node",
        name="simplify_laser_scan_class",
        output="log",
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_use_gpu_arg,
            declare_activate_obstacle_detection_arg,
            declare_x_arg,
            declare_y_arg,
            declare_z_arg,
            declare_R_arg,
            declare_P_arg,
            declare_Y_arg,
            rosbot,
            obstacle_detection
        ]
    )
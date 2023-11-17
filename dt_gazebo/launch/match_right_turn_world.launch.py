
from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
)

from launch.substitutions import (
    PathJoinSubstitution,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file = PathJoinSubstitution([get_package_share_directory("dt_gazebo"), 
                                       "worlds", 
                                       "match_right_turn_world.sdf"])
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("dt_gazebo"),
                    "launch",
                    "gazebo_world.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": world_file,
        }.items(),
    )

    return LaunchDescription(
        [
            gz_sim
        ]
    )

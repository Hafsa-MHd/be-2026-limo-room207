"""School Room 207 sim + joint_state_publisher + road_vision. No /cmd_vel.

Uses qcar_large_track by default (stable for perception). Override with:
  world:=.../ensem_room207.world spawn_z:=0.35
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    room = get_package_share_directory("room207_limo")
    simple = get_package_share_directory("limo_simple")
    qcar = get_package_share_directory("qcar_large_track")
    params = os.path.join(simple, "config", "road_vision.yaml")
    save_path = os.path.expanduser("~/ros2_ws_simple/captures/road_vision.png")
    default_world = os.path.join(qcar, "worlds", "qcar_large_track.world")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world (qcar track default; use ensem_room207 if wanted).",
    )
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.15",
        description="LIMO spawn height (0.15 on flat track, 0.35 on Room 207 platform).",
    )

    school = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(room, "launch", "room207_limo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "spawn_z": LaunchConfiguration("spawn_z"),
        }.items(),
    )

    # Wheel TFs for RViz RobotModel (Gazebo often does not publish /joint_states).
    joint_state_publisher = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": True, "rate": 30.0}],
            )
        ],
    )

    road_vision = Node(
        package="limo_simple",
        executable="road_vision",
        name="road_vision",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": True, "save_path": save_path},
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            spawn_z_arg,
            school,
            joint_state_publisher,
            road_vision,
        ]
    )

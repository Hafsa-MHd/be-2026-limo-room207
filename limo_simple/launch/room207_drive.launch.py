"""School sim + force parking-bay spawn + vision + polar/lane control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    room = get_package_share_directory("room207_limo")
    simple = get_package_share_directory("limo_simple")
    qcar = get_package_share_directory("qcar_large_track")
    vision_params = os.path.join(simple, "config", "road_vision.yaml")
    follow_params = os.path.join(simple, "config", "lane_follow.yaml")
    polar_params = os.path.join(simple, "config", "polar_follow.yaml")
    default_world = os.path.join(qcar, "worlds", "qcar_large_track.world")

    # School launch hardcodes road spawn (-0.725, -0.087). We override to parking.
    world_arg = DeclareLaunchArgument("world", default_value=default_world)
    spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.15")
    control_arg = DeclareLaunchArgument("control", default_value="polar")
    south_route_arg = DeclareLaunchArgument("d8_south_route", default_value="false")
    # Official GitLab Room 207 spawn (just below parking bay, heading N).
    park_x = DeclareLaunchArgument("park_x", default_value="-0.725347")
    park_y = DeclareLaunchArgument("park_y", default_value="-0.086613")
    park_yaw = DeclareLaunchArgument("park_yaw", default_value="1.5708")

    school = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(room, "launch", "room207_limo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "spawn_z": LaunchConfiguration("spawn_z"),
        }.items(),
    )

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

    # After school road spawn (~3s), move into parking stall ASAP.
    spawn_parking = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="limo_simple",
                executable="spawn_parking",
                name="spawn_parking",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "x": ParameterValue(LaunchConfiguration("park_x"), value_type=float),
                        "y": ParameterValue(LaunchConfiguration("park_y"), value_type=float),
                        "yaw": ParameterValue(LaunchConfiguration("park_yaw"), value_type=float),
                        "z": ParameterValue(LaunchConfiguration("spawn_z"), value_type=float),
                        "wait_school_sec": 2.0,
                        "tol_m": 0.40,
                    }
                ],
            )
        ],
    )

    road_vision = Node(
        package="limo_simple",
        executable="road_vision",
        name="road_vision",
        output="screen",
        parameters=[vision_params, {"use_sim_time": True, "save_path": ""}],
    )

    is_polar = PythonExpression(['"', LaunchConfiguration("control"), '" == "polar"'])

    polar_follow = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="limo_simple",
                executable="polar_follow",
                name="polar_follow",
                output="screen",
                parameters=[polar_params, {
                    "use_sim_time": True,
                    "d8_south_route": ParameterValue(LaunchConfiguration("d8_south_route"), value_type=bool),
                }],
            )
        ],
        condition=IfCondition(is_polar),
    )

    lane_follow = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="limo_simple",
                executable="lane_follow",
                name="lane_follow",
                output="screen",
                parameters=[follow_params, {"use_sim_time": True}],
            )
        ],
        condition=UnlessCondition(is_polar),
    )

    return LaunchDescription(
        [
            world_arg,
            spawn_z_arg,
            control_arg,
            south_route_arg,
            park_x,
            park_y,
            park_yaw,
            school,
            joint_state_publisher,
            spawn_parking,
            road_vision,
            polar_follow,
            lane_follow,
        ]
    )

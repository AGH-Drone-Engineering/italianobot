import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import TimerAction
from launch.actions import ExecuteProcess


home_dir = os.environ["HOME"]
map_file = os.path.join(
    home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/map/map"
)
minimal_publisher_path = os.path.join(
    home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/minimal_publisher.py"
)
EXPLORE_MINUTES = 6  # if this parameter is changed, please change it also in minimal_publisher in timer_callback
SAVING_TIME_SECONDS = 10  # if this parameter is changed, please change it also in minimal_publisher in timer_callback


def generate_launch_description():
    # configuration

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    itabot_nav_dir = get_package_share_directory("itabot_nav")
    itabot_nav_launch_dir = os.path.join(itabot_nav_dir, "launch")

    params_file = os.path.join(itabot_nav_dir, "config", "rosbot_autonomy.yaml")

    explore_lite_launch = os.path.join(
        get_package_share_directory("explore_lite"), "launch", "explore.launch.py"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = "True"
    use_respawn = "False"
    log_level = "Info"

    # Create our own temporary YAML files that include substitutions

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    param_substitutions = {
        "use_sim_time": use_sim_time,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # declare launch args

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Specify the actions

    bringup_cmd_group = GroupAction(
        [
            Node(
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(itabot_nav_launch_dir, "slam.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "params_file": params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": "True",
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(explore_lite_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                }.items(),
            ),
            TimerAction(
                period=EXPLORE_MINUTES * 60.0,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_saver_cli",
                        output="screen",
                        arguments=["-f", map_file],
                    ),
                ],
            ),
            TimerAction(
                period=EXPLORE_MINUTES * 60.0 + SAVING_TIME_SECONDS,
                actions=[
                    ExecuteProcess(
                        cmd=["python3", minimal_publisher_path], output="screen"
                    ),
                ],
            ),
        ]
    )

    # Create the launch description and populate

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
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
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
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

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace for all topics and tfs",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    use_gpu = LaunchConfiguration("use_gpu")
    declare_use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="False",
        description="Whether GPU acceleration is used",
    )

    simulation_engine = LaunchConfiguration("simulation_engine")
    declare_simulation_engine_arg = DeclareLaunchArgument(
        "simulation_engine",
        default_value="webots",
        description="Which simulation engine to be used",
        choices=["ignition-gazebo", "gazebo-classic", "webots"],
    )

    rosbot_controller = get_package_share_directory("rosbot_controller")
    rosbot_bringup = get_package_share_directory("rosbot_bringup")

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description=(
            "Whether to use mecanum drive controller (otherwise diff drive controller is used)"
        ),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_controller,
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim": use_sim,
            "mecanum": mecanum,
            "use_gpu": use_gpu,
            "simulation_engine": simulation_engine,
            "namespace": namespace,
        }.items(),
    )

    ekf_config = PathJoinSubstitution([rosbot_bringup, "config", "ekf.yaml"])

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        output="screen",
        parameters=[ekf_config],
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(bringup_cmd_group)
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_mecanum_arg)
    ld.add_action(declare_use_sim_arg)
    ld.add_action(declare_use_gpu_arg)
    ld.add_action(declare_simulation_engine_arg)
    ld.add_action(SetParameter(name="use_sim_time", value=use_sim))
    ld.add_action(controller_launch)
    ld.add_action(robot_localization_node)

    return ld

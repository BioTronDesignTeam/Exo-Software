from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    pckg_path = FindPackageShare("bringup_robot")
    xacro_file = PathJoinSubstitution([pckg_path, "urdf", "exo_model.xacro.urdf"])
    rviz_config_file = PathJoinSubstitution([pckg_path, "config", "display.rivz"])
    controllers_file = PathJoinSubstitution([pckg_path, "config", "controllers.yaml"])

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_file,
        ],
        output="screen",
    )

    # NEW: makes the broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # NEW: makes the controller actually do its job
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # NEW: the controller
    start_rqt_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rqt_joint_trajectory_controller",
            "rqt_joint_trajectory_controller",
        ],
        output="screen",
    )

    # NOTE: Any new feature needs to be included in the list below.
    # not having this note cost me 20 minutes
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
            start_robot_state_publisher,
            controller_manager,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,
            start_rviz,
            start_rqt_controller,
        ]
    )

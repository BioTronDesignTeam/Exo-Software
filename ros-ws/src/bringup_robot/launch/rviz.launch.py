from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    pckg_path = FindPackageShare("bringup_robot")
    xacro_file = PathJoinSubstitution([pckg_path, "urdf", "exo_model.xacro.urdf"])
    rviz_config_file = PathJoinSubstitution([pckg_path, "config", "display.rivz"])

    # load exoskeleton model, get urdf via xacro
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    start_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    start_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )
    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # NEW: Controller Manager - This loads mock hardware! i think! it does not work!
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
            start_robot_state_publisher,
            start_joint_state_publisher,
            start_rviz,
        ]
    )

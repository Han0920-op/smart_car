from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    gazebo_launch = Path(get_package_share_directory("gazebo_ros")) / "launch"
    description_dir = Path(get_package_share_directory("smart_car_description"))
    simulation_dir = Path(get_package_share_directory("smart_car_gazebo"))
    robot_description = xacro.process_file(
        str(description_dir / "urdf" / "smart_car.urdf.xacro")
    ).toxml()

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument(
            "world", default_value=str(simulation_dir / "worlds" / "patrol.world")
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch / "gzserver.launch.py")),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch / "gzclient.launch.py")),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(robot_description, value_type=str),
                "use_sim_time": True,
            }],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "smart_car", "-z", "0.01"],
            output="screen",
        ),
    ])

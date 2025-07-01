#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from limo_spline.yaml_to_gzworld import WorldConverter
import yaml

def spawn_entity_in_gzsim(context, *args, **kwargs):
    map_path = str(LaunchConfiguration("map_path").perform(context))
    world_path = str(LaunchConfiguration("world_path").perform(context))
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    MapObj = WorldConverter(map_path, world_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_path, " -r'"])

        }.items()
    )

    x_coord = MapObj.spawn_coords[0]
    y_coord = MapObj.spawn_coords[1]

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "limobot",
            "-allow_renaming", "true",
            "-z", "0.3",
            "-x", str(x_coord),
            "-y", str(y_coord),
            "-Y", "0.3"
        ]
    )
    return [gz_spawn_entity, gz_sim]

def generate_launch_description():
    limobot_path = get_package_share_directory("limo_simulation")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                'robot_description':
                    Command(
                        [
                            'xacro ', 
                            join(limobot_path, 'urdf/limo_four_diff.xacro')
                        ]
                    )
            }
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/limobot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/world/empty_world/model/limobot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        remappings=[
            ("/model/limobot/odometry", "/odom"),
            ('/world/empty_world/model/limobot/joint_state', '/joint_states'),
        ]
    )

    map_path_launch_arg = DeclareLaunchArgument (
        'map_path',
        description="path to the YAML map file"
    )

    world_path_launch_arg = DeclareLaunchArgument(
        'world_path',
        description="path to the gazebo world file"
    )

    return LaunchDescription([
        map_path_launch_arg,
        world_path_launch_arg,
        robot_state_publisher,
        OpaqueFunction(function=spawn_entity_in_gzsim),
        gz_ros2_bridge
    ])
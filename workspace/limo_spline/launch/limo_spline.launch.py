#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from limo_spline.yaml_to_gzworld import WorldConverter

def spawn_robot_in_gzsim(context, *args, **kwargs):
    map_file = str(LaunchConfiguration("map_file").perform(context))
    world_file = str(LaunchConfiguration("world_file").perform(context))

    gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_conv_obj = WorldConverter(map_file, world_file)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )

    x_coord = world_conv_obj.spawn_coords[0]
    y_coord = world_conv_obj.spawn_coords[1]

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

    tf_world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_odom",
        arguments=[
            str(x_coord), str(y_coord), "0",
            "0", "0", str(0.3),
            "world", "odom"
        ]
    )

    tf_odom_to_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_to_odom",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "odom", "base_footprint"
        ]
    )

    return [gz_sim, tf_world_to_odom, tf_odom_to_base_footprint, gz_spawn_entity]

def launch_spline_nodes(context, *args, **kwargs):
    map_file = LaunchConfiguration("map_file").perform(context)
    technique = LaunchConfiguration("technique").perform(context)

    spline_path_publisher = Node (
        package="limo_spline",
        executable="spline_path_publisher",
        name="spline_path_publisher",
        parameters = [{
            "map_file" : map_file,
            "technique" : technique
        }]
    )

    return [spline_path_publisher] 

def generate_launch_description():
    limobot_path = get_package_share_directory("limo_simulation")

    map_file_launch_arg = DeclareLaunchArgument (
        'map_file',
        description="path to the YAML map file"
    )

    world_file_launch_arg = DeclareLaunchArgument(
        'world_file',
        description="path to the gazebo world file"
    )

    technique_launch_arg = DeclareLaunchArgument(
        'technique',
        description="technique to use for creating the curves"
    )

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

    return LaunchDescription([
        map_file_launch_arg,
        world_file_launch_arg,
        technique_launch_arg,
        robot_state_publisher,
        OpaqueFunction(function=spawn_robot_in_gzsim),
        OpaqueFunction(function=launch_spline_nodes),
        gz_ros2_bridge
    ])
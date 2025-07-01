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

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def convert_map_to_world(context, *args, **kwargs):
    map_path = str(LaunchConfiguration("map_path").perform(context))
    world_path = str(LaunchConfiguration("world_path").perform(context))
    WorldConverter(map_path, world_path)

def generate_launch_description():
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
        OpaqueFunction(function=convert_map_to_world)
    ])
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_catch_ros2 import Catch2IntegrationTestNode, Catch2LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch example integration test."""
    # Catch2LaunchDescription:
    # a wrapper around LaunchDescription which adds a required "result_file"
    # argument for the launch file. This file will hold the results of the test.
    return Catch2LaunchDescription([
        # Specific launch arguments can also be included at the user's discretion
        DeclareLaunchArgument(
            name='test_duration',
            default_value='2.0',
            description='Max length of test in seconds.',
        ),
        # Auxiliary nodes can be run like normal to test integration between nodes
        Node(
            package='beginner_tutorials',
            executable='talker',
        ),
        # Catch2IntegrationTestNode:
        # a wrapper around Node which passes the "result_file" argument to Catch2.
        # There should only be one integration test node. This node will shutdown
        # the entire launch file when it exits.
        # Specific parameters and other arguments can also be passed, like the
        # "test_duration" example below.
        Catch2IntegrationTestNode(
            package='beginner_tutorials',
            executable='integration_test',
            parameters=[{
                'test_duration': LaunchConfiguration('test_duration'),
            }],
        ),
    ])
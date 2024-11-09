from launch import LaunchDescription
from launch_ros.actions import Node

# Used to pass arguments to launch files
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    publisher_freq_arg = DeclareLaunchArgument(
        "publish_freq", default_value=TextSubstitution(text="300")
    )

    talker_ = Node(
        package="beginner_tutorials",
        executable="talker",
        parameters=[{'publish_frequency': LaunchConfiguration('publish_freq')}]
    )

    return LaunchDescription([publisher_freq_arg, talker_])
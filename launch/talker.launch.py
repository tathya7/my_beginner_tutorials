from launch import LaunchDescription
from launch_ros.actions import Node

# Used to pass arguments to launch files
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    publisher_freq_arg = DeclareLaunchArgument(
        "publish_freq", default_value=TextSubstitution(text="300")
    )

    record_bag_flag = DeclareLaunchArgument(
        "record_flag", default_value=TextSubstitution(text="1")
    )

    talker_ = Node(
        package="beginner_tutorials",
        executable="talker",
        parameters=[{'publish_frequency': LaunchConfiguration('publish_freq'), 'record_bag' : LaunchConfiguration('record_bag')}]
    )

    # listener_ = Node(
    #     package="beginner_tutorials",
    #     executable="listener",
    # )

    return LaunchDescription([publisher_freq_arg, record_bag_flag, talker_])


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import sys

def generate_launch_description():
    rosbag_record = sys.argv[-1]
    actions_list = [
        Node(
            package='beginner_tutorials',
            executable='talker',
            arguments = ['0', '0', '1', '0', '0', '0', 'world', 'talk']
        ),
        Node(
            package='beginner_tutorials',
            executable='listener'
        )
    ]
    if rosbag_record=='record:=True':
        actions_list.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", "beginner_tutorials_bag", "-a"]
            )
        )
    return LaunchDescription(actions_list)
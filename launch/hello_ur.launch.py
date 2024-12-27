import sys

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch
from launch import LaunchService


def get_robot_description_kinematics():
    return PathJoinSubstitution([FindPackageShare('hello_moveit'), "config", "kinematics.yaml"])


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description_kinematics = get_robot_description_kinematics()
    demo_node = Node(
        package="hello_moveit",
        executable="hello_ur",
        name="hello_ur",
        output="screen",
        parameters=[
            robot_description_kinematics,
        ],
        #prefix='gdb -ex run --args',
        prefix=['xterm -e']
    )

    return launch.LaunchDescription([demo_node])


def main():
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    ls.run()


if __name__ == '__main__':
    sys.exit(main())

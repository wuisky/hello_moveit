# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

import ast
import os
import re

from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from launch import LaunchDescription


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    controllers = LaunchConfiguration("controllers")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_realsense = LaunchConfiguration('use_realsense')
    handeye_calibration = LaunchConfiguration('handeye_calibration')

    hand = LaunchConfiguration('hand')
    # Enable use_tool_communication when the robotiq gripper is selected
    if re.match('robotiq-', context.perform_substitution(hand)) is not None:
        use_tool_communication = 'True'
        use_robotiq_gripper = 'True'
    else:
        use_tool_communication = 'False'
        use_robotiq_gripper = 'False'

    marker_size = LaunchConfiguration("marker_size")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    tracking_base_frame = LaunchConfiguration("tracking_base_frame")
    marker_id_list = LaunchConfiguration("marker_id_list")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"])
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"])
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"])
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " ",
        "robot_ip:=",
        robot_ip,
        " ",
        "joint_limit_params:=",
        joint_limit_params,
        " ",
        "kinematics_params:=",
        kinematics_params,
        " ",
        "physical_params:=",
        physical_params,
        " ",
        "visual_params:=",
        visual_params,
        " ",
        "safety_limits:=",
        safety_limits,
        " ",
        "safety_pos_margin:=",
        safety_pos_margin,
        " ",
        "safety_k_position:=",
        safety_k_position,
        " ",
        "name:=",
        "ur",
        " ",
        "ur_type:=",
        ur_type,
        " ",
        "script_filename:=ros_control.urscript",
        " ",
        "input_recipe_filename:=rtde_input_recipe.txt",
        " ",
        "output_recipe_filename:=rtde_output_recipe.txt",
        " ",
        "prefix:=",
        prefix,
        " ",
    ])
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
        " ",
        "name:=",
        # Also ur_type parameter could be used but then the planning group names in yaml
        # configs has to be updated!
        "ur",
        " ",
        "prefix:=",
        prefix,
        " ",
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare('hello_moveit'), "config", "kinematics.yaml"])

    # robot_description_planning = {
    # "robot_description_planning": load_yaml_abs(str(joint_limit_params.perform(context)))
    # }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters":
            """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    change_controllers = context.perform_substitution(use_fake_hardware)
    if change_controllers == "true" or context.perform_substitution(controllers) == "joint_trajectory_controller":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            # robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {
                "use_sim_time": use_sim_time
            },
            warehouse_ros_config,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("hello_moveit"), "rviz", "view_robot.rviz"])
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            # robot_description_planning,
            warehouse_ros_config,
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]

    ## include ur bringup
    launch_ur_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])
        ]),
        launch_arguments={
            'ur_type': context.perform_substitution(ur_type),
            'robot_ip': context.perform_substitution(robot_ip),
            'initial_joint_controller': 'joint_trajectory_controller',
            'use_fake_hardware': context.perform_substitution(use_fake_hardware),
            'launch_rviz': 'false',
            'use_tool_communication': use_tool_communication
        }.items())
    nodes_to_start.append(launch_ur_bringup)

    ## include realsense2_camera
    launch_realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ]),
        launch_arguments={
            'pointcloud.enable': 'true',
            # 'rgb_camera.profile': '1280x720x30'
        }.items(),
        condition=IfCondition(
                    PythonExpression([
                    handeye_calibration,
                    ' or ',
                    use_realsense
                ])),
    )
    nodes_to_start.append(launch_realsense2_camera)

    ## include aruco recognition
    launch_aruco_recognition = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        PathJoinSubstitution(
            [FindPackageShare('ros2_aruco'), 'launch', 'aruco_recognition.launch.py'])
        ]),
        launch_arguments={
            'marker_size': context.perform_substitution(marker_size),
            'aruco_dictionary_id': 'DICT_5X5_250',
            'image_topic': context.perform_substitution(image_topic),
            'camera_info_topic': context.perform_substitution(camera_info_topic),
            'tracking_base_frame': context.perform_substitution(tracking_base_frame),
            'marker_id_list': marker_id_list,
        }.items(),
        condition=IfCondition(
                    PythonExpression([
                    handeye_calibration,
                    ' or ',
                    use_realsense
                    ])
                ),
    )
    nodes_to_start.append(launch_aruco_recognition)

    ## include easy_handeye2
    calibration_marker = ast.literal_eval(context.perform_substitution(marker_id_list))[0]
    launch_easy_handeye2 = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare('easy_handeye2'), 'launch', 'calibrate.launch.py'])
    ]),
                                                    launch_arguments={
                                                        'calibration_type': 'eye_in_hand',
                                                        'tracking_base_frame': str(context.perform_substitution(tracking_base_frame)),
                                                        'tracking_marker_frame': f'aruco_marker_{calibration_marker}',
                                                        'robot_base_frame': 'base_link',
                                                        'robot_effector_frame': 'tool0',
                                                        'name': 'easy_handeye2_demo_eih',
                                                    }.items(),
                                                    condition=IfCondition(handeye_calibration))
    easy_handeye2_delay = TimerAction(period=5.0, actions=[launch_easy_handeye2])
    nodes_to_start.append(easy_handeye2_delay)

    # static transform publisher
    calibration_yaml = load_yaml("hello_moveit", "config/easy_handeye2_demo_eih.calib")
    camera_tf_node = Node(package='tf2_ros', executable='static_transform_publisher', name='camera_tf_publisher',
                          condition=IfCondition(PythonExpression(['not ', handeye_calibration, ' and ', use_realsense])),
                          arguments=[
                              "--x", str(calibration_yaml["transform"]["translation"]["x"]),
                              "--y", str(calibration_yaml["transform"]["translation"]["y"]),
                              "--z", str(calibration_yaml["transform"]["translation"]["z"]),
                              "--qx", str(calibration_yaml["transform"]["rotation"]["x"]),
                              "--qy", str(calibration_yaml["transform"]["rotation"]["y"]),
                              "--qz", str(calibration_yaml["transform"]["rotation"]["z"]),
                              "--qw", str(calibration_yaml["transform"]["rotation"]["w"]),
                              "--frame-id", calibration_yaml["parameters"]["robot_effector_frame"],
                              "--child-frame-id", calibration_yaml["parameters"]["tracking_base_frame"],
                            ],
                         )
    camera_tf_node_delay = TimerAction(period=5.0, actions=[camera_tf_node])
    nodes_to_start.append(camera_tf_node_delay)

    ## robotiq 2f-140
    tool_communication_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(use_tool_communication),
        executable="tool_communication.py",
        name="tool_communication",
        output="log",
        parameters=[
            {'robot_ip': context.perform_substitution(robot_ip)},
            {'device_name': '/tmp/ttyUR'}
        ]
    )
    nodes_to_start.append(tool_communication_node)

    launch_robotiq_gripper = GroupAction(actions=[
        PushRosNamespace('gripper'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare('robotiq_85_driver'), 'launch', 'gripper_driver.launch.py'])]),
                                                      launch_arguments={
                                                          'stroke': '0.140',
                                                          'comport': '/tmp/ttyUR',
                                                          'baud': '115200',
                                                      }.items(),
                                                      condition=IfCondition(use_robotiq_gripper),
                                                    )])
    nodes_to_start.append(launch_robotiq_gripper)

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description=
            "Indicate whether robot is running with fake hardware mirroring command to its states.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers",
            default_value="joint_trajectory_controller",
            description=
            "loaded robot controller",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="yyy.yyy.yyy.yyy",
            description="IP of ur robot",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        ))
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description."                                                            ,
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config."                                                              ,
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description=
            "Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated."                            ,
        ))
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"))
    declared_arguments.append(
        DeclareLaunchArgument("handeye_calibration",
                              default_value="False",
                              description="Launch easy_handeye2_calibrationi?"))
    declared_arguments.append(
        DeclareLaunchArgument("use_realsense",
                              default_value="False",
                              description="Launch ros2_aruco and realsense2?"))
    # hand
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand",
            default_value='""',
            description="hand type",
        ))
    # aruco marker
    declared_arguments.append(
        DeclareLaunchArgument(
            "marker_size",
            default_value='0.050',
            description="aruco marker size.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "image_topic",
            description="image topic name.",
            default_value='/camera/color/image_rect_raw',
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_info_topic",
            description="camera info topic name.",
            default_value='/camera/color/camera_info',
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "tracking_base_frame",
            description=".",
            default_value='camera_color_optical_frame',
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "marker_id_list",
            description="list of marker IDs to be detected.",
            default_value='[0, 1, 2, 3]',
        ))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

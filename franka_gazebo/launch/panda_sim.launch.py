#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
# import xacro
# import yaml

# LOAD FILE:
# def load_file(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)
#     try:
#         with open(absolute_file_path, 'r') as file:
#             return file.read()
#     except EnvironmentError:
#         # parent of IOError, OSError *and* WindowsError where available.
#         return None
# # LOAD YAML:
# def load_yaml(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)
#     try:
#         with open(absolute_file_path, 'r') as file:
#             return yaml.safe_load(file)
#     except EnvironmentError:
#         # parent of IOError, OSError *and* WindowsError where available.
#         return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    # Get launch configurations
    load_gripper = LaunchConfiguration('load_gripper')
    arm_id = LaunchConfiguration('arm_id')
    
    # Launch Gazebo World  
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']))

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                # launch_arguments={'pause': 'true', 'verbose': 'true'
                # }.items(),
             )

    # PANDA ROBOT Description file package:
    # panda_description_path = os.path.join(
        # get_package_share_directory('franka_description'))
    # PANDA ROBOT ROBOT urdf file path:
    # xacro_file = os.path.join(panda_description_path,
                            #   'robots',
                            #   'panda_arm.urdf.xacro')

    # Generate ROBOT_DESCRIPTION for PANDA ROBOT:
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper, ' arm_id:=', arm_id])
    

    with open(franka_xacro_file, 'r') as infp:
        robot_desc = infp.read()

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            # robot_description,
            {"use_sim_time": True}
        ]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'panda'],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint state broadcaster:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint trajectory controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )
    # End effector controllers:
    # Panda HAND:
    # panda_handleft_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["panda_handleft_controller", "-c", "/controller_manager"],
    # )
    # panda_handright_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["panda_handright_controller", "-c", "/controller_manager"],
    # )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        DeclareLaunchArgument(
            'load_gripper',
            default_value='true',
            description='Use Franka Gripper as end-effector if true. Robot is loaded without '
                        'end-effector otherwise'),
        DeclareLaunchArgument(
            'arm_id',
            default_value='true',
            description='Use Franka Gripper as end-effector if true. Robot is loaded without '
                        'end-effector otherwise'),

        gazebo, 
        # node_robot_state_publisher,
        spawn_entity,

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action = spawn_entity,
        #         on_exit = [
        #             joint_state_broadcaster_spawner,
        #         ]
        #     )
        # ),

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action = joint_state_broadcaster_spawner,
        #         on_exit = [
        #             joint_trajectory_controller_spawner,
        #         ]
        #     )
        # ),
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action = joint_trajectory_controller_spawner,
        #         on_exit = [
        #             panda_handleft_controller_spawner,
        #         ]
        #     )
        # ),
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action = panda_handleft_controller_spawner,
        #         on_exit = [
        #             panda_handright_controller_spawner,
        #         ]
        #     )
        # ),

    ])
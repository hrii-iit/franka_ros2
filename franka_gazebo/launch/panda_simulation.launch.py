#!/usr/bin/python3

# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    arm_id = LaunchConfiguration('arm_id')

    # Launch Gazebo
    # panda_ros2_gazebo = os.path.join(
        # get_package_share_directory('panda_ros2_gazebo'),
        # 'worlds',
        # 'panda.world')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py'])
    )

    # Load robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # PathJoinSubstitution([FindPackageShare('panda_ros2_gazebo'), "urdf", 'panda.urdf.xacro']),
            PathJoinSubstitution([FindPackageShare('franka_description'), "robots", 'panda_arm.urdf.xacro']),
            # " ",
            # "safety_limits:=",
            # safety_limits,
            # " ",
            # "safety_pos_margin:=",
            # safety_pos_margin,
            # " ",
            # "safety_k_position:=",
            # safety_k_position,
            # " ",
            # "name:=",
            # "ur",
            # " ",
            # "ur_type:=",
            # ur_type,
            # " ",
            # "prefix:=",
            # prefix,
            # " ",
            # "sim_gazebo:=true",
            # " ",
            # "simulation_controllers:=",
            # initial_joint_controllers,
        ]
    )

    # Define robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {"robot_description": ParameterValue(robot_description_content, value_type=str)}, 
            {'use_sim_time': True}
        ]
    )

    # Spawn the robot in gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=[
                            '-topic', 'robot_description',
                            '-entity', 'panda'
                        ],
                        output='screen')

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn joint trajectory controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_impedance_example_controller", "-c", "/controller_manager"],
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
        
        gazebo, 
        node_robot_state_publisher,
        spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_entity,
                on_exit = [
                    joint_state_broadcaster_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_state_broadcaster_spawner,
                on_exit = [
                    joint_trajectory_controller_spawner,
                ]
            )
        ),
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
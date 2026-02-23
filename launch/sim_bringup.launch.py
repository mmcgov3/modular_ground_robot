from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('modular_ground_robot')

    # Paths
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'flat_world.sdf')
    params_path = os.path.join(pkg_share, 'config', 'controller.yaml')

    # robot_description from xacro
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_path]),
        value_type=str
    )

    # 1) Gazebo (GZ Harmonic) world
    gz = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen'
    )

    # 2) Publish robot_description for spawning
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )

    # 3) Spawn the robot into Gazebo using /robot_description
    # Delay a bit so Gazebo is up and /robot_description exists.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'modular_bot',
            '-z', '0.2'
        ]
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn]
    )

    # 4) Bridge ROS <-> Gazebo for the *Gazebo joint velocity command topics*
    # Your URDF must include the JointController plugins for these joints.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/model/modular_bot/joint/left_wheel_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/modular_bot/joint/right_wheel_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
        ]
    )

    # 5) Your controller node (loads YAML) and remaps clean wheel topics to the joint cmd topics
    controller = Node(
        package='modular_ground_robot',
        executable='wheel_cmd_controller',
        output='screen',
        parameters=[params_path, {'use_sim_time': True}],
        remappings=[
            # Keep your code publishing clean topics, remap at launch:
            ('/left_wheel_cmd', '/model/modular_bot/joint/left_wheel_joint/cmd_vel'),
            ('/right_wheel_cmd', '/model/modular_bot/joint/right_wheel_joint/cmd_vel'),
        ]
    )

    return LaunchDescription([
        gz,
        rsp,
        bridge,
        controller,
        delayed_spawn,
    ])
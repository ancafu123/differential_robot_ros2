import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument, TimerAction




def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_robot'))
    xacro_file = os.path.join(pkg_path,'description/my_robot.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}



    # Create a node for robot_state_publisher
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'   
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        remappings=[
        ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel')],
        output="screen"
    )
    
    bridge_node = Node(
        package="my_robot",
        executable="bridge",
        name = "bridge",  # o "serial_bridge_node" sin .py
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'wheel_separation': 0.17,
            'wheel_radius': 0.0325
        }],
        output="screen"
    )

    # === 4. Spawners ===
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "50"
        ],
        output="screen",
    )

    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "50"
        ],
        output="screen",
    )


    # Create a node for rviz2
    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_path, 'config', 'my_config.rviz')]]

    )

    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[load_joint_state_broadcaster]
    )

    delayed_diff_drive_controller = TimerAction(
        period=5.0,
        actions=[load_diff_drive_controller]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation time if true'),

        robot_state_publisher_node,
        #rviz2_node,
        bridge_node,
        joint_state_publisher_node,
        #controller_manager_node,
        #delayed_joint_state_broadcaster,
        #delayed_diff_drive_controller,

    ])
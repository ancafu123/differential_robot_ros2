import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
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
    
    slam_path = os.path.join(pkg_path, 'config', 'slam_config.yaml')

    # Create a node for robot_state_publisher
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    
    pico_loader = Node(
    package="my_robot",
    executable="pico_bootloader",
    output="screen"
    )
    
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
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
            'serial_port': '/dev/ttyACM1',
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
    
    rplidar_node = Node(
        package='rplidar_ros',  # Ajusta según el nombre del package
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',  # Verifica el puerto correcto
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',  # Debe coincidir con tu URDF
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'scan_frequency': 5.0,  # Reducir frecuencia a 10 Hz (default suele ser 20+)
            'auto_standby': True,
        }],
        output='screen'
    )
    

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link'
        }],
        output='screen'
    )

    
    slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_path,
            ],
            output='screen'
        )
        
    


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Use simulation time if true'),
        #pico_loader,
        bridge_node,
        robot_state_publisher_node,
        #rviz2_node,
        #joint_state_publisher_node,
        rplidar_node,
        slam_node,
        #camera_node,
        #controller_manager_node,
        #delayed_joint_state_broadcaster,
        #delayed_diff_drive_controller,

    ])
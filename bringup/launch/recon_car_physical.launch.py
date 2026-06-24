from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory('bringup')
    car_polygon_path = os.path.join(pkg_share, 'models', 'reconCarMesh.urdf')

    with open(car_polygon_path, 'r') as infp:
        robot_description_content = infp.read()

    # robot state publisher publishes current joint states
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}])

    controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_content}, 
                        PathJoinSubstitution([FindPackageShare('bringup'),'params','control_params_physical.yaml'])], output='screen')

    diff_controller_spawn_node = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['diff_drive_controller'])
    
    joint_broadcaster_spawn_node = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster'])
    
    rplidar_composition_node = Node(
        package = 'rplidar_ros',
        executable = 'rplidar_composition',
        name = 'rplidar_composition',
        output = 'screen',
        parameters=[{'frame_id': 'front_sensor', 'topic': 'scan', 
                     'angle_compensation': True, 'serial_port': '/dev/ttyUSB0'}])

    slam_launch_object = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': os.path.join(pkg_share, 'params', 'mapper_params_online_async.yaml')}.items())


    ld.add_action(robot_state_pub_node)
    ld.add_action(controller_manager)
    ld.add_action(diff_controller_spawn_node)
    ld.add_action(joint_broadcaster_spawn_node)
    ld.add_action(rplidar_composition_node)
    ld.add_action(slam_launch_object)
    return ld
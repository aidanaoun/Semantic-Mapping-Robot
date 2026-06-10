from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = get_package_share_directory('bringup')
    car_polygon_path = os.path.join(pkg_share, 'models', 'reconCarPolygon.urdf')

    with open(car_polygon_path, 'r') as infp:
        robot_description_content = infp.read()

    world_path = os.path.join(pkg_share, 'models', 'lighted_room_world.sdf')
    set_env_vars_resources = AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(pkg_share, 'models'))


    # robot state publisher publishes current joint states
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,'use_sim_time': True}])


    gazebo_launch_object = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 2 {world_path}','on_exit_shutdown': 'true'}.items())


    bridge_params = os.path.join(pkg_share, 'params', 'ros_gz_bridge.yaml')
    ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args','-p',f'config_file:={bridge_params}',],
        parameters=[{'use_sim_time': True}],
        output='screen')


    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'recon_car_polygon','-file', car_polygon_path,'-x', '0','-y', '0','-z', '0.35'], output='screen')

    
    contoller_spawn_node = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['diff_drive_controller'])
    

    joint_broadcaster_spawn_node = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster'])

    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_state_pub_node)
    ld.add_action(gazebo_launch_object)
    ld.add_action(ros_bridge_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(contoller_spawn_node)
    ld.add_action(joint_broadcaster_spawn_node)
    return ld
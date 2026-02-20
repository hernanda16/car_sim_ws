from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from xacro import process_file

def generate_launch_description():
    pkg_share = get_package_share_directory('towing')
    
    xacro_path = os.path.join(pkg_share, 'urdf', 'towing.urdf.xacro')
    robot_desc = process_file(xacro_path).toxml()

    temp_urdf_path = '/tmp/towing_generated.urdf'
    with open(temp_urdf_path, 'w') as f:
        f.write(robot_desc)
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'towing',
            '-file', temp_urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.80'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        ExecuteProcess(cmd=['gzclient', '--verbose'], output='screen'),
        node_robot_state_publisher,
        node_spawn_entity,
    ])
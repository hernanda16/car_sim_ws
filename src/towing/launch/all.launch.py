import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from xacro import process_file

path_config_buffer = os.getenv("AMENT_PREFIX_PATH", "")
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
pkg_path = ws_path + "src/towing"
mesh_path = pkg_path + "/models/towing/meshes"

def generate_launch_description():
    xacro_path = os.path.join(pkg_path, 'urdf', 'towing.urdf.xacro')
    
    robot_description = process_file(xacro_path, mappings={'mesh_path': mesh_path}).toxml()
    temp_urdf = '/tmp/towing.urdf'
    with open(temp_urdf, 'w') as f:
        f.write(robot_description)

    rviz_path = os.path.join(pkg_path, 'config', 'urdf.rviz')
    if not os.path.exists(rviz_path):
        rviz_path = os.path.join(pkg_path, 'urdf.rviz')
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )
    
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'towing',
            '-file', temp_urdf,
            '-x', '0',
            '-y', '0',
            '-z', '0.4'
        ],
        output='screen'
    )
    
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['/gazebo/joint_states'],
            'use_sim_time': True
        }],
        output='screen'
    )
    
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gzclient', '--verbose'],
            output='screen'
        ),
        node_spawn_entity,
        node_robot_state_publisher,

        # node_joint_state_publisher,
        # node_joint_state_publisher_gui,

        node_rviz2,
    ])
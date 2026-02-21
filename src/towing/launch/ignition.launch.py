import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from xacro import process_file


path_config_buffer = os.getenv("AMENT_PREFIX_PATH", "")
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
pkg_path = ws_path + "src/towing"
mesh_path = pkg_path + "/models/towing/meshes"

def generate_launch_description():

    # =========================
    # process xacro (SAMA seperti punyamu)
    # =========================
    xacro_path = os.path.join(pkg_path, 'urdf', 'towing.urdf.xacro')
    factory_model_path = os.path.join(pkg_path, 'models', 'factory', 'models', 'factory','model.sdf')

    robot_description = process_file(
        xacro_path,
        mappings={'mesh_path': mesh_path}
    ).toxml()

    temp_urdf = '/tmp/towing.urdf'
    with open(temp_urdf, 'w') as f:
        f.write(robot_description)

    # =========================
    # WAJIB untuk Ignition resource lookup
    # =========================
    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(ws_path, 'src/towing/models')
    )

    set_ign_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # =========================
    # Start Ignition Gazebo
    # =========================
    ignition = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    # =========================
    # Spawn robot (Ignition way)
    # =========================
    node_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'towing',
            '-file', temp_urdf,
            '-x', '0',
            '-y', '0',
            '-z', '0.4'
        ],
        output='screen'
    )

    node_spawn_factory = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'factory',
            '-file', factory_model_path,
            '-x', '2',
            '-y', '0',
            '-z', '0'
        ],
        output='screen'
    )

    # =========================
    # robot_state_publisher
    # =========================
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # =========================
    # Clock bridge (WAJIB untuk RViz)
    # =========================
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    joint_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/towing/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/empty/model/towing/joint_state', '/joint_states')
        ],
        output='screen'
    )

    steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_controller"],
        output="screen",
    )

    drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller"],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # =========================
    # RViz (pakai config kamu)
    # =========================
    rviz_path = os.path.join(pkg_path, 'config', 'urdf.rviz')
    if not os.path.exists(rviz_path):
        rviz_path = os.path.join(pkg_path, 'urdf.rviz')

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    pseudo_diff_node = Node(
        package='towing',
        executable='pseudo_diff_drive.py',
        output='screen',
    )

    # =========================
    # return LaunchDescription([
    #     set_ign_resource,
    #     set_ign_plugin_path,
    #     ignition,
    #     node_spawn_factory,
    # ])

    return LaunchDescription([
        set_ign_resource,
        set_ign_plugin_path,
        ignition,
        node_spawn,
        node_spawn_factory,
        node_rsp,
        clock_bridge,
        steering_spawner,
        drive_spawner,
        jsb_spawner,
        joint_bridge,
        pseudo_diff_node,
        node_rviz2,
    ])
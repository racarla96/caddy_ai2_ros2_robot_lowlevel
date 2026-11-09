import os
import tempfile
import xml.etree.ElementTree as ET

import yaml
from jinja2 import Environment, FileSystemLoader
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _sdf_ros2_control_to_urdf(sdf_str: str, robot_params: dict, prefix: str) -> str:
    """
    Extract <ros2_control> blocks from a rendered SDF and wrap them in a minimal
    valid URDF so that ros2_control_node can load the hardware plugins.

    robot_state_publisher receives the full SDF string (sdformat_urdf handles TF);
    ros2_control_node needs URDF format to find <ros2_control> tags.
    """
    root = ET.fromstring(sdf_str)
    model = root.find('.//model')

    ros2_control_xml = '\n  '.join(
        ET.tostring(rc, encoding='unicode')
        for rc in model.findall('ros2_control')
    )

    wr  = robot_params['wheel_radius']
    wb  = robot_params['wheelbase']
    msa = robot_params['max_steer_angle']
    p   = prefix

    return f'''<?xml version="1.0"?>
<robot name="caddy_ai2_control">
  <link name="{p}base_footprint"/>
  <joint name="{p}base_joint" type="fixed">
    <parent link="{p}base_footprint"/>
    <child link="{p}base_link"/>
    <origin xyz="0 0 {wr}"/>
  </joint>
  <link name="{p}base_link"/>
  <joint name="{p}steering_joint" type="revolute">
    <parent link="{p}base_link"/>
    <child link="{p}steering_link"/>
    <origin xyz="{wb/2} 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="{-msa}" upper="{msa}" effort="1000" velocity="10"/>
  </joint>
  <link name="{p}steering_link"/>
  <joint name="{p}system_traction_joint" type="continuous">
    <parent link="{p}base_link"/>
    <child link="{p}traction_link"/>
    <origin xyz="{-wb/2} 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  <link name="{p}traction_link"/>
  {ros2_control_xml}
</robot>'''


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace',  default_value='',
                              description='ROS namespace for this robot'),
        DeclareLaunchArgument('prefix',     default_value='',
                              description='Link/joint name prefix (e.g. robot1_)'),
        DeclareLaunchArgument('use_rviz',   default_value='false',
                              description='Launch RViz for visualization'),
        OpaqueFunction(function=_launch_robot),
    ])


def _launch_robot(context, *args, **kwargs):
    namespace = context.launch_configurations['namespace']
    prefix    = context.launch_configurations['prefix']

    desc_share  = get_package_share_directory('caddy_ai2_ros2_description')
    robot_share = get_package_share_directory('caddy_ai2_ros2_robot_lowlevel')

    # --- Load robot physical parameters ---
    params_file = os.path.join(desc_share, 'bringup', 'config', 'robot_params.yaml')
    with open(params_file) as f:
        robot_params = yaml.safe_load(f)

    # --- Render sensor link/joint fragments (no sensor elements — only TF frames) ---
    sensor_fragments = []
    for _sensor_name, sensor_cfg in robot_params.get('sensors', {}).items():
        if not sensor_cfg.get('enabled', True):
            continue

        sensor_type = sensor_cfg['type']
        s_share = get_package_share_directory(sensor_cfg['package'])
        with open(os.path.join(s_share, 'bringup', 'config', 'sensor_params.yaml')) as f:
            sp = yaml.safe_load(f)

        pose = sensor_cfg['pose']
        hw   = sp['hardware']
        s_env = Environment(
            loader=FileSystemLoader(os.path.join(s_share, 'description')),
            keep_trailing_newline=True,
        )
        common = dict(
            prefix=prefix,
            parent_link=f'{prefix}base_footprint',
            x=pose['x'], y=pose['y'], z=pose['z'],
            roll=pose['roll'], pitch=pose['pitch'], yaw=pose['yaw'],
            with_sensor=False,
        )

        if sensor_type == 'range_lidar':
            mesh_name = hw.get('mesh') or ''
            mesh_uri = f'file://{s_share}/meshes/{mesh_name}' if mesh_name else ''
            fragment = s_env.get_template('sensor.sdf.j2').render(
                **common,
                mesh_uri=mesh_uri,
            )
        elif sensor_type == 'imu':
            fragment = s_env.get_template('sensor.sdf.j2').render(**common)
        else:
            continue

        sensor_fragments.append(fragment)

    sensors_fragment = '\n'.join(sensor_fragments)

    # --- Render shared SDF with simulation=False ---
    sdf_dir = os.path.join(desc_share, 'description', 'model', 'sdf')
    env_sdf = Environment(loader=FileSystemLoader(sdf_dir), keep_trailing_newline=True)
    robot_description_str = env_sdf.get_template('caddy_ai2_model.sdf.j2').render(
        simulation=False,
        namespace=namespace,
        prefix=prefix,
        desc_share=desc_share,
        update_rate=100,
        sensors_fragment=sensors_fragment,
        # Steering calibration — update after measuring on the real vehicle
        counts_per_radian=100000,
        abs_zero_position=3109,
        gain_inc_to_abs=0.008078908046482845,
        **robot_params,
    )

    # --- Build minimal URDF for ros2_control_node (extracts <ros2_control> blocks) ---
    ros2_control_urdf = _sdf_ros2_control_to_urdf(
        robot_description_str, robot_params, prefix
    )

    # --- Render controllers YAML from Jinja2 template ---
    cfg_dir = os.path.join(robot_share, 'config')
    env_cfg = Environment(loader=FileSystemLoader(cfg_dir), keep_trailing_newline=True)
    rendered_ctrl = env_cfg.get_template('controllers.yaml.j2').render(
        namespace=namespace, prefix=prefix, **robot_params
    )
    tmp_ctrl = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='robot_ctrl_', delete=False
    )
    tmp_ctrl.write(rendered_ctrl)
    tmp_ctrl.close()
    ns_controllers_yaml = tmp_ctrl.name

    # ------------------------------------------------------------------ #
    # Nodes                                                               #
    # ------------------------------------------------------------------ #
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'robot_description': robot_description_str}],
    )

    node_controller_manager = Node(
        package='ros2_control',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            {'robot_description': ros2_control_urdf},
            ns_controllers_yaml,
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster'],
    )

    bicycle_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['bicycle_steering_controller',
                   '--param-file', ns_controllers_yaml],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['forward_position_command_controller',
                   '--param-file', ns_controllers_yaml,
                   '--inactive'],
    )

    forward_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['forward_velocity_command_controller',
                   '--param-file', ns_controllers_yaml,
                   '--inactive'],
    )

    # Sensor drivers — iterate enabled sensors from robot_params
    sensor_nodes = []
    for _sensor_name, sensor_cfg in robot_params.get('sensors', {}).items():
        if not sensor_cfg.get('enabled', True):
            continue
        s_share = get_package_share_directory(sensor_cfg['package'])
        real_launch = os.path.join(s_share, 'bringup', 'launch', 'real.launch.py')
        if os.path.isfile(real_launch):
            sensor_nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(real_launch),
                )
            )

    return [
        node_robot_state_publisher,
        node_controller_manager,
        *sensor_nodes,
        RegisterEventHandler(
            OnProcessExit(
                target_action=node_controller_manager,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[bicycle_steering_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=bicycle_steering_controller_spawner,
                on_exit=[
                    forward_position_controller_spawner,
                    forward_velocity_controller_spawner,
                ],
            )
        ),
    ]

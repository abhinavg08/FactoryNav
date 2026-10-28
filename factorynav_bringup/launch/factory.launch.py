"""Reuse the official Jazzy TB3 launch with a matching factory world and map."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def merge(target, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            merge(target[key], value)
        else:
            target[key] = value


def setup_launch(context):
    package = Path(__file__).resolve().parents[1]
    upstream = Path(get_package_share_directory('nav2_bringup'))
    enable_camera = LaunchConfiguration('enable_camera').perform(context).lower() == 'true'
    robot_path = Path(get_package_share_directory('nav2_minimal_tb3_sim')) / 'urdf/gz_waffle.sdf.xacro'
    temporary_paths = []
    if enable_camera:
        ET.register_namespace('xacro', 'http://www.ros.org/wiki/xacro')
        robot = ET.parse(robot_path)
        camera_link = robot.getroot().find("model/link[@name='camera_link']")
        sensor = ET.SubElement(camera_link, 'sensor', name='factory_rgb', type='camera')
        for key, value in [('pose', '0 0 0 0 0 0'), ('topic', '/camera/image_raw'),
                           ('gz_frame_id', 'camera_rgb_optical_frame'),
                           ('always_on', 'true'), ('update_rate', '5')]:
            ET.SubElement(sensor, key).text = value
        camera = ET.SubElement(sensor, 'camera')
        ET.SubElement(camera, 'horizontal_fov').text = '1.047'
        image = ET.SubElement(camera, 'image')
        for key, value in [('width', '640'), ('height', '480'), ('format', 'R8G8B8')]:
            ET.SubElement(image, key).text = value
        clip = ET.SubElement(camera, 'clip')
        ET.SubElement(clip, 'near').text = '0.05'
        ET.SubElement(clip, 'far').text = '15'
        with tempfile.NamedTemporaryFile(suffix='_factory_rgb.sdf.xacro', delete=False) as output:
            robot.write(output, encoding='utf-8', xml_declaration=True)
            robot_path = Path(output.name)
        temporary_paths.append(robot_path)
    params = yaml.safe_load((upstream / 'params/nav2_params.yaml').read_text())
    merge(params, yaml.safe_load((package / 'config/navigation_overrides.yaml').read_text()))
    with tempfile.NamedTemporaryFile(mode='w', suffix='_factorynav.yaml', delete=False) as output:
        yaml.safe_dump(params, output)
        params_path = Path(output.name)
        temporary_paths.append(params_path)
    rviz = yaml.safe_load((upstream / 'rviz/nav2_default_view.rviz').read_text())
    rviz['Panels'] = [panel for panel in rviz['Panels'] if panel['Class'] in (
        'rviz_common/Displays', 'nav2_rviz_plugins/Navigation 2')]
    rviz['Window Geometry'] = {'Width': 800, 'Height': 875, 'X': 800, 'Y': 0}
    rviz['Visualization Manager']['Views']['Current']['Scale'] = 28.0
    rviz['Visualization Manager']['Views']['Current']['X'] = 0.0
    rviz['Visualization Manager']['Views']['Current']['Y'] = 0.0
    with tempfile.NamedTemporaryFile(mode='w', suffix='_factorynav.rviz', delete=False) as output:
        yaml.safe_dump(rviz, output)
        rviz_path = Path(output.name)
        temporary_paths.append(rviz_path)

    def cleanup(_context):
        for path in temporary_paths:
            path.unlink(missing_ok=True)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(upstream / 'launch/tb3_simulation_launch.py')),
            launch_arguments={
                'world': str(package / 'worlds/factory.sdf'),
                'map': str(package / 'maps/factory.yaml'),
                'params_file': str(params_path),
                'rviz_config_file': str(rviz_path),
                'robot_sdf': str(robot_path),
                'x_pose': '-4.5', 'y_pose': '0.0', 'yaw': '0.0',
                'headless': LaunchConfiguration('headless'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items()),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=cleanup)])),
    ]
    if enable_camera:
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='factory_camera_bridge', output='screen', parameters=[{
                                'use_sim_time': True,
                                'override_frame_id': 'camera_rgb_optical_frame',
                                'config_file': str(package / 'config/camera_bridge.yaml')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('enable_camera', default_value='False'),
        OpaqueFunction(function=setup_launch),
    ])

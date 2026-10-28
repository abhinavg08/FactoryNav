"""Additional instrumented factory mode; original launch and maps are untouched."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context):
    package=Path(__file__).resolve().parents[1]; root=package.parent
    sys.path.insert(0,str(root/'factorynav_dynamic'))
    from factorynav_dynamic.models import instrument_robot
    spec=importlib.util.spec_from_file_location('baseline_launch',package/'launch/factory.launch.py')
    common=importlib.util.module_from_spec(spec); spec.loader.exec_module(common)
    upstream=Path(get_package_share_directory('nav2_bringup'))
    camera=LaunchConfiguration('enable_camera').perform(context).lower()=='true'
    profile=LaunchConfiguration('controller_profile').perform(context)
    if profile not in ('baseline','dynamic'): raise ValueError('controller_profile must be baseline or dynamic')
    x=float(LaunchConfiguration('x_pose').perform(context)); y=float(LaunchConfiguration('y_pose').perform(context)); yaw=float(LaunchConfiguration('yaw').perform(context))
    params=yaml.safe_load((upstream/'params/nav2_params.yaml').read_text())
    common.merge(params,yaml.safe_load((package/'config/dynamic_nav2.yaml').read_text()))
    if profile=='dynamic': common.merge(params,yaml.safe_load((package/'config/mppi_dynamic.yaml').read_text()))
    params['amcl']['ros__parameters']['initial_pose'].update(x=x,y=y,yaw=yaw)
    params['controller_server']['ros__parameters']['FollowPath']['visualize']=LaunchConfiguration('debug_trajectories').perform(context).lower()=='true'
    evidence=root/'log/dynamic/effective_nav2.yaml'; evidence.parent.mkdir(parents=True,exist_ok=True)
    evidence.write_text(yaml.safe_dump(params))
    paths=[]
    def save(data,suffix):
        with tempfile.NamedTemporaryFile(mode='w',suffix=suffix,delete=False) as file:
            file.write(data); path=Path(file.name); paths.append(path); return str(path)
    # Gazebo otherwise reapplies its 1200 px default after the WM's initial
    # resize, covering RViz. Configure the actual GUI window before it opens.
    gui=ET.parse(package/'worlds/dynamic_factory.sdf').getroot().find('world/gui')
    gui_path=save('<window><width>800</width><height>875</height></window>\n'+
        '\n'.join(ET.tostring(plugin,encoding='unicode') for plugin in gui.findall('plugin')),
        '_dynamic_gui.config')
    param_path=save(yaml.safe_dump(params),'_dynamic_nav2.yaml')
    ET.register_namespace('xacro','http://www.ros.org/wiki/xacro')
    robot=ET.parse(Path(get_package_share_directory('nav2_minimal_tb3_sim'))/'urdf/gz_waffle.sdf.xacro')
    contacts=instrument_robot(robot,camera)
    robot_path=save(ET.tostring(robot.getroot(),encoding='unicode'),'_dynamic_robot.sdf.xacro')
    bridges=[{'ros_topic_name':'/dynamic/robot_pose','gz_topic_name':'/model/turtlebot3_waffle/pose','ros_type_name':'tf2_msgs/msg/TFMessage','gz_type_name':'gz.msgs.Pose_V','direction':'GZ_TO_ROS'}]
    for name in ('human_a','human_b','cart'):
        bridges.extend([{'ros_topic_name':f'/dynamic/{name}/pose','gz_topic_name':f'/model/{name}/pose','ros_type_name':'tf2_msgs/msg/TFMessage','gz_type_name':'gz.msgs.Pose_V','direction':'GZ_TO_ROS'},
                        {'ros_topic_name':f'/dynamic/{name}/contacts','gz_topic_name':f'/dynamic/{name}/contacts','ros_type_name':'ros_gz_interfaces/msg/Contacts','gz_type_name':'gz.msgs.Contacts','direction':'GZ_TO_ROS'},
                        {'ros_topic_name':f'/dynamic/{name}/cmd_vel','gz_topic_name':f'/model/{name}/cmd_vel','ros_type_name':'geometry_msgs/msg/Twist','gz_type_name':'gz.msgs.Twist','direction':'ROS_TO_GZ'}])
    for topic in contacts: bridges.append({'ros_topic_name':topic,'gz_topic_name':topic,'ros_type_name':'ros_gz_interfaces/msg/Contacts','gz_type_name':'gz.msgs.Contacts','direction':'GZ_TO_ROS'})
    bridge_path=save(yaml.safe_dump(bridges),'_dynamic_bridge.yaml')
    rviz=yaml.safe_load((upstream/'rviz/nav2_default_view.rviz').read_text())
    rviz['Panels']=[p for p in rviz['Panels'] if p['Class'] in ('rviz_common/Displays','nav2_rviz_plugins/Navigation 2')]
    rviz['Window Geometry']={'Width':800,'Height':875,'X':800,'Y':0}
    rviz['Visualization Manager']['Views']['Current'].update(Scale=15.0,X=0.0,Y=0.0)
    rviz['Visualization Manager']['Displays'].append({'Class':'rviz_default_plugins/PointCloud2','Name':'2s obstacle forecast preview','Enabled':True,'Value':True,
        'Topic':{'Value':'/dynamic/predicted_obstacles','Depth':2,'History Policy':'Keep Last','Reliability Policy':'Best Effort','Durability Policy':'Volatile'},
        'Position Transformer':'XYZ','Color Transformer':'FlatColor','Color':'0; 180; 255','Style':'Points','Size (Pixels)':3,'Size (m)':0.03,'Alpha':0.8})
    debug=LaunchConfiguration('debug_trajectories').perform(context).lower()=='true'
    rviz['Visualization Manager']['Displays'].append({'Class':'rviz_default_plugins/MarkerArray','Name':'MPPI candidate trajectories','Enabled':debug,'Value':debug,
        'Topic':{'Value':'/trajectories','Depth':2,'History Policy':'Keep Last','Reliability Policy':'Reliable','Durability Policy':'Volatile'}})
    if camera:
        rviz['Visualization Manager']['Displays'].append({'Class':'rviz_default_plugins/Image','Name':'YOLO26 annotated camera','Enabled':True,'Value':True,
            'Topic':{'Value':'/perception/annotated','Depth':2,'History Policy':'Keep Last','Reliability Policy':'Reliable','Durability Policy':'Volatile'},
            'Normalize Range':True,'Min Value':0,'Max Value':1,'Median window':5})
    rviz_path=save(yaml.safe_dump(rviz),'_dynamic.rviz')
    def cleanup(_context):
        for path in paths: path.unlink(missing_ok=True)
    actions=[ExecuteProcess(cmd=['gz','sim','-r','-s',str(package/'worlds/dynamic_factory.sdf')],output='screen'),
        ExecuteProcess(cmd=['gz','sim','-g','--gui-config',gui_path],condition=IfCondition(PythonExpression(["not ",LaunchConfiguration('headless')])),output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(upstream/'launch/tb3_simulation_launch.py')),launch_arguments={
        'world':str(package/'worlds/dynamic_factory.sdf'),'map':str(package/'maps/dynamic_factory.yaml'),
        'params_file':param_path,'robot_sdf':robot_path,'rviz_config_file':rviz_path,
        'x_pose':str(x),'y_pose':str(y),'yaw':str(yaw),
        'use_simulator':'False',
        'headless':LaunchConfiguration('headless'),'use_rviz':LaunchConfiguration('use_rviz')}.items()),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='dynamic_bridge',parameters=[{'config_file':bridge_path,'use_sim_time':True}],output='screen'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]
    if camera: actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='factory_camera_bridge',parameters=[{'config_file':str(package/'config/camera_bridge.yaml'),'override_frame_id':'camera_rgb_optical_frame','use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name,default_value=value) for name,value in (
        ('headless','True'),('use_rviz','False'),('enable_camera','False'),('controller_profile','dynamic'),
        ('x_pose','-8.0'),('y_pose','-6.5'),('yaw','0.0'),('debug_trajectories','False'))]+[OpaqueFunction(function=setup)])

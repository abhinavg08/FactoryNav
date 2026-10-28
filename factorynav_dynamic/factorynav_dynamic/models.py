"""Gazebo Harmonic sensor instrumentation and physical moving models."""
import copy
import xml.etree.ElementTree as ET


def add(parent, tag, value=None, **attrs):
    result=ET.SubElement(parent,tag,attrs)
    if value is not None: result.text=str(value)
    return result


def pose_publisher(model):
    plugin=add(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    for key,value in [('publish_link_pose','false'),('publish_sensor_pose','false'),('publish_collision_pose','false'),('publish_visual_pose','false'),('publish_model_pose','true'),('publish_nested_model_pose','false'),('use_pose_vector_msg','true'),('update_frequency','20')]: add(plugin,key,value)


def instrument_robot(tree, camera=False):
    model=tree.getroot().find('model'); pose_publisher(model); contact_topics=[]
    for link in model.findall('link'):
        if link.find('collision') is None: continue
        topic='/dynamic/contacts/'+link.get('name')
        sensor=add(link,'sensor',name='contact_'+link.get('name'),type='contact')
        add(sensor,'always_on','true'); add(sensor,'update_rate','20'); add(sensor,'topic',topic)
        contact=add(sensor,'contact')
        add(contact,'topic',topic)
        for collision in link.findall('collision'): add(contact,'collision',collision.get('name'))
        contact_topics.append(topic)
    for sensor in model.findall('.//sensor'):
        if sensor.get('type')=='gpu_lidar': sensor.find('update_rate').text='10'
    if camera:
        link=model.find("link[@name='camera_link']")
        sensor=add(link,'sensor',name='factory_rgb',type='camera')
        for key,value in [('pose','0 0 0 0 0 0'),('topic','/camera/image_raw'),('gz_frame_id','camera_rgb_optical_frame'),('always_on','true'),('update_rate','5')]: add(sensor,key,value)
        cam=add(sensor,'camera'); add(cam,'horizontal_fov','1.047'); image=add(cam,'image')
        for key,value in [('width',640),('height',480),('format','R8G8B8')]: add(image,key,value)
        clip=add(cam,'clip'); add(clip,'near','.05'); add(clip,'far','15')
    return contact_topics


def moving_model(name, kind, x, y, yaw=0.0, dimensions=(1.0,.7,.8)):
    root=ET.Element('sdf',version='1.9'); model=add(root,'model',name=name)
    add(model,'pose',f'{x} {y} 0.01 0 0 {yaw}'); add(model,'static','false')
    link=add(model,'link',name='body'); add(link,'gravity','false')
    inertia=add(link,'inertial'); add(inertia,'mass','70' if kind=='human' else '45')
    add(inertia,'pose','0 0 0.9 0 0 0' if kind=='human' else '0 0 0.4 0 0 0')
    tensor=add(inertia,'inertia')
    for key,value in [('ixx',20),('iyy',20),('izz',3),('ixy',0),('ixz',0),('iyz',0)]: add(tensor,key,value)
    collision=add(link,'collision',name='traffic_collision')
    add(collision,'pose','0 0 .9 0 0 0' if kind=='human' else '0 0 .4 0 0 0')
    shape=add(add(collision,'geometry'),'cylinder' if kind=='human' else 'box')
    if kind=='human':
        add(shape,'radius','.28'); add(shape,'length','1.8')
        visible=add(link,'visual',name='worker_mesh'); add(visible,'pose','0 0 .02 .04 0 1.5708')
        add(add(add(visible,'geometry'),'mesh'),'uri','model://person_standing/meshes/standing.dae')
        proxy=add(link,'visual',name='lidar_visible_ankle_proxy'); add(proxy,'pose','0 0 .16 0 0 0')
        geo=add(add(proxy,'geometry'),'cylinder'); add(geo,'radius','.28'); add(geo,'length','.32')
        material=add(proxy,'material'); add(material,'ambient','.22 .26 .3 1'); add(material,'diffuse','.22 .26 .3 1')
    else:
        length,width,height=dimensions
        add(shape,'size',f'{length} {width} {height}')
        collision.find('pose').text=f'0 0 {height/2} 0 0 0'
        visual=add(link,'visual',name='cart_body'); add(visual,'pose',f'0 0 {height*.44} 0 0 0')
        add(add(add(visual,'geometry'),'box'),'size',f'{length} {width} {height*.62}')
        material=add(visual,'material'); add(material,'ambient','.95 .5 .08 1'); add(material,'diffuse','.95 .5 .08 1')
        load=add(link,'visual',name='cart_payload'); add(load,'pose',f'0 0 {height*.81} 0 0 0')
        add(add(add(load,'geometry'),'box'),'size',f'{length*.75} {width*.71} {height*.38}')
        material=add(load,'material'); add(material,'ambient','.45 .28 .15 1'); add(material,'diffuse','.45 .28 .15 1')
        for index,(wx,wy) in enumerate(((-1,-1),(-1,1),(1,-1),(1,1))):
            wheel=add(link,'visual',name=f'cart_wheel_{index}'); add(wheel,'pose',f'{wx*length*.35} {wy*width*.46} .10 1.5707963 0 0')
            cylinder=add(add(wheel,'geometry'),'cylinder'); add(cylinder,'radius','.1'); add(cylinder,'length','.06')
            material=add(wheel,'material'); add(material,'ambient','.06 .06 .06 1'); add(material,'diffuse','.06 .06 .06 1')
    plugin=add(model,'plugin',filename='gz-sim-velocity-control-system',name='gz::sim::systems::VelocityControl')
    add(plugin,'topic',f'/model/{name}/cmd_vel'); pose_publisher(model)
    sensor=add(link,'sensor',name='traffic_contact',type='contact')
    contact=add(sensor,'contact'); add(contact,'collision','traffic_collision'); add(contact,'topic',f'/dynamic/{name}/contacts')
    add(sensor,'always_on','true'); add(sensor,'update_rate','20')
    return ET.tostring(root,encoding='unicode')

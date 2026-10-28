"""Independent physical traffic; never subscribes to or scripts robot motion."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from .models import moving_model


def _configs(scenario,seed=0):
    variant=seed%3
    if scenario=='head_on': return [{'name':'human_b','kind':'human','route':[[5,0.12*(variant-1)],[-7.2,0.12*(variant-1)]],'speed':.24,'delay':0,'loop':True}]
    if scenario=='crossing': return [{'name':'human_a','kind':'human','route':[[0,-1.8],[0,1.8]],'speed':.25,'delay':6+variant,'loop':True}]
    if scenario=='alongside': return [{'name':'human_b','kind':'human','route':[[-6,.90+.1*(variant-1)],[7,.90+.1*(variant-1)]],'speed':.22,'delay':0,'loop':True}]
    if scenario=='cart': return [{'name':'cart','kind':'cart','route':[[0,-1.5],[0,1.5]],'speed':.18,'delay':5+variant,'loop':True}]
    if scenario=='cart_shared': return [{'name':'cart','kind':'cart','route':[[5,0],[-7,0]],'speed':.18,'delay':0,'loop':True}]
    if scenario=='blocked': return [{'name':'human_b','kind':'human','route':[[-9.925,.8],[-9.925,4.3]],'speed':.22,'delay':7,'loop':False}]
    if scenario=='contact_probe': return [{'name':'human_b','kind':'human','route':[[-5.5,0],[-8.5,0]],'speed':.22,'delay':0,'loop':False}]
    if scenario=='full': return [
        {'name':'human_a','kind':'human','route':[[-8.9,0],[-6.8,0]],'speed':.22,'delay':variant,'loop':True},
        {'name':'human_b','kind':'human','route':[[-5.5,1.1],[5.5,1.1]],'speed':.23,'delay':variant*2,'loop':True},
        {'name':'cart','kind':'cart','route':[[8,-1.3],[8,1.3]],'speed':.18,'delay':2+variant,'loop':True}]
    if scenario in ('static','station_test','clearance_test'): return []
    raise ValueError(f'Unknown scenario {scenario}')


def configs(scenario,seed=0):
    """Scenario speeds, delays, routes and cart dimensions are centralized here."""
    result=_configs(scenario,seed)
    for item in result:
        if item['kind']=='cart': item.setdefault('dimensions',[1.0,.7,.8])
    return result


def spawn(config):
    a,b=config['route'][:2]; yaw=math.atan2(b[1]-a[1],b[0]-a[0])
    sdf=moving_model(config['name'],config['kind'],*a,yaw,dimensions=config.get('dimensions',(1.0,.7,.8)))
    request=f'sdf: {json.dumps(sdf)} name: {json.dumps(config["name"])} allow_renaming: false'
    result=subprocess.run(['gz','service','-s','/world/default/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True,timeout=10)
    if result.returncode or 'data: true' not in result.stdout: raise RuntimeError(f'Spawn failed: {result.stdout} {result.stderr}')


class Traffic(Node):
    def __init__(self,configurations):
        super().__init__('factorynav_traffic_'+'_'.join(c['name'] for c in configurations),parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.configurations=configurations; self.poses={}; self.agent_publishers={}; self.targets={}; self.speeds={}; self.activation=None
        self.last_time=None; self.command_log=[]
        for config in configurations:
            name=config['name']; self.targets[name]=1; self.speeds[name]=0.0
            self.agent_publishers[name]=self.create_publisher(Twist,f'/dynamic/{name}/cmd_vel',10)
            self.create_subscription(TFMessage,f'/dynamic/{name}/pose',lambda msg,n=name:self.receive(msg,n),10)
        self.create_timer(.05,self.step)

    def receive(self,message,name):
        for transform in message.transforms:
            if transform.child_frame_id!=name: continue
            q=transform.transform.rotation; p=transform.transform.translation
            self.poses[name]=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def start_motion(self): self.activation=self.get_clock().now().nanoseconds/1e9

    def step(self):
        now=self.get_clock().now().nanoseconds/1e9
        dt=.05 if self.last_time is None else min(.15,max(0,now-self.last_time)); self.last_time=now
        for config in self.configurations:
            name=config['name']; command=Twist(); pose=self.poses.get(name)
            if pose is not None and self.activation is not None and now-self.activation>=config['delay']:
                index=self.targets[name]
                if index<len(config['route']):
                    target=config['route'][index]; dx=target[0]-pose[0]; dy=target[1]-pose[1]; distance=math.hypot(dx,dy)
                    if distance<.09:
                        index+=1
                        if index==len(config['route']) and config.get('loop',True):
                            config['route']=list(reversed(config['route'])); index=1
                        self.targets[name]=index
                    else:
                        heading=math.atan2(dy,dx); error=math.atan2(math.sin(heading-pose[2]),math.cos(heading-pose[2]))
                        desired=min(config['speed'],distance*.8)*max(0,math.cos(error))
                        speed=max(self.speeds[name]-.4*dt,min(self.speeds[name]+.4*dt,desired)); self.speeds[name]=speed
                        # VelocityControl commands are expressed in the model's own frame.
                        command.linear.x=speed*math.cos(error); command.linear.y=speed*math.sin(error)
                        command.angular.z=max(-1.2,min(1.2,error*2))
            self.agent_publishers[name].publish(command)

    def stop(self):
        for publisher in self.agent_publishers.values(): publisher.publish(Twist())


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--scenario',default='full'); parser.add_argument('--seed',type=int,default=0); parser.add_argument('--agents',nargs='+',choices=['human_a','human_b','cart']); args=parser.parse_args()
    rclpy.init(); config=configs(args.scenario,args.seed)
    if args.agents: config=[c for c in config if c['name'] in args.agents]
    if not config: raise RuntimeError('No traffic selected for this scenario')
    # GUI startup launches traffic alongside Gazebo; wait for the create service.
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        services=subprocess.run(['gz','service','-l'],capture_output=True,text=True,timeout=5)
        if '/world/default/create' in services.stdout: break
        time.sleep(.5)
    else: raise RuntimeError('Gazebo create service unavailable')
    for item in config: spawn(item)
    node=Traffic(config)
    try:
        deadline=time.monotonic()+30
        while len(node.poses)<len(config) and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
        if len(node.poses)!=len(config): raise RuntimeError('Missing agent ground truth')
        node.start_motion(); rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.stop(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()

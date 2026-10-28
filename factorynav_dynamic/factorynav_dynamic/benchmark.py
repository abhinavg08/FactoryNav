"""Real Nav2 goals, Gazebo ground truth, contacts, and timestamped trajectories."""
import argparse
import json
import math
from pathlib import Path
import time
import sys
import statistics
import threading
import copy
import os
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from ros_gz_interfaces.msg import Contacts
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from nav2_msgs.msg import CollisionMonitorState
from vision_msgs.msg import Detection2DArray
from rclpy.executors import SingleThreadedExecutor
from .navigation_io import NavigationCheck
from .geometry import circle_clearance,box_clearance,local,point_box
from .traffic import Traffic,configs,spawn


def pose_from(message,name):
    matches=[t for t in message.transforms if t.child_frame_id==name or t.child_frame_id.endswith('::'+name)]
    if not matches: return None
    t=matches[0]; q=t.transform.rotation
    return (t.transform.translation.x,t.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))


class Benchmark(NavigationCheck):
    def __init__(self,output):
        super().__init__(); self.output=output; output.mkdir(parents=True,exist_ok=True)
        self.robot=None; self.agents={}; self.contact_messages=0; self.contacts=[]; self.last_contact={}
        self.samples=[]; self.running=False; self.command=Twist(); self.recoveries=0; self.goal_results=[]
        self.min_clearance=math.inf; self.scan_latest=None; self.costmap=None; self.metrics=[]
        self.observations=[]; self.anchors={}; self.monitor_state=0
        self.agent_dimensions={}
        self.detection_messages=0; self.person_detection_messages=0
        self.command_sources={}
        self.predictions=[]
        self.odom_yaw=0.0
        self.static_obstacles=json.loads(Path('/workspace/factorynav_bringup/config/dynamic_scene.json').read_text())['obstacles']
        self.create_subscription(TFMessage,'/dynamic/robot_pose',self.robot_pose,10)
        for name in ('human_a','human_b','cart'):
            self.create_subscription(TFMessage,f'/dynamic/{name}/pose',lambda msg,n=name:self.agent_pose(msg,n),10)
            self.create_subscription(Contacts,f'/dynamic/{name}/contacts',self.on_contact,10)
        for name in ('base_link','wheel_left_link','wheel_right_link','caster_back_right_link','caster_back_left_link','camera_link'):
            self.create_subscription(Contacts,'/dynamic/contacts/'+name,self.on_contact,10)
        self.create_subscription(Twist,'/cmd_vel',self.on_command,10)
        self.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m:setattr(self,'costmap',m),10)
        self.create_subscription(String,'/perception/metrics',self.on_metrics,10)
        self.create_subscription(String,'/dynamic/prediction_metrics',self.on_prediction,10)
        self.create_subscription(Detection2DArray,'/perception/detections',self.on_detections,10)
        self.create_subscription(CollisionMonitorState,'/collision_monitor_state',lambda m:setattr(self,'monitor_state',m.action_type),10)
        self.create_timer(.1,self.sample)

    def robot_pose(self,message):
        pose=pose_from(message,'turtlebot3_waffle')
        if pose is not None: self.robot=pose

    def agent_pose(self,message,name):
        pose=pose_from(message,name)
        if pose is not None: self.agents[name]=pose

    def on_scan(self,message):
        super().on_scan(message); self.scan_latest=message

    def on_odom(self,message):
        super().on_odom(message)
        q=message.pose.pose.orientation
        self.odom_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def on_command(self,message,info):
        self.command=message
        if self.running:
            identifier=info.get('publisher_gid') if isinstance(info,dict) else getattr(info,'publisher_gid',None)
            gid=bytes(identifier).hex() if identifier is not None else 'publisher_gid_unavailable_in_rclpy_metadata'
            self.command_sources[gid]=self.command_sources.get(gid,0)+1

    def on_metrics(self,message):
        if self.running: self.metrics.append({'sim_s':self.get_clock().now().nanoseconds/1e9,**json.loads(message.data)})

    def on_detections(self,message):
        if self.running:
            self.detection_messages+=1
            if any(r.hypothesis.class_id=='person' for d in message.detections for r in d.results): self.person_detection_messages+=1

    def on_prediction(self,message):
        if self.running: self.predictions.append(json.loads(message.data))

    def on_contact(self,message):
        self.contact_messages+=1
        if not self.running: return
        now=self.get_clock().now().nanoseconds/1e9
        for contact in message.contacts:
            names=(contact.collision1.name,contact.collision2.name)
            if any('factory_floor' in name or 'ground_plane' in name for name in names): continue
            if not any('turtlebot3_waffle' in name for name in names): continue
            key='|'.join(sorted(names))
            if now-self.last_contact.get(key,-100)>0.3:
                self.contacts.append({'sim_s':now,'collisions':names,'depths':list(contact.depths)})
            self.last_contact[key]=now

    def sample(self):
        if not self.running or self.robot is None: return
        distances={name:(circle_clearance(self.robot,pose,.28) if name.startswith('human') else box_clearance(self.robot,pose,(-self.agent_dimensions.get(name,(1,.7))[0]/2,self.agent_dimensions.get(name,(1,.7))[0]/2,-self.agent_dimensions.get(name,(1,.7))[1]/2,self.agent_dimensions.get(name,(1,.7))[1]/2))) for name,pose in self.agents.items()}
        for item in self.static_obstacles:
            d=box_clearance(self.robot,(item['x'],item['y'],0),(-item['width']/2,item['width']/2,-item['depth']/2,item['depth']/2))
            distances[item['name']]=d
        self.min_clearance=min(self.min_clearance,min(distances.values(),default=math.inf))
        self.samples.append({'wall_s':time.monotonic(),'sim_s':self.get_clock().now().nanoseconds/1e9,
            'robot':self.robot,'agents':dict(self.agents),'clearances':distances,
            'odom':self.last_xy,'odom_yaw':self.odom_yaw,'cmd_v':self.command.linear.x,'cmd_w':self.command.angular.z,
            'collision_monitor_state':self.monitor_state})
        if len(self.samples)%50==0:
            (self.output/'progress.json').write_text(json.dumps(self.samples[-1]))
        if len(self.samples)%5==0: self.observe()

    def grid_peak(self,point,radius=.15):
        grid=self.costmap
        if grid is None: return None
        # Local costmap is in odom; transform ground truth map positions into odom.
        try:
            t=self.tf_buffer.lookup_transform(grid.header.frame_id,'map',rclpy.time.Time()).transform
        except Exception: return None
        q=t.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x=t.translation.x+math.cos(yaw)*point[0]-math.sin(yaw)*point[1]
        y=t.translation.y+math.sin(yaw)*point[0]+math.cos(yaw)*point[1]
        info=grid.info; ix=int((x-info.origin.position.x)/info.resolution); iy=int((y-info.origin.position.y)/info.resolution)
        if not (0<=ix<info.width and 0<=iy<info.height): return None
        cells=math.ceil(radius/info.resolution)
        values=[grid.data[row*info.width+col] for row in range(max(0,iy-cells),min(info.height,iy+cells+1)) for col in range(max(0,ix-cells),min(info.width,ix+cells+1))]
        return max(values,default=-1)

    def observe(self):
        scan=self.scan_latest
        if scan is None: return
        try:
            transform=self.tf_buffer.lookup_transform('map',scan.header.frame_id,rclpy.time.Time()).transform
        except Exception: return
        q=transform.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        points=[(transform.translation.x+r*math.cos(yaw+scan.angle_min+i*scan.angle_increment),transform.translation.y+r*math.sin(yaw+scan.angle_min+i*scan.angle_increment)) for i,r in enumerate(scan.ranges) if math.isfinite(r) and scan.range_min<=r<=scan.range_max]
        now=self.get_clock().now().nanoseconds/1e9
        for name,pose in self.agents.items():
            hits=sum(abs(math.dist(p,pose[:2])-.28)<.16 if name.startswith('human') else abs(point_box(local(p,pose),(-.5,.5,-.35,.35)))<.16 for p in points)
            cost=self.grid_peak(pose[:2],.35 if name.startswith('human') else .6); anchor=self.anchors.get(name); cleared=None; old_cost=None
            if hits and self.costmap is not None and math.dist(self.robot[:2],pose[:2])<2.5 and not (self.output/'costmap_snapshot.json').exists():
                grid=self.costmap
                t=self.tf_buffer.lookup_transform(grid.header.frame_id,'map',rclpy.time.Time()).transform
                (self.output/'costmap_snapshot.json').write_text(json.dumps({'robot':self.robot,'agent':pose,'scan_points_map':points,'map_to_grid':[t.translation.x,t.translation.y,t.rotation.z,t.rotation.w],'grid':{'frame':grid.header.frame_id,'origin':[grid.info.origin.position.x,grid.info.origin.position.y],'resolution':grid.info.resolution,'width':grid.info.width,'height':grid.info.height,'data':list(grid.data)}}))
            if anchor and math.dist(anchor['point'],pose[:2])>1.0:
                old_cost=self.grid_peak(anchor['point'],.08)
                cleared=old_cost is not None and 0<=old_cost<50
                if cleared: del self.anchors[name]
            if name not in self.anchors and cost is not None and cost>=99:
                self.anchors[name]={'point':pose[:2],'sim_s':now}
            self.observations.append({'sim_s':now,'agent':name,'distance_m':math.dist(self.robot[:2],pose[:2]),'lidar_boundary_hits':hits,'local_cost_at_agent':cost,'old_location_cost':old_cost,'old_location_cleared':cleared})

    def ready(self):
        if not self.spin_until(lambda:self.robot is not None and self.scan_count>3 and self.tf_buffer.can_transform('map','base_link',rclpy.time.Time()),90):
            raise RuntimeError('Missing simulation ground truth, sensors, or localization')
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            states={name:self.lifecycle(name) for name in ('amcl','map_server','planner_server','controller_server','bt_navigator')}
            if all(state=='active' for state in states.values()): return states
            self.spin_until(lambda:False,1)
        raise RuntimeError(f'Lifecycle activation failed: {states}')

    def goal(self,x,y,yaw=0,timeout=180):
        if not self.action.wait_for_server(timeout_sec=10): raise RuntimeError('NavigateToPose missing')
        goal=NavigateToPose.Goal(); goal.pose.header.frame_id='map'; goal.pose.header.stamp=self.get_clock().now().to_msg()
        goal.pose.pose.position.x=float(x); goal.pose.pose.position.y=float(y)
        goal.pose.pose.orientation.z=math.sin(yaw/2); goal.pose.pose.orientation.w=math.cos(yaw/2)
        self.goal_recoveries=0
        future=self.action.send_goal_async(goal,feedback_callback=lambda m:setattr(self,'goal_recoveries',max(self.goal_recoveries,m.feedback.number_of_recoveries)))
        if not self.spin_until(future.done,15): raise RuntimeError('Goal acceptance timeout')
        handle=future.result()
        if not handle.accepted: raise RuntimeError('Goal rejected')
        outcome=handle.get_result_async(); start=time.monotonic(); sim_start=self.get_clock().now().nanoseconds/1e9
        if not self.spin_until(outcome.done,timeout):
            cancel=handle.cancel_goal_async(); self.spin_until(cancel.done,3); raise RuntimeError('Goal timeout')
        response=outcome.result(); self.recoveries+=self.goal_recoveries
        record={'goal':[x,y,yaw],'status':response.status,'error_code':response.result.error_code,'recoveries':self.goal_recoveries,'wall_s':time.monotonic()-start,'simulation_s':self.get_clock().now().nanoseconds/1e9-sim_start}
        self.goal_results.append(record); print(json.dumps(record),flush=True)
        return response.status==GoalStatus.STATUS_SUCCEEDED

    def finish(self,metadata,error=None):
        self.running=False; path=0; stopped=0; moving=0; slowing=0
        for before,after in zip(self.samples,self.samples[1:]):
            dt=min(.5,max(0,after['sim_s']-before['sim_s'])); delta=math.dist(before['robot'][:2],after['robot'][:2]); path+=delta
            speed=delta/dt if dt>0 else 0
            if speed<.02: stopped+=dt
            else:
                moving+=dt
                if speed<.24: slowing+=dt
        near_stopped=0; near_progress=0; near_duration=0; max_stop=0; stop_run=0
        for before,after in zip(self.samples,self.samples[1:]):
            dt=min(.5,max(0,after['sim_s']-before['sim_s'])); delta=math.dist(before['robot'][:2],after['robot'][:2])
            near=any(math.dist(before['robot'][:2],p[:2])<2.5 for p in before['agents'].values())
            if near:
                near_duration+=dt; near_progress+=delta
                if dt>0 and delta/dt<.02: near_stopped+=dt; stop_run+=dt; max_stop=max(max_stop,stop_run)
                else: stop_run=0
            else: stop_run=0
        min_agent=min((v for s in self.samples for k,v in s['clearances'].items() if k in s['agents']),default=None)
        interactions={}
        for name in self.agents:
            samples=[s for s in self.samples if name in s['agents']]
            if not samples: continue
            first,last=samples[0],samples[-1]
            interactions[name]={'agent_travel_m':sum(math.dist(a['agents'][name][:2],b['agents'][name][:2]) for a,b in zip(samples,samples[1:])),
                'relative_x_order_changed':(first['robot'][0]-first['agents'][name][0])*(last['robot'][0]-last['agents'][name][0])<0,
                'maximum_robot_lateral_offset_m':max(abs(s['robot'][1]-first['robot'][1]) for s in samples)}
            vacated=None
            for later in samples:
                if math.dist(later['robot'][:2],later['agents'][name][:2])<1: continue
                for earlier in samples[::10]:
                    if later['sim_s']-earlier['sim_s']<3: break
                    if math.dist(later['robot'][:2],earlier['agents'][name][:2])<.28:
                        vacated={'earlier_sim_s':earlier['sim_s'],'reentry_sim_s':later['sim_s'],'location':earlier['agents'][name][:2]}; break
                if vacated: break
            interactions[name]['reentered_vacated_agent_footprint']=vacated
        sensor_summary={name:{'lidar_observed_samples':sum(o['lidar_boundary_hits']>0 for o in self.observations if o['agent']==name),'lethal_costmap_samples':sum(o['local_cost_at_agent'] is not None and o['local_cost_at_agent']>=99 for o in self.observations if o['agent']==name),'clearing_observations':sum(o['old_location_cleared'] is True for o in self.observations if o['agent']==name)} for name in self.agents}
        nav_success=bool(self.goal_results) and all(g['status']==4 for g in self.goal_results) and error is None
        safety=not self.contacts and self.min_clearance>=0
        probe=metadata['scenario']=='contact_probe'
        scores=[d['score'] for m in self.metrics for d in m.get('detections',[]) if d['class']=='person']
        moving_visible_frames=0
        for metric in self.metrics:
            if not any(d['class']=='person' for d in metric.get('detections',[])) or not self.samples: continue
            stamp=metric.get('source_stamp_s',metric['sim_s'])
            index=min(range(len(self.samples)),key=lambda i:abs(self.samples[i]['sim_s']-stamp))
            sample=self.samples[index]; prior=self.samples[max(0,index-5)]
            dt=sample['sim_s']-prior['sim_s']
            if dt<=0: continue
            for name,pose in sample['agents'].items():
                if not name.startswith('human') or name not in prior['agents']: continue
                dx=pose[0]-sample['robot'][0]; dy=pose[1]-sample['robot'][1]
                bearing=math.atan2(math.sin(math.atan2(dy,dx)-sample['robot'][2]),math.cos(math.atan2(dy,dx)-sample['robot'][2]))
                if math.dist(pose[:2],prior['agents'][name][:2])/dt>.05 and abs(bearing)<.65 and math.hypot(dx,dy)<12:
                    moving_visible_frames+=1; break
        directions=[1 if s['cmd_w']>0 else -1 for s in self.samples if abs(s['cmd_w'])>.25]
        continuous=metadata['scenario']=='head_on' and nav_success and safety and near_progress>.5 and max_stop<1.5 and any(i['relative_x_order_changed'] and i['maximum_robot_lateral_offset_m']>.45 for i in interactions.values())
        simulation_log=Path('/workspace/log/dynamic/simulation.log')
        temporal_active=simulation_log.exists() and 'DynamicObstacleCritic scored' in simulation_log.read_text(errors='replace')
        acceptance={'navigation_succeeded':nav_success,'no_contact_or_overlap':safety,'physical_clearance_at_least_0_08m':self.min_clearance>=.08,
                    'actors_moved':all(i['agent_travel_m']>.5 for i in interactions.values()),
                    'actors_lidar_observed':all(s['lidar_observed_samples']>0 for s in sensor_summary.values()),
                    'local_costmap_marking':not self.agents or any(s['lethal_costmap_samples']>0 for s in sensor_summary.values()),
                    'head_on_continuous_passing':metadata['scenario']!='head_on' or continuous,
                    'blocked_wait_and_progress':metadata['scenario']!='blocked' or (near_stopped>1 and path>1),
                    'temporal_critic_scored_live_tracks':metadata['controller_profile']!='dynamic' or not self.agents or temporal_active}
        result={**metadata,'success':(self.contact_messages>0 and bool(self.contacts)) if probe else all(acceptance.values()),
            'gui_enabled':os.environ.get('FACTORYNAV_GUI','false')=='true',
            'acceptance_checks':acceptance,
            'observed_cmd_vel_sources':self.command_sources,
            'advertised_cmd_vel_publishers':{bytes(p.endpoint_gid).hex():p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel')},
            'navigation_success':nav_success,'safety_pass':safety,
            'goals':self.goal_results,'error':error,'samples':len(self.samples),'path_length_m':path,
            'minimum_estimated_footprint_clearance_m':None if not math.isfinite(self.min_clearance) else self.min_clearance,
            'contact_events':len(self.contacts),'contact_messages_received':self.contact_messages,
            'stopped_sim_s':stopped,'slowing_sim_s':slowing,'moving_sim_s':moving,'recovery_count':self.recoveries,
            'near_agent_stopped_sim_s':near_stopped,'near_agent_progress_m':near_progress,'near_agent_duration_sim_s':near_duration,'longest_near_agent_stop_s':max_stop,
            'minimum_agent_footprint_clearance_m':min_agent,'interactions':interactions,'sensor_validation':sensor_summary,
            'clearance_margin_0_08m_pass':self.min_clearance>=.08,
            'angular_command_sign_changes_over_0_25rad_s':sum(a!=b for a,b in zip(directions,directions[1:])),
            'continuous_passing_evidence':continuous,
            'contact_measurement':'Gazebo contact messages and conservative ground-truth oriented rectangle/circle clearances sampled at 10 Hz. Isolated contact_probe validates sensor transport.',
            'perception_messages':len(self.metrics),'person_detection_frames':sum(any(d['class']=='person' for d in m.get('detections',[])) for m in self.metrics),
            'ros_detection_messages':self.detection_messages,'person_frames_with_moving_human_in_camera_frustum':moving_visible_frames,
            'ros_person_detection_messages':self.person_detection_messages,
            'forecast_enabled':os.environ.get('FACTORYNAV_FORECAST','true')=='true','forecast_messages':len(self.predictions),
            'forecast_active_messages':sum(m['projected_points']>0 for m in self.predictions),
            'tracking_active_messages':sum(any(t['forecast_active'] for t in m['tracks']) for m in self.predictions),
            'temporal_critic_active':temporal_active,
            'maximum_person_confidence':max(scores,default=None),'median_inference_ms':statistics.median([m['latency_ms'] for m in self.metrics]) if self.metrics else None}
        (self.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        (self.output/'trajectory.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in self.samples))
        (self.output/'contacts.json').write_text(json.dumps(self.contacts,indent=2)+'\n')
        (self.output/'perception.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in self.metrics))
        (self.output/'sensor_observations.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in self.observations))
        (self.output/'prediction_observations.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in self.predictions))
        print(json.dumps(result,indent=2),flush=True); return result


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--scenario',default='static'); parser.add_argument('--output',type=Path,default=Path('/workspace/docs/dynamic_test_runs/static')); parser.add_argument('--profile',default='dynamic'); parser.add_argument('--seed',type=int,default=0); parser.add_argument('--external-traffic',action='store_true')
    args=parser.parse_args(); rclpy.init(); node=Benchmark(args.output); error=None; traffic=None; executor=None; worker=None
    config=configs(args.scenario,args.seed); original_config=copy.deepcopy(config)
    node.agent_dimensions={c['name']:c.get('dimensions',(1,.7,.8)) for c in config}
    try:
        node.ready()
        if not args.external_traffic:
            for item in config: spawn(item)
        if config and not args.external_traffic:
            traffic=Traffic(config); executor=SingleThreadedExecutor(); executor.add_node(traffic)
            worker=threading.Thread(target=executor.spin,daemon=True); worker.start()
            if not node.spin_until(lambda:len(node.agents)==len(config) and len(traffic.poses)==len(config),30): raise RuntimeError('Missing traffic poses')
        elif config and not node.spin_until(lambda:len(node.agents)==len(config),30): raise RuntimeError('Missing externally controlled traffic poses')
        node.running=True
        if traffic: traffic.start_motion()
        stations=json.loads(Path('/workspace/factorynav_bringup/config/dynamic_scene.json').read_text())['stations']
        if args.scenario=='contact_probe': node.spin_until(lambda:False,20)
        elif args.scenario in ('static','station_test','full'):
            names=('assembly','quality','delivery') if args.scenario=='full' else ('assembly','cnc','quality','storage','packing','delivery')
            for name in names:
                station=next(s for s in stations if s['name']==name)
                if not node.goal(station['x'],station['y'],timeout=240): raise RuntimeError(f'Failed station {name}')
        else:
            goal={'head_on':(7,0,0),'crossing':(5,0,0),'alongside':(7,0,0),'cart':(7,0,0),'cart_shared':(7,0,0),'blocked':(-9.925,3.6,math.pi/2)}[args.scenario]
            if not node.goal(*goal,timeout=180): raise RuntimeError('Navigation action failed')
            node.spin_until(lambda:False,1)
    except Exception as exc: error=str(exc)
    finally:
        if traffic: traffic.stop()
        result=node.finish({'scenario':args.scenario,'controller_profile':args.profile,'seed':args.seed,'traffic_configuration':original_config},error)
        if executor: executor.shutdown()
        if worker: worker.join(timeout=2)
        if traffic: traffic.destroy_node()
        node.destroy_node(); rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__=='__main__': raise SystemExit(main())

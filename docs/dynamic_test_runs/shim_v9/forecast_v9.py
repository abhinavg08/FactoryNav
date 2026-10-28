"""Short constant-velocity forecasts from real unmapped LiDAR returns only.

No ground-truth, actor-route, YOLO, or robot-velocity publisher is used.
Projected points are explicitly separate from the unmodified raw scan.
"""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header,String
from tf2_ros import Buffer,TransformListener


def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def transform(point,t):
    angle=yaw(t.rotation); c=math.cos(angle); s=math.sin(angle)
    return t.translation.x+c*point[0]-s*point[1],t.translation.y+s*point[0]+c*point[1]


def clusters(points,threshold=.35):
    result=[]; group=[]
    for point in points:
        if point is None:
            if len(group)>=3: result.append(group)
            group=[]; continue
        if group and math.dist(group[-1],point)>threshold:
            if len(group)>=3: result.append(group)
            group=[]
        group.append(point)
    if len(group)>=3: result.append(group)
    if len(result)>1 and math.dist(result[0][0],result[-1][-1])<threshold:
        result[0]=result[-1]+result[0]; result.pop()
    return [g for g in result if max(math.dist(g[0],p) for p in g)<1.8]


class Forecast(Node):
    def __init__(self):
        super().__init__('factorynav_lidar_forecast',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.grid=None; self.tracks={}; self.next_id=0
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.cloud=self.create_publisher(PointCloud2,'/dynamic/predicted_obstacles',10)
        self.metrics=self.create_publisher(String,'/dynamic/prediction_metrics',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'grid',m),qos)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)

    def mapped(self,point):
        grid=self.grid; info=grid.info
        x=int((point[0]-info.origin.position.x)/info.resolution); y=int((point[1]-info.origin.position.y)/info.resolution)
        radius=math.ceil(.20/info.resolution)
        if not (radius<=x<info.width-radius and radius<=y<info.height-radius): return True
        return any(grid.data[row*info.width+col]!=0 for row in range(y-radius,y+radius+1) for col in range(x-radius,x+radius+1))

    def scan(self,message):
        if self.grid is None: return
        try:
            sensor_to_odom=self.buffer.lookup_transform('odom',message.header.frame_id,rclpy.time.Time()).transform
            odom_to_map=self.buffer.lookup_transform('map','odom',rclpy.time.Time()).transform
        except Exception: return
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        points=[]
        for index,distance in enumerate(message.ranges):
            if not math.isfinite(distance) or not message.range_min<distance<min(4.0,message.range_max): points.append(None); continue
            angle=message.angle_min+index*message.angle_increment
            point=transform((distance*math.cos(angle),distance*math.sin(angle)),sensor_to_odom)
            points.append(None if self.mapped(transform(point,odom_to_map)) else point)
        groups=clusters(points); available=set(self.tracks); updated={}; forecasts=[]; measurements=[]
        for group in groups:
            center=(sum(p[0] for p in group)/len(group),sum(p[1] for p in group)/len(group))
            candidate=min(available,key=lambda i:math.dist(center,self.tracks[i]['center']),default=None)
            previous=self.tracks.get(candidate)
            if previous and math.dist(center,previous['center'])<.65 and 0<stamp-previous['stamp']<.5:
                available.remove(candidate); dt=stamp-previous['stamp']
                measured=((center[0]-previous['center'][0])/dt,(center[1]-previous['center'][1])/dt)
                velocity=tuple(.75*previous['velocity'][i]+.25*measured[i] for i in range(2)); frames=previous['frames']+1
            else:
                candidate=self.next_id; self.next_id+=1; velocity=(0.0,0.0); frames=1
            speed=math.hypot(*velocity)
            if speed>.65: velocity=tuple(v*.65/speed for v in velocity); speed=.65
            updated[candidate]={'center':center,'velocity':velocity,'stamp':stamp,'frames':frames}
            active=frames>=6 and .08<speed<=.65
            if active:
                # Translate observed surfaces; do not assume a semantic human/cart shape.
                for horizon in (.5,1.0,1.5,2.0):
                    forecasts.extend((p[0]+velocity[0]*horizon,p[1]+velocity[1]*horizon,.20) for p in group[::max(1,len(group)//32)])
            measurements.append({'track_id':candidate,'observed_center_odom':center,'velocity_odom_m_s':velocity,'frames':frames,'forecast_active':active,'surface_points':len(group)})
        self.tracks=updated
        header=Header(); header.frame_id='odom'; header.stamp=message.header.stamp
        self.cloud.publish(create_cloud_xyz32(header,forecasts))
        metrics=String(); metrics.data=json.dumps({'source_stamp_s':stamp,'forecast_horizon_s':2.0,'projected_points':len(forecasts),'tracks':measurements})
        self.metrics.publish(metrics)


def main():
    rclpy.init(); node=Forecast()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()

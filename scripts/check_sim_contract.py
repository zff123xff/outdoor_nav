#!/usr/bin/env python3
"""Inspect real Classic messages and bounded navigation in an isolated fixture."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--domain-id',type=int,required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']=str(args.domain_id)
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    check=subprocess.run(['ros2','topic','list','--no-daemon','--spin-time','3'],capture_output=True,text=True,timeout=12)
    if check.returncode or any(t not in ('/parameter_events','/rosout') for t in check.stdout.split()):raise RuntimeError('Domain is not empty')
    sock=socket.socket();sock.bind(('127.0.0.1',0));os.environ['GAZEBO_MASTER_URI']='http://127.0.0.1:'+str(sock.getsockname()[1]);sock.close()
    os.environ['ROS_LOG_DIR']=str(output/'ros')
    log=(output/'launch.log').open('w')
    process=subprocess.Popen(['ros2','launch',str(Path(__file__).with_name('simulation_fixture.launch.py')),'world:=empty.world','robot_namespace:=robot_1','gui:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('zny_classic_audit',parameter_overrides=[Parameter('use_sim_time',value=True)])
    messages={};stamps={};nav_commands=[];action_ids={};subs=[]
    def receive(key,msg):
        messages[key]=msg
        if hasattr(msg,'header'):stamps.setdefault(key,[]).append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        if key=='nav_cmd':nav_commands.append([msg.linear.x,msg.linear.y,msg.angular.z])
        if key=='status':
            for item in msg.status_list:action_ids[bytes(item.goal_info.goal_id.uuid).hex()]=item.status
    for topic,typ,key in [('mavros/odom',Odometry,'odom'),('livox/lidar',PointCloud2,'cloud'),('livox/imu',Imu,'imu'),('sensors/gps/fix',NavSatFix,'gps'),('simulation/truth/odom',Odometry,'truth'),('ekf/status',String,'ekf'),('cmd_vel_nav',Twist,'nav_cmd'),('follow_path/_action/status',GoalStatusArray,'status'),('goal_pose',PoseStamped,'goal')]:
        subs.append(node.create_subscription(typ,'/robot_1/'+topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data))
    subs.append(node.create_subscription(OccupancyGrid,'/robot_1/local_costmap/costmap',lambda m:receive('costmap',m),qos_profile_sensor_data))
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    clients=[node.create_client(GetState,'/robot_1/'+name+'/get_state') for name in ('controller_server','straight_planner')];futures=[None,None]
    pose_pub=node.create_publisher(PoseStamped,'/robot_1/goal_pose',10)
    gps_pub=node.create_publisher(NavSatFix,'/robot_1/gp_goal',10)
    stop=node.create_publisher(Bool,'/robot_1/target_detected',10)
    result={}
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)
            if process.poll() is not None:raise RuntimeError('Classic launch exited')
    def tf(parent,child):return buffer.lookup_transform('robot_1/'+parent,'robot_1/'+child,rclpy.time.Time())
    def point():
        t=tf('map','base_link').transform.translation
        return [t.x,t.y,t.z]
    def quat(q):return [q.x,q.y,q.z,q.w]
    try:
        deadline=time.monotonic()+160
        while time.monotonic()<deadline:
            spin(.1)
            for i,c in enumerate(clients):
                if futures[i] is None and c.service_is_ready():futures[i]=c.call_async(GetState.Request())
                if futures[i] is not None and futures[i].done() and (futures[i].exception() or futures[i].result().current_state.id!=3):futures[i]=None
            active=all(f is not None and f.done() and f.exception() is None and f.result().current_state.id==3 for f in futures)
            if active and all(k in messages for k in ('odom','cloud','truth','ekf')) and 'ready=1' in messages['ekf'].data:
                try:
                    transform=tf('map','base_link');age=(node.get_clock().now()-rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds*1e-9
                    if 0<=age<1:break
                except Exception:pass
        else:raise RuntimeError('Real data / navigation / TF not ready')
        spin(2.)
        m=messages['odom'];cloud=messages['cloud'];mount=tf('base_link','livox_frame').transform
        result['odom']={'frame_id':m.header.frame_id,'child_frame_id':m.child_frame_id,'pose_z':m.pose.pose.position.z,'tf_z':point()[2]}
        result['rates']={k:(len(v)-1)/(v[-1]-v[0]) for k,v in stamps.items() if len(v)>2 and v[-1]>v[0]}
        result['pointcloud']={'frame_id':cloud.header.frame_id,'point_step':cloud.point_step,'row_step':cloud.row_step,'width':cloud.width,'data_bytes':len(cloud.data),'fields':[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in cloud.fields]}
        result['mount']={'xyz':[mount.translation.x,mount.translation.y,mount.translation.z],'quaternion_xyzw':quat(mount.rotation)}
        def endpoints(topic,subscriber=False):
            items=node.get_subscriptions_info_by_topic(topic) if subscriber else node.get_publishers_info_by_topic(topic)
            return [dict(node=i.node_namespace+'/'+i.node_name,reliability=str(i.qos_profile.reliability),type=i.topic_type) for i in items]
        result['qos']={t:endpoints('/robot_1/'+t) for t in ('mavros/odom','livox/lidar','cmd_vel')}
        result['cloud_consumers']=endpoints('/robot_1/livox/lidar',True)
        if 'costmap' in messages:
            grid=messages['costmap'];result['costmap']={'frame':grid.header.frame_id,'width':grid.info.width,'height':grid.info.height,'nonzero':sum(v>0 for v in grid.data),'lethal':sum(v>=99 for v in grid.data)}
        result['cmd_consumers']=endpoints('/robot_1/cmd_vel',True)
        result['set_gp_origin_subscribers']=node.count_subscribers('/robot_1/mavros/set_gp_origin')
        result['task_subscribers']=node.count_subscribers('/robot_1/task')
        result['robot_task_status_available']=any(name == '/robot_1/robot_task_status' for name, types in node.get_topic_names_and_types())
        # Transform actual recorded points with actual TF, without injecting any TF or odometry.
        sensor_tf=tf('map','livox_frame').transform;q=sensor_tf.rotation
        row=[2*(q.x*q.z-q.w*q.y),2*(q.y*q.z+q.w*q.x),1-2*(q.x*q.x+q.y*q.y)]
        field={f.name:f.offset for f in cloud.fields};heights=[]
        for i in range(cloud.width):
            xyz=[struct.unpack_from('<f',cloud.data,i*cloud.point_step+field[a])[0] for a in ('x','y','z')]
            if all(math.isfinite(v) for v in xyz):heights.append(sum(a*b for a,b in zip(row,xyz))+sensor_tf.translation.z)
        result['cloud_world_heights']={'points':len(heights),'in_obstacle_height_band':sum(.05<=z<=1 for z in heights),'min':min(heights,default=0),'max':max(heights,default=0)}
        # At-position yaw goal: check whether the existing planner actually sends a new action.
        pos=point();before=set(action_ids);nav_commands.clear()
        q=tf('map','base_link').transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        target=PoseStamped();target.header.frame_id='robot_1/map';target.header.stamp=node.get_clock().now().to_msg();target.pose.position.x=pos[0];target.pose.position.y=pos[1];target.pose.orientation.z=math.sin((yaw+.5)/2);target.pose.orientation.w=math.cos((yaw+.5)/2)
        pose_pub.publish(target);spin(2.5)
        result['yaw_only']={'new_action_ids':list(set(action_ids)-before),'nonzero_nav_commands':sum(abs(v[0])+abs(v[1])+abs(v[2])>1e-3 for v in nav_commands)}
        # GPS goal is derived from the current ZERO origin, not the document's remote fixed coordinates.
        start=point();goal=[start[0]+1.,start[1]];before=set(action_ids);nav_commands.clear()
        fix=NavSatFix();fix.header.frame_id='gps';fix.status.status=0;fix.status.service=1;fix.latitude=math.degrees(goal[1]/6371000.);fix.longitude=math.degrees(goal[0]/6371000.);fix.altitude=0.
        gps_pub.publish(fix)
        deadline=time.monotonic()+24
        while time.monotonic()<deadline:
            spin(.1)
            statuses={k:v for k,v in action_ids.items() if k not in before}
            p=point();error=math.hypot(p[0]-goal[0],p[1]-goal[1])
            if error<.32 and 4 in statuses.values():break
            if statuses and all(v in (4,5,6) for v in statuses.values()) and 6 in statuses.values():break
        p=point();converted=messages.get('goal')
        result['gps_goal']={'start':start,'target':goal,'final':p,'position_error':math.hypot(p[0]-goal[0],p[1]-goal[1]),'new_action_statuses':{k:v for k,v in action_ids.items() if k not in before},'max_nav_vx':max((abs(v[0]) for v in nav_commands),default=0),'max_nav_vy':max((abs(v[1]) for v in nav_commands),default=0),'nav_command_count':len(nav_commands),'converted_goal_xy':None if converted is None else [converted.pose.position.x,converted.pose.position.y]}
        # Cancel a fresh action while it is moving; stopping an already-complete
        # goal would not prove the target_detected behavior.
        start_stop=point();before_stop=set(action_ids);nav_commands.clear()
        fix.latitude=math.degrees(start_stop[1]/6371000.)
        fix.longitude=math.degrees((start_stop[0]+1.5)/6371000.)
        gps_pub.publish(fix)
        stop_deadline=time.monotonic()+5
        while time.monotonic()<stop_deadline:
            spin(.1)
            if any(abs(v[0])+abs(v[2])>.02 for v in nav_commands) and any(k not in before_stop and v==2 for k,v in action_ids.items()):break
        stop_was_active=any(k not in before_stop and v==2 for k,v in action_ids.items())
        stop_start=point()
        stop.publish(Bool(data=True));spin(2.)
        v=messages['truth'].twist.twist.linear
        result['target_detected_stop']={'was_active':stop_was_active,'stop_start':stop_start,'stop_end':point(),'new_action_statuses':{k:v for k,v in action_ids.items() if k not in before_stop},'actual_speed':math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),'action_statuses':dict(action_ids)}
        result['completed']=True
    except Exception as exc:
        result['error']=str(exc);raise
    finally:
        (output/'result.json').write_text(json.dumps(result,indent=2))
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=18)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8)
        log.close();node.destroy_node();rclpy.try_shutdown()
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()

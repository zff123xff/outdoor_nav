#!/usr/bin/env python3
"""Start an isolated Gazebo robot and verify hybrid control with short movements.

For this integration fixture only, source ROS, outdoor_nav/install/local_setup.bash
and sim_ws/install/local_setup.bash first. This intentionally moves only the simulator it
starts, in an explicitly selected, otherwise empty ROS domain.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2, NavSatFix, Imu
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from lifecycle_msgs.srv import GetState
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id',type=int,required=True)
    parser.add_argument('--world',default='empty.world')
    parser.add_argument('--stationary',action='store_true')
    parser.add_argument('--output',default='/tmp/go2w_hybrid_smoke')
    args=parser.parse_args()
    if not 0<=args.domain_id<=232: parser.error('invalid ROS domain')
    os.environ['ROS_DOMAIN_ID']=str(args.domain_id)
    master_socket=socket.socket();master_socket.bind(('127.0.0.1',0))
    os.environ['GAZEBO_MASTER_URI']='http://127.0.0.1:'+str(master_socket.getsockname()[1])
    master_socket.close()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_LOG_DIR']=str(output/'ros')
    # No robot should be present in this domain before this test owns a launch.
    probe=subprocess.run(['ros2','topic','list','--no-daemon','--spin-time','3'],capture_output=True,text=True,timeout=12)
    if probe.returncode or any(t not in ('/parameter_events','/rosout') for t in probe.stdout.split()):
        raise RuntimeError('Test domain is not empty or discovery failed: '+probe.stdout+probe.stderr)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('',0));sock.close()
    ns='hybrid_test'
    log=(output/'launch.log').open('w')
    launch=subprocess.Popen(['ros2','launch',str(Path(__file__).with_name('simulation_fixture.launch.py')),
        'profile:=robotdog_nav','locomotion_mode:=hybrid','world:='+args.world,
        'robot_namespace:='+ns,'gui:=false'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init()
    node=Node('hybrid_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    received={};subs=[];hist=[];counts={};stamps={}
    def receive(key,msg):
        received[key]=msg
        counts[key]=counts.get(key,0)+1
        if hasattr(msg,'header'):
            stamps.setdefault(key,[]).append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        if key=='trajectory' and msg.points: hist.append(list(msg.points[-1].positions))
    for topic,typ,key in [('simulation/truth/odom',Odometry,'truth'),('ekf/status',String,'ekf'),('cmd_vel_legs',Twist,'legs'),('livox/lidar',PointCloud2,'cloud'),('livox/imu',Imu,'imu'),('sensors/gps/fix',NavSatFix,'gps')]:
        subs.append(node.create_subscription(typ,'/'+ns+'/'+topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data))
    for topic,typ,key in [('/joint_states',JointState,'joints'),('/wheel_velocity_controller/commands',Float64MultiArray,'wheels'),('/leg_trajectory_controller/joint_trajectory',JointTrajectory,'trajectory')]:
        subs.append(node.create_subscription(typ,topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data))
    pub=node.create_publisher(Twist,'/'+ns+'/cmd_vel_remote',10)
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    clients=[node.create_client(GetState,'/'+ns+'/'+n+'/get_state') for n in ('controller_server','straight_planner')]
    future=[None,None]
    controllers=node.create_client(ListControllers,'/controller_manager/list_controllers')
    result={}
    def spin():
        rclpy.spin_once(node,timeout_sec=.03)
        if launch.poll() is not None: raise RuntimeError('Simulator launch exited early')
        if 'truth' in received:
            q=received['truth'].pose.pose.orientation
            tilt=math.acos(min(1.,max(-1.,1-2*(q.x*q.x+q.y*q.y))))
            if tilt>.55: raise RuntimeError('Robot tilted %.3f rad; test stopped' % tilt)
    def pose():
        m=received['truth'];p=m.pose.pose.position
        return [p.x,p.y,p.z,yaw(m.pose.pose.orientation)]
    def run_stage(name,vx=0.,vy=0.,wz=0.,duration=2.,publish=True):
        start=pose();start_time=node.get_clock().now().nanoseconds;deadline=time.monotonic()+40
        hist.clear();wheel_peak=0.;leg_y_peak=0.;leg_yaw_peak=0.
        command=Twist();command.linear.x=vx;command.linear.y=vy;command.angular.z=wz
        while (node.get_clock().now().nanoseconds-start_time)*1e-9<duration:
            if time.monotonic()>deadline: raise RuntimeError('Simulation clock stalled')
            if publish: pub.publish(command)
            spin()
            if 'wheels' in received:wheel_peak=max(wheel_peak,max(map(abs,received['wheels'].data),default=0.))
            if 'legs' in received:
                leg_y_peak=max(leg_y_peak,abs(received['legs'].linear.y))
                leg_yaw_peak=max(leg_yaw_peak,abs(received['legs'].angular.z))
        finish=pose();variation=max((max(j)-min(j) for j in zip(*hist)),default=0.)
        velocity=received['truth'].twist.twist.linear
        speed=math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
        result[name]={'stopped_speed':speed,'start':start,'finish':finish,'leg_joint_span':variation,'wheel_command_peak':wheel_peak,'leg_y_peak':leg_y_peak,'leg_yaw_peak':leg_yaw_peak}
        if not publish:
            assert speed<.05, 'Robot still moving after command timeout: '+str(speed)
            assert max(map(abs,received['wheels'].data))<1e-9, 'Wheel commands remained nonzero'
        print(name,json.dumps(result[name]),flush=True)
        return start,finish
    try:
        deadline=time.monotonic()+180
        while time.monotonic()<deadline:
            spin()
            for i,c in enumerate(clients):
                if c.service_is_ready() and future[i] is None: future[i]=c.call_async(GetState.Request())
                if future[i] is not None and future[i].done() and (future[i].exception() or future[i].result().current_state.id!=3):future[i]=None
            active=all(f is not None and f.done() and f.exception() is None and f.result().current_state.id==3 for f in future)
            if not (active and 'ekf' in received and 'ready=1' in received['ekf'].data and all(k in received for k in ('truth','joints','wheels','trajectory','cloud','imu','gps'))):continue
            try:
                tf=buffer.lookup_transform(ns+'/map',ns+'/base_link',rclpy.time.Time())
            except Exception:
                continue
            age=(node.get_clock().now()-rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds*1e-9
            truth_age=(node.get_clock().now()-rclpy.time.Time.from_msg(received['truth'].header.stamp)).nanoseconds*1e-9
            if 0<=age<1 and 0<=truth_age<1 and pub.get_subscription_count()>0:break
        else:raise RuntimeError('EKF, navigation, fresh TF or actual sensor receipt not ready')
        future_controllers=controllers.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node,future_controllers,timeout_sec=8)
        states={c.name:c.state for c in future_controllers.result().controller}
        assert states.get('leg_trajectory_controller')=='active' and states.get('wheel_velocity_controller')=='active',states
        assert 'diff_drive_controller' not in states and 'leg_position_controller' not in states,states
        assert len(received['trajectory'].joint_names)==12
        assert received['cloud'].header.frame_id==ns+'/livox_frame' and received['cloud'].width>0
        result['readiness']={'controllers':states,'ekf':received['ekf'].data,'pose':pose()}
        print('READY',json.dumps(result['readiness']),flush=True)
        if args.stationary:
            run_stage('stationary',duration=5,publish=False)
            result['passed']=True
            return
        run_stage('idle',duration=1)
        a,b=run_stage('forward',vx=.12,duration=2)
        assert (b[0]-a[0])*math.cos(a[3])+(b[1]-a[1])*math.sin(a[3])>.08,'No forward progress'
        run_stage('forward_watchdog',duration=1.5,publish=False)
        assert max(map(abs,received['wheels'].data))<1e-9,'Wheel watchdog did not stop'
        a,b=run_stage('turn',wz=.15,duration=2)
        assert abs(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3])))>.04,'No rotation'
        assert result['turn']['leg_joint_span']>.01,'CHAMP legs did not step during turn'
        run_stage('turn_watchdog',duration=1.5,publish=False)
        a,b=run_stage('lateral',vy=.06,duration=3)
        lateral=-(b[0]-a[0])*math.sin(a[3])+(b[1]-a[1])*math.cos(a[3])
        assert lateral>.015,'No lateral progress: '+str(lateral)
        assert result['lateral']['leg_joint_span']>.01,'CHAMP legs did not step laterally'
        run_stage('lateral_watchdog',duration=2,publish=False)
        assert max(map(abs,received['wheels'].data))<1e-9
        result['passed']=True
    except Exception as exc:
        result['error']=str(exc)
        raise
    finally:
        pub.publish(Twist())
        result['sensor_counts']=counts
        result['sensor_rates']={k:(len(v)-1)/(v[-1]-v[0]) for k,v in stamps.items() if len(v)>1 and v[-1]>v[0]}
        if 'cloud' in received:result['pointcloud']={'frame':received['cloud'].header.frame_id,'width':received['cloud'].width}
        (output/'result.json').write_text(json.dumps(result,indent=2))
        if launch.poll() is None:
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()

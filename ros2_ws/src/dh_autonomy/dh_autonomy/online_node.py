"""CAD-free ROS shell; online_stationary hard-locks the dry backend.

Reuses organizer odometry/VO/depth/TF; starts none of those providers. No course
file parameter exists. Measured TF and verified depth geometry are mandatory.
Normal online_mission also defaults dry; future explicit physical mode supports
only the organizer Sky Beta interface. No physical deployment is authorized.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.time import Time
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,PolygonStamped,Point32
from std_msgs.msg import Empty,String,Header
from tf2_ros import Buffer,TransformListener
from dh_core.camera_geom import quat_to_rot
from dh_core.online_geometry import RigidHistory,validate_mount,register_depth_to_color
from dh_core.online_map import camera_rays
from dh_core.online_navigation import OnlineNavigator,GoalObservation
from dh_core.landing_pad import PadTrack,detect_blue_pad,landing_readiness
from .backends import DryRunBackend,SkybetaBackend,verify_isolation


def stamp(msg): return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def decode_depth(msg):
    if msg.encoding not in ('16UC1','32FC1'): raise ValueError('Unsupported depth encoding')
    dtype=np.dtype(('>' if msg.is_bigendian else '<')+('u2' if msg.encoding=='16UC1' else 'f4'))
    size=dtype.itemsize
    if msg.step<msg.width*size or msg.step%size or len(msg.data)<msg.step*msg.height: raise ValueError('Depth buffer/stride')
    d=np.frombuffer(bytes(msg.data),dtype=dtype,count=msg.height*msg.step//size).reshape(msg.height,msg.step//size)[:,:msg.width].astype(float)
    if msg.encoding=='16UC1': d*=.001
    d[(d<=.1)|(d>8.)|(~np.isfinite(d))]=np.nan
    return d


def decode_color(msg):
    if msg.encoding not in ('bgr8','rgb8'): raise ValueError('Blue detection requires genuine color, not mono')
    if msg.step<msg.width*3 or len(msg.data)<msg.step*msg.height: raise ValueError('Color buffer/stride')
    data=np.frombuffer(bytes(msg.data),np.uint8,count=msg.step*msg.height).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
    return data[:,:,::-1].copy() if msg.encoding=='rgb8' else data.copy()


class OnlineMissionNode(Node):
    force_dry_run=False

    def __init__(self):
        super().__init__('dh_mission')
        p=lambda key,value:self.declare_parameter(key,value).value
        self.planner=p('planner','clothoid'); self.speed=float(p('speed',.5))
        requested_dry=bool(p('dry_run',True)); self.dry_run=self.force_dry_run or requested_dry
        backend=p('backend','skybeta')
        self.autostart=bool(p('autostart_on_offboard',False))
        self.options=dict(res=float(p('voxel_resolution',.2)),size=float(p('map_size',16.)),
                          radius=float(p('robot_radius',.27)),half_height=float(p('robot_half_height',.12)),margin=float(p('safety_margin',.05)))
        legal_min=float(p('legal_min_altitude',0.)); legal_max=float(p('legal_max_altitude',0.))
        legal_verified=bool(p('legal_altitude_verified',False))
        self.options['legal_band']=(legal_min,legal_max) if legal_verified else None
        floor_z=float(p('floor_z',0.))
        self.options['floor_z']=floor_z if bool(p('floor_height_verified',False)) else None
        self.required_passages=tuple(kind for kind in ('tunnel','window') if bool(p('require_'+kind,True)))
        self.expected_tilt=float(p('expected_camera_tilt_deg',0.))
        self.tilt_tolerance=float(p('mount_tolerance_deg',3.))
        self.geometry_verified=bool(p('depth_geometry_verified',False))
        self.publish_map=bool(p('publish_map',False))
        self.depth_std=float(p('depth_std',.03)); self.approach_height=float(p('pad_approach_height',.65))
        self.pose_std_floor=float(p('pose_std_floor',.03))
        self.base_frame=p('base_frame','base_link'); self.odom_frame=p('odom_frame','odom')
        self.nav=OnlineNavigator(self.planner,self.options,self.speed,self.required_passages); self.history=RigidHistory(); self.pad=PadTrack()
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.depths=deque(maxlen=16); self.colors=deque(maxlen=16); self.depth_info=None; self.color_info=None
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='dh_plan'); self.pending=None
        self.last_tick=self.now(); self.rejected={}; self.calibration_state='NOT_VERIFIED'; self.pad_state='NO_COLOR_OR_PAD'
        if self.dry_run: self.backend=DryRunBackend(self)
        elif backend=='skybeta': self.backend=SkybetaBackend(self)
        else: raise ValueError('Online physical mode supports organizer Sky Beta only, never raw FMU')
        self.create_subscription(Odometry,p('odom_topic','/visual_slam/tracking/odometry'),self.on_odom,SENSOR)
        self.create_subscription(Image,p('depth_topic','/oak/stereo/image_raw'),self.depths.append,SENSOR)
        self.create_subscription(CameraInfo,p('depth_info_topic','/oak/stereo/camera_info'),lambda m:setattr(self,'depth_info',m),SENSOR)
        self.create_subscription(Image,p('color_topic','/oak/rgb/image_raw'),self.colors.append,SENSOR)
        self.create_subscription(CameraInfo,p('color_info_topic','/oak/rgb/camera_info'),lambda m:setattr(self,'color_info',m),SENSOR)
        self.create_subscription(Empty,'/dh/start',self.start,10)
        self.create_subscription(Empty,'/dh/reset_online',self.reset,10)
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.create_subscription(VisualSlamStatus,p('vo_status_topic','/visual_slam/status'),
                lambda m:self.nav.tracking(stamp(m),int(m.vo_state),self.now()),SENSOR)
        except ImportError:
            self.get_logger().warn('VO status unavailable: required health will HOLD')
        self.pub_status=self.create_publisher(String,'/dh/online_status',10)
        self.pub_plan=self.create_publisher(Path,'/dh/online_plan',1)
        self.pub_pad=self.create_publisher(PoseWithCovarianceStamped,'/dh/landing_pad',5)
        self.pub_bounds=self.create_publisher(PolygonStamped,'/dh/landing_pad_bounds',5)
        self.pub_occupied=self.create_publisher(PointCloud2,'/dh/map/occupied',1)
        self.pub_free=self.create_publisher(PointCloud2,'/dh/map/known_free',1)
        self.pub_metadata=self.create_publisher(String,'/dh/map/metadata',1)
        self.create_timer(.2,self.process_depth); self.create_timer(.05,self.tick); self.create_timer(1.,self.map_products)
        verify_isolation(self,self.dry_run)

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def reject(self,reason): self.rejected[reason]=self.rejected.get(reason,0)+1

    def on_odom(self,msg):
        if msg.header.frame_id!=self.odom_frame or msg.child_frame_id!=self.base_frame: return self.reject('ODOM_FRAME')
        q=msg.pose.pose.orientation; xyz=msg.pose.pose.position; quat=np.array([q.x,q.y,q.z,q.w])
        diag=np.asarray(msg.pose.covariance).reshape(6,6).diagonal()
        if (not np.isfinite(diag).all() or (diag<0).any() or diag[3:].max()>.2**2
                or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>.01): return self.reject('ODOM_INVALID')
        t=stamp(msg); old=self.nav.epoch
        if self.nav.odometry([xyz.x,xyz.y,xyz.z],math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),t,self.now(),max(self.pose_std_floor,float(np.sqrt(diag[:3].max()))),quat):
            if self.nav.epoch!=old: self.pad=PadTrack(); self.history=RigidHistory(); self.depths.clear(); self.colors.clear()
            self.history.add(t,[xyz.x,xyz.y,xyz.z],quat)

    def transform(self,frame,t):
        tf=self.tf.lookup_transform(self.base_frame,frame,Time(nanoseconds=int(t*1e9)))
        q=tf.transform.rotation; v=tf.transform.translation
        if abs(np.linalg.norm([q.x,q.y,q.z,q.w])-1)>.01 or not np.isfinite([q.x,q.y,q.z,q.w,v.x,v.y,v.z]).all(): raise ValueError('Invalid measured TF')
        R=quat_to_rot(q.x,q.y,q.z,q.w); offset=np.array([v.x,v.y,v.z])
        if not validate_mount(R,self.expected_tilt,self.tilt_tolerance): raise ValueError('Measured TF does not verify intended tilt')
        return R,offset

    @staticmethod
    def intrinsics(info,msg):
        if info is None or (info.width,info.height)!=(msg.width,msg.height) or info.header.frame_id!=msg.header.frame_id: raise ValueError('CameraInfo/frame/raster mismatch')
        K=np.asarray(info.k,float).reshape(3,3)
        if not np.isfinite(K).all() or min(K[0,0],K[1,1])<=0 or any(abs(d)>1e-8 for d in info.d): raise ValueError('Require valid rectified calibrated image')
        return K

    def process_depth(self):
        now=self.now()
        if not self.geometry_verified: self.calibration_state='DEPTH_GEOMETRY_UNVERIFIED'; return
        choices=[m for m in self.depths if 0<=now-stamp(m)<=.5 and self.history.at(stamp(m)) is not None]
        if not choices: return
        msg=max(choices,key=stamp); t=stamp(msg)
        self.depths=deque((m for m in self.depths if stamp(m)>t),maxlen=16)
        try:
            depth=decode_depth(msg); K=self.intrinsics(self.depth_info,msg); Rbo,tbo=self.transform(msg.header.frame_id,t)
            pos,Rwb=self.history.at(t); Rwo=Rwb@Rbo; origin=pos+Rwb@tbo
            dirs,ranges=camera_rays(depth,(K[0,0],K[1,1],K[0,2],K[1,2]),stride=8)
            if not self.nav.map.observe(origin,dirs@Rwo.T,ranges,t,now): return self.reject('INVALID_OR_REPLAYED_DEPTH')
            self.calibration_state='TF_TILT_AND_DECLARED_GEOMETRY_VERIFIED'
            try: self.process_pad(t,depth,K,Rwo,origin,now)
            except Exception as e: self.pad_state='PAD_REJECTED:'+type(e).__name__; self.reject('PAD:'+str(e))
        except Exception as e:
            self.calibration_state='REJECTED:'+type(e).__name__; self.reject(str(e))

    def process_pad(self,t,depth,Kd,Rwd,twd,now):
        choices=[m for m in self.colors if abs(stamp(m)-t)<=.005 and self.history.at(stamp(m)) is not None]
        if not choices: self.pad_state='NO_SYNCHRONIZED_COLOR'; return
        msg=min(choices,key=lambda m:abs(stamp(m)-t)); image=decode_color(msg); Kc=self.intrinsics(self.color_info,msg)
        # Bound registration cost; scale color intrinsics with the resized raster.
        scale=min(1.,320./image.shape[1]); shape=(int(round(image.shape[1]*scale)),int(round(image.shape[0]*scale)))
        image=cv2.resize(image,shape,interpolation=cv2.INTER_NEAREST); Kc=Kc.copy(); Kc[:2,:]*=scale
        pos,Rwb=self.history.at(stamp(msg)); Rbo,tbo=self.transform(msg.header.frame_id,stamp(msg))
        Rwc=Rwb@Rbo; twc=pos+Rwb@tbo
        registered=register_depth_to_color(depth,Kd,Kc,Rwc.T@Rwd,Rwc.T@(twd-twc),image.shape[:2])
        obs=detect_blue_pad(image,registered,Kc,Rwc,twc,stamp(msg),registered=True,depth_std=self.depth_std)
        if obs is not None and self.nav.map.floor_z is not None and abs(obs.centre[2]-self.nav.map.floor_z)>.1:
            self.pad_state='PAD_NOT_ON_VERIFIED_FLOOR'; return
        if not self.pad.update(obs,now): self.pad_state='PAD_NOT_CONFIRMED'; return
        estimate=self.pad.estimate(now,self.nav.pos_std)
        if estimate is None: self.pad_state='PAD_CONFIRMING'; return
        self.pad_state='PAD_TRACKED'
        self.nav.set_goal(GoalObservation(estimate.centre+np.array([0,0,self.approach_height]),estimate.stamp,estimate.confidence,'blue_pad'),now)
        pose=PoseWithCovarianceStamped(); pose.header=Header(stamp=msg.header.stamp,frame_id=self.odom_frame)
        pose.pose.pose.position.x,pose.pose.pose.position.y,pose.pose.pose.position.z=map(float,estimate.centre)
        pose.pose.pose.orientation.w=1.
        for i in (0,7,14): pose.pose.covariance[i]=estimate.std**2
        self.pub_pad.publish(pose)
        bounds=PolygonStamped(); bounds.header=pose.header
        bounds.polygon.points=[Point32(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in estimate.corners]
        self.pub_bounds.publish(bounds)

    def start(self,_):
        if self.nav.aborted: return
        self.nav.active=True
        if self.dry_run: self.backend.grant_authority()

    def reset(self,_):
        if not self.dry_run and (not self.backend.state_fresh() or self.backend.armed() is not False):
            return self.reject('RESET_REQUIRES_FRESH_DISARMED_STATE')
        epoch=self.nav.epoch+1
        self.nav=OnlineNavigator(self.planner,self.options,self.speed,self.required_passages); self.nav.epoch=epoch
        self.history=RigidHistory(); self.pad=PadTrack(); self.hold_requested=False
        self.depths.clear(); self.colors.clear()
        if self.dry_run: self.backend.revoke_authority()

    def tick(self):
        now=self.now(); dt=now-self.last_tick; self.last_tick=now
        if not self.dry_run and not self.nav.aborted:
            ready=self.nav.health_ok(now)[0] and self.nav.position is not None
            if ready: ready=bool(self.nav.map.snapshot(now,self.nav.pos_std).points_safe([self.nav.position])[0])
            if ready and not self.nav.had_authority and not self.backend.offboard_active():
                self.backend.send_velocity(0.,0.,0.,0.)  # Fresh readiness TWIST, never a floored pose.
            if ready and self.autostart and self.backend.offboard_active(): self.nav.active=True
        if self.pending is not None and self.pending.done():
            try:
                if self.nav.accept(self.pending.result(),now): self.publish_plan()
            except Exception as e: self.reject('PLANNER:'+type(e).__name__)
            self.pending=None
        if self.pending is None:
            request=self.nav.request(now)
            if request is not None: self.pending=self.executor.submit(OnlineNavigator.compute,request)
        velocity,yaw_rate=self.nav.tick(now,dt,self.backend.offboard_active())
        if self.nav.passages.done and self.nav.goal is not None and self.nav.position is not None and np.linalg.norm(self.nav.position-self.nav.goal.position)<.15:
            estimate=self.pad.estimate(now,self.nav.pos_std)
            clear=False
            if estimate is not None:
                down=self.nav.position.copy(); down[2]=estimate.centre[2]+.4
                clear=self.nav.map.snapshot(now,self.nav.pos_std).segment_safe(self.nav.position,down)
            self.pad_state=landing_readiness(self.pad,self.nav.position,now,clear,False,self.options['radius'],self.options['margin'],self.nav.pos_std)
            velocity[:]=0.; yaw_rate=0.  # No verified landing capability: never invent descent.
        if self.nav.active and not self.nav.aborted:
            self.backend.send_velocity(*map(float,velocity),float(yaw_rate))
        elif self.nav.aborted and self.nav.had_authority and self.backend.offboard_active() and not getattr(self,'hold_requested',False):
            self.backend.hold(); self.hold_requested=True
        self.pub_status.publish(String(data=json.dumps(dict(state=self.nav.state,planner=self.planner,
            calibration=self.calibration_state,pad=self.pad_state,landing_authority='UNVERIFIED_NO_COMMAND',
            required_passages=sorted(self.nav.passages.required),completed_passages=sorted(self.nav.passages.completed),
            replans=self.nav.replans,map_bytes=self.nav.map.memory_bytes,rejected=self.rejected))))

    def publish_plan(self):
        message=Path(); message.header=Header(frame_id=self.odom_frame); message.header.stamp=self.get_clock().now().to_msg()
        for p in self.nav.follower.traj.p:
            pose=PoseStamped(); pose.header=message.header
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,p); pose.pose.orientation.w=1.
            message.poses.append(pose)
        self.pub_plan.publish(message)

    def map_products(self):
        if not self.publish_map: return
        view=self.nav.map.snapshot(self.now(),self.nav.pos_std)
        header=Header(frame_id=self.odom_frame); header.stamp=self.get_clock().now().to_msg()
        for state,publisher in ((0,self.pub_free),(1,self.pub_occupied)):
            cells=np.argwhere(view.states==state); points=(view.lo+(cells+.5)*view.res).astype('<f4')
            message=PointCloud2(header=header,height=1,width=len(points),is_bigendian=False,is_dense=True,
                fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))],
                point_step=12,row_step=len(points)*12,data=points.tobytes())
            publisher.publish(message)
        self.pub_metadata.publish(String(data=json.dumps(dict(frame=self.odom_frame,stamp=self.now(),revision=view.revision,
            origin=view.lo.tolist(),shape=list(view.safe.shape),resolution=view.res,axis_order='xyz',unknown='all nonobserved voxels; never infer free from occupied cloud alone'))))


class OnlineStationaryNode(OnlineMissionNode):
    """Hard lock: even false/PX4 parameter overrides cannot construct a physical backend."""
    force_dry_run=True


def stationary_main(): main(OnlineStationaryNode)


def main(node_class=OnlineMissionNode):
    rclpy.init(); node=node_class()
    try: rclpy.spin(node)
    finally:
        node.executor.shutdown(wait=False,cancel_futures=True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

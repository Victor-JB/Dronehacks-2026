"""Executable reuse-first local policy. One existing team MapNode, no CAD.

Requires the team's dronehacks_nav package. No camera/SLAM/flight_interface is
started. Disable a separately running map_node: this node IS that mapper with
a small planning/control adapter. Default dry; stationary entrypoint hard locked.
"""
import math
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data as SENSOR
from rclpy.time import Time
from std_msgs.msg import Empty,String
from sensor_msgs.msg import Image
from dronehacks_nav.map_node import MapNode
from dronehacks_nav.mapping import VoxelMap
from refnav import depth as D
from dh_core.live_map_adapter import LocalPolicy,from_team_map,clearance_at_current_pose,InitialVolume,swept_cylinder_clearance
from dh_core.online_geometry import RigidHistory,validate_mount
from dh_core.camera_geom import quat_to_rot
from dh_core.health import LocalizationHealth,LocHealthParams,stamp_to_sec
from dh_core.safety_filter import SensorInputs
from .backends import DryRunBackend,SkybetaBackend,verify_isolation


class TimedTeamMap(VoxelMap):
    """Keep existing integration, add observation ages and immediate hit veto."""
    def __init__(self,source):
        self.__dict__.update(source.__dict__)
        self.observed_at=np.full(self.shape,-np.inf,np.float64); self.stamp=None

    def integrate(self,origin,dirs,ranges,max_range_m=4.):
        if self.stamp is None: return
        r=np.asarray(ranges); d=np.asarray(dirs)
        good=np.isfinite(r)&(r>0)&np.isfinite(d).all(axis=1)
        d,r=d[good],r[good]
        if not len(r): return
        ts=np.arange(self.voxel/2,max_range_m,self.voxel/2)
        pts=origin+d[:,None,:]*ts[None,:,None]
        pts=pts[ts[None,:]<np.minimum(r,max_range_m)[:,None]-self.voxel]
        hit=self._index(origin+d[r<=max_range_m]*r[r<=max_range_m,None])
        touched=np.r_[self._index(pts),hit]
        super().integrate(origin,d,r,max_range_m)
        self.grid.reshape(-1)[hit]=np.maximum(self.grid.reshape(-1)[hit],self.l_occ+.001)
        self.observed_at.reshape(-1)[touched]=self.stamp


class LocalPolicyNode(MapNode):
    force_dry=False

    def __init__(self):
        super().__init__()
        p=lambda k,v:self.declare_parameter(k,v).value
        self.map=TimedTeamMap(self.map)
        self.policy=LocalPolicy(); self.history=RigidHistory()
        self.health=LocalizationHealth(LocHealthParams(require_vo_status=True))
        self.geometry_verified=bool(p('depth_geometry_verified',False))
        self.rules_verified=bool(p('below_wall_height_verified',False))
        self.legal=(float(p('legal_min_altitude',0.)),float(p('legal_max_altitude',0.)))
        self.cruise=float(p('cruise_z',.85)); self.radius=float(p('robot_radius',.27))
        self.half_height=float(p('robot_half_height',.12)); self.margin=float(p('safety_margin',.10))
        self.vertical_margin=float(p('vertical_clearance_margin',self.margin))
        self.policy.ev.prm.vehicle_radius=self.radius; self.policy.ev.prm.uncertainty_margin=self.margin
        self.policy.ev.prm.latency_s=float(p('sensor_to_actuation_latency_s',.8))
        self.policy.ev.prm.brake_decel=float(p('guaranteed_braking_decel',.6))
        self.pose_std_floor=float(p('pose_std_floor',.03))
        self.map_free_age=float(p('static_free_max_age_s',.5))
        self.map_decay=float(p('map_confidence_decay_s',30.))
        self.map_min_free=float(p('minimum_free_logodds',0.))
        self.verified_placement=bool(p('initial_placement_verified',False))
        self.placement_radius=float(p('initial_clearance_radius',0.))
        self.placement_half_height=float(p('initial_clearance_half_height',0.))
        self.initial_volume=None
        self.active_perception=bool(p('active_perception_enabled',False))
        self.policy.allow_turn=self.active_perception
        self.dense_free_evidence=bool(p('dense_free_evidence_enabled',False))
        self.scan_remaining=0.;self.scan_last_yaw=None
        self.blocked_since=None
        from dh_core.landing_pad import PadTrack
        self.pad_track=PadTrack();self.rgb_frames={};self.pad_detections=0
        self.rgb_registered=bool(p('rgb_depth_registration_verified',False))
        self.create_subscription(Image,p('registered_rgb_topic','/oak/rgb/image_raw'),self.on_rgb,SENSOR)
        requested_dry=bool(p('dry_run',True)); self.dry_run=self.force_dry or requested_dry
        reviewed=bool(p('physical_configuration_reviewed',False))
        if not self.dry_run and not reviewed:
            raise ValueError('Physical policy requires a separately reviewed configuration')
        self.backend=DryRunBackend(self) if self.dry_run else SkybetaBackend(self)
        self.armed_start=False; self.pose=None; self.pos_std=.03
        self.odom_valid=False
        self.last_capture=None; self.tf_ok=False; self.current_msg=None; self.info=None
        self.clearance_points=None; self.valid_fraction=0.
        self.clearance_origin=None; self.capture_yaw=None
        self.startup_ready=False; self.startup_reason='WAIT_FRESH_SENSORS'
        self.prev_tick=self.now(); self.hold_requested=False; self.last_plan=-math.inf
        self.pub_policy=self.create_publisher(String,'/dh/local_policy/status',10)
        self.create_subscription(Empty,'/dh/start',self.on_start,10)
        try:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            self.create_subscription(VisualSlamStatus,p('vo_status_topic','/visual_slam/status'),self.on_vo,SENSOR)
        except ImportError: self.get_logger().warning('Missing VO status interface: policy will HOLD')
        self.create_timer(.05,self.control_tick)
        verify_isolation(self,self.dry_run)

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def on_vo(self,m):
        t=stamp_to_sec(m.header.stamp)
        if 0<=self.now()-t<=1. and (self.health.vo_rx is None or t>self.health.vo_rx):
            self.health.on_vo_status(t,int(m.vo_state))

    def on_rgb(self,m):
        if not self.rgb_registered:return
        self.rgb_frames[stamp_to_sec(m.header.stamp)]=m
        if len(self.rgb_frames)>12:del self.rgb_frames[min(self.rgb_frames)]

    def _on_info(self,m):
        self.info=m
        super()._on_info(m)

    def _on_odom(self,m):
        if not hasattr(self,'health'): return
        self.odom_valid=False
        t=stamp_to_sec(m.header.stamp); now=self.now(); q=m.pose.pose.orientation; xyz=m.pose.pose.position
        quat=np.array([q.x,q.y,q.z,q.w]); cov=np.array(m.pose.covariance).reshape(6,6).diagonal()
        if (m.header.frame_id!=self.frame or m.child_frame_id!=self.base_frame
                or not np.isfinite(np.r_[quat,cov,xyz.x,xyz.y,xyz.z]).all() or (cov<0).any()
                or abs(np.linalg.norm(quat)-1)>.01 or cov[3:].max()>.04 or not 0<=now-t<=.4
                or (self.health.last_odom_t is not None and t<=self.health.last_odom_t)): return
        position=np.array([xyz.x,xyz.y,xyz.z]); yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pos_std=max(self.pose_std_floor,float(np.sqrt(cov[:3].max())))
        self.health.on_odom(now,t,position,self.pos_std,yaw=yaw)
        if self.history.samples:
            previous=self.history.samples[-1]
            angle=2*math.acos(float(np.clip(abs(quat@previous[2]),0.,1.)))
            if angle>1.5*(t-previous[0])+.35: self.policy.aborted=True
        if self.health.jump_since is not None:
            self.policy.aborted=True; self.policy.abort_reason='POSE_JUMP'
            self.map.grid.fill(0); self.map.observed_at.fill(-np.inf)
        self.history.add(t,position,quat); self.pose=(position,yaw)
        if (self.initial_volume is None and self.verified_placement and self.placement_radius>0
                and self.placement_half_height>0 and abs(position[2]-self.cruise)<.03):
            self.initial_volume=InitialVolume(position.copy(),self.placement_radius,self.placement_half_height,t)
        self.odom_valid=True
        super()._on_odom(m)

    def _pose_at(self,t):
        sample=self.history.at(t)
        return None if sample is None else (t,*sample)

    def _next_depth(self):
        m=super()._next_depth()
        self.tf_ok=False
        if m is None or not self.geometry_verified: return None
        t=stamp_to_sec(m.header.stamp)
        if (not 0<=self.now()-t<=.3 or (self.last_capture is not None and t<=self.last_capture)
                or m.is_bigendian or m.encoding not in ('16UC1','32FC1')
                or self.info is None or (m.width,m.height)!=(self.info.width,self.info.height)
                or not np.isfinite(list(self.info.k)+list(self.info.d)).all()
                or m.header.frame_id!=self.info.header.frame_id or any(abs(x)>1e-8 for x in self.info.d)):
            return None
        self.current_msg=m; self.map.stamp=t
        return m

    def _calibration(self,frame_id):
        try:
            t=stamp_to_sec(self.current_msg.header.stamp)
            tf=self.tf_buffer.lookup_transform(self.base_frame,frame_id,Time(nanoseconds=int(t*1e9)))
            q=tf.transform.rotation; v=tf.transform.translation
            if not np.isfinite([q.x,q.y,q.z,q.w,v.x,v.y,v.z]).all() or abs(np.linalg.norm([q.x,q.y,q.z,q.w])-1)>.01: return None
            R=quat_to_rot(q.x,q.y,q.z,q.w)
            if not validate_mount(R,expected_deg=0.,tolerance_deg=3.) or R[0,2]<math.cos(math.radians(3.)): return None
            self.tf_ok=True
            self.extrinsic=(frame_id,R,np.array([v.x,v.y,v.z]))
            return frame_id,R,np.array([v.x,v.y,v.z])
        except Exception: return None

    def _integrate(self):
        if not hasattr(self,'policy'): return
        before=self.map.frames
        super()._integrate()
        if self.map.frames==before:
            self.tf_ok=False; self.clearance_points=None; self.valid_fraction=0.; return
        self.last_capture=self.map.stamp
        try:
            m=self.current_msg; depth=D.decode_depth(m.encoding,m.height,m.width,m.step,bytes(m.data))
            # Existing refnav projection/clearance, not another stereo pipeline.
            _,_,Rwb=self._pose_at(self.last_capture)
            yaw=math.atan2(Rwb[1,0],Rwb[0,0]); c,s=math.cos(yaw),math.sin(yaw)
            level=np.diag([1.,-1.,-1.])@np.array([[c,s,0],[-s,c,0],[0,0,1]])@Rwb
            _,R,t=self.extrinsic
            self.clearance_points=D.level_points(depth,self.K,R,t,level,stride=4)
            _,capture_position,_=self._pose_at(self.last_capture)
            if self.dense_free_evidence:
                from dh_core.depth_evidence import carve_visible_band
                hh=self.half_height+self.vertical_margin+3*self.pos_std
                carve_visible_band(self.map,self.map.observed_at,depth,self.K,Rwb@R,
                    capture_position+Rwb@t,self.last_capture,self.cruise-hh,self.cruise+hh,
                    max_range=min(2.5,self.max_range),error=.03+3*self.pos_std)
            self.clearance_origin=capture_position.copy(); self.capture_yaw=yaw
            color=self.rgb_frames.pop(self.last_capture,None)
            if (color is not None and color.header.frame_id==m.header.frame_id
                    and color.encoding in ('rgb8','bgr8') and not color.is_bigendian
                    and (color.width,color.height)==(m.width,m.height)
                    and color.step>=color.width*3 and len(color.data)==color.step*color.height):
                from dh_core.landing_pad import detect_blue_pad
                bgr=np.frombuffer(bytes(color.data),np.uint8).reshape(color.height,color.step)[:,:color.width*3].reshape(color.height,color.width,3)
                if color.encoding=='rgb8':bgr=bgr[:,:,::-1].copy()
                fx,fy,cx,cy=self.K
                obs=detect_blue_pad(bgr,depth,np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]]),
                    Rwb@R,capture_position+Rwb@t,self.last_capture,registered=True,depth_std=.03)
                if self.pad_track.update(obs,self.now()):self.pad_detections+=1
            # Conservative central ROI validity, independent of occupied returns.
            cy,cx=depth.shape[0]//2,depth.shape[1]//2
            roi=depth[max(0,cy-depth.shape[0]//4):cy+depth.shape[0]//4,
                      max(0,cx-depth.shape[1]//4):cx+depth.shape[1]//4]
            self.valid_fraction=float((np.isfinite(roi)&(roi>.1)&(roi<8.)).mean()) if roi.size else 0.
        except Exception:
            self.clearance_points=None; self.valid_fraction=0.; self.tf_ok=False
        try:
            view=from_team_map(self.map,self.map.observed_at,self.last_capture,self.now(),self.cruise,
                legal_band=self.legal,height_below_walls_verified=self.rules_verified,radius=self.radius,
                half_height=self.half_height,margin=self.margin,pos_std=self.pos_std,revision=self.map.frames,
                free_max_age_s=self.map_free_age,decay_time_s=self.map_decay,min_free_logodds=self.map_min_free,
                initial_volume=self.initial_volume,vertical_margin=self.vertical_margin)
            self.policy.update_map(view)
        except ValueError as e:
            self.policy.view=None; self.get_logger().warning(str(e),throttle_duration_sec=2.)

    def on_start(self,_):
        if self.armed_start:return
        if not self.startup_ready or self.policy.aborted:
            self.get_logger().warning('Start refused: '+self.startup_reason)
            return
        self.armed_start=True
        if self.active_perception:
            self.scan_remaining=2*math.pi;self.scan_last_yaw=self.pose[1]
        if self.dry_run: self.backend.grant_authority()

    def control_tick(self):
        now=self.now(); dt=now-self.prev_tick; self.prev_tick=now
        mode,_,reason=self.health.evaluate(now)
        if not self.odom_valid and mode!='ABORT': mode,reason='HOVER','invalid or missing latest odometry'
        if mode=='ABORT': self.policy.aborted=True; self.policy.abort_reason='LOCALIZATION'
        rng,rs,rr=None,None,None
        if (self.clearance_points is not None and self.pose is not None
                and self.clearance_origin is not None and self.capture_yaw is not None):
            bearing=0.
            if self.policy.gate is not None:
                reference=self.policy.gate.follower.traj.sample(self.policy.gate.follower.tau)[1]
                if np.linalg.norm(reference[:2])>.01:
                    bearing=math.atan2(reference[1],reference[0])-self.pose[1]
            capture_bearing=(bearing+self.pose[1]-self.capture_yaw+math.pi)%(2*math.pi)-math.pi
            points=clearance_at_current_pose(self.clearance_points,self.clearance_origin,self.capture_yaw,*self.pose)
            if D.in_view(capture_bearing,self.K,self.current_msg.width):
                rng=swept_cylinder_clearance(points,-bearing,
                    self.radius+self.margin+3*self.pos_std,
                    self.half_height+self.vertical_margin+3*self.pos_std)
                rs=rr=self.last_capture
            else: rng=None
        inp=SensorInputs(now,free_distance=rng,depth_stamp=rs,depth_rx=rr,
            odom_stamp=self.health.last_odom_t,odom_rx=self.health.t_odom_rx,pos_std=self.pos_std,
            transform_valid=self.tf_ok and self.last_capture is not None and 0<=now-self.last_capture<=.3,loc_mode=mode)
        inp.depth_valid_fraction=self.valid_fraction
        active=self.backend.offboard_active()
        pad=self.pad_track.estimate(now,self.pos_std)
        self.policy.goal_hint=None if pad is None else np.r_[pad.centre[:2],self.cruise]
        if self.pose is not None:
            position,yaw=self.pose
            ownership_ok=True
            if not self.dry_run:
                ownership_ok=self.get_node_names().count('flight_interface')==1
                for topic in ('/skybeta/setpoint/twist','/skybeta/setpoint/pose'):
                    publishers=self.get_publishers_info_by_topic(topic)
                    ownership_ok &= len(publishers)==1 and all(
                        info.node_name==self.get_name() and info.node_namespace==self.get_namespace() for info in publishers)
            if not ownership_ok:
                if self.policy.had_authority:
                    self.policy.aborted=True; self.policy.abort_reason='COMMAND_OWNERSHIP'
                self.startup_ready=False; self.startup_reason='WAIT_EXCLUSIVE_POLICY_AND_ONE_FLIGHT_INTERFACE'
                self.pub_policy.publish(String(data='{"startup_ready":false,"startup_reason":"WAIT_EXCLUSIVE_POLICY_AND_ONE_FLIGHT_INTERFACE"}'))
                return
            self.startup_ready,self.startup_reason=self.startup_check(position,inp)
            if (self.armed_start and not self.policy.aborted and self.scan_remaining<=0
                    and (self.policy.gate is None or self.policy.gate.follower.done) and now-self.last_plan>=1.):
                self.last_plan=now; self.policy.plan(position,yaw,now)
                if self.active_perception and self.policy.gate is None:
                    self.scan_remaining=math.pi/2;self.scan_last_yaw=yaw
            heading_error=0.
            if self.policy.gate is not None:
                follower=self.policy.gate.follower
                remaining=follower.traj.p[follower.traj.nearest_index(position):]-position
                ahead=np.flatnonzero(np.linalg.norm(remaining[:,:2],axis=1)>=.25)
                look=remaining[ahead[0]] if len(ahead) else remaining[-1]
                if np.linalg.norm(look[:2])>.03:
                    heading_error=(math.atan2(look[1],look[0])-yaw+math.pi)%(2*math.pi)-math.pi
            scanning=self.scan_remaining>0
            align=self.active_perception and self.policy.gate is not None and abs(heading_error)>.22
            if (scanning or align) and self.armed_start and active and not self.policy.aborted:
                if scanning:
                    delta=(yaw-self.scan_last_yaw+math.pi)%(2*math.pi)-math.pi
                    self.scan_remaining-=delta;self.scan_last_yaw=yaw
                rotation_safe=(self.policy.view is not None and 0<=now-self.policy.view.stamp<=.5
                    and self.policy.view.safe([position])[0] and mode in ('OK','DEGRADED')
                    and inp.transform_valid and self.valid_fraction>=.5
                    and self.last_capture is not None and 0<=now-self.last_capture<=.3)
                v=np.zeros(3);yr=(.35 if scanning else float(np.clip(heading_error,-.35,.35))) if rotation_safe else 0.
                self.policy.state=('OBSERVING_YAW' if scanning else 'OBSERVING_ROUTE_HEADING') if rotation_safe else 'HOLD_SCAN_EVIDENCE'
                self.policy.had_authority=True
            else:
                v,yr=self.policy.step(position,yaw,dt,inp,authority=active and self.armed_start)
                # A safety stop must not pin a permanently unusable reference.
                # Discard it, observe while stationary, then ask the SAME planner again.
                if (self.active_perception and active and self.armed_start and not self.policy.aborted
                        and self.policy.state.startswith('HOLD_') and np.linalg.norm(v)<.005):
                    if self.blocked_since is None:self.blocked_since=now
                    if now-self.blocked_since>=1.:
                        if self.policy.gate is not None:
                            self.policy.recent=(self.policy.recent+[self.policy.gate.follower.traj.p[-1,:2].copy()])[-8:]
                        self.policy.gate=None;self.scan_remaining=math.pi/2;self.scan_last_yaw=yaw
                        self.blocked_since=None
                else:self.blocked_since=None
            ready=(self.policy.view is not None and self.policy.view.safe([position])[0]
                   and self.policy.ev.evaluate(inp,.3).allows_motion)
            if (not self.dry_run and not self.policy.had_authority and self.policy.view is not None
                    and ready):
                self.backend.send_velocity(0.,0.,0.,0.)
            if self.armed_start and not self.policy.aborted and (active or ready): self.backend.send_velocity(*map(float,v),float(yr))
            elif self.policy.aborted and active and not self.hold_requested:
                self.backend.hold(); self.hold_requested=True
        import json
        self.pub_policy.publish(String(data=json.dumps(dict(state=self.policy.state,health=mode,reason=reason,
            startup_ready=self.startup_ready,startup_reason=self.startup_reason,
            replans=self.policy.replans,dry_run=self.dry_run,cad_required=False))))

    def startup_check(self,position,inputs):
        """Passive evidence gate. Never seed free voxels or move to collect data."""
        if not self.geometry_verified: return False,'WAIT_VERIFIED_CAMERA_GEOMETRY'
        if not self.rules_verified: return False,'WAIT_VERIFIED_BELOW_WALL_ALTITUDE'
        if abs(position[2]-self.cruise)>=.03: return False,'WAIT_REVIEWED_CRUISE_HEIGHT'
        view=self.policy.view
        if view is None or not 0<=inputs.now-view.stamp<=.5: return False,'WAIT_FRESH_MAP'
        if not view.safe([position])[0]: return False,'WAIT_MEASURED_FULL_START_VOLUME'
        if not self.policy.ev.evaluate(inputs,.3).allows_motion: return False,'WAIT_FRESH_CLEARANCE_AND_LOCALIZATION'
        return True,'READY_FOR_START'


class StationaryLocalPolicyNode(LocalPolicyNode): force_dry=True


def stationary_main(): main(StationaryLocalPolicyNode)


def main(node_class=LocalPolicyNode):
    rclpy.init(); node=node_class()
    try: rclpy.spin(node)
    finally:
        if not node.dry_run and node.policy.had_authority and node.backend.offboard_active(): node.backend.hold()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

#!/usr/bin/env python3
"""box_route.live_view - the drone's live view on a plain web page, served by the Jetson itself.

Ported from dronehacks main (dronehacks_nav/camera_web.py) for the box_route mission. View only:
it has no buttons and publishes nothing, so it cannot change what the drone does.

Drawn on the page (top-down, odometry frame):
  green line    the track: where VIO (odometry) says the drone has been
  yellow arrow  where VIO says the drone is now, and its heading
  blue dashed   the route the mission wants to fly (numbered waypoints). Before Offboard it is a
                preview from where the drone is standing now; once the mission starts it is the
                real one, anchored where Offboard started
  orange ring   the waypoint it is going to now
  pink ring     the setpoint it is sending right now (the "carrot" sliding along the leg)
plus the mission phase, PX4 armed / Offboard, and the left camera.

    http://skybeta-02.local:8080/            everything on one page
    http://skybeta-02.local:8080/state.json  odometry, track, mission status, skybeta state
    http://skybeta-02.local:8080/camera.mjpg
    http://skybeta-02.local:8080/depth.mjpg  only with enable_depth:=true
    http://skybeta-02.local:8080/map.mjpg    only with enable_depth:=true (and dronehacks_nav map_node)

Depth is OFF by default: subscribing to the 30 Hz depth images costs the Jetson time next to
cuVSLAM. fly.sh start --depth turns it (and the map) on.

Frames are re-encoded at most max_fps per stream, and only while someone is watching.

Run:  ros2 run box_route_v2 live_view --ros-args -p port:=8080
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

FIG_CAMERA = """<figure><img src="/camera.mjpg"><figcaption>camera (left)</figcaption><div class="s" id="camera">-</div></figure>"""
FIG_DEPTH = """<figure><img src="/depth.mjpg"><figcaption>depth (black = no return; + = centre)</figcaption><div class="s" id="depth">-</div></figure>
<figure><img src="/map.mjpg"><figcaption>map from above (dronehacks_nav map_node: grey = unknown, light = free, red = occupied)</figcaption><div class="s" id="map">-</div></figure>"""

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>box_route_v2 live view</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif}
div{display:flex;flex-wrap:wrap;gap:8px;padding:8px}figure{margin:0;flex:1 1 400px}
img{width:100%;image-rendering:pixelated;background:#000}
.s{font:16px monospace;color:#8f8;white-space:pre}
#pos{display:flex;flex-wrap:wrap;gap:8px;padding:8px}canvas{background:#000;flex:1 1 420px;max-width:640px;aspect-ratio:1}
#st{flex:1 1 320px;font:15px monospace;white-space:pre;color:#ddd}
#bar{display:flex;flex-wrap:wrap;align-items:center;gap:12px;padding:10px 8px;background:#1b1b1b;position:sticky;top:0;z-index:1}
#banner{flex:1 1 420px;font:bold 22px sans-serif}.bad{color:#f66}.ok{color:#8f8}.warn{color:#fd5}</style></head><body>
<div id="bar"><div id="banner">connecting...</div><span style="color:#888">view only</span></div>
<div id="pos"><canvas id="c" width="640" height="640"></canvas><div id="st">waiting for /state.json</div></div>
<div>
@FIGURES@
</div><script>
const STREAMS=@STREAMS@;
function f(x){if(!x||!x.fps&&x.fps!==0)return "no messages";
 let t=x.fps.toFixed(1)+" Hz   "+x.width+"x"+x.height+" "+x.encoding;
 if("centre_m" in x)t+="\\ncentre "+(x.centre_m==null?"no return":x.centre_m.toFixed(2)+" m")
  +"   nearest "+(x.min_m==null?"-":x.min_m.toFixed(2)+" m")+"   valid "+(100*x.valid).toFixed(0)+"%";
 return t}
setInterval(async()=>{try{const d=await (await fetch("/stats.json")).json();
 for(const k of STREAMS)document.getElementById(k).textContent=f(d[k]);}catch(e){}},500);
// --- top-down, the odometry frame's +x (the nose at stack start) up, +y left
const cv=document.getElementById("c"),g=cv.getContext("2d"),st=document.getElementById("st");
function fx(v,n){return v==null?"-":v.toFixed(n==null?2:n)}
// Fixed zoom: the view is fitted ONCE to the take-off spot + route (+1 m margin), and only re-fitted
// when a new mission starts. Re-fitting every frame made the drone marker look like it was jumping.
let view=null,viewKey=null;
function draw(d){
 const m=d.mission||{},route=m.route||[];
 const key=m.start?JSON.stringify(m.start):(route.length?"preview":null);
 if(key&&key!==viewKey){
  const pts=[[0,0]].concat(route.map(p=>[p[0],p[1]])); if(d.pos)pts.push([d.pos[0],d.pos[1]]);
  let a=1e9,b=-1e9,c=1e9,e=-1e9;for(const p of pts){a=Math.min(a,p[0]);b=Math.max(b,p[0]);c=Math.min(c,p[1]);e=Math.max(e,p[1]);}
  view={cx:(a+b)/2,cy:(c+e)/2,r:Math.max(1.5,(b-a)/2+1.0,(e-c)/2+1.0)};viewKey=key;}
 if(!view)view={cx:0,cy:0,r:3};
 const cx=view.cx,cy=view.cy,r=view.r;
 const W=cv.width,s=W/2/r,cl=v=>Math.max(4,Math.min(W-4,v)),X=p=>cl(W/2-(p[1]-cy)*s),Y=p=>cl(W/2-(p[0]-cx)*s);   // off-view points pinned to the edge   // +y left -> screen left, +x up
 g.clearRect(0,0,W,W);g.strokeStyle="#222";g.lineWidth=1;
 const step=r>6?2:r>3?1:r>1.5?0.5:0.25;
 for(let v=Math.floor((cy-r)/step)*step;v<=cy+r;v+=step){g.beginPath();g.moveTo(X([0,v]),0);g.lineTo(X([0,v]),W);g.stroke();}
 for(let v=Math.floor((cx-r)/step)*step;v<=cx+r;v+=step){g.beginPath();g.moveTo(0,Y([v,0]));g.lineTo(W,Y([v,0]));g.stroke();}
 g.fillStyle="#777";g.font="12px monospace";g.fillText(step+" m grid   +x (nose at stack start) up, +y left",6,14);
 if(route.length){g.strokeStyle=m.route_preview?"#357":"#4af";g.lineWidth=2;g.setLineDash([6,4]);g.beginPath();
  route.forEach((p,i)=>i?g.lineTo(X(p),Y(p)):g.moveTo(X(p),Y(p)));g.stroke();g.setLineDash([]);
  g.fillStyle=m.route_preview?"#357":"#4af";route.forEach((p,i)=>{if(!i)return;g.beginPath();g.arc(X(p),Y(p),4,0,7);g.fill();
   g.fillStyle=(m.wall||[])[i]?"#f84":(m.route_preview?"#579":"#8cf");g.fillText(`${i} (${fx(p[0])}, ${fx(p[1])})`,X(p)+6,Y(p)-6);g.fillStyle=m.route_preview?"#357":"#4af"});}
 g.strokeStyle="#3c3";g.lineWidth=2;g.beginPath();(d.trail||[]).forEach((p,i)=>i?g.lineTo(X(p),Y(p)):g.moveTo(X(p),Y(p)));g.stroke();
 const o=m.start||[0,0];g.strokeStyle="#0f0";g.beginPath();g.moveTo(X(o)-8,Y(o));g.lineTo(X(o)+8,Y(o));g.moveTo(X(o),Y(o)-8);g.lineTo(X(o),Y(o)+8);g.stroke();
 if(m.target){g.strokeStyle="#f93";g.lineWidth=2;g.beginPath();g.arc(X(m.target),Y(m.target),10,0,7);g.stroke();}
 if(m.setpoint){g.strokeStyle="#f4f";g.lineWidth=2;g.beginPath();g.arc(X(m.setpoint),Y(m.setpoint),6,0,7);g.stroke();}
 if(d.pos){const p=d.pos,a=d.yaw||0,L=18;g.fillStyle=d.odom_age>0.5?"#f44":"#fd3";g.save();g.translate(X(p),Y(p));g.rotate(-a);
  g.beginPath();g.moveTo(0,-L);g.lineTo(-8,8);g.lineTo(8,8);g.closePath();g.fill();g.restore();}
}
function text(d){
 const sk=d.skybeta||{},m=d.mission||{},c=(ok,t)=>`<span class="${ok?"ok":"bad"}">${t}</span>`;
 let t=`<b>where it thinks it is</b> (VIO odometry, metres)\\n`;
 t+=d.pos?`x ${fx(d.pos[0])}  y ${fx(d.pos[1])}  z ${fx(d.pos[2])}  yaw ${fx(d.yaw*57.2958,0)} deg\\n`:"no odometry\\n";
 t+=`speed ${fx(d.speed)} m/s   odometry ${fx(d.odom_hz,1)} Hz, age `+c(d.odom_age!=null&&d.odom_age<0.5,fx(d.odom_age)+" s")+"\\n";
 if(m.start&&d.pos)t+=`height above take-off ${fx(d.pos[2]-m.start[2])} m (target ${fx(m.height_m)})\\n`;
 const le=sk.local_enu; if(le)t+=`PX4 z ${fx(le[2])}  (PX4 - VIO ${d.pos?fx(le[2]-d.pos[2]):"-"} m)\\n`;
 t+=`\\n<b>PX4</b>   armed ${c(!!sk.armed,sk.armed?"YES":"no")}   Offboard ${c(!!sk.offboard_active,sk.offboard_active?"YES":"no")}   ${sk.nav_state_name??sk.nav_state??"-"}${sk.clamped&&sk.clamped.length?'   <span class="warn">CLAMPED '+sk.clamped.join(",")+'</span>':""}\\n`;
 t+=`PX4 link ${c(!!sk.px4_link,sk.px4_link?"up":"DOWN")}   landed ${sk.landed??"-"}\\n\\n`;
 t+=`<b>mission</b>   ${(m.phase||"-").toUpperCase()}`+(m.legs?`   waypoint ${m.index}/${m.legs}`:"")+"\\n";
 if(m.height_m!=null)t+=`${m.height_m} m up, scale ${m.scale}, ${m.speed_mps} m/s, wall legs ${m.wall_speed_mps} m/s\\n`;
 if(m.config_error)t+='<span class="bad">route.yaml: '+m.config_error+'</span>\\n';
 if(m.config_pending)t+='<span class="warn">route.yaml changed: applied after disarm</span>\\n';
 if(m.route&&m.body){t+=`<b>waypoints</b> ${m.route_preview?"(preview from here)":""}   # (x, y) odometry  [fwd, left] offset\\n`;
  m.route.forEach((p,i)=>{if(!i)return;const b=m.body[i]||[0,0],cur=m.phase=="waypoints"&&m.index==i;
   t+=(cur?"<b>&gt;":" ")+String(i).padStart(2)+`  (${fx(p[0])}, ${fx(p[1])})  [${fx(b[0])}, ${fx(b[1])}]`+((m.wall||[])[i]?'  <span class="warn">wall</span>':"")+(cur?"</b>":"")+"\\n";});}
 if(m.target&&d.pos)t+=`to waypoint: ${fx(Math.hypot(m.target[0]-d.pos[0],m.target[1]-d.pos[1]))} m across, ${fx(d.pos[2]-m.target[2])} m height\\n`;
 if(m.stale_offboard)t+='<span class="bad">Offboard was already on at arming: mission will NOT start</span>\\n';
 const hl=(m.limits||{}).height_check_m??0.4;if(m.height_err!=null)t+=`VIO vs PX4 height check: `+c(Math.abs(m.height_err)<=hl,fx(m.height_err)+" m")+` (limit ${hl})\\n`;
 if(m.limits)t+=`land if VIO quiet ${m.limits.vio_quiet_s} s / jumps ${m.limits.vio_jump_m} m / off PX4 ${m.limits.vio_px4_xy_m} m side, ${m.limits.vio_px4_z_m} m height / waypoint ${m.limits.waypoint_timeout_s} s\\nturn early: ${m.turn_early?"ON, "+m.limits.turn_ahead_m+" m ahead":"OFF (stop, turn, go)"}\\n`;
 if(m.height_refused)t+='<span class="bad">VIO height disagrees with PX4: mission REFUSED, restart the stack</span>\\n';
 t+=`\\n<span style="color:#3c3">green</span> track   <span style="color:#fd3">yellow</span> drone + heading\\n<span style="color:#4af">blue</span> route (dim = preview from here)   <span style="color:#f93">orange</span> next waypoint   <span style="color:#f4f">pink</span> setpoint`;
 if(d.stale)t='<span class="bad">NO DATA from the drone for '+fx(d.stale,0)+" s</span>\\n\\n"+t;
 st.innerHTML=t;
}
const bn=document.getElementById("banner");
function words(d){const m=d.mission||{},sk=d.skybeta||{},ph=m.phase;
 if(d.stale||!ph)return["#f44","NO DATA from the mission"];
 if(m.stale_offboard)return["#f44","OFFBOARD LEFT ON: disarm, power-cycle the Pixhawk"];
 if(m.height_refused)return["#f44","REFUSED: VIO height wrong. Pilot, land. Restart the stack"];
 if(!sk.px4_link)return["#fd5","NOT READY: no PX4 link"];
 if(ph=="idle")return sk.armed?["#4af","ARMED: pilot, switch to Position now"]:["#3c3","READY: pilot, arm on Altitude, then switch to Position"];
 if(ph=="waypoints")return["#4af",m.index==0?"TAKING OFF to "+m.height_m+" m":"FLYING ROUTE: waypoint "+m.index+" of "+m.legs];
 if(ph=="settle")return["#4af","AT THE LAST WAYPOINT: settling before landing"];
 if(ph=="land")return["#f84","LANDING ITSELF (pilot can take over)"];
 if(ph=="done")return["#aaa","DONE"+(sk.armed?": pilot, land if it is still flying":" (ready again once re-armed)")];
 return["#ddd",ph];}
function bar(d){const [c,t]=words(d);bn.style.color=c;bn.textContent=t;}
setInterval(async()=>{try{const d=await (await fetch("/state.json")).json();draw(d);text(d);bar(d);}catch(e){st.innerHTML='<span class="bad">page cannot reach the drone</span>';bn.style.color="#f44";bn.textContent="page cannot reach the drone";}},200);
</script></body></html>"""


def to_array(m):
    """sensor_msgs/Image -> numpy (honours the row step). mono8 / bgr8 / rgb8 / 16UC1."""
    enc = m.encoding.lower()
    if enc in ("16uc1", "mono16"):
        return np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
    if enc in ("mono8", "8uc1"):
        return a[:, :m.width]
    if enc in ("bgr8", "rgb8"):
        a = a[:, :m.width * 3].reshape(m.height, m.width, 3)
        return a if enc == "bgr8" else a[:, :, ::-1]
    raise ValueError(f"unsupported encoding {m.encoding!r}")


class Stream:
    """The latest JPEG of one topic, and a condition the HTTP threads wait on."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.viewers = 0
        self.t_last = 0.0
        self.arrivals = deque(maxlen=60)
        self.info = {}
        self.t_stats = 0.0

    def put(self, jpeg):
        with self.cond:
            self.jpeg, self.seq = jpeg, self.seq + 1
            self.cond.notify_all()


class LiveView(Node):
    def __init__(self):
        super().__init__("box_route_live_view")
        self.port = int(self.declare_parameter("port", 8080).value)
        self.address = self.declare_parameter("address", "0.0.0.0").value
        # /oak/left/image_raw straight from the driver, not image_mono (that one feeds cuVSLAM;
        # reading it slowed VIO: dronehacks PICKUP.md 2.6)
        camera_topic = self.declare_parameter("camera_topic", "/oak/left/image_raw").value
        self.enable_depth = bool(self.declare_parameter("enable_depth", False).value)
        depth_topic = self.declare_parameter("depth_topic", "/oak/stereo/image_raw").value
        map_topic = self.declare_parameter("map_topic", "/map/image").value
        self.max_fps = float(self.declare_parameter("max_fps", 10.0).value)
        self.depth_max_mm = 1000.0 * float(self.declare_parameter("depth_max_m", 4.0).value)
        self.quality = int(self.declare_parameter("jpeg_quality", 70).value)
        self.streams = {"camera": Stream()}
        self.create_subscription(Image, camera_topic, lambda m: self._on_image("camera", m), qos_profile_sensor_data)
        if self.enable_depth:
            self.streams.update(depth=Stream(), map=Stream())
            self.create_subscription(Image, depth_topic, lambda m: self._on_image("depth", m), qos_profile_sensor_data)
            self.create_subscription(Image, map_topic, lambda m: self._on_image("map", m), 1)
        names = list(self.streams)
        self.page = (PAGE.replace("@FIGURES@", FIG_CAMERA + (FIG_DEPTH if self.enable_depth else ""))
                     .replace("@STREAMS@", json.dumps(names))).encode()
        odom_topic = self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry").value
        self.lock = threading.Lock()
        self.odom = None                   # (t, x, y, z, yaw, speed) ENU
        self.odom_times = deque(maxlen=60)
        self.trail = deque(maxlen=4000)    # ENU (x, y), one point per 5 cm moved
        self.mission_status, self.sky = {}, {}
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/box_route/status", lambda m: self._on_json("mission_status", m), 10)
        self.create_subscription(String, "/skybeta/state", lambda m: self._on_json("sky", m), 10)
        self.get_logger().info(f"streams {names} (depth {'ON' if self.enable_depth else 'off'}): "
                               f"http://{self.address}:{self.port}/")

    def _on_odom(self, m):
        p, o, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        t = time.monotonic()
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        with self.lock:
            self.odom = (t, p.x, p.y, p.z, yaw, float(math.hypot(v.x, v.y)))
            self.odom_times.append(t)
            if not self.trail or math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) >= 0.05:
                self.trail.append((round(p.x, 3), round(p.y, 3)))

    def _on_json(self, attr, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            setattr(self, attr, d)

    def state(self):
        """/state.json: everything the position panel draws, in the odometry frame (ENU)."""
        now = time.monotonic()
        with self.lock:
            od, trail, ms, sky = self.odom, list(self.trail), dict(self.mission_status), dict(self.sky)
            n, t0 = len(self.odom_times), (self.odom_times[0] if self.odom_times else None)
        out = {"trail": trail, "skybeta": sky, "mission": ms}
        if od:
            out.update(pos=od[1:4], yaw=od[4], speed=od[5], odom_age=round(now - od[0], 3),
                       odom_hz=(n - 1) / (od[0] - t0) if n > 1 and od[0] > t0 else None)
            out["stale"] = now - od[0] if now - od[0] > 2.0 else None
        else:
            out["stale"] = None
        return out

    def _depth_stats(self, a):
        h, w = a.shape
        r = max(2, min(h, w) // 20)
        patch = a[h // 2 - r:h // 2 + r, w // 2 - r:w // 2 + r]
        patch = patch[patch > 0]
        valid = a[a > 0]
        return {
            "centre_m": float(np.median(patch)) / 1000.0 if patch.size else None,
            "min_m": float(np.percentile(valid, 1)) / 1000.0 if valid.size else None,
            "valid": float(valid.size) / a.size,
        }

    def _on_image(self, name, m):
        s = self.streams[name]
        t = time.monotonic()
        s.arrivals.append(t)
        n = len(s.arrivals)
        fps = (n - 1) / (t - s.arrivals[0]) if n > 1 and t > s.arrivals[0] else 0.0
        s.info.update(fps=fps, width=m.width, height=m.height, encoding=m.encoding)
        if name == "depth" and t - s.t_stats >= 0.2:
            s.t_stats = t
            try:
                s.info.update(self._depth_stats(to_array(m)))
            except ValueError:
                pass
        if s.viewers == 0 or t - s.t_last < 1.0 / self.max_fps:
            return
        s.t_last = t
        try:
            a = to_array(m)
        except ValueError as e:
            self.get_logger().warning(f"{name}: {e}", throttle_duration_sec=10.0)
            return
        if name == "depth":
            scaled = np.clip(a.astype(np.float32) * (255.0 / self.depth_max_mm), 0, 255).astype(np.uint8)
            colour = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            colour[a == 0] = 0
            h, w = a.shape
            cv2.drawMarker(colour, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, max(8, w // 15), 1)
            a = colour
        ok, buf = cv2.imencode(".jpg", a, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            s.put(buf.tobytes())

    def serve(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _send(self, ctype, body):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    self._send("text/html", node.page)
                    return
                if path == "/state.json":
                    self._send("application/json", json.dumps(node.state()).encode())
                    return
                if path == "/stats.json":
                    now = time.monotonic()
                    out = {k: None if not st.arrivals or now - st.arrivals[-1] > 2.0 else dict(st.info)
                           for k, st in node.streams.items()}
                    self._send("application/json", json.dumps(out).encode())
                    return
                name = path.strip("/").removesuffix(".mjpg")
                if name not in node.streams:
                    self.send_error(404)
                    return
                s = node.streams[name]
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                with s.cond:
                    s.viewers += 1
                seq = s.seq
                try:
                    while rclpy.ok():
                        with s.cond:
                            s.cond.wait_for(lambda: s.seq != seq, timeout=2.0)
                            if s.seq == seq:
                                continue
                            seq, jpeg = s.seq, s.jpeg
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    with s.cond:
                        s.viewers -= 1

        server = ThreadingHTTPServer((self.address, self.port), Handler)
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return server


def main(argv=None):
    rclpy.init(args=list(argv) if argv is not None else sys.argv)
    node = LiveView()
    server = node.serve()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

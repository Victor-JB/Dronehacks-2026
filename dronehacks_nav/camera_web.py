#!/usr/bin/env python3
"""dronehacks_nav.camera_web - the drone's live view on a plain web page, served by the Jetson itself:
where VIO thinks the drone is (track, heading, setpoint, planned route), the mission's phase, armed /
Offboard, and the camera, depth and map images.

Two buttons drive the mission (start_gate): START ROUTE (leave the hover after takeoff and fly the
route) and STOP (end the mission: it lands itself; the pilot can take over). They publish "start" / "stop" on
/refnav/command and work only with the key this node makes at startup (written to key_file; fly.sh
start prints the link with it), because anyone on the WiFi can open the page. Without the key the
page is view-only. Arming and the switch to Position stay with the pilot: no button can take off.

    http://skybeta-02.local:8080/            everything on one page
    http://skybeta-02.local:8080/state.json  odometry, track, mission status, skybeta state
    http://skybeta-02.local:8080/camera.mjpg
    http://skybeta-02.local:8080/depth.mjpg
    http://skybeta-02.local:8080/map.mjpg     top-down occupancy map from dronehacks_nav.map_node
    http://skybeta-02.local:8080/stats.json   topic rate, resolution, depth at the image centre

Frames are re-encoded at most max_fps per stream, and only while someone is watching, so an idle
page costs nothing but the subscriptions. Depth (16UC1 millimetres) is coloured with TURBO from
0 to depth_max_m; pixels with no return are black.

Run:  ros2 run dronehacks_nav camera_web --ros-args -p port:=8080
"""
from __future__ import annotations

import json
import secrets
import sys
import threading
from urllib.parse import parse_qs, urlparse
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

from dronehacks_nav import frames as F

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>skybeta camera</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif}
div{display:flex;flex-wrap:wrap;gap:8px;padding:8px}figure{margin:0;flex:1 1 400px}
img{width:100%;image-rendering:pixelated;background:#000}
.s{font:16px monospace;color:#8f8;white-space:pre}
#pos{display:flex;flex-wrap:wrap;gap:8px;padding:8px}canvas{background:#000;flex:1 1 420px;max-width:640px;aspect-ratio:1}
#st{flex:1 1 320px;font:15px monospace;white-space:pre;color:#ddd}
#bar{display:flex;flex-wrap:wrap;align-items:center;gap:12px;padding:10px 8px;background:#1b1b1b;position:sticky;top:0;z-index:1}
#banner{flex:1 1 420px;font:bold 22px sans-serif}button{font:bold 20px sans-serif;padding:12px 22px;border:0;border-radius:6px;cursor:pointer}
#go{background:#2a2;color:#fff}#stop{background:#c22;color:#fff}button:disabled{background:#444;color:#888;cursor:default}#msg{font:14px monospace;color:#fd5}.bad{color:#f66}.ok{color:#8f8}.warn{color:#fd5}</style></head><body>
<div id="bar"><div id="banner">connecting...</div>
<button id="go" disabled>START ROUTE</button><button id="stop" disabled>STOP</button><span id="msg"></span></div>
<div id="pos"><canvas id="c" width="640" height="640"></canvas><div id="st">waiting for /state.json</div></div>
<div>
<figure><img src="/camera.mjpg"><figcaption>camera (left)</figcaption><div class="s" id="camera">-</div></figure>
<figure><img src="/depth.mjpg"><figcaption>depth (black = no return; + = centre)</figcaption><div class="s" id="depth">-</div></figure>
<figure><img src="/map.mjpg"><figcaption>map from above, north up (grey = unknown, light = free, red = occupied; green + = start, green line = track, yellow = drone, blue = planned route, pink = setpoint)</figcaption><div class="s" id="map">-</div></figure>
</div><script>
function f(x){if(!x||!x.fps&&x.fps!==0)return "no messages";
 let t=x.fps.toFixed(1)+" Hz   "+x.width+"x"+x.height+" "+x.encoding;
 if("centre_m" in x)t+="\\ncentre "+(x.centre_m==null?"no return":x.centre_m.toFixed(2)+" m")
  +"   nearest "+(x.min_m==null?"-":x.min_m.toFixed(2)+" m")+"   valid "+(100*x.valid).toFixed(0)+"%";
 return t}
setInterval(async()=>{try{const d=await (await fetch("/stats.json")).json();
 for(const k of ["camera","depth","map"])document.getElementById(k).textContent=f(d[k]);}catch(e){}},500);
// --- where VIO thinks the drone is: top-down, the odometry frame's +x (the nose at stack start) up, +y left
const cv=document.getElementById("c"),g=cv.getContext("2d"),st=document.getElementById("st");
function fx(v,n){return v==null?"-":v.toFixed(n==null?2:n)}
function draw(d){
 const pts=[[0,0]].concat(d.trail||[],(d.route||[]).map(p=>[p[0],p[1]]));
 if(d.pos)pts.push([d.pos[0],d.pos[1]]); if(d.setpoint)pts.push([d.setpoint[0],d.setpoint[1]]);
 let r=1.5;for(const p of pts)r=Math.max(r,Math.abs(p[0])+0.5,Math.abs(p[1])+0.5);
 const W=cv.width,s=W/2/r,X=p=>W/2-p[1]*s,Y=p=>W/2-p[0]*s;   // +y left -> screen left, +x up
 g.clearRect(0,0,W,W);g.strokeStyle="#222";g.lineWidth=1;
 const step=r>6?2:r>3?1:0.5;
 for(let v=-Math.ceil(r/step)*step;v<=r;v+=step){g.beginPath();g.moveTo(X([0,v]),0);g.lineTo(X([0,v]),W);g.moveTo(0,Y([v,0]));g.lineTo(W,Y([v,0]));g.stroke();}
 g.fillStyle="#777";g.font="12px monospace";g.fillText(step+" m grid   +x (nose at stack start) up, +y left",6,14);
 if(d.route&&d.route.length){g.strokeStyle="#4af";g.setLineDash([6,4]);g.beginPath();
  const h=d.home?[d.home[0],d.home[1]]:[0,0];g.moveTo(X(h),Y(h));for(const p of d.route)g.lineTo(X(p),Y(p));g.stroke();g.setLineDash([]);
  g.fillStyle="#4af";d.route.forEach((p,i)=>{g.beginPath();g.arc(X(p),Y(p),4,0,7);g.fill();g.fillText(String(i+1),X(p)+6,Y(p)-6)});}
 g.strokeStyle="#3c3";g.lineWidth=2;g.beginPath();(d.trail||[]).forEach((p,i)=>i?g.lineTo(X(p),Y(p)):g.moveTo(X(p),Y(p)));g.stroke();
 g.strokeStyle="#0f0";g.beginPath();g.moveTo(X([0,0])-8,Y([0,0]));g.lineTo(X([0,0])+8,Y([0,0]));g.moveTo(X([0,0]),Y([0,0])-8);g.lineTo(X([0,0]),Y([0,0])+8);g.stroke();
 if(d.setpoint){g.strokeStyle="#f4f";g.lineWidth=2;g.beginPath();g.arc(X(d.setpoint),Y(d.setpoint),7,0,7);g.stroke();}
 if(d.pos){const p=d.pos,a=d.yaw||0,L=18;g.fillStyle=d.odom_age>0.5?"#f44":"#fd3";g.save();g.translate(X(p),Y(p));g.rotate(-a);
  g.beginPath();g.moveTo(0,-L);g.lineTo(-8,8);g.lineTo(8,8);g.closePath();g.fill();g.restore();}
}
function text(d){
 const sk=d.skybeta||{},m=d.mission||{},c=(ok,t)=>`<span class="${ok?"ok":"bad"}">${t}</span>`;
 let t=`<b>where it thinks it is</b> (VIO odometry, metres)\\n`;
 t+=d.pos?`x ${fx(d.pos[0])}  y ${fx(d.pos[1])}  z ${fx(d.pos[2])}  yaw ${fx(d.yaw*57.2958,0)} deg\\n`:"no odometry\\n";
 t+=`speed ${fx(d.speed)} m/s   odometry ${fx(d.odom_hz,1)} Hz, age `+c(d.odom_age!=null&&d.odom_age<0.5,fx(d.odom_age)+" s")+"\\n\\n";
 t+=`<b>PX4</b>   armed ${c(!!sk.armed,sk.armed?"YES":"no")}   Offboard ${c(!!sk.offboard_active,sk.offboard_active?"YES":"no")}   nav_state ${sk.nav_state??"-"}${sk.clamped?'   <span class="warn">CLAMPED</span>':""}\\n\\n`;
 t+=`<b>mission</b>   ${(m.phase||"-").toUpperCase()}`+(m.legs?`   leg ${Math.min(m.leg+1,m.legs)}/${m.legs}`:"")+"\\n";
 if(m.why)t+=m.why+"\\n"; if(m.result)t+=`<span class="warn">result: ${m.result}</span>\\n`;
 t+=`clearance ${m.clearance_m??"-"} m   depth age ${m.depth_age_s??"-"} s   motion check ${d.motion_check??"-"}\\n`;
 t+=`\\n<span style="color:#3c3">green</span> track   <span style="color:#fd3">yellow</span> drone + heading   <span style="color:#f4f">pink</span> setpoint   <span style="color:#4af">blue</span> planned route`;
 if(d.stale)t='<span class="bad">NO DATA from the drone for '+fx(d.stale,0)+" s</span>\\n\\n"+t;
 st.innerHTML=t;
}
// --- the state in plain words, and the two buttons (only with ?key=..., from fly.sh start)
const KEY=new URLSearchParams(location.search).get("key"),bn=document.getElementById("banner"),
 go=document.getElementById("go"),sp=document.getElementById("stop"),msg=document.getElementById("msg");
if(!KEY){go.style.display=sp.style.display="none";msg.textContent="view only: use the link fly.sh start prints for the buttons";}
const FLY=["turn","check","go","hold"],AIR=["climb","hover","await_start"].concat(FLY);
function words(d){const m=d.mission||{},sk=d.skybeta||{},ph=m.phase;
 if(d.stale||!ph)return["#f44","NO DATA from the mission"];
 if(ph=="wait"&&(m.why||"").startsWith("walkthrough"))return["#4af","WALKTHROUGH: recording only, this mode never flies"];
 if(ph=="wait")return["#fd5","NOT READY: "+(m.why||"")];
 if(ph=="arm")return sk.armed?["#4af","ARMED: pilot, switch to Position now"]:["#3c3","READY: pilot, arm on Altitude, then switch to Position"];
 if(ph=="climb")return["#4af","TAKING OFF to "+(m.setpoint?(-m.setpoint[2]).toFixed(2):"?")+" m"];
 if(ph=="hover")return["#4af","HOVERING"];
 if(ph=="await_start")return["#3c3","HOVERING: press START ROUTE"+(m.start_wait_left_s!=null?" (ends in "+m.start_wait_left_s.toFixed(0)+" s)":"")];
 if(FLY.includes(ph))return["#4af","FLYING ROUTE: leg "+Math.min(m.leg+1,m.legs)+" of "+m.legs];
 if(ph.startsWith("vault"))return["#4af","WALL VAULT: "+ph.slice(6)];
 if(ph=="land")return["#f84","MISSION ENDED: pilot, take over and land"];
 if(ph=="done")return["#aaa","DONE: "+(m.result||"")+" (ready again once disarmed)"];
 return["#ddd",ph];}
function bar(d){const [c,t]=words(d),ph=(d.mission||{}).phase;bn.style.color=c;bn.textContent=t;
 go.disabled=ph!="await_start";sp.disabled=!(ph=="wait"||ph=="arm"||AIR.includes(ph)||(ph||"").startsWith("vault"));}
async function send(cmd){
 if(cmd=="start"&&!confirm("START ROUTE: fly the route now?"))return;
 try{const r=await fetch("/command?cmd="+cmd+"&key="+encodeURIComponent(KEY),{method:"POST"});
  const j=await r.json();msg.textContent=j.ok?(cmd.toUpperCase()+" sent"):("refused: "+j.error);}catch(e){msg.textContent="could not send "+cmd;}}
go.onclick=()=>send("start");sp.onclick=()=>send("stop");
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
        self.arrivals = deque(maxlen=60)   # receipt times of every message, for the topic rate
        self.info = {}                     # what /stats.json reports for this stream
        self.t_stats = 0.0

    def put(self, jpeg):
        with self.cond:
            self.jpeg, self.seq = jpeg, self.seq + 1
            self.cond.notify_all()


class CameraWeb(Node):
    def __init__(self):
        super().__init__("camera_web")
        self.port = int(self.declare_parameter("port", 8080).value)
        self.address = self.declare_parameter("address", "0.0.0.0").value
        # /oak/left/image_raw straight from the driver, not image_mono: that one is the output of the
        # organisers' GPU converter that also feeds cuVSLAM, and reading it slowed VIO (PICKUP.md 2.6)
        camera_topic = self.declare_parameter("camera_topic", "/oak/left/image_raw").value
        depth_topic = self.declare_parameter("depth_topic", "/oak/stereo/image_raw").value
        map_topic = self.declare_parameter("map_topic", "/map/image").value
        self.max_fps = float(self.declare_parameter("max_fps", 10.0).value)
        self.depth_max_mm = 1000.0 * float(self.declare_parameter("depth_max_m", 4.0).value)
        self.quality = int(self.declare_parameter("jpeg_quality", 70).value)
        self.streams = {"camera": Stream(), "depth": Stream(), "map": Stream()}
        self.create_subscription(Image, camera_topic, lambda m: self._on_image("camera", m), qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, lambda m: self._on_image("depth", m), qos_profile_sensor_data)
        self.create_subscription(Image, map_topic, lambda m: self._on_image("map", m), 1)
        # where VIO thinks the drone is, and what the mission and PX4 are doing (all cheap to read)
        odom_topic = self.declare_parameter("odometry_topic", "/visual_slam/tracking/odometry").value
        self.lock = threading.Lock()
        self.odom = None                   # (t, x, y, z, yaw, speed) ENU
        self.odom_times = deque(maxlen=60)
        self.trail = deque(maxlen=4000)    # ENU (x, y), one point per 5 cm moved
        self.mission_status, self.sky = {}, {}
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/refnav/status", lambda m: self._on_json("mission_status", m), 10)
        self.create_subscription(String, "/skybeta/state", lambda m: self._on_json("sky", m), 10)
        # the buttons: a key per run, so only whoever has fly.sh start's link can press them
        self.pub_cmd = self.create_publisher(String, "/refnav/command", 10)
        self.key = secrets.token_urlsafe(6)
        key_file = self.declare_parameter("key_file", "/tmp/dronehacks_live_key").value
        try:
            with open(key_file, "w") as f:
                f.write(self.key + "\n")
        except OSError as e:
            self.get_logger().warning(f"cannot write {key_file}: {e}")
        self.get_logger().info(f"camera {camera_topic}, depth {depth_topic}, map {map_topic}: "
                               f"http://{self.address}:{self.port}/")

    def _on_odom(self, m):
        p, o, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        t = time.monotonic()
        with self.lock:
            self.odom = (t, p.x, p.y, p.z, F.yaw_from_quat_xyzw(o.x, o.y, o.z, o.w), float(np.hypot(v.x, v.y)))
            self.odom_times.append(t)
            if not self.trail or np.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) >= 0.05:
                self.trail.append((round(p.x, 3), round(p.y, 3)))

    def _on_json(self, attr, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            setattr(self, attr, d)
            setattr(self, attr + "_t", time.monotonic())

    def state(self):
        """/state.json: everything the position panel draws, in the odometry frame (ENU)."""
        now = time.monotonic()
        with self.lock:
            od, trail, ms, sky = self.odom, list(self.trail), dict(self.mission_status), dict(self.sky)
            n, t0 = len(self.odom_times), (self.odom_times[0] if self.odom_times else None)
        out = {"trail": trail, "skybeta": sky, "mission": ms, "motion_check": ms.get("motion_check")}
        if od:
            out.update(pos=od[1:4], yaw=od[4], speed=od[5], odom_age=round(now - od[0], 3),
                       odom_hz=(n - 1) / (od[0] - t0) if n > 1 and od[0] > t0 else None)
            out["stale"] = now - od[0] if now - od[0] > 2.0 else None
        else:
            out["stale"] = None
        enu = lambda q: list(F.ned_to_enu(*q[:3]))           # refnav works in NED; the page draws ENU
        if ms.get("setpoint"):
            out["setpoint"] = enu(ms["setpoint"])
        if ms.get("home"):
            out["home"] = enu(ms["home"])
        if ms.get("targets") and ms.get("home"):
            d = ms["home"][2]
            out["route"] = [enu((n_, e_, d)) for n_, e_ in ms["targets"]]
        return out

    def _depth_stats(self, a):
        """Centre distance (median of a small patch), nearest return, fraction of valid pixels."""
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
        if name == "depth" and t - s.t_stats >= 0.2:   # 5 Hz is plenty for a readout
            s.t_stats = t
            try:
                s.info.update(self._depth_stats(to_array(m)))
            except ValueError:
                pass
        now = self.get_clock().now().nanoseconds * 1e-9
        if s.viewers == 0 or now - s.t_last < 1.0 / self.max_fps:
            return
        s.t_last = now
        try:
            a = to_array(m)
        except ValueError as e:
            self.get_logger().warning(f"{name}: {e}", throttle_duration_sec=10.0)
            return
        if name == "depth":
            scaled = np.clip(a.astype(np.float32) * (255.0 / self.depth_max_mm), 0, 255).astype(np.uint8)
            colour = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            colour[a == 0] = 0                      # no return: black, not "very near"
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

            def _json(self, code, obj):
                body = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                url = urlparse(self.path)
                q = parse_qs(url.query)
                if url.path != "/command":
                    self.send_error(404)
                    return
                if not secrets.compare_digest(q.get("key", [""])[0], node.key):
                    self._json(403, {"ok": False, "error": "wrong or missing key: use the link fly.sh start prints"})
                    return
                cmd = q.get("cmd", [""])[0]
                if cmd not in ("start", "stop"):
                    self._json(400, {"ok": False, "error": f"unknown command {cmd!r}"})
                    return
                node.pub_cmd.publish(String(data=cmd))
                node.get_logger().info(f"live view: {cmd.upper()} pressed ({self.client_address[0]})")
                self._json(200, {"ok": True})

            def do_GET(self):
                self.path = urlparse(self.path).path       # the page's own ?key=... is for the browser
                if self.path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)
                    return
                if self.path == "/state.json":
                    body = json.dumps(node.state()).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path == "/stats.json":
                    now = time.monotonic()
                    out = {}
                    for k, st in node.streams.items():
                        stale = not st.arrivals or now - st.arrivals[-1] > 2.0
                        out[k] = None if stale else dict(st.info)
                    body = json.dumps(out).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                name = self.path.strip("/").removesuffix(".mjpg")
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
    node = CameraWeb()
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

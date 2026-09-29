"""Browser map and goal selector that avoids WSLg/OpenGL windows."""

import json
import math
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


HTML = r"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Go2 SLAM — выбор цели</title>
<style>
html,body{height:100%;margin:0;background:#17191d;color:#eef1f5;font:16px system-ui,Segoe UI,sans-serif;overflow:hidden}
header{height:58px;box-sizing:border-box;padding:9px 18px;background:#242831;display:flex;gap:22px;align-items:center}
h1{font-size:19px;margin:0}.hint{color:#bdc6d3}.status{margin-left:auto;color:#7edc91;font-weight:600}
#wrap{height:calc(100% - 58px);padding:12px;box-sizing:border-box;display:grid;grid-template-columns:minmax(0,1fr) minmax(300px,380px);gap:12px}
#mapPanel{position:relative;min-width:0;min-height:0}canvas{width:100%;height:100%;display:block;background:#23262c;border-radius:8px;cursor:crosshair}
#cameraPanel{align-self:start;background:#242831;border-radius:8px;padding:12px;box-sizing:border-box;box-shadow:0 8px 24px #0005}
#cameraTitle{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;font-weight:650}.cameraStatus{font-size:12px;color:#ff8a70}
#camera{display:block;width:100%;aspect-ratio:4/3;object-fit:cover;background:#101216;border-radius:6px}.cameraHelp{font-size:12px;color:#9ea9b8;margin-top:9px}
.legend{position:absolute;left:14px;bottom:12px;background:#17191ddd;padding:8px 12px;border-radius:6px;font-size:13px}
.blue{color:#3d9cff}.orange{color:#ff8a32}.cyan{color:#21d4e8}
@media(max-width:850px){html,body{overflow:auto}#wrap{height:auto;min-height:calc(100% - 58px);grid-template-columns:1fr;grid-template-rows:65vh auto}#cameraPanel{width:100%;max-width:520px;justify-self:center}}
</style></head><body>
<header><h1>Go2 SLAM</h1><span class="hint">Щелчок — ехать прямо к точке; протянуть — задать конечное направление</span><span id="status" class="status">Ожидание карты…</span></header>
<div id="wrap"><div id="mapPanel"><canvas id="map"></canvas><div class="legend"><span class="blue">●</span> робот &nbsp; <span class="cyan">•</span> сырой лидар &nbsp; <span class="orange">➜</span> цель</div></div><aside id="cameraPanel"><div id="cameraTitle"><span>Фронтальная камера</span><span id="cameraStatus" class="cameraStatus">ожидание…</span></div><img id="camera" alt="Камера Unitree Go2"><div class="cameraHelp">Поток /camera/image_raw · 320 × 240</div></aside></div>
<script>
const canvas=document.getElementById('map'),ctx=canvas.getContext('2d'),statusEl=document.getElementById('status'),cameraEl=document.getElementById('camera'),cameraStatusEl=document.getElementById('cameraStatus');
let state=null,img=null,view=null,drag=null;
function resize(){const r=canvas.getBoundingClientRect(),d=devicePixelRatio||1;canvas.width=Math.round(r.width*d);canvas.height=Math.round(r.height*d);draw()}
addEventListener('resize',resize);resize();
function rebuild(){if(!state?.map)return;const m=state.map,off=document.createElement('canvas');off.width=m.width;off.height=m.height;const c=off.getContext('2d'),im=c.createImageData(m.width,m.height);for(let y=0;y<m.height;y++)for(let x=0;x<m.width;x++){const v=m.data[y*m.width+x],sy=m.height-1-y,i=(sy*m.width+x)*4;let q=v<0?68:v===0?238:Math.max(18,238-v*2);im.data[i]=im.data[i+1]=im.data[i+2]=q;im.data[i+3]=255}c.putImageData(im,0,0);img=off}
function calcView(){if(!img)return null;const s=Math.min(canvas.width/img.width,canvas.height/img.height),w=img.width*s,h=img.height*s;return{x:(canvas.width-w)/2,y:(canvas.height-h)/2,w,h,s}}
function worldToScreen(x,y){const m=state.map,o=m.origin,dx=x-o.x,dy=y-o.y,co=Math.cos(o.yaw),si=Math.sin(o.yaw),gx=(co*dx+si*dy)/m.resolution,gy=(-si*dx+co*dy)/m.resolution;return{x:view.x+gx*view.s,y:view.y+(m.height-1-gy)*view.s}}
function screenToWorld(x,y){const m=state.map;if(!view||x<view.x||y<view.y||x>view.x+view.w||y>view.y+view.h)return null;const gx=(x-view.x)/view.s,gy=m.height-1-(y-view.y)/view.s,o=m.origin,co=Math.cos(o.yaw),si=Math.sin(o.yaw);return{x:o.x+co*gx*m.resolution-si*gy*m.resolution,y:o.y+si*gx*m.resolution+co*gy*m.resolution,gx:Math.floor(gx),gy:Math.floor(gy)}}
function free(p){const m=state.map;if(!p||p.gx<0||p.gy<0||p.gx>=m.width||p.gy>=m.height)return false;const v=m.data[p.gy*m.width+p.gx];return v>=0&&v<50}
function arrow(a,b,color){const ang=Math.atan2(b.y-a.y,b.x-a.x);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=4*(devicePixelRatio||1);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.beginPath();ctx.moveTo(b.x,b.y);ctx.lineTo(b.x-16*Math.cos(ang-.55),b.y-16*Math.sin(ang-.55));ctx.lineTo(b.x-16*Math.cos(ang+.55),b.y-16*Math.sin(ang+.55));ctx.closePath();ctx.fill()}
function drawScan(){if(!state?.scan)return;const d=devicePixelRatio||1;ctx.fillStyle='#20d7e8bb';for(const q of state.scan){const p=worldToScreen(q[0],q[1]);ctx.fillRect(p.x-.7*d,p.y-.7*d,1.4*d,1.4*d)}}
function draw(){ctx.fillStyle='#23262c';ctx.fillRect(0,0,canvas.width,canvas.height);if(!img||!state?.map)return;view=calcView();ctx.imageSmoothingEnabled=false;ctx.drawImage(img,view.x,view.y,view.w,view.h);drawScan();if(state.robot){const p=worldToScreen(state.robot.x,state.robot.y),tip=worldToScreen(state.robot.x+.4*Math.cos(state.robot.yaw),state.robot.y+.4*Math.sin(state.robot.yaw)),r=8*(devicePixelRatio||1);ctx.fillStyle='#198cff';ctx.beginPath();ctx.arc(p.x,p.y,r,0,Math.PI*2);ctx.fill();arrow(p,tip,'#198cff')}if(drag)arrow(drag.a,drag.b,'#ff6d24')}
function pos(e){const r=canvas.getBoundingClientRect(),d=devicePixelRatio||1;return{x:(e.clientX-r.left)*d,y:(e.clientY-r.top)*d}}
canvas.onmousedown=e=>{if(e.button===0)drag={a:pos(e),b:pos(e)}};canvas.onmousemove=e=>{if(drag){drag.b=pos(e);draw()}};
canvas.onmouseup=async e=>{if(!drag)return;drag.b=pos(e);const a=screenToWorld(drag.a.x,drag.a.y),b=screenToWorld(drag.b.x,drag.b.y),dragLength=Math.hypot(drag.b.x-drag.a.x,drag.b.y-drag.a.y);drag=null;draw();if(!free(a)){statusEl.textContent='Выберите светлую свободную клетку';statusEl.style.color='#ff8a70';return}let yaw;if(dragLength<15*(devicePixelRatio||1)&&state.robot)yaw=Math.atan2(a.y-state.robot.y,a.x-state.robot.x);else yaw=b?Math.atan2(b.y-a.y,b.x-a.x):0;const res=await fetch('/api/goal',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:a.x,y:a.y,yaw})});statusEl.textContent=res.ok?`Цель отправлена: ${a.x.toFixed(2)}, ${a.y.toFixed(2)}`:'Ошибка отправки цели';statusEl.style.color=res.ok?'#7edc91':'#ff8a70'};
async function update(){try{const n=await(await fetch('/api/state',{cache:'no-store'})).json(),changed=!state?.map||state.map.seq!==n.map?.seq;state=n;if(changed)rebuild();if(state.map)statusEl.textContent=statusEl.textContent==='Ожидание карты…'?'Карта готова':statusEl.textContent;draw()}catch(e){statusEl.textContent='Нет связи с ROS';statusEl.style.color='#ff8a70'}}
function updateCamera(){cameraEl.onload=()=>{cameraStatusEl.textContent='онлайн';cameraStatusEl.style.color='#7edc91';setTimeout(updateCamera,100)};cameraEl.onerror=()=>{cameraStatusEl.textContent='нет сигнала';cameraStatusEl.style.color='#ff8a70';setTimeout(updateCamera,500)};cameraEl.src='/api/camera.bmp?t='+Date.now()}
setInterval(update,300);update();
updateCamera();
</script></body></html>"""


class GoalWebNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_goal_web_ui")
        self._lock = threading.Lock()
        self._map = None
        self._map_seq = 0
        self._robot = None
        self._camera_bmp = None
        self._scan_points = []
        self._goal_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.create_subscription(Image, "/camera/image_raw", self._on_camera, 5)
        self.create_subscription(LaserScan, "/scan_raw", self._on_raw_scan, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.1, self._update_robot)
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                return

            def _send(self, status, content_type, body):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/":
                    self._send(200, "text/html; charset=utf-8", HTML.encode())
                elif self.path == "/api/state":
                    self._send(200, "application/json", json.dumps(node.state()).encode())
                elif self.path.startswith("/api/camera.bmp"):
                    frame = node.camera_frame()
                    if frame is None:
                        self._send(503, "text/plain", b"Camera frame is not ready")
                    else:
                        self._send(200, "image/bmp", frame)
                else:
                    self._send(404, "text/plain", b"Not found")

            def do_POST(self):
                if self.path != "/api/goal":
                    self._send(404, "text/plain", b"Not found")
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    goal = json.loads(self.rfile.read(size))
                    node.send_goal(float(goal["x"]), float(goal["y"]), float(goal["yaw"]))
                    self._send(200, "application/json", b'{"ok":true}')
                except (ValueError, KeyError, json.JSONDecodeError):
                    self._send(400, "application/json", b'{"ok":false}')

        self._server = ThreadingHTTPServer(("0.0.0.0", 8765), Handler)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info("Goal map: http://localhost:8765")

    def _on_map(self, message: OccupancyGrid) -> None:
        q = message.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._map_seq += 1
            self._map = {
                "seq": self._map_seq,
                "width": message.info.width,
                "height": message.info.height,
                "resolution": message.info.resolution,
                "origin": {"x": message.info.origin.position.x, "y": message.info.origin.position.y, "yaw": yaw},
                "data": list(message.data),
            }

    def _on_camera(self, message: Image) -> None:
        """Wrap Webots BGRA pixels in a top-down 32-bit BMP for the browser."""
        if message.encoding.lower() != "bgra8" or message.width <= 0 or message.height <= 0:
            return
        row_size = message.width * 4
        if message.step < row_size or len(message.data) < message.step * message.height:
            return
        if message.step == row_size:
            pixels = bytes(message.data)
        else:
            pixels = b"".join(
                bytes(message.data[row * message.step : row * message.step + row_size])
                for row in range(message.height)
            )
        pixel_offset = 14 + 40
        file_size = pixel_offset + len(pixels)
        file_header = struct.pack("<2sIHHI", b"BM", file_size, 0, 0, pixel_offset)
        # Negative height tells BMP readers that the first row is the top row.
        info_header = struct.pack(
            "<IiiHHIIiiII",
            40, message.width, -message.height, 1, 32, 0, len(pixels), 2835, 2835, 0, 0,
        )
        with self._lock:
            self._camera_bmp = file_header + info_header + pixels

    def camera_frame(self):
        with self._lock:
            return self._camera_bmp

    def _on_raw_scan(self, message: LaserScan) -> None:
        frame_id = message.header.frame_id or "lidar_link"
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", frame_id, Time.from_msg(message.header.stamp)
            )
        except TransformException:
            try:
                transform = self._tf_buffer.lookup_transform("map", frame_id, Time())
            except TransformException:
                return
        origin = transform.transform.translation
        q = transform.transform.rotation
        sensor_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        points = []
        angle = message.angle_min
        for distance in message.ranges:
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                world_angle = sensor_yaw + angle
                points.append(
                    [
                        round(origin.x + distance * math.cos(world_angle), 3),
                        round(origin.y + distance * math.sin(world_angle), 3),
                    ]
                )
            angle += message.angle_increment
        with self._lock:
            self._scan_points = points

    def _update_robot(self) -> None:
        try:
            t = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return
        p, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._robot = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw}

    def state(self):
        with self._lock:
            return {"map": self._map, "robot": self._robot, "scan": self._scan_points}

    def send_goal(self, x: float, y: float, yaw: float) -> None:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y = x, y
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self._goal_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError("Nav2 action server is not ready")
        request = NavigateToPose.Goal()
        request.pose = goal
        self._goal_client.send_goal_async(request)
        self.get_logger().info(f"Browser goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalWebNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

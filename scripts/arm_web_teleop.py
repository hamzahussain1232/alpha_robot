#!/usr/bin/env python3
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ArmWebTeleop(Node):
    def __init__(self) -> None:
        super().__init__('arm_web_teleop')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8090)
        self.declare_parameter('command_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('joint_min_rad', [-1.57, -1.57, -1.57, -1.57, -1.57, -1.57])
        self.declare_parameter('joint_max_rad', [1.57, 1.57, 1.57, 1.57, 1.57, 1.57])
        self.declare_parameter('default_move_duration_sec', 1.0)
        self.declare_parameter('nudge_step_deg', 5.0)
        self.declare_parameter('gripper_joint_index', 5)
        self.declare_parameter('gripper_open_rad', 0.0)
        self.declare_parameter('gripper_close_rad', 1.2)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.joint_names = [str(v) for v in self.get_parameter('joint_names').value]
        self.joint_min = [float(v) for v in self.get_parameter('joint_min_rad').value]
        self.joint_max = [float(v) for v in self.get_parameter('joint_max_rad').value]
        self.default_move_duration = float(self.get_parameter('default_move_duration_sec').value)
        self.nudge_step_deg = float(self.get_parameter('nudge_step_deg').value)
        self.gripper_joint_index = int(self.get_parameter('gripper_joint_index').value)
        self.gripper_open_rad = float(self.get_parameter('gripper_open_rad').value)
        self.gripper_close_rad = float(self.get_parameter('gripper_close_rad').value)

        expected = len(self.joint_names)
        if len(self.joint_min) != expected or len(self.joint_max) != expected:
            raise RuntimeError('joint_min_rad / joint_max_rad length must match joint_names length')

        self._lock = threading.Lock()
        self._current_positions = [0.0] * expected

        self.cmd_pub = self.create_publisher(JointTrajectory, self.command_topic, 10)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 10)

        self._http = self._start_http_server()
        self.get_logger().info(
            f'Arm web teleop ready at http://{self.host}:{self.port} (topic: {self.command_topic})'
        )

    def destroy_node(self):
        if hasattr(self, '_http') and self._http is not None:
            self._http.shutdown()
            self._http.server_close()
        return super().destroy_node()

    def _joint_state_cb(self, msg: JointState) -> None:
        if not msg.position:
            return
        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        with self._lock:
            for i, joint_name in enumerate(self.joint_names):
                idx = name_to_idx.get(joint_name)
                if idx is None or idx >= len(msg.position):
                    continue
                self._current_positions[i] = _clamp(float(msg.position[idx]), self.joint_min[i], self.joint_max[i])

    def _publish_pose(self, positions: List[float], duration_sec: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names[:]
        point = JointTrajectoryPoint()
        point.positions = [
            _clamp(float(value), self.joint_min[i], self.joint_max[i])
            for i, value in enumerate(positions)
        ]
        duration_sec = max(0.05, float(duration_sec))
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        traj.points = [point]
        self.cmd_pub.publish(traj)

        with self._lock:
            self._current_positions = point.positions[:]

    def _state_dict(self):
        with self._lock:
            current = self._current_positions[:]

        return {
            'joint_names': self.joint_names,
            'current_rad': current,
            'current_deg': [round(math.degrees(v), 1) for v in current],
            'min_rad': self.joint_min,
            'max_rad': self.joint_max,
            'min_deg': [round(math.degrees(v), 1) for v in self.joint_min],
            'max_deg': [round(math.degrees(v), 1) for v in self.joint_max],
            'default_move_duration_sec': self.default_move_duration,
            'nudge_step_deg': self.nudge_step_deg,
            'gripper_joint_index': self.gripper_joint_index,
        }

    def _start_http_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _write_json(self, payload, status=200):
                body = json.dumps(payload).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self):
                content_len = int(self.headers.get('Content-Length', '0'))
                raw = self.rfile.read(content_len) if content_len > 0 else b'{}'
                return json.loads(raw.decode('utf-8'))

            def do_GET(self):
                if self.path == '/state':
                    self._write_json(node._state_dict())
                    return
                if self.path == '/':
                    html = _build_html()
                    body = html.encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self._write_json({'error': 'not found'}, status=404)

            def do_POST(self):
                try:
                    payload = self._read_json()
                except Exception:
                    self._write_json({'ok': False, 'error': 'invalid json'}, status=400)
                    return

                if self.path == '/api/pose':
                    positions = payload.get('positions', [])
                    duration = payload.get('duration_sec', node.default_move_duration)
                    if not isinstance(positions, list) or len(positions) != len(node.joint_names):
                        self._write_json({'ok': False, 'error': 'positions must be a 6-value list'}, status=400)
                        return
                    node._publish_pose([float(v) for v in positions], float(duration))
                    self._write_json({'ok': True, 'state': node._state_dict()})
                    return

                if self.path == '/api/nudge':
                    idx = int(payload.get('joint_index', -1))
                    delta_deg = float(payload.get('delta_deg', node.nudge_step_deg))
                    duration = float(payload.get('duration_sec', node.default_move_duration))
                    if idx < 0 or idx >= len(node.joint_names):
                        self._write_json({'ok': False, 'error': 'invalid joint_index'}, status=400)
                        return
                    with node._lock:
                        target = node._current_positions[:]
                    target[idx] = target[idx] + math.radians(delta_deg)
                    node._publish_pose(target, duration)
                    self._write_json({'ok': True, 'state': node._state_dict()})
                    return

                if self.path == '/api/home':
                    node._publish_pose([0.0] * len(node.joint_names), node.default_move_duration)
                    self._write_json({'ok': True, 'state': node._state_dict()})
                    return

                if self.path == '/api/gripper_open':
                    with node._lock:
                        target = node._current_positions[:]
                    if 0 <= node.gripper_joint_index < len(target):
                        target[node.gripper_joint_index] = node.gripper_open_rad
                    node._publish_pose(target, node.default_move_duration)
                    self._write_json({'ok': True, 'state': node._state_dict()})
                    return

                if self.path == '/api/gripper_close':
                    with node._lock:
                        target = node._current_positions[:]
                    if 0 <= node.gripper_joint_index < len(target):
                        target[node.gripper_joint_index] = node.gripper_close_rad
                    node._publish_pose(target, node.default_move_duration)
                    self._write_json({'ok': True, 'state': node._state_dict()})
                    return

                self._write_json({'ok': False, 'error': 'not found'}, status=404)

            def log_message(self, _fmt, *_args):
                return

        server = ThreadingHTTPServer((self.host, self.port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        return server


def _build_html() -> str:
    return """<!doctype html>
<html lang='en'>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Nano Arm Web Control</title>
  <style>
    :root { --bg:#0f172a; --card:#111827; --fg:#e5e7eb; --muted:#94a3b8; --accent:#22c55e; }
    body { margin:0; font-family: ui-sans-serif,system-ui,sans-serif; background:linear-gradient(135deg,#0f172a,#1e293b); color:var(--fg); }
    .wrap { max-width:860px; margin:20px auto; padding:16px; }
    .card { background:var(--card); border-radius:14px; padding:16px; box-shadow:0 10px 30px rgba(0,0,0,.35); }
    h1 { margin:0 0 8px; font-size:1.35rem; }
    .hint { color:var(--muted); margin-bottom:14px; }
    .row { display:grid; grid-template-columns:120px 1fr 80px; gap:10px; align-items:center; margin:10px 0; }
    input[type=range]{ width:100%; }
    .btns { display:flex; flex-wrap:wrap; gap:8px; margin-top:14px; }
    button { border:0; border-radius:9px; padding:10px 14px; color:#fff; background:#334155; cursor:pointer; }
    button.primary { background:var(--accent); color:#052e16; font-weight:700; }
    .status { margin-top:10px; color:var(--muted); font-size:.95rem; }
    @media (max-width: 640px) { .row { grid-template-columns:1fr; } }
  </style>
</head>
<body>
  <div class='wrap'>
    <div class='card'>
      <h1>Nano Arm 6-DOF Web Control</h1>
      <div class='hint'>Move sliders in degrees and press <b>Send Pose</b>.</div>
      <div id='joints'></div>
      <div class='btns'>
        <button class='primary' onclick='sendPose()'>Send Pose</button>
        <button onclick='homePose()'>Home (All 0)</button>
        <button onclick='gripperOpen()'>Gripper Open</button>
        <button onclick='gripperClose()'>Gripper Close</button>
      </div>
      <div class='status' id='status'>Loading...</div>
    </div>
  </div>
<script>
let state = null;

function rad(deg){ return deg * Math.PI / 180.0; }

async function api(path, method='GET', body=null){
  const res = await fetch(path, {
    method,
    headers: {'Content-Type': 'application/json'},
    body: body ? JSON.stringify(body) : undefined,
  });
  return await res.json();
}

function jointRow(i, name, minDeg, maxDeg, curDeg){
  return `
    <div class='row'>
      <label>${name}</label>
      <input type='range' id='j${i}' min='${minDeg}' max='${maxDeg}' step='1' value='${curDeg}' oninput='updateValue(${i})'>
      <span id='v${i}'>${curDeg}°</span>
    </div>`;
}

function updateValue(i){
  const s = document.getElementById('j' + i);
  document.getElementById('v' + i).textContent = `${s.value}°`;
}

function render(){
  const c = document.getElementById('joints');
  c.innerHTML = state.joint_names.map((name, i) => jointRow(i, name, state.min_deg[i], state.max_deg[i], state.current_deg[i])).join('');
  document.getElementById('status').textContent = 'Ready';
}

async function refresh(){
  state = await api('/state');
  render();
}

async function sendPose(){
  const positions = state.joint_names.map((_, i) => rad(parseFloat(document.getElementById('j' + i).value)));
  const out = await api('/api/pose', 'POST', {positions, duration_sec: state.default_move_duration_sec});
  document.getElementById('status').textContent = out.ok ? 'Pose sent' : (`Error: ${out.error}`);
  if (out.state){ state = out.state; }
}

async function homePose(){
  const out = await api('/api/home', 'POST', {});
  if (out.state){ state = out.state; render(); }
  document.getElementById('status').textContent = out.ok ? 'Home sent' : (`Error: ${out.error}`);
}

async function gripperOpen(){
  const out = await api('/api/gripper_open', 'POST', {});
  if (out.state){ state = out.state; render(); }
  document.getElementById('status').textContent = out.ok ? 'Gripper open sent' : (`Error: ${out.error}`);
}

async function gripperClose(){
  const out = await api('/api/gripper_close', 'POST', {});
  if (out.state){ state = out.state; render(); }
  document.getElementById('status').textContent = out.ok ? 'Gripper close sent' : (`Error: ${out.error}`);
}

refresh().catch(err => {
  document.getElementById('status').textContent = `Failed to load: ${err}`;
});
</script>
</body>
</html>
"""


def main(args=None):
    rclpy.init(args=args)
    node = ArmWebTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

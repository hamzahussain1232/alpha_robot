#!/usr/bin/env python3
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Camera + Detections</title>
  <style>
    :root {
      --bg: #0e1318;
      --panel: #15222d;
      --panel2: #1a2b37;
      --text: #eef4f8;
      --muted: #9fb4c3;
      --accent: #59b3ff;
      --good: #75d39a;
      --warn: #f1c86b;
      --border: #284152;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "DejaVu Sans", "Noto Sans", sans-serif;
      background: radial-gradient(circle at top, #1c2a36, var(--bg) 58%);
      color: var(--text);
    }
    .wrap {
      max-width: 1280px;
      margin: 0 auto;
      padding: 22px 16px 40px;
    }
    .hero {
      display: flex;
      justify-content: space-between;
      align-items: end;
      gap: 16px;
      margin-bottom: 16px;
    }
    .hero h1 {
      margin: 0;
      font-size: 2rem;
    }
    .hero p {
      margin: 8px 0 0;
      color: var(--muted);
    }
    .pill {
      padding: 10px 14px;
      border-radius: 999px;
      background: rgba(89, 179, 255, 0.12);
      border: 1px solid rgba(89, 179, 255, 0.25);
      color: var(--accent);
      white-space: nowrap;
    }
    .grid {
      display: grid;
      grid-template-columns: 1.3fr 1fr;
      gap: 14px;
    }
    .card {
      background: linear-gradient(180deg, var(--panel2), var(--panel));
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 16px;
      box-shadow: 0 12px 26px rgba(0, 0, 0, 0.2);
    }
    .card h2 {
      margin: 0 0 10px;
      font-size: 0.95rem;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    .camera {
      width: 100%;
      aspect-ratio: 4 / 3;
      object-fit: contain;
      background: #000;
      border-radius: 14px;
      border: 1px solid rgba(255,255,255,0.08);
    }
    .sub {
      margin-top: 10px;
      color: var(--muted);
      font-size: 0.92rem;
      line-height: 1.5;
    }
    .table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 10px;
    }
    .table th,
    .table td {
      text-align: left;
      padding: 10px 6px;
      border-bottom: 1px solid rgba(255,255,255,0.06);
      font-size: 0.94rem;
    }
    .badge {
      display: inline-block;
      padding: 4px 10px;
      border-radius: 999px;
      font-size: 0.84rem;
      font-weight: 700;
    }
    .ok { background: rgba(117, 211, 154, 0.14); color: var(--good); }
    .muted { color: var(--muted); }
    .footer {
      margin-top: 14px;
      color: var(--muted);
      font-size: 0.88rem;
    }
    @media (max-width: 960px) {
      .grid { grid-template-columns: 1fr; }
      .hero { flex-direction: column; align-items: start; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <div>
        <h1>Robot Camera + Detections</h1>
        <p>Live compressed camera stream with detector labels beside it.</p>
      </div>
      <div class="pill" id="statusPill">Waiting for camera...</div>
    </div>

    <div class="grid">
      <div class="card">
        <h2>Live Camera</h2>
        <img id="cameraFeed" class="camera" src="/stream.mjpg" alt="camera feed">
        <div class="sub" id="cameraMeta">No frame yet.</div>
      </div>

      <div class="card">
        <h2>Detections</h2>
        <div class="sub" id="detSummary">No detections yet.</div>
        <table class="table">
          <thead><tr><th>Label</th><th>Score</th><th>Pose</th></tr></thead>
          <tbody id="detRows"></tbody>
        </table>
      </div>
    </div>

    <div class="footer">
      The page uses the live compressed camera topic, so it stays light and avoids raw-image RViz by default.
    </div>
  </div>

  <script>
    const cameraFeed = document.getElementById('cameraFeed');
    const cameraMeta = document.getElementById('cameraMeta');
    const detSummary = document.getElementById('detSummary');
    const detRows = document.getElementById('detRows');
    const statusPill = document.getElementById('statusPill');

    function fmtPose(pose) {
      if (!pose) return 'n/a';
      const x = Number(pose.x || 0).toFixed(2);
      const y = Number(pose.y || 0).toFixed(2);
      return `x ${x}, y ${y}`;
    }

    async function refreshDetections() {
      try {
        const res = await fetch('/api/detections', {cache: 'no-store'});
        const data = await res.json();
        const objects = Array.isArray(data.objects) ? data.objects : [];
        detSummary.textContent = data.summary || (objects.length ? `${objects.length} object(s)` : 'No detections yet.');

        detRows.innerHTML = '';
        if (!objects.length) {
          const tr = document.createElement('tr');
          tr.innerHTML = '<td colspan="3" class="muted">Nothing detected yet.</td>';
          detRows.appendChild(tr);
        } else {
          objects.slice(0, 8).forEach(obj => {
            const tr = document.createElement('tr');
            const label = obj.label || 'unknown';
            const score = Number(obj.score || 0).toFixed(2);
            tr.innerHTML = `<td><span class="badge ok">${label}</span></td><td>${score}</td><td class="muted">${fmtPose(obj.pose)}</td>`;
            detRows.appendChild(tr);
          });
        }

        if (data.camera_age_sec !== undefined && data.camera_age_sec !== null) {
          const age = Number(data.camera_age_sec).toFixed(2);
          cameraMeta.textContent = `Camera frame age: ${age}s`;
          statusPill.textContent = objects.length ? `Live detections: ${objects.length}` : 'Camera online';
        }
      } catch (err) {
        detSummary.textContent = 'Detection feed not ready.';
        statusPill.textContent = 'Waiting for detector...';
      }
    }

    function tick() {
      refreshDetections();
    }

    tick();
    setInterval(tick, 700);
  </script>
</body>
</html>
"""


class CameraDetectionWeb(Node):
    def __init__(self) -> None:
        super().__init__("camera_detection_web")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8091)
        self.declare_parameter("camera_topic", "/camera/image_raw/compressed")
        self.declare_parameter("detections_topic", "/perception/detections")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._lock = threading.Lock()
        self._latest_camera_bytes: Optional[bytes] = None
        self._latest_camera_ts = 0.0
        self._latest_detections: List[Dict[str, Any]] = []
        self._latest_detections_ts = 0.0
        self._latest_detections_raw = ""

        self.create_subscription(CompressedImage, self.camera_topic, self._on_camera, sensor_qos)
        self.create_subscription(String, self.detections_topic, self._on_detections, 10)

        self.get_logger().info(
            f"Camera web ready at http://{self.host}:{self.port} "
            f"(camera={self.camera_topic}, detections={self.detections_topic})"
        )

    def _on_camera(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_camera_bytes = bytes(msg.data)
            self._latest_camera_ts = time.time()

    def _on_detections(self, msg: String) -> None:
        raw = msg.data or ""
        try:
            payload = json.loads(raw) if raw else {}
        except Exception:
            payload = {}
        objects = payload.get("objects", []) if isinstance(payload, dict) else []
        if not isinstance(objects, list):
            objects = []

        cleaned: List[Dict[str, Any]] = []
        for obj in objects[:12]:
            if not isinstance(obj, dict):
                continue
            cleaned.append(
                {
                    "label": obj.get("label", "unknown"),
                    "score": float(obj.get("score", 0.0) or 0.0),
                    "pose": obj.get("pose", {}),
                }
            )

        with self._lock:
            self._latest_detections = cleaned
            self._latest_detections_ts = time.time()
            self._latest_detections_raw = raw

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            age = None if self._latest_camera_ts <= 0 else max(0.0, time.time() - self._latest_camera_ts)
            detections_age = (
                None if self._latest_detections_ts <= 0 else max(0.0, time.time() - self._latest_detections_ts)
            )
            return {
                "camera_ready": self._latest_camera_bytes is not None,
                "camera_age_sec": age,
                "detections_age_sec": detections_age,
                "count": len(self._latest_detections),
                "objects": list(self._latest_detections),
            }

    def camera_bytes(self) -> Optional[bytes]:
        with self._lock:
            if self._latest_camera_bytes is None:
                return None
            return bytes(self._latest_camera_bytes)

    def detections_payload(self) -> Dict[str, Any]:
        snapshot = self.snapshot()
        objects = snapshot["objects"]
        labels = [str(obj.get("label", "unknown")) for obj in objects[:8]]
        summary = "No detections yet."
        if labels:
            summary = f"{len(objects)} object(s): " + ", ".join(labels)
        return {
            "camera_age_sec": snapshot["camera_age_sec"],
            "detections_age_sec": snapshot["detections_age_sec"],
            "count": snapshot["count"],
            "objects": objects,
            "summary": summary,
            "camera_ready": snapshot["camera_ready"],
        }


node: Optional[CameraDetectionWeb] = None


class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if node is None:
            self.send_response(503)
            self.end_headers()
            return

        path = urlparse(self.path).path

        if path == "/":
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/camera.jpg":
            frame = node.camera_bytes()
            if not frame:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"camera not ready")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
            return

        if path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    frame = node.camera_bytes()
                    if frame is None:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    time.sleep(0.08)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return
            except Exception:
                return

        if path == "/api/detections":
            payload = json.dumps(node.detections_payload()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if path == "/api/status":
            payload = json.dumps(node.snapshot()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, _fmt, *_args):
        return


def main() -> None:
    global node

    rclpy.init()
    node = CameraDetectionWeb()
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    server = ThreadingHTTPServer((node.host, node.port), DashboardHandler)
    node.get_logger().info(f"Open http://{node.host}:{node.port} in your browser")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            server.shutdown()
            server.server_close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

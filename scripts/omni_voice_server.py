#!/usr/bin/env python3
"""
Alpha Robot Command Center
One mobile dashboard for:
- Annotated YOLO camera stream
- Detection panel
- Safe hold-to-move controls
- Emergency stop
- Browser microphone and text commands
- Arm preset commands
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from flask import Flask, Response, jsonify, render_template_string, request

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


app = Flask(__name__)
dashboard: Optional["AlphaDashboard"] = None


HTML = r"""
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Alpha Robot Command Center</title>
<style>
:root {
  --bg: #07111f;
  --panel: #102236;
  --panel2: #132a43;
  --line: #29435d;
  --text: #eef6ff;
  --muted: #9eb4ca;
  --blue: #4cb0ff;
  --green: #40d39c;
  --yellow: #ffc857;
  --red: #ff5c72;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: radial-gradient(circle at top, #132c49 0%, var(--bg) 55%);
  color: var(--text);
  font-family: Arial, Helvetica, sans-serif;
}
header {
  padding: 18px 20px;
  border-bottom: 1px solid var(--line);
  background: rgba(7, 17, 31, 0.92);
  position: sticky;
  top: 0;
  z-index: 10;
}
h1 { margin: 0; font-size: clamp(1.35rem, 3vw, 2rem); }
.subtitle { color: var(--muted); margin-top: 6px; }
.wrap {
  max-width: 1500px;
  margin: 0 auto;
  padding: 18px;
}
.status-row {
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
  margin-bottom: 16px;
}
.badge {
  padding: 8px 11px;
  border: 1px solid var(--line);
  background: rgba(16, 34, 54, 0.92);
  border-radius: 999px;
  color: var(--muted);
  font-size: 0.9rem;
}
.badge.ok { color: var(--green); border-color: rgba(64, 211, 156, 0.45); }
.badge.warn { color: var(--yellow); border-color: rgba(255, 200, 87, 0.45); }
.badge.danger { color: var(--red); border-color: rgba(255, 92, 114, 0.5); }
.grid {
  display: grid;
  grid-template-columns: minmax(0, 1.55fr) minmax(330px, 0.95fr);
  gap: 16px;
}
.card {
  background: linear-gradient(150deg, rgba(19,42,67,.98), rgba(10,27,45,.98));
  border: 1px solid var(--line);
  border-radius: 16px;
  padding: 15px;
  box-shadow: 0 12px 35px rgba(0,0,0,.22);
}
.card h2 {
  margin: 0 0 12px;
  font-size: 1.05rem;
  letter-spacing: .04em;
  text-transform: uppercase;
  color: #c5dcf3;
}
.camera-box {
  min-height: 270px;
  background: #000;
  border-radius: 12px;
  overflow: hidden;
  border: 1px solid #203a55;
  display: grid;
  place-items: center;
}
.camera-box img {
  display: block;
  width: 100%;
  height: auto;
  max-height: 65vh;
  object-fit: contain;
}
.camera-note { color: var(--muted); margin: 10px 2px 0; font-size: .9rem; }
.detections {
  max-height: 300px;
  overflow: auto;
}
table { width: 100%; border-collapse: collapse; }
th, td {
  text-align: left;
  padding: 10px 7px;
  border-bottom: 1px solid rgba(74,111,143,.35);
}
th { color: #c7dff8; font-size: .82rem; text-transform: uppercase; }
td { color: var(--muted); }
.controls-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 16px;
  margin-top: 16px;
}
.control-card {
  background: linear-gradient(150deg, rgba(19,42,67,.98), rgba(10,27,45,.98));
  border: 1px solid var(--line);
  border-radius: 16px;
  padding: 15px;
}
.control-card h2 {
  margin: 0 0 12px;
  font-size: 1.02rem;
  color: #c5dcf3;
}
.dpad {
  width: min(275px, 100%);
  margin: 0 auto;
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 9px;
}
.dpad button:nth-child(1) { grid-column: 2; }
.dpad button:nth-child(2) { grid-column: 1; grid-row: 2; }
.dpad button:nth-child(3) { grid-column: 2; grid-row: 2; }
.dpad button:nth-child(4) { grid-column: 3; grid-row: 2; }
.dpad button:nth-child(5) { grid-column: 2; grid-row: 3; }
button {
  border: 1px solid #3d6388;
  border-radius: 12px;
  background: #173a5d;
  color: var(--text);
  min-height: 54px;
  font-size: 1rem;
  font-weight: 700;
  cursor: pointer;
  touch-action: none;
}
button:hover { background: #1d4c78; }
button.active { background: #286da7; border-color: #74c7ff; }
button.stop { background: #5e2534; border-color: #d75a70; }
button.stop:hover { background: #803043; }
button.estop { background: #94263a; border-color: #ff8193; }
button.estop:hover { background: #bb3049; }
button.release { background: #1d5c4a; border-color: #53d7a7; }
button.small { min-height: 44px; padding: 8px 10px; font-size: .9rem; }
.row { display: flex; gap: 9px; flex-wrap: wrap; }
.row > * { flex: 1; min-width: 110px; }
label { display:block; color: var(--muted); margin-top: 10px; font-size: .9rem; }
input[type="range"] { width:100%; }
input[type="text"] {
  width: 100%;
  padding: 12px;
  border-radius: 10px;
  border: 1px solid #3d6388;
  background: #071522;
  color: white;
  font-size: 1rem;
}
.help {
  color: var(--muted);
  font-size: .88rem;
  line-height: 1.45;
}
.value { color: var(--blue); font-weight: 700; }
.footer {
  color: var(--muted);
  padding: 20px 0 8px;
  text-align: center;
  font-size: .85rem;
}
@media (max-width: 930px) {
  .grid { grid-template-columns: 1fr; }
  .controls-grid { grid-template-columns: 1fr; }
}
</style>
</head>
<body>
<header>
  <h1>Alpha Robot Command Center</h1>
  <div class="subtitle">Camera, YOLO detections, voice, movement, arm control and emergency safety</div>
</header>

<main class="wrap">
  <section class="status-row">
    <div class="badge" id="cameraStatus">Camera: checking...</div>
    <div class="badge" id="detectorStatus">Detector: checking...</div>
    <div class="badge" id="driveStatus">Drive: stopped</div>
    <div class="badge" id="estopStatus">Emergency stop: released</div>
    <div class="badge" id="tempStatus">Pi temp: checking...</div>
    <div class="badge" id="cpuStatus">CPU: checking...</div>
    <div class="badge" id="ramStatus">RAM: checking...</div>
    <div class="badge" id="diskStatus">Disk: checking...</div>
    <div class="badge" id="uptimeStatus">Uptime: checking...</div>
    <div class="badge" id="throttleStatus">Power: checking...</div>
  </section>

  <section class="grid">
    <article class="card">
      <h2>Live YOLO Camera</h2>
      <div class="camera-box">
        <img src="/stream.mjpg" alt="Live annotated camera stream">
      </div>
      <div class="camera-note">
        This stream uses <b>/perception/annotated_image/compressed</b>.
        Bounding boxes appear when YOLO detects supported objects.
      </div>
    </article>

    <article class="card">
      <h2>Detected Objects</h2>
      <div class="detections">
        <table>
          <thead>
            <tr><th>Label</th><th>Confidence</th><th>Pose</th></tr>
          </thead>
          <tbody id="detectionRows">
            <tr><td colspan="3">Waiting for detections...</td></tr>
          </tbody>
        </table>
      </div>
      <p class="help" id="detectionSummary">No detections yet.</p>
    </article>
  </section>

  <section class="controls-grid">
    <article class="control-card">
      <h2>Drive Control</h2>
      <p class="help">Press and hold a direction button. Releasing it sends a stop command.</p>

      <div class="dpad">
        <button id="forward">▲ Forward</button>
        <button id="left">◀ Left</button>
        <button id="stop" class="stop">■ Stop</button>
        <button id="right">Right ▶</button>
        <button id="back">▼ Back</button>
      </div>

      <label>Linear speed: <span class="value" id="linearValue">0.10 m/s</span></label>
      <input id="linearSpeed" type="range" min="0.04" max="0.16" step="0.01" value="0.10">

      <label>Turning speed: <span class="value" id="angularValue">0.45 rad/s</span></label>
      <input id="angularSpeed" type="range" min="0.20" max="1.70" step="0.05" value="0.45">

      <div class="row" style="margin-top:14px;">
        <button id="estop" class="estop">Emergency Stop</button>
        <button id="releaseEstop" class="release">Release E-Stop</button>
      </div>
    </article>

    <article class="control-card">
      <h2>Voice and Text Command</h2>
      <p class="help">Examples: “robot move forward”, “robot stop”, “robot wave”, “robot raise arm”, “give me cup”.</p>
      <div class="row">
        <button id="micButton">🎤 Speak Command</button>
        <button id="sendText">Send Text</button>
      </div>
      <label for="commandText">Command text</label>
      <input id="commandText" type="text" placeholder="Type a robot command...">

      <h2 style="margin-top:18px;">Arm Presets</h2>
      <div class="row">
        <button class="small armButton" data-command="robot wave">Wave</button>
        <button class="small armButton" data-command="robot raise arm">Raise</button>
        <button class="small armButton" data-command="robot lower arm">Lower</button>
      </div>
      <div class="row" style="margin-top:9px;">
        <button class="small armButton" data-command="robot reset arm">Reset Arm</button>
        <button class="small armButton" data-command="robot give me cup">Cup Sequence</button>
      </div>

      <p class="help" id="commandFeedback" style="margin-top:16px;">Dashboard ready.</p>
    </article>
  </section>

  <div class="footer">
    Alpha Robot • Hold-to-move safety • YOLO annotated stream • Browser microphone requires accepted HTTPS certificate
  </div>
</main>

<script>
const linear = document.getElementById("linearSpeed");
const angular = document.getElementById("angularSpeed");
const linearValue = document.getElementById("linearValue");
const angularValue = document.getElementById("angularValue");
const feedback = document.getElementById("commandFeedback");

let activeMotion = null;
let motionTimer = null;

function updateSpeedLabels() {
  linearValue.textContent = `${Number(linear.value).toFixed(2)} m/s`;
  angularValue.textContent = `${Number(angular.value).toFixed(2)} rad/s`;
}
linear.addEventListener("input", updateSpeedLabels);
angular.addEventListener("input", updateSpeedLabels);
updateSpeedLabels();

async function postJson(path, payload) {
  const response = await fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload)
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.message || "Request failed");
  return data;
}

function setFeedback(text) {
  feedback.textContent = text;
}

async function sendDrive(action) {
  return postJson("/api/drive", {
    action,
    linear_speed: Number(linear.value),
    angular_speed: Number(angular.value)
  });
}

function stopMotion(sendStop = true) {
  if (motionTimer) {
    clearInterval(motionTimer);
    motionTimer = null;
  }
  document.querySelectorAll(".dpad button").forEach(btn => btn.classList.remove("active"));
  activeMotion = null;
  if (sendStop) {
    sendDrive("stop").catch(err => setFeedback(`Stop error: ${err.message}`));
  }
}

function startMotion(action, button) {
  if (activeMotion === action) return;
  stopMotion(false);
  activeMotion = action;
  button.classList.add("active");

  sendDrive(action).catch(err => setFeedback(`Drive error: ${err.message}`));
  motionTimer = setInterval(() => {
    sendDrive(action).catch(() => {});
  }, 180);
}

function bindHold(id, action) {
  const button = document.getElementById(id);
  button.addEventListener("pointerdown", event => {
    event.preventDefault();
    startMotion(action, button);
  });
  ["pointerup", "pointercancel", "pointerleave"].forEach(name => {
    button.addEventListener(name, event => {
      event.preventDefault();
      if (activeMotion === action) stopMotion(true);
    });
  });
}

bindHold("forward", "forward");
bindHold("back", "back");
bindHold("left", "left");
bindHold("right", "right");

document.getElementById("stop").addEventListener("click", () => {
  stopMotion(true);
  setFeedback("Robot stop command sent.");
});

window.addEventListener("blur", () => stopMotion(true));
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopMotion(true);
});

document.getElementById("estop").addEventListener("click", async () => {
  try {
    stopMotion(false);
    await postJson("/api/estop", {enabled: true});
    setFeedback("Emergency stop is active.");
  } catch (err) {
    setFeedback(`Emergency stop error: ${err.message}`);
  }
});

document.getElementById("releaseEstop").addEventListener("click", async () => {
  try {
    await postJson("/api/estop", {enabled: false});
    setFeedback("Emergency stop released. Robot is still stopped.");
  } catch (err) {
    setFeedback(`Release error: ${err.message}`);
  }
});

async function sendCommand(text) {
  const clean = String(text || "").trim();
  if (!clean) return;
  try {
    await postJson("/api/command", {text: clean});
    setFeedback(`Command sent: ${clean}`);
    document.getElementById("commandText").value = "";
  } catch (err) {
    setFeedback(`Command error: ${err.message}`);
  }
}

document.getElementById("sendText").addEventListener("click", () => {
  sendCommand(document.getElementById("commandText").value);
});

document.getElementById("commandText").addEventListener("keydown", event => {
  if (event.key === "Enter") sendCommand(event.target.value);
});

document.querySelectorAll(".armButton").forEach(button => {
  button.addEventListener("click", () => sendCommand(button.dataset.command));
});

document.getElementById("micButton").addEventListener("click", () => {
  const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SpeechRecognition) {
    setFeedback("Speech recognition is not supported in this browser. Use the text box.");
    return;
  }

  const recognition = new SpeechRecognition();
  recognition.lang = "en-US";
  recognition.interimResults = false;
  recognition.maxAlternatives = 1;

  setFeedback("Listening...");
  recognition.start();

  recognition.onresult = event => {
    const text = event.results[0][0].transcript;
    document.getElementById("commandText").value = text;
    sendCommand(text);
  };

  recognition.onerror = event => {
    setFeedback(`Microphone error: ${event.error}`);
  };
});

function setBadge(id, text, style) {
  const badge = document.getElementById(id);
  badge.textContent = text;
  badge.className = `badge ${style || ""}`;
}

function renderDetections(objects) {
  const body = document.getElementById("detectionRows");
  body.innerHTML = "";

  if (!objects || objects.length === 0) {
    body.innerHTML = '<tr><td colspan="3">No detections yet.</td></tr>';
    return;
  }

  objects.forEach(obj => {
    const row = document.createElement("tr");
    const label = document.createElement("td");
    const score = document.createElement("td");
    const pose = document.createElement("td");

    label.textContent = obj.label || "unknown";
    score.textContent = `${Math.round((Number(obj.score) || 0) * 100)}%`;

    if (obj.pose && typeof obj.pose === "object" && Object.keys(obj.pose).length) {
      pose.textContent = JSON.stringify(obj.pose);
    } else {
      pose.textContent = "—";
    }

    row.append(label, score, pose);
    body.appendChild(row);
  });
}

async function refreshStatus() {
  try {
    const response = await fetch("/api/status", {cache: "no-store"});
    const data = await response.json();

    setBadge(
      "cameraStatus",
      data.camera_ready ? `Camera: live (${data.camera_age_sec ?? "?"}s)` : "Camera: waiting",
      data.camera_ready ? "ok" : "warn"
    );

    setBadge(
      "detectorStatus",
      `Detector: ${data.count || 0} object(s)`,
      data.count ? "ok" : "warn"
    );

    setBadge(
      "driveStatus",
      data.drive_active ? "Drive: active" : "Drive: stopped",
      data.drive_active ? "warn" : ""
    );

    setBadge(
      "estopStatus",
      data.emergency_stop ? "Emergency stop: ACTIVE" : "Emergency stop: released",
      data.emergency_stop ? "danger" : "ok"
    );

    const health = data.health || {};

    if (typeof health.temperature_c === "number") {
      const tempStyle = health.temperature_c >= 80 ? "danger" :
                        health.temperature_c >= 70 ? "warn" : "ok";
      setBadge("tempStatus", `Pi temp: ${health.temperature_c.toFixed(1)} °C`, tempStyle);
    } else {
      setBadge("tempStatus", "Pi temp: unavailable", "warn");
    }

    if (typeof health.cpu_percent === "number") {
      const cpuStyle = health.cpu_percent >= 85 ? "danger" :
                       health.cpu_percent >= 70 ? "warn" : "ok";
      setBadge("cpuStatus", `CPU: ${health.cpu_percent.toFixed(0)}%`, cpuStyle);
    } else {
      setBadge("cpuStatus", "CPU: checking...", "warn");
    }

    if (typeof health.ram_percent === "number") {
      const ramStyle = health.ram_percent >= 90 ? "danger" :
                       health.ram_percent >= 75 ? "warn" : "ok";
      setBadge(
        "ramStatus",
        `RAM: ${health.ram_percent.toFixed(0)}% (${health.ram_used_mb || "?"}/${health.ram_total_mb || "?"} MB)`,
        ramStyle
      );
    } else {
      setBadge("ramStatus", "RAM: unavailable", "warn");
    }

    if (typeof health.disk_percent === "number") {
      const diskStyle = health.disk_percent >= 92 ? "danger" :
                        health.disk_percent >= 80 ? "warn" : "ok";
      setBadge(
        "diskStatus",
        `Disk: ${health.disk_percent.toFixed(0)}% used • ${health.disk_free_gb || "?"} GB free`,
        diskStyle
      );
    } else {
      setBadge("diskStatus", "Disk: unavailable", "warn");
    }

    setBadge(
      "uptimeStatus",
      `Uptime: ${health.uptime || "checking..."}`,
      "ok"
    );

    const throttle = String(health.throttle || "unavailable");
    const throttleStyle = throttle === "0x0" ? "ok" :
                          throttle === "unavailable" ? "warn" : "danger";
    setBadge(
      "throttleStatus",
      throttle === "0x0" ? "Power: normal" : `Power: ${throttle}`,
      throttleStyle
    );

    renderDetections(data.objects || []);
    document.getElementById("detectionSummary").textContent = data.summary || "No detections yet.";
  } catch (_) {
    setBadge("cameraStatus", "Dashboard: waiting for ROS...", "warn");
  }
}

setInterval(refreshStatus, 1000);
refreshStatus();
</script>
</body>
</html>
"""


class AlphaDashboard(Node):
    def __init__(self) -> None:
        super().__init__("omni_voice_server")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5000)
        self.declare_parameter(
            "camera_topic",
            "/perception/annotated_image/compressed",
        )
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("web_velocity_topic", "/web_vel")
        self.declare_parameter("voice_text_topic", "/omni/voice/text")
        self.declare_parameter("emergency_stop_topic", "/emergency_stop")

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
        self._latest_camera: Optional[bytes] = None
        self._latest_camera_time = 0.0
        self._objects: List[Dict[str, Any]] = []
        self._detections_time = 0.0

        self._linear = 0.0
        self._angular = 0.0
        self._drive_deadline = 0.0
        self._drive_active = False
        self._last_motion_nonzero = False
        self._emergency_stop = False
        self._last_estop_publish = 0.0

        self._health: Dict[str, Any] = {}
        self._last_health_update = 0.0
        self._previous_cpu_total: Optional[int] = None
        self._previous_cpu_idle: Optional[int] = None
        self._update_health()

        self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self._camera_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            self.detections_topic,
            self._detections_callback,
            10,
        )

        self.web_velocity_pub = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("web_velocity_topic").value),
            10,
        )
        self.voice_text_pub = self.create_publisher(
            String,
            str(self.get_parameter("voice_text_topic").value),
            10,
        )
        self.estop_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            10,
        )

        self.create_timer(0.10, self._control_tick)

        self.get_logger().info(
            "Alpha Robot Command Center ready. "
            f"camera={self.camera_topic}, detections={self.detections_topic}"
        )

    def _read_temperature(self) -> Optional[float]:
        try:
            raw = open("/sys/class/thermal/thermal_zone0/temp", "r").read().strip()
            return round(float(raw) / 1000.0, 1)
        except Exception:
            return None

    def _read_cpu_percent(self) -> Optional[float]:
        try:
            fields = open("/proc/stat", "r").readline().split()[1:]
            values = [int(value) for value in fields[:8]]
            total = sum(values)
            idle = values[3] + values[4]

            if self._previous_cpu_total is None or self._previous_cpu_idle is None:
                self._previous_cpu_total = total
                self._previous_cpu_idle = idle
                return None

            delta_total = total - self._previous_cpu_total
            delta_idle = idle - self._previous_cpu_idle

            self._previous_cpu_total = total
            self._previous_cpu_idle = idle

            if delta_total <= 0:
                return None

            return round(100.0 * (1.0 - (delta_idle / delta_total)), 1)
        except Exception:
            return None

    @staticmethod
    def _read_memory() -> Tuple[Optional[int], Optional[int], Optional[float]]:
        try:
            values: Dict[str, int] = {}
            for line in open("/proc/meminfo", "r"):
                key, raw_value, *_ = line.split()
                values[key.rstrip(":")] = int(raw_value)

            total_kb = values.get("MemTotal", 0)
            available_kb = values.get("MemAvailable", 0)

            if total_kb <= 0:
                return None, None, None

            used_kb = max(0, total_kb - available_kb)
            used_mb = round(used_kb / 1024)
            total_mb = round(total_kb / 1024)
            percent = round((used_kb / total_kb) * 100.0, 1)

            return used_mb, total_mb, percent
        except Exception:
            return None, None, None

    @staticmethod
    def _read_disk() -> Tuple[Optional[float], Optional[float]]:
        try:
            usage = shutil.disk_usage("/")
            free_gb = round(usage.free / (1024 ** 3), 1)
            used_percent = round((usage.used / usage.total) * 100.0, 1)
            return free_gb, used_percent
        except Exception:
            return None, None

    @staticmethod
    def _read_uptime() -> str:
        try:
            total_seconds = int(float(open("/proc/uptime", "r").read().split()[0]))
            days, remainder = divmod(total_seconds, 86400)
            hours, remainder = divmod(remainder, 3600)
            minutes = remainder // 60

            if days:
                return f"{days}d {hours}h {minutes}m"
            return f"{hours}h {minutes}m"
        except Exception:
            return "unavailable"

    @staticmethod
    def _read_throttle() -> str:
        try:
            result = subprocess.run(
                ["vcgencmd", "get_throttled"],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            output = result.stdout.strip()

            if "=" in output:
                return output.split("=", 1)[1].strip()

            return output or "unavailable"
        except Exception:
            return "unavailable"

    def _update_health(self) -> None:
        temperature_c = self._read_temperature()
        cpu_percent = self._read_cpu_percent()
        ram_used_mb, ram_total_mb, ram_percent = self._read_memory()
        disk_free_gb, disk_percent = self._read_disk()

        health = {
            "temperature_c": temperature_c,
            "cpu_percent": cpu_percent,
            "ram_used_mb": ram_used_mb,
            "ram_total_mb": ram_total_mb,
            "ram_percent": ram_percent,
            "disk_free_gb": disk_free_gb,
            "disk_percent": disk_percent,
            "uptime": self._read_uptime(),
            "throttle": self._read_throttle(),
        }

        with self._lock:
            self._health = health
            self._last_health_update = time.monotonic()

    def _camera_callback(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_camera = bytes(msg.data)
            self._latest_camera_time = time.time()

    def _detections_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or "{}")
        except Exception:
            payload = {}

        raw_objects = payload.get("objects", []) if isinstance(payload, dict) else []
        if not isinstance(raw_objects, list):
            raw_objects = []

        cleaned: List[Dict[str, Any]] = []
        for obj in raw_objects[:15]:
            if not isinstance(obj, dict):
                continue
            try:
                score = float(obj.get("score", 0.0) or 0.0)
            except (TypeError, ValueError):
                score = 0.0

            cleaned.append(
                {
                    "label": str(obj.get("label", "unknown")),
                    "score": max(0.0, min(1.0, score)),
                    "pose": obj.get("pose", {}),
                }
            )

        with self._lock:
            self._objects = cleaned
            self._detections_time = time.time()

    @staticmethod
    def _clamp(value: Any, low: float, high: float, fallback: float) -> float:
        try:
            return max(low, min(high, float(value)))
        except (TypeError, ValueError):
            return fallback

    def set_drive(self, action: str, linear_speed: Any, angular_speed: Any) -> Tuple[bool, str]:
        linear_speed = self._clamp(linear_speed, 0.04, 0.16, 0.10)
        angular_speed = self._clamp(angular_speed, 0.20, 1.70, 0.45)

        motion_map = {
            "forward": (linear_speed, 0.0),
            "back": (-linear_speed, 0.0),
            "left": (0.0, angular_speed),
            "right": (0.0, -angular_speed),
        }

        if action == "stop":
            self.stop_drive()
            return True, "Stop command sent."

        if action not in motion_map:
            return False, "Unknown drive action."

        with self._lock:
            if self._emergency_stop:
                return False, "Emergency stop is active."
            self._linear, self._angular = motion_map[action]
            self._drive_deadline = time.monotonic() + 0.45
            self._drive_active = True

        return True, f"{action} command active."

    def stop_drive(self) -> None:
        with self._lock:
            self._linear = 0.0
            self._angular = 0.0
            self._drive_deadline = 0.0
            self._drive_active = False

        self._publish_velocity(0.0, 0.0)

    def set_emergency_stop(self, enabled: bool) -> None:
        with self._lock:
            self._emergency_stop = bool(enabled)
            self._linear = 0.0
            self._angular = 0.0
            self._drive_deadline = 0.0
            self._drive_active = False

        self._publish_velocity(0.0, 0.0)
        self.estop_pub.publish(Bool(data=bool(enabled)))

    def publish_voice_command(self, text: str) -> Tuple[bool, str]:
        clean = " ".join(str(text).strip().split())
        if not clean:
            return False, "Command is empty."
        if len(clean) > 180:
            return False, "Command is too long."

        self.voice_text_pub.publish(String(data=clean.lower()))
        self.get_logger().info(f"Dashboard command: {clean}")
        return True, "Command sent."

    def _publish_velocity(self, linear: float, angular: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.web_velocity_pub.publish(msg)

    def _control_tick(self) -> None:
        now = time.monotonic()

        with self._lock:
            emergency_stop = self._emergency_stop
            active = (
                self._drive_active
                and not emergency_stop
                and now < self._drive_deadline
            )

            linear = self._linear if active else 0.0
            angular = self._angular if active else 0.0

            if self._drive_active and now >= self._drive_deadline:
                self._drive_active = False
                self._linear = 0.0
                self._angular = 0.0

            send_final_stop = (not active) and self._last_motion_nonzero
            if active:
                self._last_motion_nonzero = True
            elif send_final_stop:
                self._last_motion_nonzero = False

        if active:
            self._publish_velocity(linear, angular)
        elif send_final_stop:
            self._publish_velocity(0.0, 0.0)

        if (now - self._last_estop_publish) >= 0.20:
            self.estop_pub.publish(Bool(data=emergency_stop))
            self._last_estop_publish = now

        if (now - self._last_health_update) >= 2.0:
            self._update_health()

    def get_camera(self) -> Optional[bytes]:
        with self._lock:
            return None if self._latest_camera is None else bytes(self._latest_camera)

    def status(self) -> Dict[str, Any]:
        with self._lock:
            camera_age = None
            if self._latest_camera_time > 0:
                camera_age = round(max(0.0, time.time() - self._latest_camera_time), 2)

            detections_age = None
            if self._detections_time > 0:
                detections_age = round(max(0.0, time.time() - self._detections_time), 2)

            objects = list(self._objects)
            health = dict(self._health)
            camera_ready = self._latest_camera is not None
            drive_active = self._drive_active and time.monotonic() < self._drive_deadline
            emergency_stop = self._emergency_stop

        labels = [str(obj.get("label", "unknown")) for obj in objects[:8]]
        summary = "No detections yet."
        if labels:
            summary = f"{len(objects)} object(s): " + ", ".join(labels)

        return {
            "camera_ready": camera_ready,
            "camera_age_sec": camera_age,
            "detections_age_sec": detections_age,
            "count": len(objects),
            "objects": objects,
            "summary": summary,
            "drive_active": drive_active,
            "emergency_stop": emergency_stop,
            "health": health,
        }


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/stream.mjpg")
def stream():
    def frame_generator():
        while True:
            if dashboard is None:
                time.sleep(0.10)
                continue

            frame = dashboard.get_camera()
            if frame is None:
                time.sleep(0.10)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8")
                + frame
                + b"\r\n"
            )
            time.sleep(0.10)

    return Response(
        frame_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/status")
def api_status():
    if dashboard is None:
        return jsonify({"message": "Dashboard is starting."}), 503
    return jsonify(dashboard.status())


@app.route("/api/drive", methods=["POST"])
def api_drive():
    if dashboard is None:
        return jsonify({"message": "Dashboard is starting."}), 503

    payload = request.get_json(silent=True) or {}
    success, message = dashboard.set_drive(
        str(payload.get("action", "")).lower(),
        payload.get("linear_speed", 0.10),
        payload.get("angular_speed", 0.45),
    )
    return jsonify({"ok": success, "message": message}), 200 if success else 400


@app.route("/api/command", methods=["POST"])
def api_command():
    if dashboard is None:
        return jsonify({"message": "Dashboard is starting."}), 503

    payload = request.get_json(silent=True) or {}
    success, message = dashboard.publish_voice_command(payload.get("text", ""))
    return jsonify({"ok": success, "message": message}), 200 if success else 400


@app.route("/api/estop", methods=["POST"])
def api_estop():
    if dashboard is None:
        return jsonify({"message": "Dashboard is starting."}), 503

    payload = request.get_json(silent=True) or {}
    enabled = bool(payload.get("enabled", True))
    dashboard.set_emergency_stop(enabled)
    return jsonify(
        {
            "ok": True,
            "message": "Emergency stop active." if enabled else "Emergency stop released.",
        }
    )


def main() -> None:
    global dashboard

    rclpy.init()
    dashboard = AlphaDashboard()

    web_thread = threading.Thread(
        target=lambda: app.run(
            host=dashboard.host,
            port=dashboard.port,
            ssl_context="adhoc",
            threaded=True,
            use_reloader=False,
        ),
        daemon=True,
    )
    web_thread.start()

    dashboard.get_logger().info(
        f"Open https://<PI-IP>:{dashboard.port} in your phone browser"
    )

    try:
        rclpy.spin(dashboard)
    except KeyboardInterrupt:
        pass
    finally:
        if dashboard is not None:
            dashboard.stop_drive()
            dashboard.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

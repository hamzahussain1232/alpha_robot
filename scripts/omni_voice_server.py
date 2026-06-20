#!/usr/bin/env python3
import argparse
import threading
from pathlib import Path

from flask import Flask, jsonify, render_template_string, request

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String


app = Flask(__name__)
node = None


HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Dashboard</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b1220;
      --panel: #111a2d;
      --panel-2: #162238;
      --text: #e5eefb;
      --muted: #93a4bf;
      --accent: #4ade80;
      --accent-2: #60a5fa;
      --danger: #ef4444;
      --border: rgba(148, 163, 184, 0.18);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background:
        radial-gradient(circle at top, rgba(96, 165, 250, 0.12), transparent 34%),
        linear-gradient(180deg, #09101d, var(--bg));
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    .wrap {
      max-width: 980px;
      margin: 0 auto;
      padding: 16px;
    }
    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      padding: 6px 2px 16px;
    }
    h1 {
      margin: 0;
      font-size: 22px;
      letter-spacing: 0;
    }
    .status {
      color: var(--muted);
      font-size: 13px;
      text-align: right;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 14px;
    }
    .panel {
      background: rgba(17, 26, 45, 0.92);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px;
    }
    .section-title {
      font-size: 14px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin: 0 0 12px;
    }
    .row {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 10px;
      align-items: center;
    }
    input[type="text"] {
      width: 100%;
      min-width: 0;
      border: 1px solid var(--border);
      background: var(--panel-2);
      color: var(--text);
      border-radius: 10px;
      padding: 12px 14px;
      font-size: 16px;
      outline: none;
    }
    button {
      border: 0;
      border-radius: 10px;
      padding: 12px 14px;
      font-size: 16px;
      font-weight: 650;
      color: white;
      background: var(--panel-2);
      cursor: pointer;
      touch-action: manipulation;
      user-select: none;
    }
    button:active { transform: translateY(1px); }
    .mic {
      width: 100%;
      margin-top: 10px;
      background: linear-gradient(180deg, #16a34a, #15803d);
    }
    .mic.listening {
      background: linear-gradient(180deg, #dc2626, #b91c1c);
    }
    .hint {
      margin-top: 10px;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.45;
    }
    .sliders {
      display: grid;
      grid-template-columns: 1fr;
      gap: 12px;
      margin-bottom: 14px;
    }
    .slider-line {
      display: grid;
      grid-template-columns: 96px 1fr 88px;
      gap: 10px;
      align-items: center;
    }
    .slider-line label {
      color: var(--muted);
      font-size: 14px;
    }
    .slider-line input[type="range"] {
      width: 100%;
    }
    .value {
      text-align: right;
      font-variant-numeric: tabular-nums;
      color: var(--text);
    }
    .pad {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 10px;
      max-width: 420px;
      margin: 0 auto;
    }
    .pad button {
      min-height: 72px;
      font-size: 22px;
      background: #1b2944;
      border: 1px solid var(--border);
    }
    .pad .stop {
      background: linear-gradient(180deg, #ef4444, #dc2626);
      font-size: 18px;
    }
    .pad .spacer {
      visibility: hidden;
    }
    .quick {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-top: 12px;
    }
    .quick button {
      flex: 1 1 140px;
      background: #19314b;
      border: 1px solid var(--border);
    }
    .voice-readout {
      margin-top: 10px;
      color: var(--muted);
      font-size: 13px;
      word-break: break-word;
    }
    @media (min-width: 760px) {
      .grid { grid-template-columns: 1fr 1fr; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="topbar">
      <h1>Robot Dashboard</h1>
      <div class="status" id="status">Ready</div>
    </div>

    <div class="grid">
      <section class="panel">
        <div class="section-title">Voice</div>
        <div class="row">
          <input id="voiceText" type="text" placeholder="Say or type a command">
          <button id="sendVoice">Send</button>
        </div>
        <button id="micBtn" class="mic">Mic</button>
        <div class="hint">
          Voice goes to the robot command topic. Use it for move, bring, fetch, go to, stop, and other commands you add later.
        </div>
        <div class="voice-readout" id="voiceReadout"></div>
      </section>

      <section class="panel">
        <div class="section-title">Drive</div>
        <div class="sliders">
          <div class="slider-line">
            <label for="linearSpeed">Forward</label>
            <input id="linearSpeed" type="range" min="0.03" max="0.25" step="0.01" value="0.10">
            <div class="value" id="linearValue">0.10 m/s</div>
          </div>
          <div class="slider-line">
            <label for="angularSpeed">Turn</label>
            <input id="angularSpeed" type="range" min="0.10" max="1.20" step="0.05" value="0.40">
            <div class="value" id="angularValue">0.40 rad/s</div>
          </div>
        </div>
        <div class="pad">
          <div class="spacer">.</div>
          <button data-drive="forward">▲</button>
          <div class="spacer">.</div>
          <button data-drive="left">◀</button>
          <button class="stop" id="stopBtn">STOP</button>
          <button data-drive="right">▶</button>
          <div class="spacer">.</div>
          <button data-drive="backward">▼</button>
          <div class="spacer">.</div>
        </div>
        <div class="quick">
          <button data-preset="slow">Slow</button>
          <button data-preset="map">Map</button>
          <button data-preset="normal">Normal</button>
        </div>
      </section>

      <section class="panel">
        <div class="section-title">System</div>
        <div style="font-size: 28px; font-weight: 700;" id="piTempValue">--.- °C</div>
        <div class="hint" id="piTempSource">Reading Pi temperature...</div>
      </section>
    </div>
  </div>

  <script>
    const statusText = document.getElementById('status');
    const voiceReadout = document.getElementById('voiceReadout');
    const voiceInput = document.getElementById('voiceText');
    const sendVoice = document.getElementById('sendVoice');
    const micBtn = document.getElementById('micBtn');
    const linearSlider = document.getElementById('linearSpeed');
    const angularSlider = document.getElementById('angularSpeed');
    const linearValue = document.getElementById('linearValue');
    const angularValue = document.getElementById('angularValue');
    const stopBtn = document.getElementById('stopBtn');
    const piTempValue = document.getElementById('piTempValue');
    const piTempSource = document.getElementById('piTempSource');
    let holdTimer = null;
    let activeDrive = null;

    function setStatus(text) {
      statusText.textContent = text;
    }

    function updateSliderLabels() {
      linearValue.textContent = Number(linearSlider.value).toFixed(2) + ' m/s';
      angularValue.textContent = Number(angularSlider.value).toFixed(2) + ' rad/s';
    }

    function postJson(path, payload) {
      return fetch(path, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload)
      });
    }

    async function sendVoiceText(text) {
      const cleaned = (text || '').trim();
      if (!cleaned) return;
      voiceReadout.textContent = 'Sending: "' + cleaned + '"';
      setStatus('Sending voice command');
      await postJson('/api/voice', { text: cleaned });
      setStatus('Ready');
    }

    async function sendDrive(direction) {
      const payload = {
        direction: direction,
        linear: Number(linearSlider.value),
        angular: Number(angularSlider.value)
      };
      await postJson('/api/drive', payload);
    }

    async function refreshPiStatus() {
      try {
        const response = await fetch('/api/status');
        const data = await response.json();
        if (!data.ok) throw new Error(data.error || 'status unavailable');

        if (data.pi_temp_c === null || data.pi_temp_c === undefined) {
          piTempValue.textContent = '--.- °C';
          piTempSource.textContent = 'Pi temperature unavailable';
        } else {
          piTempValue.textContent = Number(data.pi_temp_c).toFixed(1) + ' °C';
          piTempSource.textContent = 'Source: ' + (data.pi_temp_source || 'unknown');
        }
      } catch (error) {
        piTempValue.textContent = '--.- °C';
        piTempSource.textContent = 'Unable to read Pi temperature';
      }
    }

    function stopDrive() {
      if (holdTimer) {
        clearInterval(holdTimer);
        holdTimer = null;
      }
      activeDrive = null;
      sendDrive('stop');
    }

    function startDrive(direction) {
      activeDrive = direction;
      sendDrive(direction);
      if (holdTimer) clearInterval(holdTimer);
      holdTimer = setInterval(() => {
        if (activeDrive) sendDrive(activeDrive);
      }, 120);
    }

    async function applyPreset(name) {
      if (name === 'slow') {
        linearSlider.value = '0.05';
        angularSlider.value = '0.20';
      } else if (name === 'map') {
        linearSlider.value = '0.08';
        angularSlider.value = '0.35';
      } else {
        linearSlider.value = '0.12';
        angularSlider.value = '0.50';
      }
      updateSliderLabels();
      setStatus('Preset applied: ' + name);
      await sendDrive('stop');
    }

    sendVoice.onclick = () => sendVoiceText(voiceInput.value);
    voiceInput.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') {
        event.preventDefault();
        sendVoice.click();
      }
    });
    linearSlider.oninput = updateSliderLabels;
    angularSlider.oninput = updateSliderLabels;
    stopBtn.onclick = stopDrive;

    document.querySelectorAll('[data-drive]').forEach((button) => {
      button.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        startDrive(button.dataset.drive);
      });
      button.addEventListener('pointerup', stopDrive);
      button.addEventListener('pointercancel', stopDrive);
      button.addEventListener('pointerleave', () => {
        if (activeDrive === button.dataset.drive) stopDrive();
      });
    });

    document.querySelectorAll('[data-preset]').forEach((button) => {
      button.addEventListener('click', () => applyPreset(button.dataset.preset));
    });

    const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SpeechRecognition) {
      micBtn.textContent = 'Mic unavailable';
      micBtn.disabled = true;
    } else {
      const recognition = new SpeechRecognition();
      recognition.continuous = false;
      recognition.lang = 'en-US';

      micBtn.onclick = () => {
        recognition.start();
        micBtn.classList.add('listening');
        micBtn.textContent = 'Listening...';
      };

      recognition.onresult = (event) => {
        const text = event.results[0][0].transcript;
        voiceInput.value = text;
        sendVoiceText(text);
      };

      recognition.onend = () => {
        micBtn.classList.remove('listening');
        micBtn.textContent = 'Mic';
      };
    }

    window.addEventListener('blur', stopDrive);
    window.addEventListener('pagehide', stopDrive);

    updateSliderLabels();
    refreshPiStatus();
    setInterval(refreshPiStatus, 2000);
  </script>
</body>
</html>
"""


class OmniVoiceServer(Node):
    def __init__(self):
        super().__init__('omni_voice_server')
        self.declare_parameter('voice_topic', '/voice/text')
        self.declare_parameter('drive_topic', '/web_vel')
        self.declare_parameter('drive_frame_id', 'base_link')
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 1.2)

        self.voice_topic = str(self.get_parameter('voice_topic').value)
        self.drive_topic = str(self.get_parameter('drive_topic').value)
        self.drive_frame_id = str(self.get_parameter('drive_frame_id').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.voice_pub = self.create_publisher(String, self.voice_topic, 10)
        self.drive_pub = self.create_publisher(TwistStamped, self.drive_topic, 10)

    def read_pi_temperature_celsius(self):
        thermal_root = Path('/sys/class/thermal')
        preferred_types = ('cpu_thermal', 'soc_thermal', 'x86_pkg_temp')

        thermal_zones = sorted(thermal_root.glob('thermal_zone*'))
        if not thermal_zones:
            return None, 'unavailable'

        for zone in thermal_zones:
            type_path = zone / 'type'
            temp_path = zone / 'temp'
            try:
                zone_type = type_path.read_text().strip()
                if zone_type in preferred_types and temp_path.exists():
                    raw_temp = temp_path.read_text().strip()
                    return float(raw_temp) / 1000.0, zone_type
            except (OSError, ValueError):
                continue

        for zone in thermal_zones:
            temp_path = zone / 'temp'
            try:
                if temp_path.exists():
                    raw_temp = temp_path.read_text().strip()
                    return float(raw_temp) / 1000.0, zone.name
            except (OSError, ValueError):
                continue

        return None, 'unavailable'

    def publish_voice(self, text: str):
        cleaned = ' '.join((text or '').strip().split())
        if not cleaned:
            return False
        msg = String()
        msg.data = cleaned.lower()
        self.voice_pub.publish(msg)
        self.get_logger().info(f"Voice command sent to {self.voice_topic}: {msg.data!r}")
        return True

    def publish_drive(self, direction: str, linear: float, angular: float):
        direction = (direction or '').strip().lower()
        linear = max(0.0, min(self.max_linear_speed, float(linear)))
        angular = max(0.0, min(self.max_angular_speed, float(angular)))

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.drive_frame_id

        if direction == 'forward':
            twist.twist.linear.x = linear
        elif direction == 'backward':
            twist.twist.linear.x = -linear
        elif direction == 'left':
            twist.twist.angular.z = angular
        elif direction == 'right':
            twist.twist.angular.z = -angular
        elif direction == 'stop':
            pass
        else:
            return False

        self.drive_pub.publish(twist)
        if direction == 'stop':
            self.get_logger().info(f"Drive stop sent to {self.drive_topic}")
        else:
            self.get_logger().info(
                f"Drive {direction} sent to {self.drive_topic}: v={twist.twist.linear.x:.3f}, "
                f"w={twist.twist.angular.z:.3f}"
            )
        return True


@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route('/api/voice', methods=['POST'])
def api_voice():
    if node is None:
        return jsonify({'ok': False, 'error': 'node not ready'}), 503

    payload = request.get_json(silent=True) or {}
    text = payload.get('text')
    if text is None:
        text = request.form.get('text', '')
    if node.publish_voice(text):
        return jsonify({'ok': True})
    return jsonify({'ok': False, 'error': 'empty text'}), 400


@app.route('/api/drive', methods=['POST'])
def api_drive():
    if node is None:
        return jsonify({'ok': False, 'error': 'node not ready'}), 503

    payload = request.get_json(silent=True) or {}
    direction = payload.get('direction', '')
    linear = payload.get('linear', 0.0)
    angular = payload.get('angular', 0.0)
    if node.publish_drive(direction, linear, angular):
        return jsonify({'ok': True})
    return jsonify({'ok': False, 'error': 'invalid direction'}), 400


@app.route('/api/status', methods=['GET'])
def api_status():
    if node is None:
        return jsonify({'ok': False, 'error': 'node not ready'}), 503

    temp_c, source = node.read_pi_temperature_celsius()
    return jsonify({
        'ok': True,
        'pi_temp_c': temp_c,
        'pi_temp_source': source,
    })


def _start_flask(host: str, port: int):
    app.run(host=host, port=port, ssl_context='adhoc', use_reloader=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5000)
    args = parser.parse_args()

    global node
    rclpy.init()
    node = OmniVoiceServer()

    flask_thread = threading.Thread(target=_start_flask, args=(args.host, args.port), daemon=True)
    flask_thread.start()

    node.get_logger().info('Secure Mobile Dashboard running!')
    node.get_logger().info(f'Open https://<RaspberryPi-IP>:{args.port} in your phone browser')
    node.get_logger().info(f'Voice topic: {node.voice_topic}')
    node.get_logger().info(f'Drive topic: {node.drive_topic}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()

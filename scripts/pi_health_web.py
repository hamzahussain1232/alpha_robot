#!/usr/bin/env python3
import argparse
import json
import os
import shutil
import socket
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


WATCH_TARGETS = [
    ("serial_diffdrive", "serial_diffdrive_node.py"),
    ("wasd_teleop", "wasd_teleop.py"),
    ("slam_toolbox", "slam_toolbox"),
    ("pi_camera", "camera_node"),
    ("detector", "object_detector_node.py"),
    ("arm_driver", "nano_arm_driver.py"),
    ("web_video", "web_video_server"),
]

THROTTLE_FLAGS = {
    0: "Under-voltage now",
    1: "ARM freq capped now",
    2: "Currently throttled",
    3: "Soft temp limit now",
    16: "Under-voltage occurred",
    17: "ARM freq capped occurred",
    18: "Throttling occurred",
    19: "Soft temp limit occurred",
}

HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Alpha Dashboard</title>
  <style>
    :root {
      --bg: #0f1419;
      --panel: #162029;
      --panel2: #1b2a36;
      --text: #ebf2f7;
      --muted: #9db0bf;
      --ok: #6dd3a0;
      --warn: #f4c86a;
      --bad: #ef7d7d;
      --accent: #59b3ff;
      --border: #2a3d4c;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "DejaVu Sans", "Noto Sans", sans-serif;
      background: radial-gradient(circle at top, #1b2730, #0f1419 55%);
      color: var(--text);
    }
    .wrap {
      max-width: 1180px;
      margin: 0 auto;
      padding: 24px 18px 40px;
    }
    .hero {
      display: flex;
      justify-content: space-between;
      align-items: end;
      gap: 16px;
      margin-bottom: 18px;
    }
    .hero h1 {
      margin: 0;
      font-size: 2rem;
      font-weight: 700;
      letter-spacing: 0.02em;
    }
    .hero p {
      margin: 8px 0 0;
      color: var(--muted);
    }
    .status-pill {
      padding: 10px 14px;
      border-radius: 999px;
      background: rgba(89, 179, 255, 0.12);
      color: var(--accent);
      border: 1px solid rgba(89, 179, 255, 0.25);
      font-size: 0.95rem;
      white-space: nowrap;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 14px;
      margin-bottom: 18px;
    }
    .card {
      background: linear-gradient(180deg, var(--panel2), var(--panel));
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 16px;
      box-shadow: 0 10px 24px rgba(0, 0, 0, 0.18);
    }
    .card h2 {
      margin: 0 0 8px;
      font-size: 0.95rem;
      color: var(--muted);
      font-weight: 600;
      text-transform: uppercase;
      letter-spacing: 0.06em;
    }
    .value {
      font-size: 1.9rem;
      font-weight: 700;
    }
    .sub {
      margin-top: 8px;
      color: var(--muted);
      font-size: 0.95rem;
    }
    .section {
      margin-top: 14px;
      display: grid;
      grid-template-columns: 1.1fr 1fr;
      gap: 14px;
    }
    .table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 8px;
    }
    .table th,
    .table td {
      text-align: left;
      padding: 10px 6px;
      border-bottom: 1px solid rgba(255,255,255,0.06);
      font-size: 0.95rem;
    }
    .badge {
      display: inline-block;
      padding: 4px 10px;
      border-radius: 999px;
      font-size: 0.85rem;
      font-weight: 700;
    }
    .ok { background: rgba(109, 211, 160, 0.14); color: var(--ok); }
    .warn { background: rgba(244, 200, 106, 0.14); color: var(--warn); }
    .bad { background: rgba(239, 125, 125, 0.14); color: var(--bad); }
    .muted { color: var(--muted); }
    .flags {
      margin: 10px 0 0;
      padding-left: 18px;
      color: var(--muted);
    }
    .footer {
      margin-top: 18px;
      color: var(--muted);
      font-size: 0.9rem;
    }
    @media (max-width: 900px) {
      .section { grid-template-columns: 1fr; }
      .hero { flex-direction: column; align-items: start; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <div>
        <h1>Robot Alpha Dashboard</h1>
        <p>Live Pi 5 health, robot services, and network condition in one place.</p>
      </div>
      <div class="status-pill" id="updated">Connecting...</div>
    </div>

    <div class="grid">
      <div class="card">
        <h2>CPU</h2>
        <div class="value" id="cpu">--</div>
        <div class="sub" id="load">Load average --</div>
      </div>
      <div class="card">
        <h2>Temperature</h2>
        <div class="value" id="temp">--</div>
        <div class="sub" id="uptime">Uptime --</div>
      </div>
      <div class="card">
        <h2>Memory</h2>
        <div class="value" id="memory">--</div>
        <div class="sub" id="swap">Swap --</div>
      </div>
      <div class="card">
        <h2>Disk</h2>
        <div class="value" id="disk">--</div>
        <div class="sub" id="host">Host --</div>
      </div>
    </div>

    <div class="section">
      <div class="card">
        <h2>Robot Processes</h2>
        <table class="table">
          <thead><tr><th>Service</th><th>Status</th><th>Details</th></tr></thead>
          <tbody id="processes"></tbody>
        </table>
      </div>
      <div class="card">
        <h2>System Health</h2>
        <div class="sub"><strong>Network</strong></div>
        <table class="table">
          <thead><tr><th>Interface</th><th>RX</th><th>TX</th></tr></thead>
          <tbody id="network"></tbody>
        </table>
        <div class="sub" style="margin-top: 10px;"><strong>Throttle / Undervoltage</strong></div>
        <div class="sub" id="throttle_raw">raw: --</div>
        <ul class="flags" id="flags"></ul>
      </div>
    </div>

    <div class="footer">Open this page from your laptop browser while the robot runs on the Pi.</div>
  </div>

  <script>
    function humanBytes(bytes) {
      const units = ['B', 'KB', 'MB', 'GB', 'TB'];
      let value = Number(bytes || 0);
      let unit = 0;
      while (value >= 1024 && unit < units.length - 1) {
        value /= 1024;
        unit += 1;
      }
      return `${value.toFixed(1)} ${units[unit]}`;
    }

    function badge(status) {
      if (status === 'RUNNING') return '<span class="badge ok">RUNNING</span>';
      if (status === 'WARNING') return '<span class="badge warn">WARNING</span>';
      return '<span class="badge bad">STOPPED</span>';
    }

    async function refresh() {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();

        document.getElementById('updated').textContent = `Updated ${data.updated_hms}`;
        document.getElementById('cpu').textContent = `${data.cpu_percent.toFixed(1)}%`;
        document.getElementById('load').textContent =
          `Load average ${data.loadavg.map(v => v.toFixed(2)).join('  ')}`;
        document.getElementById('temp').textContent =
          data.temp_c === null ? 'Unavailable' : `${data.temp_c.toFixed(1)} C`;
        document.getElementById('uptime').textContent = `Uptime ${data.uptime}`;
        document.getElementById('memory').textContent =
          `${humanBytes(data.memory.used_bytes)} / ${humanBytes(data.memory.total_bytes)}`;
        document.getElementById('swap').textContent =
          `Swap ${humanBytes(data.swap.used_bytes)} / ${humanBytes(data.swap.total_bytes)}`;
        document.getElementById('disk').textContent = `${data.disk.percent.toFixed(1)}% used`;
        document.getElementById('host').textContent =
          `${data.hostname}  ·  ${humanBytes(data.disk.used_bytes)} / ${humanBytes(data.disk.total_bytes)}`;

        const pbody = document.getElementById('processes');
        pbody.innerHTML = '';
        data.processes.forEach(proc => {
          const tr = document.createElement('tr');
          tr.innerHTML = `<td>${proc.label}</td><td>${badge(proc.state)}</td><td class="muted">${proc.details}</td>`;
          pbody.appendChild(tr);
        });

        const nbody = document.getElementById('network');
        nbody.innerHTML = '';
        data.network.forEach(net => {
          const tr = document.createElement('tr');
          tr.innerHTML = `<td>${net.interface}</td><td>${humanBytes(net.rx_per_sec)}/s</td><td>${humanBytes(net.tx_per_sec)}/s</td>`;
          nbody.appendChild(tr);
        });
        if (!data.network.length) {
          const tr = document.createElement('tr');
          tr.innerHTML = '<td colspan="3" class="muted">No active network interfaces</td>';
          nbody.appendChild(tr);
        }

        document.getElementById('throttle_raw').textContent = `raw: ${data.throttle.raw}`;
        const flags = document.getElementById('flags');
        flags.innerHTML = '';
        if (data.throttle.flags.length) {
          data.throttle.flags.forEach(flag => {
            const li = document.createElement('li');
            li.textContent = flag;
            flags.appendChild(li);
          });
        } else {
          const li = document.createElement('li');
          li.textContent = 'No throttle / undervoltage flags';
          flags.appendChild(li);
        }
      } catch (err) {
        document.getElementById('updated').textContent = `Disconnected: ${err}`;
      }
    }

    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>
"""


def read_proc_stat():
    with open("/proc/stat", "r", encoding="utf-8") as handle:
        fields = handle.readline().strip().split()[1:]
    values = [int(v) for v in fields[:8]]
    idle = values[3] + values[4]
    total = sum(values)
    return idle, total


def cpu_percent(previous, current):
    prev_idle, prev_total = previous
    curr_idle, curr_total = current
    total_delta = max(1, curr_total - prev_total)
    idle_delta = max(0, curr_idle - prev_idle)
    return 100.0 * (1.0 - (idle_delta / total_delta))


def read_meminfo():
    info = {}
    with open("/proc/meminfo", "r", encoding="utf-8") as handle:
        for line in handle:
            key, value = line.split(":", 1)
            info[key] = int(value.strip().split()[0])
    return info


def read_net_bytes():
    stats = {}
    with open("/proc/net/dev", "r", encoding="utf-8") as handle:
        for line in handle.readlines()[2:]:
            iface, rest = line.split(":", 1)
            iface = iface.strip()
            if iface == "lo":
                continue
            fields = rest.split()
            stats[iface] = (int(fields[0]), int(fields[8]))
    return stats


def diff_net_rates(previous, current, interval):
    rates = []
    for iface, (curr_rx, curr_tx) in current.items():
        prev_rx, prev_tx = previous.get(iface, (curr_rx, curr_tx))
        rates.append(
            {
                "interface": iface,
                "rx_per_sec": max(0.0, (curr_rx - prev_rx) / max(interval, 0.1)),
                "tx_per_sec": max(0.0, (curr_tx - prev_tx) / max(interval, 0.1)),
            }
        )
    return sorted(rates, key=lambda item: item["interface"])


def read_temperature_c():
    for path in (
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/devices/virtual/thermal/thermal_zone0/temp",
    ):
        if not os.path.exists(path):
            continue
        with open(path, "r", encoding="utf-8") as handle:
            raw = handle.read().strip()
        try:
            value = float(raw)
            return value / 1000.0 if value > 1000.0 else value
        except ValueError:
            return None
    return None


def read_uptime():
    with open("/proc/uptime", "r", encoding="utf-8") as handle:
        return float(handle.read().split()[0])


def format_duration(seconds):
    seconds = int(seconds)
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    if days > 0:
        return f"{days}d {hours:02}:{minutes:02}:{seconds:02}"
    return f"{hours:02}:{minutes:02}:{seconds:02}"


def read_throttled():
    if shutil.which("vcgencmd") is None:
        return {"raw": "unavailable", "flags": ["vcgencmd not installed"]}
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            check=True,
            capture_output=True,
            text=True,
        )
        output = result.stdout.strip()
        if "=" not in output:
            return {"raw": output, "flags": []}
        raw_value = output.split("=", 1)[1]
        value = int(raw_value, 16)
        flags = [label for bit, label in THROTTLE_FLAGS.items() if value & (1 << bit)]
        return {"raw": raw_value, "flags": flags}
    except Exception as exc:
        return {"raw": f"error:{exc}", "flags": []}


def read_processes():
    try:
        result = subprocess.run(
            ["ps", "-eo", "pid=,comm=,args="],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return []

    lines = result.stdout.splitlines()
    processes = []
    for label, token in WATCH_TARGETS:
        match = next((line.strip() for line in lines if token in line), None)
        processes.append(
            {
                "label": label,
                "state": "RUNNING" if match else "STOPPED",
                "details": match or "not running",
            }
        )
    return processes


class MetricsStore:
    def __init__(self):
        self.prev_cpu = read_proc_stat()
        self.prev_net = read_net_bytes()
        self.prev_time = time.time()

    def sample(self):
        now = time.time()
        cpu_now = read_proc_stat()
        net_now = read_net_bytes()
        interval = max(0.1, now - self.prev_time)
        mem = read_meminfo()
        disk = shutil.disk_usage("/")
        payload = {
            "updated_hms": time.strftime("%H:%M:%S", time.localtime(now)),
            "hostname": socket.gethostname(),
            "cpu_percent": cpu_percent(self.prev_cpu, cpu_now),
            "loadavg": list(os.getloadavg()),
            "temp_c": read_temperature_c(),
            "uptime": format_duration(read_uptime()),
            "memory": {
                "used_bytes": max(0, (mem.get("MemTotal", 0) - mem.get("MemAvailable", 0)) * 1024),
                "total_bytes": mem.get("MemTotal", 0) * 1024,
            },
            "swap": {
                "used_bytes": max(0, (mem.get("SwapTotal", 0) - mem.get("SwapFree", 0)) * 1024),
                "total_bytes": mem.get("SwapTotal", 0) * 1024,
            },
            "disk": {
                "used_bytes": disk.used,
                "total_bytes": disk.total,
                "percent": 100.0 * disk.used / max(1, disk.total),
            },
            "network": diff_net_rates(self.prev_net, net_now, interval),
            "throttle": read_throttled(),
            "processes": read_processes(),
        }
        self.prev_cpu = cpu_now
        self.prev_net = net_now
        self.prev_time = now
        return payload


class DashboardHandler(BaseHTTPRequestHandler):
    store = MetricsStore()

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/api/status":
            payload = json.dumps(self.store.sample()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, _format, *_args):
        return


def main():
    parser = argparse.ArgumentParser(description="Lightweight Pi web dashboard")
    parser.add_argument("--host", default="0.0.0.0", help="host interface to bind")
    parser.add_argument("--port", type=int, default=8090, help="TCP port to serve")
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"Robot Alpha dashboard running on http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()

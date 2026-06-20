#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
import time


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
            info[key] = int(value.strip().split()[0])  # kB
    return info


def read_net_bytes():
    stats = {}
    with open("/proc/net/dev", "r", encoding="utf-8") as handle:
        for line in handle.readlines()[2:]:
            iface, rest = line.split(":", 1)
            if iface.strip() == "lo":
                continue
            fields = rest.split()
            rx = int(fields[0])
            tx = int(fields[8])
            stats[iface.strip()] = (rx, tx)
    return stats


def diff_net_rates(previous, current, interval):
    rates = {}
    for iface, (curr_rx, curr_tx) in current.items():
        prev_rx, prev_tx = previous.get(iface, (curr_rx, curr_tx))
        rates[iface] = (
            max(0.0, (curr_rx - prev_rx) / max(interval, 0.1)),
            max(0.0, (curr_tx - prev_tx) / max(interval, 0.1)),
        )
    return rates


def read_temperature_c():
    candidates = [
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/devices/virtual/thermal/thermal_zone0/temp",
    ]
    for path in candidates:
        if os.path.exists(path):
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


def human_bytes(value):
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(value)
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            return f"{size:.1f} {unit}"
        size /= 1024.0
    return f"{value} B"


def read_loadavg():
    return os.getloadavg()


def read_disk():
    return shutil.disk_usage("/")


def read_throttled():
    if shutil.which("vcgencmd") is None:
        return None
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            check=True,
            capture_output=True,
            text=True,
        )
        output = result.stdout.strip()
        if "=" not in output:
            return output, []
        value_text = output.split("=", 1)[1]
        value = int(value_text, 16)
        flags = [label for bit, label in THROTTLE_FLAGS.items() if value & (1 << bit)]
        return value_text, flags
    except Exception as exc:
        return f"error:{exc}", []


def read_processes():
    try:
        result = subprocess.run(
            ["ps", "-eo", "pid=,comm=,args="],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return {}

    found = {}
    for line in result.stdout.splitlines():
        for label, token in WATCH_TARGETS:
            if token in line:
                found[label] = line.strip()
    return found


def render(cpu, mem, disk, temp_c, uptime, loadavg, net_rates, throttled, processes):
    mem_total = mem.get("MemTotal", 0) * 1024
    mem_avail = mem.get("MemAvailable", 0) * 1024
    mem_used = max(0, mem_total - mem_avail)

    swap_total = mem.get("SwapTotal", 0) * 1024
    swap_free = mem.get("SwapFree", 0) * 1024
    swap_used = max(0, swap_total - swap_free)

    disk_used = disk.used
    disk_total = disk.total
    disk_pct = (100.0 * disk_used / max(1, disk_total))

    lines = [
        "\033[2J\033[H",
        "Pi 5 Health Monitor",
        "",
        f"Uptime       : {format_duration(uptime)}",
        f"CPU load     : {cpu:5.1f}%    loadavg {loadavg[0]:.2f} {loadavg[1]:.2f} {loadavg[2]:.2f}",
        f"Temperature  : {temp_c:.1f} C" if temp_c is not None else "Temperature  : unavailable",
        f"Memory       : {human_bytes(mem_used)} / {human_bytes(mem_total)}",
        f"Swap         : {human_bytes(swap_used)} / {human_bytes(swap_total)}",
        f"Disk /       : {human_bytes(disk_used)} / {human_bytes(disk_total)}  ({disk_pct:.1f}%)",
        "",
        "Network",
    ]

    if net_rates:
        for iface, (rx_rate, tx_rate) in sorted(net_rates.items()):
            lines.append(
                f"  {iface:<10} RX {human_bytes(rx_rate)}/s   TX {human_bytes(tx_rate)}/s"
            )
    else:
        lines.append("  no active interfaces")

    lines.extend(["", "Pi throttle state"])
    if throttled is None:
        lines.append("  vcgencmd not available")
    else:
        value_text, flags = throttled
        lines.append(f"  raw: {value_text}")
        if flags:
            for flag in flags:
                lines.append(f"  - {flag}")
        else:
            lines.append("  no throttle / undervoltage flags")

    lines.extend(["", "Robot processes"])
    for label, _token in WATCH_TARGETS:
        if label in processes:
            lines.append(f"  {label:<14} RUNNING  {processes[label]}")
        else:
            lines.append(f"  {label:<14} stopped")

    lines.extend([
        "",
        "Keys",
        "  Ctrl+C to quit",
    ])
    print("\n".join(lines), flush=True)


def main():
    parser = argparse.ArgumentParser(description="Live Pi 5 health monitor")
    parser.add_argument("--interval", type=float, default=1.0, help="refresh interval in seconds")
    args = parser.parse_args()

    interval = max(0.2, args.interval)
    prev_cpu = read_proc_stat()
    prev_net = read_net_bytes()
    time.sleep(interval)

    try:
        while True:
            curr_cpu = read_proc_stat()
            curr_net = read_net_bytes()
            render(
                cpu=cpu_percent(prev_cpu, curr_cpu),
                mem=read_meminfo(),
                disk=read_disk(),
                temp_c=read_temperature_c(),
                uptime=read_uptime(),
                loadavg=read_loadavg(),
                net_rates=diff_net_rates(prev_net, curr_net, interval),
                throttled=read_throttled(),
                processes=read_processes(),
            )
            prev_cpu = curr_cpu
            prev_net = curr_net
            time.sleep(interval)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

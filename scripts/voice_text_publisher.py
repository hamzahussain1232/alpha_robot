#!/usr/bin/env python3
import argparse
import queue
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def default_phone_html_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(
            Path(get_package_share_directory("articubot_one"))
            / "assets"
            / "voice"
            / "phone_voice.html"
        )
    except Exception:
        return str(
            Path(__file__).resolve().parent.parent
            / "assets"
            / "voice"
            / "phone_voice.html"
        )


class VoiceTextPublisher(Node):
    def __init__(self):
        super().__init__("voice_text_publisher")
        self.pub = self.create_publisher(String, "/voice/text", 10)
        self._queue = queue.Queue()
        self.create_timer(0.05, self._drain_queue)

    def publish_once(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f'Published /voice/text: "{text}"')

    def enqueue(self, text: str):
        if text:
            self._queue.put(text)

    def _drain_queue(self):
        while not self._queue.empty():
            text = self._queue.get_nowait()
            self.publish_once(text)


def _make_handler(node: VoiceTextPublisher, html_content: str):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed = urlparse(self.path)
            params = parse_qs(parsed.query)
            text_vals = params.get("text", [])
            text = text_vals[0] if text_vals else ""

            if parsed.path in ("/", "/index.html", "/phone_voice.html"):
                if html_content:
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(html_content.encode("utf-8"))
                    return
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"HTML not found")
                return

            if parsed.path not in ("/say",):
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"Not found")
                return

            if not text:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b"Missing text. Use /say?text=... ")
                return

            node.enqueue(text)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")

        def log_message(self, format, *args):
            return

    return Handler


def start_http_server(node: VoiceTextPublisher, host: str, port: int, html_content: str):
    server = HTTPServer((host, port), _make_handler(node, html_content))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f"Phone text server: http://{host}:{port}/say?text=...")
    return server


def main():
    parser = argparse.ArgumentParser(description="Publish voice text or run phone text server")
    parser.add_argument("--text", help='Command text, e.g. --text "move forward"')
    parser.add_argument("--serve", action="store_true", help="Run HTTP server for phone text input")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host for HTTP server")
    parser.add_argument("--port", type=int, default=5000, help="Bind port for HTTP server")
    parser.add_argument("--html", default=default_phone_html_path(), help="Path to phone HTML page")
    args = parser.parse_args()

    rclpy.init()
    node = VoiceTextPublisher()

    server = None
    try:
        if args.text:
            node.publish_once(args.text)
            rclpy.spin_once(node, timeout_sec=0.2)
        elif args.serve:
            html_content = ""
            html_path = Path(args.html)
            if html_path.exists():
                html_content = html_path.read_text(encoding="utf-8")
            server = start_http_server(node, args.host, args.port, html_content)
            rclpy.spin(node)
        else:
            node.get_logger().error("Provide --text or --serve")
    finally:
        if server:
            server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

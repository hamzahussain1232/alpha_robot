#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, render_template_string
import threading
import ssl

app = Flask(__name__)
node = None

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Omni Voice Robot</title>
    <style>
        body { font-family: -apple-system, sans-serif; background: #0f172a; color: white; display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; margin: 0; }
        .mic-btn { width: 140px; height: 140px; border-radius: 50%; background: #22c55e; border: none; font-size: 50px; box-shadow: 0 10px 25px rgba(34, 197, 94, 0.5); cursor: pointer; transition: transform 0.1s; }
        .mic-btn:active { transform: scale(0.95); }
        .mic-btn.listening { background: #ef4444; box-shadow: 0 10px 25px rgba(239, 68, 68, 0.5); animation: pulse 1.5s infinite; }
        @keyframes pulse { 0% { transform: scale(1); } 50% { transform: scale(1.05); } 100% { transform: scale(1); } }
        #status { margin-top: 30px; font-size: 22px; font-weight: bold; text-align: center; }
        .help { margin-top: 40px; font-size: 14px; color: #9ca3af; text-align: center; line-height: 1.6; }
    </style>
</head>
<body>
    <h2 style="margin-bottom: 40px;">Robot Command Center</h2>
    <button id="mic" class="mic-btn">🎙️</button>
    <div id="status">Tap the Mic to Speak</div>
    <div class="help">
        "Robot move forward" <br>
        "Robot turn left / right" <br>
        "Robot wave at me" <br>
        "Robot raise your arm" <br>
        "Robot stop"
    </div>

    <script>
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        if (!SpeechRecognition) {
            document.getElementById('status').innerText = 'Speech Not Supported on this Browser';
        } else {
            const recognition = new SpeechRecognition();
            recognition.continuous = false;
            recognition.lang = 'en-US';
            const micBtn = document.getElementById('mic');
            const statusText = document.getElementById('status');

            micBtn.onclick = () => {
                recognition.start();
                micBtn.classList.add('listening');
                statusText.innerText = 'Listening...';
            };

            recognition.onresult = (event) => {
                const text = event.results[0][0].transcript;
                statusText.innerText = 'Sending: "' + text + '"';
                fetch('/command', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: 'text=' + encodeURIComponent(text)
                });
            };

            recognition.onend = () => {
                micBtn.classList.remove('listening');
                setTimeout(() => { if (statusText.innerText.startsWith('Sending')) statusText.innerText = 'Tap the Mic to Speak'; }, 2000);
            };
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/command', methods=['POST'])
def command():
    text = request.form.get('text', '').lower()
    if text and node:
        node.get_logger().info(f"Phone heard: '{text}'")
        msg = String()
        msg.data = text
        node.pub.publish(msg)
    return "OK"

class OmniVoiceServer(Node):
    def __init__(self):
        super().__init__('omni_voice_server')
        self.pub = self.create_publisher(String, '/omni/voice/text', 10)

def main():
    global node
    rclpy.init()
    node = OmniVoiceServer()

    # Run flask in a thread with adhoc SSL
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, ssl_context='adhoc', use_reloader=False))
    flask_thread.daemon = True
    flask_thread.start()

    node.get_logger().info('Secure Mobile Voice Server running!')
    node.get_logger().info('Open https://<RaspberryPi-IP>:5000 in your phone browser')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Flask server for receiving voice commands from mobile app.
Communicates with ROS2 robot via REST API.

Usage:
    python3 voice_server.py --port 5000 --host 0.0.0.0
"""

import json
import argparse
import logging
from flask import Flask, request, jsonify
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
import time

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global ROS2 node reference
ros_node = None
command_publisher = None

app = Flask(__name__)
CORS(app)  # Allow cross-origin requests from mobile app


class VoiceCommandBridge(Node):
    """ROS2 node that publishes voice commands to the robot."""
    
    def __init__(self):
        super().__init__('voice_command_bridge')
        
        # Publisher for voice commands
        self.command_pub = self.create_publisher(String, '/robot/voice_command', 10)
        self.status_pub = self.create_publisher(String, '/robot/command_status', 10)
        
        logger.info("Voice Command Bridge initialized")
    
    def publish_command(self, command):
        """Publish a voice command to ROS2."""
        try:
            msg = String()
            msg.data = command
            self.command_pub.publish(msg)
            logger.info(f"Published command: {command}")
            return True
        except Exception as e:
            logger.error(f"Error publishing command: {e}")
            return False
    
    def publish_status(self, status):
        """Publish status update."""
        try:
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)
        except Exception as e:
            logger.error(f"Error publishing status: {e}")


def ros2_spin_thread():
    """Run ROS2 node in a separate thread."""
    global ros_node
    rclpy.spin(ros_node)


# ==================== REST API ENDPOINTS ====================

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint."""
    return jsonify({
        'status': 'ok',
        'timestamp': time.time(),
        'robot': 'articubot_one'
    })


@app.route('/api/command', methods=['POST'])
def receive_voice_command():
    """
    Receive voice command from mobile app.
    
    Expected JSON:
    {
        "command": "give me medicine",
        "language": "english",
        "urgency": "normal"
    }
    """
    try:
        data = request.get_json()
        
        if not data or 'command' not in data:
            return jsonify({'error': 'Missing command field'}), 400
        
        command = data['command'].lower().strip()
        language = data.get('language', 'english')
        urgency = data.get('urgency', 'normal')
        
        logger.info(f"Received command: '{command}' (urgency: {urgency})")
        
        # Validate command
        if not is_valid_command(command):
            return jsonify({'error': f'Unknown command: {command}'}), 400
        
        # Publish to ROS2
        success = ros_node.publish_command(command)
        
        if success:
            status = {
                'command': command,
                'status': 'processing',
                'timestamp': time.time()
            }
            ros_node.publish_status(status)
            
            return jsonify({
                'status': 'accepted',
                'command': command,
                'message': 'Command sent to robot'
            }), 202
        else:
            return jsonify({'error': 'Failed to send command to robot'}), 500
            
    except Exception as e:
        logger.error(f"Error in receive_voice_command: {e}")
        return jsonify({'error': str(e)}), 500


@app.route('/api/commands', methods=['GET'])
def list_available_commands():
    """List all available voice commands."""
    return jsonify({
        'commands': [
            'give me medicine',
            'fetch water',
            'pick up bottle',
            'bring me cup',
            'get object',
            'return home',
            'stop',
            'cancel'
        ],
        'description': 'Available voice commands for the robot'
    })


@app.route('/api/status', methods=['GET'])
def get_robot_status():
    """Get current robot status (stub - extend with ROS2 subscribers)."""
    return jsonify({
        'robot': 'articubot_one',
        'mode': 'autonomous',
        'battery': 85.0,
        'position': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'arm_position': 'home',
        'status': 'ready'
    })


@app.route('/api/emergency-stop', methods=['POST'])
def emergency_stop():
    """Emergency stop robot."""
    try:
        ros_node.publish_command('emergency_stop')
        return jsonify({'status': 'emergency stop activated'}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# ==================== HELPERS ====================

def is_valid_command(command):
    """Check if command is valid and recognized."""
    valid_commands = [
        'give me medicine', 'fetch water', 'pick up bottle',
        'bring me cup', 'get object', 'return home',
        'stop', 'cancel', 'emergency_stop'
    ]
    return any(cmd.lower() in command.lower() for cmd in valid_commands)


# ==================== MAIN ====================

def main():
    global ros_node
    
    parser = argparse.ArgumentParser(description='Voice command server for articubot_one')
    parser.add_argument('--port', type=int, default=5000, help='Flask server port')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Flask server host')
    parser.add_argument('--ros-namespace', type=str, default='articubot', help='ROS namespace')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    ros_node = VoiceCommandBridge()
    
    # Start ROS2 spinner in separate thread
    spinner_thread = Thread(target=ros2_spin_thread, daemon=True)
    spinner_thread.start()
    
    logger.info(f"Starting Flask server on {args.host}:{args.port}")
    logger.info("Voice Command Bridge ready for mobile app connections")
    
    # Run Flask app
    app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == '__main__':
    main()

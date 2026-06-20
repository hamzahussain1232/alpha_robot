#!/usr/bin/env python3
"""
Voice Command Receiver Node - Processes voice commands and orchestrates autonomous tasks.

This node:
1. Listens for voice commands from mobile app via Flask server
2. Parses commands and validates them
3. Orchestrates autonomous task execution (detect object, navigate, grasp, return)
4. Publishes status updates

Usage:
    ros2 run articubot_one voice_command_receiver_node
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState
import time
from enum import Enum


class TaskState(Enum):
    """Autonomous task states."""
    IDLE = "idle"
    DETECTING = "detecting"
    NAVIGATING = "navigating"
    GRASPING = "grasping"
    RETURNING = "returning"
    COMPLETED = "completed"
    FAILED = "failed"


class VoiceCommandReceiverNode(Node):
    """Main autonomous task orchestrator."""
    
    def __init__(self):
        super().__init__('voice_command_receiver')
        
        # Declare parameters
        self.declare_parameter('home_pose_x', 0.0)
        self.declare_parameter('home_pose_y', 0.0)
        self.declare_parameter('home_pose_yaw', 0.0)
        self.declare_parameter('arm_safe_height', 0.5)
        
        # Get parameters
        self.home_x = self.get_parameter('home_pose_x').value
        self.home_y = self.get_parameter('home_pose_y').value
        self.home_yaw = self.get_parameter('home_pose_yaw').value
        
        # Subscriptions
        self.voice_command_sub = self.create_subscription(
            String, '/robot/voice_command', self.voice_command_callback, 10)
        
        self.detection_sub = self.create_subscription(
            String, '/perception/detections', self.detection_callback, 10)
        
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/robot/command_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # Action clients
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State variables
        self.current_task_state = TaskState.IDLE
        self.current_task = None
        self.detected_objects = []
        self.current_joint_states = {}
        self.task_start_time = None
        
        self.get_logger().info("Voice Command Receiver Node initialized")
        
    def voice_command_callback(self, msg):
        """Process incoming voice command."""
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received voice command: {command}")
        
        # Parse and execute command
        try:
            if 'medicine' in command or 'give me' in command:
                self.execute_fetch_task('medicine box')
            elif 'water' in command or 'fetch water' in command:
                self.execute_fetch_task('bottle')
            elif 'pick up' in command:
                self.execute_fetch_task('any')
            elif 'return home' in command or 'go home' in command:
                self.return_home()
            elif 'stop' in command or 'cancel' in command:
                self.stop_task()
            elif 'emergency_stop' in command:
                self.emergency_stop()
            else:
                self.get_logger().warn(f"Unknown command: {command}")
                self.publish_status('unknown_command', 'Command not recognized')
        except Exception as e:
            self.get_logger().error(f"Error executing command: {e}")
            self.publish_status('error', f'Command failed: {str(e)}')
    
    def detection_callback(self, msg):
        """Process object detections from vision system."""
        try:
            self.detected_objects = json.loads(msg.data)
            self.get_logger().debug(f"Objects detected: {len(self.detected_objects)}")
        except:
            pass
    
    def joint_state_callback(self, msg):
        """Track arm joint states."""
        self.current_joint_states = {
            name: pos for name, pos in zip(msg.name, msg.position)
        }
    
    # ==================== TASK EXECUTION ====================
    
    def execute_fetch_task(self, target_object):
        """
        Execute autonomous fetch task:
        1. Detect object
        2. Navigate to object location
        3. Grasp object with arm
        4. Return home
        """
        self.current_task = target_object
        self.task_start_time = time.time()
        
        self.get_logger().info(f"Starting fetch task for: {target_object}")
        self.publish_status('task_started', f'Fetching {target_object}')
        
        try:
            # Step 1: Detect object
            self.current_task_state = TaskState.DETECTING
            self.publish_status('detecting', f'Looking for {target_object}')
            target_pose = self.detect_object(target_object, timeout=10.0)
            
            if target_pose is None:
                raise Exception(f"Object '{target_object}' not found in scene")
            
            self.get_logger().info(f"Object detected at: x={target_pose['x']:.2f}, y={target_pose['y']:.2f}")
            
            # Step 2: Move to object location
            self.current_task_state = TaskState.NAVIGATING
            self.publish_status('navigating', f'Moving to {target_object}')
            self.navigate_to_object(target_pose)
            
            # Step 3: Approach and grasp
            self.current_task_state = TaskState.GRASPING
            self.publish_status('grasping', f'Picking up {target_object}')
            self.move_arm_to_grasp(target_pose)
            self.grasp_object()
            
            # Step 4: Return home
            self.current_task_state = TaskState.RETURNING
            self.publish_status('returning', 'Returning home')
            self.return_home()
            
            # Task completed
            self.current_task_state = TaskState.COMPLETED
            self.publish_status('completed', f'Successfully fetched {target_object}')
            self.get_logger().info(f"Task completed in {time.time() - self.task_start_time:.2f}s")
            
        except Exception as e:
            self.current_task_state = TaskState.FAILED
            self.publish_status('failed', str(e))
            self.get_logger().error(f"Task failed: {e}")
            self.move_arm_to_safe_position()
    
    def detect_object(self, object_name, timeout=10.0):
        """
        Detect object in the scene.
        Returns: dict with x, y, z, theta or None if not found
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Check latest detections
            for obj in self.detected_objects:
                if object_name.lower() in obj.get('label', '').lower() or object_name == 'any':
                    # Return object pose
                    pose = obj.get('pose', {})
                    return {
                        'x': pose.get('x', 0),
                        'y': pose.get('y', 0),
                        'z': pose.get('z', 0),
                        'theta': pose.get('yaw', 0),
                        'label': obj.get('label', 'unknown')
                    }
            
            time.sleep(0.2)
        
        return None
    
    def navigate_to_object(self, target_pose):
        """Navigate robot to object location using Nav2."""
        # Create nav2 goal
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = target_pose['x']
        pose.pose.pose.position.y = target_pose['y']
        
        self.get_logger().info(f"Navigating to x={target_pose['x']:.2f}, y={target_pose['y']:.2f}")
        
        # Use Nav2 (simplified - would need full action implementation)
        # For now, just publish status
        self.publish_status('navigating', f"Moving to x={target_pose['x']:.2f}, y={target_pose['y']:.2f}")
        time.sleep(3)  # Simulated navigation time
    
    def move_arm_to_grasp(self, target_pose):
        """Move arm to grasp position."""
        self.get_logger().info("Moving arm to grasp position")
        
        # Move arm to approach position
        self.move_arm_to_position([
            1.57, 0.785, 0.785, -0.785, -0.785, 1.57
        ], duration=2.0)
        
        time.sleep(2)
    
    def grasp_object(self):
        """Grasp object by closing gripper."""
        self.get_logger().info("Closing gripper to grasp object")
        
        # Close gripper (joint 6 to ~180 degrees = ~3.14 radians)
        current_state = list(self.current_joint_states.values())[:6]
        current_state[5] = 3.14  # Close gripper
        
        self.move_arm_to_position(current_state, duration=0.5)
        time.sleep(1)
    
    def move_arm_to_safe_position(self):
        """Move arm to safe home position."""
        self.get_logger().info("Moving arm to safe position")
        self.move_arm_to_position([1.57, 0.785, 0.785, -0.785, -0.785, 1.57], duration=1.0)
    
    def return_home(self):
        """Return robot and arm to home position."""
        self.get_logger().info("Returning to home position")
        
        # Move arm to safe position first
        self.move_arm_to_safe_position()
        time.sleep(1)
        
        # Navigate to home
        self.navigate_to_object({'x': self.home_x, 'y': self.home_y, 'theta': self.home_yaw})
        
        # Open gripper to release object
        current_state = list(self.current_joint_states.values())[:6]
        current_state[5] = 0.0  # Open gripper
        self.move_arm_to_position(current_state, duration=0.5)
    
    def stop_task(self):
        """Stop current task."""
        self.get_logger().info("Stopping current task")
        self.current_task_state = TaskState.IDLE
        
        # Stop arm movement
        self.move_arm_to_safe_position()
        
        # Stop robot movement
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        self.publish_status('stopped', 'Task stopped by user')
    
    def emergency_stop(self):
        """Emergency stop - immediate halt."""
        self.get_logger().error("EMERGENCY STOP ACTIVATED")
        
        # Send zero velocities
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.publish_status('emergency_stop', 'EMERGENCY STOP')
    
    # ==================== ARM CONTROL ====================
    
    def move_arm_to_position(self, joint_angles, duration=1.0):
        """
        Move arm to target joint angles.
        
        Args:
            joint_angles: list of 6 joint angles in radians
            duration: movement duration in seconds
        """
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'base_link'
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]
        
        # Create waypoint
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points = [point]
        self.arm_trajectory_pub.publish(trajectory)
    
    # ==================== STATUS PUBLISHING ====================
    
    def publish_status(self, status_code, message):
        """Publish task status update."""
        status_msg = {
            'status': status_code,
            'message': message,
            'task': self.current_task,
            'timestamp': time.time(),
            'task_state': self.current_task_state.value
        }
        
        msg = String()
        msg.data = json.dumps(status_msg)
        self.status_pub.publish(msg)
        
        self.get_logger().info(f"Status: {status_code} - {message}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

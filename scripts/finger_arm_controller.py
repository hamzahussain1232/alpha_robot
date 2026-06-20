#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from sensor_msgs.msg import CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import mediapipe as mp

class FingerArmController(Node):
    def __init__(self):
        super().__init__('finger_arm_controller')
        
        self.get_logger().info("Loading MediaPipe Hands...")
        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.55,
            min_tracking_confidence=0.55
        )
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/detections/compressed',
            self.image_callback,
            10
        )
        
        self.arm_pub_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.last_arm_move_time = 0.0

    @staticmethod
    def _count_fingers(landmarks, handedness_label: str) -> int:
        tip_ids = [4, 8, 12, 16, 20]
        pip_ids = [3, 6, 10, 14, 18]

        count = 0
        thumb_tip = landmarks[tip_ids[0]]
        thumb_ip = landmarks[pip_ids[0]]
        if handedness_label.lower() == "right":
            thumb_up = thumb_tip.x < thumb_ip.x
        else:
            thumb_up = thumb_tip.x > thumb_ip.x
        if thumb_up: count += 1

        for i in range(1, 5):
            if landmarks[tip_ids[i]].y < landmarks[pip_ids[i]].y:
                count += 1
        return count

    def image_callback(self, msg):
        # Convert compressed image back to OpenCV Image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.mp_hands.process(rgb)
        
        if result.multi_hand_landmarks and result.multi_handedness:
            # ONLY RUNS WHEN HAND IS DETECTED
            hand_landmarks = result.multi_hand_landmarks[0]
            handedness_label = result.multi_handedness[0].classification[0].label
            fingers = self._count_fingers(hand_landmarks.landmark, handedness_label)
            
            current_time = time.time()
            if current_time - self.last_arm_move_time > 2.0:
                traj_msg = JointTrajectory()
                traj_msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
                point = JointTrajectoryPoint()
                point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                valid_command = False
                
                if fingers == 1:
                    self.get_logger().info("1 Finger Detected -> Moving Servo 1")
                    point.positions[0] = -0.5
                    valid_command = True
                elif fingers == 2:
                    self.get_logger().info("2 Fingers Detected -> Moving Servo 2")
                    point.positions[1] = -0.5
                    valid_command = True
                elif fingers == 3:
                    self.get_logger().info("3 Fingers Detected -> Moving Servo 3")
                    point.positions[2] = 0.5
                    valid_command = True
                elif fingers == 4:
                    self.get_logger().info("4 Fingers Detected -> Moving Servo 4")
                    point.positions[3] = 0.5
                    valid_command = True
                elif fingers == 5:
                    self.get_logger().info("5 Fingers Detected -> Resetting all servos")
                    valid_command = True
                    
                if valid_command:
                    point.time_from_start.sec = 1
                    point.time_from_start.nanosec = 0
                    traj_msg.points.append(point)
                    self.arm_pub_.publish(traj_msg)
                    self.last_arm_move_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = FingerArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

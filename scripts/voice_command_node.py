#!/usr/bin/env python3
import difflib
import re
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class VoiceCommandNode(Node):
    AMBIGUOUS_INTENT = "__ambiguous__"

    def __init__(self):
        super().__init__("voice_command_node")

        self.declare_parameter("linear_speed", 0.25)
        self.declare_parameter("angular_speed", 0.8)
        self.declare_parameter("motion_duration_sec", 1.0)
        self.declare_parameter("arm_move_duration_sec", 2.0)
        self.declare_parameter("arm_home", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("arm_pick", [0.0, 0.6, -1.1, 0.6, 0.0, 0.0])
        self.declare_parameter("arm_place", [0.0, 0.3, -0.6, 0.2, 0.0, 0.0])
        self.declare_parameter("command_match_threshold", 0.84)
        self.declare_parameter("require_wake_word", True)
        self.declare_parameter("wake_word", "robot")
        self.declare_parameter("command_cooldown_sec", 0.8)
        self.declare_parameter("publish_unknown_feedback", False)
        self.declare_parameter("strict_keyword_mode", True)
        self.declare_parameter("prefer_last_keyword", True)
        self.declare_parameter("motion_conflict_policy", "rotation")
        self.declare_parameter("continuous_motion_mode", True)
        self.declare_parameter("allow_steering_while_moving", True)
        self.declare_parameter("turn_in_place_when_idle", True)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.motion_duration_sec = float(self.get_parameter("motion_duration_sec").value)
        self.arm_move_duration_sec = float(self.get_parameter("arm_move_duration_sec").value)
        self.command_match_threshold = float(self.get_parameter("command_match_threshold").value)
        self.require_wake_word = bool(self.get_parameter("require_wake_word").value)
        self.wake_word = self.normalize_text(str(self.get_parameter("wake_word").value))
        self.command_cooldown_sec = float(self.get_parameter("command_cooldown_sec").value)
        self.publish_unknown_feedback = bool(self.get_parameter("publish_unknown_feedback").value)
        self.strict_keyword_mode = bool(self.get_parameter("strict_keyword_mode").value)
        self.prefer_last_keyword = bool(self.get_parameter("prefer_last_keyword").value)
        self.motion_conflict_policy = str(self.get_parameter("motion_conflict_policy").value).lower().strip()
        self.continuous_motion_mode = bool(self.get_parameter("continuous_motion_mode").value)
        self.allow_steering_while_moving = bool(
            self.get_parameter("allow_steering_while_moving").value
        )
        self.turn_in_place_when_idle = bool(self.get_parameter("turn_in_place_when_idle").value)

        self.arm_home = list(self.get_parameter("arm_home").value)
        self.arm_pick = list(self.get_parameter("arm_pick").value)
        self.arm_place = list(self.get_parameter("arm_place").value)

        self.text_sub = self.create_subscription(String, "/voice/text", self.text_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/key_vel_in", 10)
        self.arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.feedback_pub = self.create_publisher(String, "/voice/feedback", 10)
        self.timer = self.create_timer(0.1, self.publish_active_twist)

        self.active_twist = Twist()
        self.motion_until = self.get_clock().now()
        self.motion_is_continuous = False
        self.is_moving = False
        self.last_command_ts = 0.0

        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.intent_phrases = {
            "stop": ["stop", "halt", "freeze", "hold on", "wait"],
            "forward": ["move forward", "go forward", "forward", "go ahead", "move ahead", "front"],
            "backward": ["move backward", "go backward", "backward", "reverse", "move back", "back"],
            "left": ["turn left", "go left", "left"],
            "right": ["turn right", "go right", "right"],
            "arm_home": ["arm home", "home arm", "arm reset"],
            "arm_pick": ["arm pick", "pick pose", "pickup pose", "grab pose"],
            "arm_place": ["arm place", "place pose", "drop pose", "put pose"],
        }
        self.keyword_aliases = {
            "stop": {"stop", "halt", "freeze", "wait"},
            "forward": {"forward", "front", "ahead"},
            "backward": {"backward", "back", "reverse"},
            "left": {"left"},
            "right": {"right"},
            "arm_home": {"home", "reset"},
            "arm_pick": {"pick", "pickup", "grab"},
            "arm_place": {"place", "drop", "put"},
        }
        self.get_logger().info("Voice command node ready. Listening on /voice/text")

    @staticmethod
    def normalize_text(text):
        text = text.lower().strip()
        text = re.sub(r"[^a-z0-9\s]", " ", text)
        text = re.sub(r"\s+", " ", text)
        return text

    def text_cb(self, msg):
        text = self.normalize_text(msg.data)
        if not text:
            return
        self.get_logger().info(f'Voice text: "{text}"')

        # Optional wake word gate to prevent random speech triggering robot motion.
        if self.require_wake_word:
            if not self.wake_word or self.wake_word not in text.split():
                return
            text = self.normalize_text(text.replace(self.wake_word, " "))
            if not text:
                return

        now = time.monotonic()
        if now - self.last_command_ts < self.command_cooldown_sec:
            return

        self.handle_command(text)

    def publish_feedback(self, text):
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)

    def set_motion(self, linear_x=0.0, angular_z=0.0, continuous=False):
        tw = Twist()
        tw.linear.x = linear_x
        tw.angular.z = angular_z
        self.active_twist = tw
        self.motion_is_continuous = bool(continuous)
        if not self.motion_is_continuous:
            self.motion_until = self.get_clock().now() + Duration(seconds=self.motion_duration_sec)
        self.is_moving = True

    def stop_motion(self):
        self.active_twist = Twist()
        self.motion_is_continuous = False
        try:
            self.cmd_pub.publish(self.active_twist)
        except Exception:
            # Node may be shutting down.
            pass
        self.is_moving = False
        self.publish_feedback("stop")

    def publish_active_twist(self):
        if not self.is_moving:
            return

        if self.motion_is_continuous:
            self.cmd_pub.publish(self.active_twist)
            return

        if self.get_clock().now() <= self.motion_until:
            self.cmd_pub.publish(self.active_twist)
            return

        self.cmd_pub.publish(Twist())
        self.is_moving = False

    def publish_arm_pose(self, positions):
        if len(positions) != len(self.joint_names):
            self.get_logger().warn("Arm pose has wrong size, expected 6 joints")
            return

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(self.arm_move_duration_sec)
        point.time_from_start.nanosec = int(
            (self.arm_move_duration_sec - int(self.arm_move_duration_sec)) * 1e9
        )
        traj.points = [point]
        self.arm_pub.publish(traj)

    def _intent_from_token(self, token):
        for intent, words in self.keyword_aliases.items():
            if token in words:
                return intent
        return None

    def _extract_intent_from_keywords(self, text, tokens):
        if not self.strict_keyword_mode:
            return None

        # Handle explicit two-word arm commands first.
        if "arm" in tokens:
            for arm_intent in ("arm_home", "arm_pick", "arm_place"):
                for phrase in self.intent_phrases[arm_intent]:
                    if phrase in text:
                        return arm_intent

        # Build list of detected motion keywords in order.
        detected = []
        for t in tokens:
            intent = self._intent_from_token(t)
            if intent in {"stop", "forward", "backward", "left", "right"}:
                detected.append(intent)

        return self._resolve_motion_keywords(detected)

    def _resolve_motion_keywords(self, detected):
        if not detected:
            return None

        # Safety first: stop always has highest priority when present.
        if "stop" in detected:
            return "stop"

        # Keep order but collapse immediate duplicates.
        ordered = []
        for intent in detected:
            if not ordered or ordered[-1] != intent:
                ordered.append(intent)

        unique = set(ordered)
        if len(unique) == 1:
            return ordered[-1] if self.prefer_last_keyword else ordered[0]

        has_rot = any(i in {"left", "right"} for i in ordered)
        has_lin = any(i in {"forward", "backward"} for i in ordered)
        policy = self.motion_conflict_policy

        # Common ASR error for "robot left/right" is adding an extra "forward".
        # Rotation policy prioritizes turning intents when mixed commands appear.
        if policy == "rotation" and has_rot:
            candidates = [i for i in ordered if i in {"left", "right"}]
            return candidates[-1] if self.prefer_last_keyword else candidates[0]

        if policy == "translation" and has_lin:
            candidates = [i for i in ordered if i in {"forward", "backward"}]
            return candidates[-1] if self.prefer_last_keyword else candidates[0]

        if policy == "last":
            return ordered[-1]
        if policy == "first":
            return ordered[0]

        # reject (or unknown policy) -> no motion command execution
        return self.AMBIGUOUS_INTENT

    def _execute_motion_intent(self, intent):
        if intent == "stop":
            self.stop_motion()
            return True
        if intent == "forward":
            self.set_motion(
                linear_x=self.linear_speed,
                angular_z=0.0,
                continuous=self.continuous_motion_mode,
            )
            self.publish_feedback("moving forward")
            return True
        if intent == "backward":
            self.set_motion(
                linear_x=-self.linear_speed,
                angular_z=0.0,
                continuous=self.continuous_motion_mode,
            )
            self.publish_feedback("moving backward")
            return True
        if intent == "left":
            if (
                self.continuous_motion_mode
                and self.allow_steering_while_moving
                and self.is_moving
                and self.motion_is_continuous
                and abs(self.active_twist.linear.x) > 1e-6
            ):
                self.set_motion(
                    linear_x=self.active_twist.linear.x,
                    angular_z=self.angular_speed,
                    continuous=True,
                )
                self.publish_feedback("steering left")
                return True

            if self.turn_in_place_when_idle:
                self.set_motion(
                    linear_x=0.0,
                    angular_z=self.angular_speed,
                    continuous=self.continuous_motion_mode,
                )
            else:
                self.set_motion(linear_x=0.0, angular_z=self.angular_speed, continuous=False)
            self.publish_feedback("turning left")
            return True
        if intent == "right":
            if (
                self.continuous_motion_mode
                and self.allow_steering_while_moving
                and self.is_moving
                and self.motion_is_continuous
                and abs(self.active_twist.linear.x) > 1e-6
            ):
                self.set_motion(
                    linear_x=self.active_twist.linear.x,
                    angular_z=-self.angular_speed,
                    continuous=True,
                )
                self.publish_feedback("steering right")
                return True

            if self.turn_in_place_when_idle:
                self.set_motion(
                    linear_x=0.0,
                    angular_z=-self.angular_speed,
                    continuous=self.continuous_motion_mode,
                )
            else:
                self.set_motion(linear_x=0.0, angular_z=-self.angular_speed, continuous=False)
            self.publish_feedback("turning right")
            return True
        return False

    def _execute_arm_intent(self, intent):
        if intent == "arm_home":
            self.publish_arm_pose(self.arm_home)
            self.publish_feedback("arm home")
            return True
        if intent == "arm_pick":
            self.publish_arm_pose(self.arm_pick)
            self.publish_feedback("arm pick pose")
            return True
        if intent == "arm_place":
            self.publish_arm_pose(self.arm_place)
            self.publish_feedback("arm place pose")
            return True
        return False

    def _intent_score(self, text, tokens, intent_key):
        phrases = self.intent_phrases[intent_key]

        # Exact/contains phrase match is highest confidence.
        for phrase in phrases:
            if phrase in text:
                return 1.0

        # Phrase-level similarity is safer than raw single-word similarity.
        phrase_score = max(difflib.SequenceMatcher(None, text, p).ratio() for p in phrases)

        # Token overlap boosts scores when key command words are present.
        overlap_score = 0.0
        token_set = set(tokens)
        for phrase in phrases:
            pset = set(phrase.split())
            if pset:
                overlap_score = max(overlap_score, len(token_set & pset) / len(pset))

        return 0.7 * phrase_score + 0.3 * overlap_score

    def handle_command(self, text):
        tokens = text.split()
        if not tokens:
            return

        keyword_intent = self._extract_intent_from_keywords(text, tokens)
        if keyword_intent is not None:
            if keyword_intent == self.AMBIGUOUS_INTENT:
                if self.publish_unknown_feedback:
                    self.publish_feedback("ambiguous command, say one direction only")
                self.get_logger().warn(f'Ambiguous motion command: "{text}"')
                return
            self.last_command_ts = time.monotonic()
            if self._execute_motion_intent(keyword_intent):
                return
            if self._execute_arm_intent(keyword_intent):
                return

        # Compute scores for motion intents first.
        scores = {
            "stop": self._intent_score(text, tokens, "stop"),
            "forward": self._intent_score(text, tokens, "forward"),
            "backward": self._intent_score(text, tokens, "backward"),
            "left": self._intent_score(text, tokens, "left"),
            "right": self._intent_score(text, tokens, "right"),
        }

        best_motion = max(scores, key=scores.get)
        if scores[best_motion] >= self.command_match_threshold:
            self.last_command_ts = time.monotonic()
            if self._execute_motion_intent(best_motion):
                return

        # Arm intents require "arm" or explicit pose words to avoid accidental triggers.
        arm_context = ("arm" in tokens) or ("pose" in tokens)
        if arm_context:
            arm_scores = {
                "arm_home": self._intent_score(text, tokens, "arm_home"),
                "arm_pick": self._intent_score(text, tokens, "arm_pick"),
                "arm_place": self._intent_score(text, tokens, "arm_place"),
            }
            best_arm = max(arm_scores, key=arm_scores.get)
            if arm_scores[best_arm] >= self.command_match_threshold:
                self.last_command_ts = time.monotonic()
                if self._execute_arm_intent(best_arm):
                    return

        if self.publish_unknown_feedback:
            self.publish_feedback(f"unknown command: {text}")
        self.get_logger().warn(f'Unknown command: "{text}"')


def main():
    rclpy.init()
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_motion()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()

#! /usr/bin/env python3
#
# Run:
#   ros2 run articubot_one xy_waypoint_follower.py --file sim_waypoints
#
# YAML format:
# waypoints:
#   - x: 0.0
#     y: 0.0
#     yaw: 0.0
#     frame_id: map  # optional

import argparse
import math
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class YamlWaypointParser:
    def __init__(self, file_name):
        self.file_path = os.path.join(
            get_package_share_directory("articubot_one"),
            "assets",
            "waypoints",
            f"{file_name}.yaml",
        )
        with open(self.file_path, "r", encoding="utf-8") as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_waypoints(self):
        waypoints = self.wps_dict.get("waypoints", [])
        if not waypoints:
            raise ValueError("No waypoints found in YAML file")
        return waypoints


def build_goal_poses(navigator, waypoints):
    goal_poses = []
    now = navigator.get_clock().now().to_msg()
    for wp in waypoints:
        x = float(wp["x"])
        y = float(wp["y"])
        yaw = float(wp.get("yaw", 0.0))
        frame_id = str(wp.get("frame_id", "map"))
        _, _, qz, qw = quaternion_from_yaw(yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = now
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        goal_poses.append(goal_pose)
    return goal_poses


def main(file_name):
    rclpy.init()
    navigator = BasicNavigator()

    # In your navigation mode, AMCL is the localizer.
    navigator.waitUntilNav2Active(localizer="amcl")

    parser = YamlWaypointParser(file_name)
    waypoints = parser.get_waypoints()
    goal_poses = build_goal_poses(navigator, waypoints)

    print(f"Loaded {len(goal_poses)} waypoints from {parser.file_path}")
    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            print(f"Executing waypoint {feedback.current_waypoint + 1}/{len(goal_poses)}")

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Waypoints succeeded")
    elif result == TaskResult.CANCELED:
        print("Waypoints canceled")
    elif result == TaskResult.FAILED:
        print("Waypoints failed")
    else:
        print("Unknown waypoint result")

    rclpy.shutdown()


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="Follow map-frame waypoints from YAML")
    arg_parser.add_argument(
        "--file",
        default="sim_waypoints",
        help='YAML file name in assets/waypoints without extension, e.g. "sim_waypoints"',
    )
    args = arg_parser.parse_args()
    main(args.file)

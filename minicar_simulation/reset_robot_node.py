#!/usr/bin/env python3
"""
Robot Reset Node - Resets the robot to its initial spawn position.

Uses Gazebo's /set_entity_state service to reset the robot position
without restarting the simulation.
"""

import json
import os
import math

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion


SPAWN_POSE_FILE = "/tmp/minicar_simulation/output/spawn_pose.json"


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class ResetRobotNode(Node):
    def __init__(self):
        super().__init__("reset_robot_node")

        # Parameters
        self.declare_parameter("entity_name", "minicar")
        self.declare_parameter("spawn_pose_file", SPAWN_POSE_FILE)

        self.entity_name = self.get_parameter("entity_name").value
        self.spawn_pose_file = self.get_parameter("spawn_pose_file").value

        # Load initial spawn pose
        self.spawn_pose = self._load_spawn_pose()

        # Gazebo set_entity_state client
        self.set_state_client = self.create_client(
            SetEntityState, "/set_entity_state"
        )

        # Reset service
        self.reset_service = self.create_service(
            Empty, "/reset_robot", self.reset_callback
        )

        self.get_logger().info(
            f"Reset robot node started. Entity: {self.entity_name}, "
            f"Spawn pose: x={self.spawn_pose['x']:.3f}, y={self.spawn_pose['y']:.3f}, "
            f"yaw={self.spawn_pose['yaw']:.3f}"
        )

    def _load_spawn_pose(self) -> dict:
        """Load spawn pose from JSON file."""
        if os.path.exists(self.spawn_pose_file):
            with open(self.spawn_pose_file, "r") as f:
                pose = json.load(f)
                self.get_logger().info(f"Loaded spawn pose from {self.spawn_pose_file}")
                return pose
        else:
            self.get_logger().warn(
                f"Spawn pose file not found: {self.spawn_pose_file}. Using defaults."
            )
            return {"x": 0.0, "y": 1.5, "z": 0.05, "yaw": 0.0}

    def reload_spawn_pose(self):
        """Reload spawn pose from file (useful after course regeneration)."""
        self.spawn_pose = self._load_spawn_pose()

    def reset_callback(self, request, response):
        """Handle reset service request."""
        self.get_logger().info("Reset requested...")

        # Reload spawn pose in case course was regenerated
        self.reload_spawn_pose()

        # Wait for Gazebo service
        if not self.set_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Gazebo /set_entity_state service not available")
            return response

        # Create entity state message
        state = EntityState()
        state.name = self.entity_name
        state.pose = Pose()
        state.pose.position = Point(
            x=self.spawn_pose["x"],
            y=self.spawn_pose["y"],
            z=self.spawn_pose["z"]
        )
        state.pose.orientation = euler_to_quaternion(0.0, 0.0, self.spawn_pose["yaw"])

        # Zero velocity
        state.twist = Twist()

        # Reference frame (empty = world frame)
        state.reference_frame = ""

        # Call service
        req = SetEntityState.Request()
        req.state = state

        future = self.set_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(
                    f"Robot reset to x={self.spawn_pose['x']:.3f}, "
                    f"y={self.spawn_pose['y']:.3f}, yaw={self.spawn_pose['yaw']:.3f}"
                )
            else:
                self.get_logger().error("Failed to reset robot state")
        else:
            self.get_logger().error("Service call failed")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRemapNode(Node):
    def __init__(self):
        super().__init__("joint_state_remap")
        self.declare_parameter("robot_description", "")
        robot_description = self.get_parameter("robot_description").get_parameter_value().string_value
        self._name_map = self._build_name_map(robot_description)

        self._sub = self.create_subscription(
            JointState, "joint_states_in", self._on_joint_state, 10
        )
        self._pub = self.create_publisher(JointState, "joint_states_out", 10)

    def _build_name_map(self, robot_description):
        name_map = {}
        if not robot_description:
            self.get_logger().warn("robot_description is empty; joint name remap disabled")
            return name_map

        try:
            robot = ET.fromstring(robot_description)
        except ET.ParseError:
            self.get_logger().warn("robot_description parse failed; joint name remap disabled")
            return name_map

        for joint in robot.findall("joint"):
            joint_name = joint.get("name", "")
            if not joint_name:
                continue
            parts = joint_name.split("_", 1)
            if len(parts) == 2 and parts[0].isdigit():
                stripped = parts[1]
                name_map[stripped] = joint_name

        if not name_map:
            self.get_logger().info("No numbered joints found; joint name remap not needed")
        return name_map

    def _on_joint_state(self, msg):
        if not self._name_map:
            self._pub.publish(msg)
            return

        remapped = JointState()
        remapped.header = msg.header
        remapped.name = [self._name_map.get(name, name) for name in msg.name]
        remapped.position = list(msg.position)
        remapped.velocity = list(msg.velocity)
        remapped.effort = list(msg.effort)
        self._pub.publish(remapped)


def main():
    rclpy.init()
    node = JointStateRemapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

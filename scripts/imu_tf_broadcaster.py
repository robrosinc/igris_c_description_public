#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__("imu_tf_broadcaster")
        # Create a TransformBroadcaster object
        self.tf_broadcaster = TransformBroadcaster(self)
        # Subscribe to the IMU topic (adjust the topic name as needed)
        self.create_subscription(Imu, "/igris_c/imu", self.imu_callback, 10)
        self.get_logger().info("IMU TF Broadcaster has been started.")

    def imu_callback(self, msg: Imu):
        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        # Set the parent frame (for example, the pelvis frame)
        t.header.frame_id = "base_link"  # or "pelvis" depending on your URDF
        # Set the child frame to be the IMU's frame
        t.child_frame_id = "base_link"  # or whichever frame represents the sensor

        # If you only care about orientation, you can set translation to zero.
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Use the orientation provided by the IMU message.
        t.transform.rotation = msg.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

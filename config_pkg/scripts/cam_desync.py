#!/usr/bin/env python3
"""Re-publie le topic CAMERA d'un bag avec un decalage + jitter temporel, pour
simuler une camera et une IMU venant de capteurs INDEPENDANTS (non synchronises
materiellement) -- le cas du ROV : cam USB sur la Pi vs IMU sur le Navigator.

Le contenu visuel est INCHANGE ; seul le header.stamp de l'image est modifie :
    nouveau_stamp = ancien_stamp + offset + jitter_aleatoire

VINS lit ce topic decale + l'IMU d'origine. On mesure combien de desync il
encaisse :
  - offset CONSTANT  -> estimate_td devrait le rattraper (jusqu'a une limite)
  - jitter aleatoire -> non rattrapable -> degrade progressivement l'ATE

Usage:
  python3 cam_desync.py --ros-args \
      -p in_topic:=/alphasense_driver_ros/cam0 \
      -p out_topic:=/cam0_desync \
      -p offset_ms:=50.0 -p jitter_ms:=0.0
"""
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CamDesync(Node):
    def __init__(self):
        super().__init__("cam_desync")
        self.declare_parameter("in_topic", "/alphasense_driver_ros/cam0")
        self.declare_parameter("out_topic", "/cam0_desync")
        self.declare_parameter("offset_ms", 0.0)
        self.declare_parameter("jitter_ms", 0.0)
        in_t = self.get_parameter("in_topic").value
        out_t = self.get_parameter("out_topic").value
        self.offset_ns = int(self.get_parameter("offset_ms").value * 1e6)
        self.jitter_ns = int(self.get_parameter("jitter_ms").value * 1e6)
        self.pub = self.create_publisher(Image, out_t, 50)
        self.sub = self.create_subscription(Image, in_t, self.cb, 50)
        self.get_logger().info(
            f"desync {in_t} -> {out_t} | offset={self.offset_ns/1e6:.1f} ms "
            f"jitter=+/-{self.jitter_ns/1e6:.1f} ms")

    def cb(self, msg):
        t = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        j = random.randint(-self.jitter_ns, self.jitter_ns) if self.jitter_ns > 0 else 0
        t2 = max(0, t + self.offset_ns + j)
        msg.header.stamp.sec = t2 // 1_000_000_000
        msg.header.stamp.nanosec = t2 % 1_000_000_000
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CamDesync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

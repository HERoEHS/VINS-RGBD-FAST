#!/usr/bin/env python3
"""OpenLORIS D400 accel(/d400/accel/sample)+gyro(/d400/gyro/sample)를
합쳐 /d400/imu0 (sensor_msgs/Imu)로 발행. VINS-RGBD-FAST openloris config 입력용.
gyro(고레이트) 콜백마다 최신 accel을 끼워 발행."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE, depth=200)

class Merge(Node):
    def __init__(self):
        super().__init__("imu_merge")
        self.acc = None
        self.pub = self.create_publisher(Imu, "/d400/imu0", QOS)
        self.create_subscription(Imu, "/d400/accel/sample", self.on_acc, QOS)
        self.create_subscription(Imu, "/d400/gyro/sample",  self.on_gyro, QOS)
        self.n = 0
    def on_acc(self, m):
        self.acc = m.linear_acceleration
    def on_gyro(self, m):
        if self.acc is None:
            return
        out = Imu()
        out.header = m.header              # gyro 타임스탬프 기준
        out.angular_velocity = m.angular_velocity
        out.linear_acceleration = self.acc # 최신 accel
        out.orientation_covariance[0] = -1.0
        self.pub.publish(out)
        self.n += 1
        if self.n % 400 == 0:
            self.get_logger().info(f"published /d400/imu0 x{self.n}")

def main():
    rclpy.init(); n = Merge()
    n.get_logger().info("imu_merge: accel+gyro -> /d400/imu0 (BEST_EFFORT)")
    rclpy.spin(n)

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Keep the arm at its current joint position via forward_position_controller.

This is a direct position command to ros2_control; the hardware driver sends
POS_VEL frames (CAN 0x100+ID, vlim from the xacro).  The node ramps any target
change at --speed rad/s and holds the last command on exit.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class Holder(Node):
    def __init__(self, speed):
        super().__init__('posvel_hold')
        self.speed = speed
        self.cur = None
        self.cmd = None
        self.target = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/forward_position_controller/commands', 10)
        self.create_timer(0.01, self.tick)
        self.last_print = 0.0

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.cur = [d[j] for j in JOINTS]

    def tick(self):
        if self.cur is None:
            return
        if self.cmd is None:
            self.cmd = list(self.cur)
            self.target = list(self.cur)
            self.get_logger().info('holding at ' + ' '.join(f'{v:+.4f}' for v in self.cur))
        step = self.speed * 0.01
        for i in range(6):
            d = self.target[i] - self.cmd[i]
            if abs(d) <= step:
                self.cmd[i] = self.target[i]
            else:
                self.cmd[i] += step if d > 0 else -step
        self.pub.publish(Float64MultiArray(data=self.cmd))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.25)
    ap.add_argument('--seconds', type=float, default=0.0)
    args = ap.parse_args()
    rclpy.init()
    n = Holder(args.speed)
    t0 = n.get_clock().now()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            if args.seconds > 0:
                if (n.get_clock().now() - t0).nanoseconds / 1e9 > args.seconds:
                    break
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

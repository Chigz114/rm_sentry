#!/usr/bin/env python3
"""WASD keyboard teleop for omnidirectional sentry robot.

Controls:
  W/S  : forward / backward
  A/D  : left / right strafe
  Q/E  : turn left / turn right
  Space: emergency stop
  R/F  : increase / decrease speed
  Ctrl+C: quit

Publishes geometry_msgs/Twist to /cmd_vel_chassis (planar_move plugin).
"""

import sys
import select
import termios
import tty
from threading import Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP_TEXT = r"""
==================== Sentry Keyboard Teleop ====================
  W/S  : forward / backward (hold)
  A/D  : strafe left / right (hold)
  Q/E  : turn left / turn right (hold)
  Space: stop all motion
  R/F  : speed up / slow down
  Ctrl+C: quit
  Release all keys to stop
================================================================
"""

KEYBindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}


KEY_TIMEOUT = 0.15  # seconds without key input before stopping


def get_key_nonblocking(settings):
    """Read a key with timeout. Returns None if no key within KEY_TIMEOUT."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], KEY_TIMEOUT)
    if rlist:
        ch = sys.stdin.read(1)
    else:
        ch = None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return ch


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_chassis')
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('speed_step', 0.2)
        self.declare_parameter('publish_rate', 20.0)

        topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.speed_step = self.get_parameter('speed_step').value
        rate = self.get_parameter('publish_rate').value

        self.pub = self.create_publisher(Twist, topic, 10)
        self.lock = Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.timer = self.create_timer(1.0 / rate, self.publish_twist)
        self.get_logger().info(HELP_TEXT)
        self.get_logger().info(
            f'Publishing to "{topic}" | linear={self.linear_speed:.2f} m/s '
            f'angular={self.angular_speed:.2f} rad/s')

    def publish_twist(self):
        msg = Twist()
        with self.lock:
            msg.linear.x = self.vx
            msg.linear.y = self.vy
            msg.angular.z = self.wz
        self.pub.publish(msg)

    def set_vel(self, vx, vy, wz):
        with self.lock:
            self.vx = vx
            self.vy = vy
            self.wz = wz

    def stop(self):
        self.set_vel(0.0, 0.0, 0.0)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTeleop()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            key = get_key_nonblocking(settings)
            if key is None:
                node.stop()
                continue
            if key in KEYBindings:
                dx, dy, dw = KEYBindings[key]
                node.set_vel(
                    dx * node.linear_speed,
                    dy * node.linear_speed,
                    dw * node.angular_speed)
            elif key == ' ':
                node.stop()
            elif key == 'r':
                node.linear_speed += node.speed_step
                node.angular_speed += node.speed_step
                node.get_logger().info(
                    f'Speed up: linear={node.linear_speed:.2f} '
                    f'angular={node.angular_speed:.2f}')
            elif key == 'f':
                node.linear_speed = max(0.1, node.linear_speed - node.speed_step)
                node.angular_speed = max(0.1, node.angular_speed - node.speed_step)
                node.get_logger().info(
                    f'Speed down: linear={node.linear_speed:.2f} '
                    f'angular={node.angular_speed:.2f}')
            elif key == '\x03':
                break
    except Exception as e:
        node.get_logger().error(f'{e}')
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

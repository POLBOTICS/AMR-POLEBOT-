#!/usr/bin/env python3
"""
POLEBOT Keyboard Teleop Node
Controls the robot using keyboard inputs (WASD + arrow keys)
"""
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Key bindings
MOVE_BINDINGS = {
    'w': ( 1.0,  0.0),   # forward
    's': (-1.0,  0.0),   # backward
    'a': ( 0.0,  1.0),   # turn left
    'd': ( 0.0, -1.0),   # turn right
    'q': ( 1.0,  1.0),   # forward-left
    'e': ( 1.0, -1.0),   # forward-right
    'z': (-1.0,  1.0),   # backward-left
    'x': (-1.0, -1.0),   # backward-right
}

SPEED_BINDINGS = {
    '+': ( 1.1, 1.0),    # increase linear
    '-': ( 0.9, 1.0),    # decrease linear
    ']': ( 1.0, 1.1),    # increase angular
    '[': ( 1.0, 0.9),    # decrease angular
}

MSG = """
╔══════════════════════════════════════╗
║     POLEBOT Keyboard Teleop          ║
║     AMR - Polman Bandung             ║
╠══════════════════════════════════════╣
║  Movement:                           ║
║    W / ↑    : Forward                ║
║    S / ↓    : Backward               ║
║    A / ←    : Turn Left              ║
║    D / →    : Turn Right             ║
║    Q        : Forward-Left diagonal  ║
║    E        : Forward-Right diagonal ║
║                                      ║
║  Speed:                              ║
║    + / -    : Linear speed ±10%      ║
║    ] / [    : Angular speed ±10%     ║
║                                      ║
║  SPACE      : Emergency STOP         ║
║  CTRL+C     : Quit                   ║
╚══════════════════════════════════════╝
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        cmd_topic          = self.get_parameter('cmd_vel_topic').value

        # Publisher
        self.pub = self.create_publisher(Twist, cmd_topic, 10)

        self.get_logger().info(f'Publishing to: {cmd_topic}')
        self.get_logger().info(
            f'Linear: {self.linear_speed:.2f} m/s | Angular: {self.angular_speed:.2f} rad/s'
        )

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(MSG)

        try:
            while rclpy.ok():
                key = get_key(settings)
                twist = Twist()

                if key in MOVE_BINDINGS:
                    lin, ang = MOVE_BINDINGS[key]
                    twist.linear.x  = lin * self.linear_speed
                    twist.angular.z = ang * self.angular_speed

                elif key in SPEED_BINDINGS:
                    lin_mult, ang_mult = SPEED_BINDINGS[key]
                    self.linear_speed  *= lin_mult
                    self.angular_speed *= ang_mult
                    self.get_logger().info(
                        f'Speed — Linear: {self.linear_speed:.2f} m/s | '
                        f'Angular: {self.angular_speed:.2f} rad/s'
                    )
                    continue

                elif key == ' ':  # Emergency stop
                    self.get_logger().warn('EMERGENCY STOP!')

                elif key == '\x03':  # CTRL+C
                    break

                self.pub.publish(twist)

        except Exception as e:
            self.get_logger().error(str(e))

        finally:
            # Publish stop command
            self.pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

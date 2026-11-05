"""
Nudge the pendulum like a person tapping it with a finger.

A nudge is a short horizontal push on the pole: the force rises and falls as a
half-sine over nudge_duration seconds and acts at a point somewhere along the
upper part of the rod. Gazebo's ApplyLinkWrench system (bridged by
ros_gz_bridge) only takes a wrench at the link origin, which is the pivot, so
the push is applied as the equivalent torque: force * contact height.

Nudges happen:
  * once at startup (a very small one) so the pole starts to fall on its own;
    after that gravity keeps the let-go / catch cycle going without any nudges
  * from the arrow keys, either in the Gazebo window (Key Publisher plugin) or
    in a terminal running keyboard_nudge; both publish Qt key codes on
    /keyboard/keypress. Up/Down push along +X/-X (robot forward/back),
    Left/Right along +Y/-Y, and +/- change the push strength
  * on demand, by publishing the push force (N) to /nudge, e.g.
      ros2 topic pub --once /nudge geometry_msgs/msg/Vector3 "{x: 0.5, y: 0.0}"

Gazebo adds persistent wrenches together rather than replacing them, so this
node only ever publishes the *change* in total torque since the last update,
and clears the accumulated wrenches shortly after each nudge ends.
"""

import math
import random

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity, EntityWrench
from std_msgs.msg import Int32

# Qt key codes, as published by Gazebo's Key Publisher GUI plugin
KEY_LEFT, KEY_UP, KEY_RIGHT, KEY_DOWN = 0x01000012, 0x01000013, 0x01000014, 0x01000015
KEY_DIRECTIONS = {KEY_UP: (1.0, 0.0), KEY_DOWN: (-1.0, 0.0),
                  KEY_LEFT: (0.0, 1.0), KEY_RIGHT: (0.0, -1.0)}
KEY_STRONGER = (ord('+'), ord('='))
KEY_WEAKER = (ord('-'), ord('_'))


class Disturbance(Node):

    def __init__(self):
        super().__init__('disturbance')

        self.declare_parameter('world', 'balance_world')
        self.declare_parameter('link', 'omni_pendulum::pendulum_link')
        self.declare_parameter('release_delay', 4.0)
        self.declare_parameter('release_force', 0.1)
        self.declare_parameter('nudge_duration', 0.15)
        self.declare_parameter('min_contact_height', 0.3)
        self.declare_parameter('max_contact_height', 0.5)
        self.declare_parameter('key_force', 0.8)
        self.declare_parameter('key_force_step', 0.2)

        world = self.get_parameter('world').value
        self.link = self.get_parameter('link').value
        self.release_delay = self.get_parameter('release_delay').value
        self.release_force = self.get_parameter('release_force').value
        self.duration = self.get_parameter('nudge_duration').value
        self.height_range = (self.get_parameter('min_contact_height').value,
                             self.get_parameter('max_contact_height').value)
        self.key_force = self.get_parameter('key_force').value
        self.key_force_step = self.get_parameter('key_force_step').value

        self.wrench_pub = self.create_publisher(
            EntityWrench, f'/world/{world}/wrench/persistent', 10)
        self.clear_pub = self.create_publisher(
            Entity, f'/world/{world}/wrench/clear', 10)
        self.create_subscription(Vector3, '/nudge', self.nudge_callback, 10)
        self.create_subscription(Int32, '/keyboard/keypress', self.key_callback, 10)

        self.start_time = None
        self.released = False
        self.nudge = None           # (start time, peak torque about x, about y)
        self.pending = []           # manual nudges waiting for the next tick
        self.applied = [0.0, 0.0]   # total torque Gazebo currently applies
        self.clear_time = None      # when to clear wrenches after a nudge

        self.create_timer(0.01, self.tick)

    def entity(self):
        ent = Entity()
        ent.name = self.link
        ent.type = Entity.LINK
        return ent

    def nudge_callback(self, msg):
        self.pending.append((msg.x, msg.y))

    def key_callback(self, msg):
        if msg.data in KEY_DIRECTIONS:
            # Holding a key auto-repeats; don't let presses pile up
            if len(self.pending) < 2:
                dx, dy = KEY_DIRECTIONS[msg.data]
                self.pending.append((dx * self.key_force, dy * self.key_force))
        elif msg.data in KEY_STRONGER + KEY_WEAKER:
            step = self.key_force_step if msg.data in KEY_STRONGER else -self.key_force_step
            self.key_force = min(max(self.key_force + step, self.key_force_step), 3.0)
            self.get_logger().info(f'Arrow-key nudge strength: {self.key_force:.1f} N')

    def start_nudge(self, t, fx, fy, label):
        height = random.uniform(*self.height_range)
        # A force along +X at height h tips the pole about +Y, and +Y about -X
        self.nudge = (t, -fy * height, fx * height)
        self.get_logger().info(
            f'{label}: {math.hypot(fx, fy):.2f} N toward '
            f'{math.degrees(math.atan2(fy, fx)):.0f} deg at {height:.2f} m up the pole')

    def random_force(self, magnitude):
        heading = random.uniform(-math.pi, math.pi)
        return magnitude * math.cos(heading), magnitude * math.sin(heading)

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if t == 0.0:
            return  # /clock not received yet
        if self.start_time is None:
            self.start_time = t
            self.clear_pub.publish(self.entity())
            return

        if self.nudge is None:
            if not self.released and t - self.start_time >= self.release_delay:
                self.released = True
                self.start_nudge(t, *self.random_force(self.release_force), 'Release nudge')
            elif self.pending:
                self.start_nudge(t, *self.pending.pop(0), 'Manual nudge')

        target = [0.0, 0.0]
        if self.nudge is not None:
            start, tx, ty = self.nudge
            phase = (t - start) / self.duration
            if phase >= 1.0:
                self.nudge = None
                self.clear_time = t + 0.1
            else:
                shape = math.sin(math.pi * phase)
                target = [tx * shape, ty * shape]

        delta = [tg - ap for tg, ap in zip(target, self.applied)]
        if abs(delta[0]) + abs(delta[1]) > 1e-6:
            msg = EntityWrench()
            msg.entity = self.entity()
            msg.wrench.torque.x = delta[0]
            msg.wrench.torque.y = delta[1]
            self.wrench_pub.publish(msg)
            self.applied = target

        # Wait for the last update to arrive first; clear and wrench are separate topics
        if self.nudge is None and self.clear_time is not None and t >= self.clear_time:
            self.clear_pub.publish(self.entity())
            self.clear_time = None


def main(args=None):
    rclpy.init(args=args)
    node = Disturbance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

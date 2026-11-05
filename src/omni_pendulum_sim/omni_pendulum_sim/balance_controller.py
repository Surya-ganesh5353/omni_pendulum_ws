"""
Balance a 2-DOF inverted pendulum on a mecanum (omnidirectional) AgileX LIMO.

Each tilt axis is treated as an independent cart-pole:
  pitch (tilt toward +X)  -> drive the base along X
  roll  (tilt toward -Y)  -> strafe the base along Y
so whichever way the pole starts to fall, the robot drives in that direction to
get back underneath it.

The base only accepts velocity commands (/cmd_vel), but a pendulum is balanced
by *accelerating* under it, so the controller computes a desired acceleration
and integrates it into a velocity command. It is an LQR state feedback written
as a cascade, so the parts can be limited separately:

  lean_ref = -(k_pos * (p - p_ref) + k_vel * (v - v_ref)) / k_theta
  a = k_theta * (theta - lean_ref) + k_omega * theta_dot
  v += a * dt,  p += v * dt

The inner line keeps the pole up and is never limited. The outer line is how
the robot slows down and returns home: to brake it has to tip the pole slightly
backwards first (an inverted pendulum cannot be stopped any other way). Capping
lean_ref at max_lean_rad stops the robot from braking so hard after a big
nudge that the pole swings over to the other side and falls; capping the
position term at max_return_speed keeps the drive back home gentle.

At startup the controller stays idle and lets the pole fall freely until it
has tipped past engage_tilt_rad, then catches it.

An upright pole held by a good controller barely moves, so balancing runs as a
repeating cycle: once the pole is back under control the robot lets go (no
acceleration) and gravity tips the pole again; at catch_tilt_rad the robot
drives under it and brings it back.

The default gains come from a deliberately soft LQR design (so the pole visibly
sways while the robot works to keep it up) on the linearised model
  theta_ddot = (m g l / J) * theta - (m l / J) * a
for the pendulum in robot.urdf.xacro (rod 0.5 m / 0.1 kg + 0.1 kg ball on top).
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class AxisController:
    """LQR balancer for one tilt axis."""

    def __init__(self, k_theta, k_omega, k_pos, k_vel):
        self.k_theta = k_theta
        self.k_omega = k_omega
        self.k_pos = k_pos
        self.k_vel = k_vel
        self.reset()

    def reset(self):
        self.vel = 0.0
        self.pos = 0.0
        self.pos_ref = 0.0

    def lean_ref(self, vel_ref, max_return_speed, dt):
        """Lean the pole should have to brake / return home (before limiting)."""
        self.pos_ref += vel_ref * dt
        pos_term = self.k_pos * (self.pos - self.pos_ref)
        pos_limit = self.k_vel * max_return_speed
        pos_term = max(min(pos_term, pos_limit), -pos_limit)
        return -(pos_term + self.k_vel * (self.vel - vel_ref)) / self.k_theta

    def accel(self, theta, theta_dot, lean_ref):
        return self.k_theta * (theta - lean_ref) + self.k_omega * theta_dot

    def integrate(self, accel, dt):
        self.vel += accel * dt
        self.pos += self.vel * dt


def limit_norm(x, y, limit):
    """Scale the vector (x, y) down to length `limit`, keeping its direction."""
    norm = math.hypot(x, y)
    if norm <= limit:
        return x, y
    return x * limit / norm, y * limit / norm


class OmnidirectionalBalanceController(Node):

    def __init__(self):
        super().__init__('balance_controller')

        self.declare_parameter('k_theta', 33.4)
        self.declare_parameter('k_omega', 7.1)
        self.declare_parameter('k_pos', 1.3)
        self.declare_parameter('k_vel', 3.1)
        # Hard safety limit; normally the lean limit keeps the robot well below it
        self.declare_parameter('max_vel', 2.0)
        # How far the controller may deliberately tip the pole to brake or
        # return home, and how fast it drives back to where it started
        self.declare_parameter('max_lean_rad', 0.06)
        self.declare_parameter('max_return_speed', 0.3)
        # What the mecanum wheels can actually deliver in any direction (the
        # diagonal is the weakest). The controller assumes the base follows its
        # commands, so asking for more than this makes it lose track and overshoot.
        self.declare_parameter('max_accel', 6.0)
        self.declare_parameter('max_tilt_rad', 0.6)
        self.declare_parameter('engage_tilt_rad', 0.14)
        self.declare_parameter('control_period', 0.005)
        # Let-go / catch cycle: catch at this tilt, or sooner if the pole moves faster
        # than gravity alone would make it (a nudge). Let go again once the pole is
        # within release_tilt_rad of the requested lean and slower than release_rate
        self.declare_parameter('catch_tilt_rad', 0.1)
        self.declare_parameter('catch_rate', 0.6)
        self.declare_parameter('release_tilt_rad', 0.015)
        self.declare_parameter('release_rate', 0.1)

        gains = [self.get_parameter(n).value for n in ('k_theta', 'k_omega', 'k_pos', 'k_vel')]
        self.x_axis = AxisController(*gains)
        self.y_axis = AxisController(*gains)
        self.max_vel = self.get_parameter('max_vel').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_lean = self.get_parameter('max_lean_rad').value
        self.max_return_speed = self.get_parameter('max_return_speed').value
        self.max_tilt_rad = self.get_parameter('max_tilt_rad').value
        self.engage_tilt_rad = self.get_parameter('engage_tilt_rad').value
        self.control_period = self.get_parameter('control_period').value
        self.catch_tilt_rad = self.get_parameter('catch_tilt_rad').value
        self.catch_rate = self.get_parameter('catch_rate').value
        self.release_tilt_rad = self.get_parameter('release_tilt_rad').value
        self.release_rate = self.get_parameter('release_rate').value

        # Optional user velocity request (e.g. from teleop_twist_keyboard)
        self.vel_ref_x = 0.0
        self.vel_ref_y = 0.0
        self.last_stamp = None
        self.engaged = False
        self.catching = True
        self.fallen = False

        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_ref', self.ref_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Balance controller started')

    def ref_callback(self, msg):
        self.vel_ref_x = msg.linear.x
        self.vel_ref_y = msg.linear.y

    def joint_callback(self, msg):
        try:
            pitch_idx = msg.name.index('pendulum_pitch_joint')
            roll_idx = msg.name.index('pendulum_roll_joint')
        except ValueError:
            return

        theta_pitch = msg.position[pitch_idx]
        theta_roll = msg.position[roll_idx]
        omega_pitch = msg.velocity[pitch_idx] if msg.velocity else 0.0
        omega_roll = msg.velocity[roll_idx] if msg.velocity else 0.0

        # Use the message's simulation timestamp so dt matches simulated physics
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt = stamp - self.last_stamp
        if dt < self.control_period:
            return
        self.last_stamp = stamp

        cmd = Twist()

        # Let the pole fall freely until it has tipped far enough, then catch it
        if not self.engaged:
            if max(abs(theta_pitch), abs(theta_roll)) < self.engage_tilt_rad:
                self.pub.publish(cmd)
                return
            self.engaged = True
            self.get_logger().info(
                f'Pole tipped past {math.degrees(self.engage_tilt_rad):.0f} deg, balancing')

        # Safety cutoff once the pendulum has fallen past recovery
        if abs(theta_pitch) > self.max_tilt_rad or abs(theta_roll) > self.max_tilt_rad:
            if not self.fallen:
                self.get_logger().warn('Pendulum fell over, stopping base')
                self.fallen = True
            self.x_axis.reset()
            self.y_axis.reset()
            self.pub.publish(cmd)
            return
        self.fallen = False

        # Positive pitch tips toward +X; positive roll tips toward -Y, so the
        # y axis works in negated coordinates. Limits are applied to the 2D
        # vectors so diagonal moves keep their direction.
        lean_x = self.x_axis.lean_ref(self.vel_ref_x, self.max_return_speed, dt)
        lean_y = self.y_axis.lean_ref(-self.vel_ref_y, self.max_return_speed, dt)
        lean_x, lean_y = limit_norm(lean_x, lean_y, self.max_lean)

        # Gravity takes over while letting go; the robot just keeps its velocity
        rate = math.hypot(omega_pitch, omega_roll)
        if self.catching:
            error = math.hypot(theta_pitch - lean_x, theta_roll - lean_y)
            self.catching = error > self.release_tilt_rad or rate > self.release_rate
        else:
            tilt = math.hypot(theta_pitch, theta_roll)
            self.catching = tilt > self.catch_tilt_rad or rate > self.catch_rate

        ax = ay = 0.0
        if self.catching:
            ax = self.x_axis.accel(theta_pitch, omega_pitch, lean_x)
            ay = -self.y_axis.accel(theta_roll, omega_roll, lean_y)
            ax, ay = limit_norm(ax, ay, self.max_accel)
        self.x_axis.integrate(ax, dt)
        self.y_axis.integrate(-ay, dt)

        vx, vy = limit_norm(self.x_axis.vel, -self.y_axis.vel, self.max_vel)
        self.x_axis.vel, self.y_axis.vel = vx, -vy
        cmd.linear.x = vx
        cmd.linear.y = vy
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = OmnidirectionalBalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""
Nudge the pendulum from a terminal with the arrow keys.

Run in its own terminal while the simulation is running:
    ros2 run omni_pendulum_sim keyboard_nudge

Key presses are published as Qt key codes on /keyboard/keypress, the same
topic Gazebo's Key Publisher plugin uses, and the disturbance node turns them
into nudges.
"""

import os
import select
import sys
import termios
import tty

from omni_pendulum_sim.disturbance import KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_UP
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


ARROWS = {'A': KEY_UP, 'B': KEY_DOWN, 'C': KEY_RIGHT, 'D': KEY_LEFT}
NAMES = {KEY_UP: 'forward +X', KEY_DOWN: 'back -X', KEY_LEFT: 'left +Y', KEY_RIGHT: 'right -Y'}

HELP = """
Nudge the pendulum:
   Up / Down     push forward / back
   Left / Right  push left / right
   + / -         stronger / weaker push
   q             quit
"""


def read_key(timeout):
    # Read raw bytes: Python's buffered stdin would swallow the rest of an
    # arrow key's escape sequence and hide it from select()
    fd = sys.stdin.fileno()
    if not select.select([fd], [], [], timeout)[0]:
        return None
    data = os.read(fd, 8).decode(errors='ignore')
    if data.startswith('\x1b'):  # arrow keys arrive as ESC [ A..D
        while len(data) < 3 and select.select([fd], [], [], 0.05)[0]:
            data += os.read(fd, 8).decode(errors='ignore')
        return ARROWS.get(data[2]) if data[1:2] == '[' and len(data) >= 3 else None
    return data[0] if data else None


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_nudge needs an interactive terminal; run it with ros2 run.')
        return

    rclpy.init(args=args)
    node = Node('keyboard_nudge')
    pub = node.create_publisher(Int32, '/keyboard/keypress', 10)
    print(HELP)

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.1)
            if key is None:
                continue
            if key in ('q', '\x03'):
                break
            code = key if isinstance(key, int) else ord(key)
            if code in NAMES or key in '+=-_':
                pub.publish(Int32(data=code))
                if code in NAMES:
                    print(f'Nudge {NAMES[code]}', flush=True)
                else:
                    print('Stronger push' if key in '+=' else 'Weaker push', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

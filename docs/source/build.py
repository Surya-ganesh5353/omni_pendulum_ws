"""Build the project guide as HTML, then print it to PDF with headless Chrome.

usage: python3 build.py <package_dir> <output.pdf>
"""
import html
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from expl_python import BALANCE_CONTROLLER, DISTURBANCE, KEYBOARD_NUDGE  # noqa: E402
from expl_other import LAUNCH, SETUP, URDF  # noqa: E402

PKG, OUT = sys.argv[1], sys.argv[2]
HERE = os.path.dirname(os.path.abspath(__file__))

FILES = [
    ('balance_controller.py', 'omni_pendulum_sim/balance_controller.py', BALANCE_CONTROLLER,
     'The brain of the robot: reads the pendulum angles and decides how the base moves.'),
    ('disturbance.py', 'omni_pendulum_sim/disturbance.py', DISTURBANCE,
     'Simulates a person nudging the pole: automatic random taps, arrow keys and /nudge.'),
    ('keyboard_nudge.py', 'omni_pendulum_sim/keyboard_nudge.py', KEYBOARD_NUDGE,
     'Optional terminal program that turns arrow-key presses into nudges.'),
    ('sim.launch.py', 'launch/sim.launch.py', LAUNCH,
     'Starts everything: Gazebo, the ROS-Gazebo bridge, the robot and the nodes.'),
    ('robot.urdf.xacro', 'urdf/robot.urdf.xacro', URDF,
     'The robot model: LIMO chassis, mecanum wheels, platform, pendulum and Gazebo plugins.'),
    ('setup.py', 'setup.py', SETUP,
     'Tells colcon how to build and install the package and its programs.'),
]


def fmt(text):
    """Escape text, keep <sup>/<sub>, turn `x` into <code>x</code>."""
    t = html.escape(text, quote=False)
    t = re.sub(r'&lt;(/?)(sup|sub)&gt;', r'<\1\2>', t)
    return re.sub(r'`([^`]+)`', r'<code>\1</code>', t)


def listing(title, rel, expl, blurb):
    lines = open(os.path.join(PKG, rel)).read().split('\n')
    if lines and lines[-1] == '':
        lines.pop()
    # map: last line of a group -> (first, text); single lines -> (n, text)
    after = {}
    for key, text in expl.items():
        first, last = (key, key) if isinstance(key, int) else key
        assert 1 <= first <= last <= len(lines), (title, key, len(lines))
        after[last] = (first, text)
    explained = set()
    for key in expl:
        first, last = (key, key) if isinstance(key, int) else key
        explained.update(range(first, last + 1))
    missing = [i + 1 for i, l in enumerate(lines) if l.strip() and (i + 1) not in explained]
    assert not missing, (title, 'unexplained lines', missing)

    out = [f'<h2 class="file" id="{title}">{html.escape(title)}</h2>',
           f'<p class="path"><code>src/omni_pendulum_sim/{rel}</code> &middot; {len(lines)} lines</p>',
           f'<p>{fmt(blurb)}</p>', '<div class="listing">']
    for n, line in enumerate(lines, 1):
        cls = 'code blank' if not line.strip() else 'code'
        out.append(f'<div class="{cls}"><span class="ln">{n}</span>'
                   f'<span class="src">{html.escape(line) or "&nbsp;"}</span></div>')
        if n in after:
            first, text = after[n]
            rng = f'<span class="rng">Lines {first}&ndash;{n}</span> ' if first != n else ''
            out.append(f'<div class="ex">{rng}{fmt(text)}</div>')
    out.append('</div>')
    return '\n'.join(out)


NARRATIVE = open(os.path.join(HERE, 'narrative.html')).read()
CSS = open(os.path.join(HERE, 'style.css')).read()

toc_files = ''.join(f'<li><a href="#{t}">{t}</a></li>' for t, *_ in FILES)
body = NARRATIVE.replace('<!--TOC_FILES-->', toc_files)
body += '<section class="code-section"><h1 id="code">Part 6 &mdash; Line-by-line code walkthrough</h1>'
body += ('<p>Each source line is shown with its line number. The grey box under a line (or under a '
         'group of lines) explains what it does and why. Blank lines are shown but not explained; '
         'docstrings and closing brackets are explained as a group.</p>')
for f in FILES:
    body += listing(*f)
body += '</section>'

doc = f'''<!doctype html><html><head><meta charset="utf-8">
<title>Omni Pendulum Robot - Project Guide</title><style>{CSS}</style></head>
<body>{body}</body></html>'''

html_path = os.path.join(HERE, 'guide.html')
open(html_path, 'w').write(doc)
subprocess.run(['google-chrome', '--headless=new', '--disable-gpu', '--no-pdf-header-footer',
                f'--print-to-pdf={OUT}', f'file://{html_path}'],
               check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=180)
print('wrote', OUT)

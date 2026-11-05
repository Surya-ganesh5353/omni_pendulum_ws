"""Draw the guide's figures from the recorded runs in data/ (headless Gazebo)."""
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
BLUE, ORANGE = '#2a78d6', '#eb6834'
INK, INK2, RULE = '#0b0b0b', '#52514e', '#e3e2de'
plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 9, 'text.color': INK, 'axes.labelcolor': INK2,
    'xtick.color': INK2, 'ytick.color': INK2, 'axes.edgecolor': RULE, 'axes.grid': True,
    'grid.color': RULE, 'grid.linewidth': 0.6, 'axes.axisbelow': True, 'lines.linewidth': 1.5,
    'axes.spines.top': False, 'axes.spines.right': False, 'svg.fonttype': 'path',
    'xtick.color': INK2, 'axes.titlesize': 9.5, 'axes.titleweight': 'bold',
    'axes.titlelocation': 'left'})


def load(name):
    rows = list(csv.DictReader(open(os.path.join(HERE, 'data', name + '.csv'))))
    d = {k: [float(r[k]) for r in rows] for k in rows[0]}
    d['tilt'] = [math.degrees(math.hypot(p, r)) for p, r in zip(d['pitch'], d['roll'])]
    d['speed'] = [math.hypot(a, b) for a, b in zip(d['vx'], d['vy'])]
    return d


def save(fig, name):
    fig.savefig(os.path.join(HERE, 'figures', name + '.svg'), bbox_inches='tight')
    plt.close(fig)


def window(d, a, b):
    return [i for i, t in enumerate(d['t']) if a <= t <= b]


def fig_fall():
    d = load('fall')
    alpha = 9.81 * 0.075 / 0.03335833
    i0 = next(i for i, a in enumerate(d['tilt']) if a >= 1.0)
    idx = [i for i in range(i0, len(d['t'])) if d['tilt'][i] <= 90.0]
    t0 = d['t'][i0]
    th = math.radians(d['tilt'][i0])
    w = math.hypot(d['wp'][i0], d['wr'][i0])
    tt, yy, t, dt, marks = [], [], 0.0, 1e-4, {}
    while th < math.pi / 2:
        w += alpha * math.sin(th) * dt
        th += w * dt
        t += dt
        tt.append(t)
        yy.append(math.degrees(th))
        for m in (10, 45, 90):
            if m not in marks and yy[-1] >= m:
                marks[m] = t
    fig, ax = plt.subplots(figsize=(6.6, 2.6))
    ax.plot([d['t'][i] - t0 for i in idx], [d['tilt'][i] for i in idx], color=BLUE,
            label='Gazebo simulation')
    ax.plot(tt, yy, color=ORANGE, linestyle=(0, (4, 2)), label='Theory (fixed pivot)')
    ax.set(xlabel='Time since the pole passed 1° (s)', ylabel='Tilt (°)', ylim=(0, 95))
    ax.set_title('Free fall with the controller off')
    ax.legend(frameon=False, loc='upper left')
    save(fig, 'fall')
    meas = {m: d['t'][next(i for i, a in enumerate(d['tilt']) if a >= m)] - t0 for m in marks}
    print('free fall theory', {k: round(v, 3) for k, v in marks.items()},
          'sim', {k: round(v, 3) for k, v in meas.items()})


def tilt_speed(d, a, b, title, name, marks=(), height=3.5):
    idx = window(d, a, b)
    t = [d['t'][i] for i in idx]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.6, height), sharex=True,
                                   gridspec_kw={'hspace': 0.28})
    ax1.plot(t, [d['tilt'][i] for i in idx], color=BLUE)
    ax1.axhline(math.degrees(0.1), color=INK2, linewidth=0.8, linestyle=(0, (3, 3)))
    ax1.text(t[-1], 7.3, 'dashed line: catch at 5.7°', ha='right', va='bottom',
             color=INK2, fontsize=8)
    ax1.set(ylabel='Tilt from vertical (°)', ylim=(0, 8.8))
    ax1.set_title(title)
    ax2.plot(t, [d['speed'][i] for i in idx], color=BLUE)
    ax2.set(ylabel='Robot speed (m/s)', xlabel='Simulation time (s)', ylim=(0, None))
    for x, label in marks:
        for ax in (ax1, ax2):
            ax.axvline(x, color=INK2, linewidth=0.8)
        ax1.annotate(label, (x, 1), xycoords=('data', 'axes fraction'), xytext=(3, -2),
                     textcoords='offset points', va='top', fontsize=8, color=INK)
    save(fig, name)


def fig_path():
    d = load('gravity')
    x0, y0 = d['gx'][0], d['gy'][0]
    x = [a - x0 for a in d['gx']]
    y = [b - y0 for b in d['gy']]
    fig, ax = plt.subplots(figsize=(6.6, 1.9))
    ax.plot(y, x, color=BLUE, linewidth=1.0)
    ax.plot([0], [0], 'o', markersize=6, color=INK, markeredgecolor='white')
    ax.annotate('start', (0, 0), xytext=(4, 8), textcoords='offset points', fontsize=8)
    ax.set_aspect('equal')
    ax.set(xlabel='Sideways position, y (m)', ylabel='Forward, x (m)',
           xlim=(-0.35, 0.85), ylim=(-0.12, 0.17))
    ax.set_title('Where the robot went in 120 s of gravity-only balancing')
    save(fig, 'path')


fig_fall()
g = load('gravity')
tilt_speed(g, 60, 70, 'The let-go / catch cycle, gravity only (10 s of a 120 s run)', 'cycle')
n = load('nudge')
tilt_speed(n, 20, 50, 'Response to two manual nudges', 'nudge',
           marks=[(25, '0.8 N'), (40, '1.2 N')])
fig_path()

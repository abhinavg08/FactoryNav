"""Draw the normal factory's real geometry, station colours and demo goals."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle


ROOT = Path(__file__).resolve().parents[1]


def main():
    scene = json.loads((ROOT / 'factorynav_bringup/config/factory_scene.json').read_text())
    fig = plt.figure(figsize=(14, 9), facecolor='#f4f7fb')
    grid = fig.add_gridspec(1, 2, width_ratios=(3.8, 1.8), wspace=.08,
                           left=.055, right=.97, top=.87, bottom=.13)
    ax = fig.add_subplot(grid[0, 0])
    key = fig.add_subplot(grid[0, 1])
    ax.set_facecolor('#ffffff')
    ax.set(xlim=(-6.5, 6.5), ylim=(-5.5, 5.5), aspect='equal',
           xlabel='Factory x (m)', ylabel='Factory y (m)')
    ax.set_xticks(range(-6, 7, 2)); ax.set_yticks(range(-5, 6))
    ax.grid(color='#dbe2eb', linewidth=.6, zorder=0)
    for side in ax.spines.values(): side.set_color('#cbd5e1')

    for item in scene['obstacles']:
        ax.add_patch(Rectangle((item['x'] - item['width']/2,
                                item['y'] - item['depth']/2),
                               item['width'], item['depth'],
                               facecolor=item['color'], edgecolor='#253747',
                               linewidth=1.1, zorder=2))
        label = {'storage_rack_north': 'Storage rack',
                 'storage_rack_south': 'Storage rack',
                 'workbench_A': 'Workbench A', 'workbench_B': 'Workbench B',
                 'pallets': 'Pallets'}.get(item['name'])
        if label:
            ax.text(item['x'], item['y'], label, ha='center', va='center',
                    fontsize=9, weight='bold', color='white', zorder=3)
    for lane_y in (-.8, .8):
        ax.plot([-5.15, 5.15], [lane_y, lane_y], color='#e9bb22',
                linewidth=2.2, zorder=1)
    for station in scene['stations']:
        ax.add_patch(Rectangle((station['x']-.6, station['y']-.6), 1.2, 1.2,
                               facecolor=station['color'], edgecolor='#1e293b',
                               linewidth=1.5, zorder=4))

    ax.text(-4.5, 1.12, 'START\nLoading station', ha='center', va='center',
            fontsize=11, weight='bold', color='#08785d', zorder=5)
    ax.text(4.5, 1.12, 'STOP / GOAL\nWorkstation B', ha='center', va='center',
            fontsize=11, weight='bold', color='#165db9', zorder=5)
    ax.annotate('Workstation A\noptional goal', xy=(-4.2, 2.1), xytext=(-3.15, 2.1),
                ha='left', va='center', fontsize=10, color='#4b3808',
                arrowprops={'arrowstyle': '-', 'color': '#8a6a13'}, zorder=5)
    ax.add_patch(Circle((-4.5, 0), .19, facecolor='#374151',
                        edgecolor='white', linewidth=1.2, zorder=6))
    ax.add_patch(FancyArrowPatch((-4.5, 0), (-4.17, 0), arrowstyle='-|>',
                                 mutation_scale=12, color='white', zorder=7))
    ax.add_patch(FancyArrowPatch((-3.65, 0), (3.65, 0), arrowstyle='-|>',
                                 mutation_scale=23, color='#2563eb',
                                 linewidth=2.6, linestyle='--', zorder=5))
    ax.text(0, .32, 'Default mission direction', ha='center', fontsize=10,
            weight='bold', color='#1d4ed8')
    ax.text(0, -.58, 'Open driving aisle', ha='center', fontsize=10, color='#64748b')

    key.set_axis_off(); key.set(xlim=(0, 1), ylim=(0, 1))
    key.text(.03, .94, 'STATION PADS', fontsize=14, weight='bold', color='#17283c')
    entries = [
        (.85, (.1, .65, .5), 'Green: Loading station', 'Start at (-4.5, 0).'),
        (.70, (.95, .68, .1), 'Yellow: Workstation A', 'Optional goal at (-4.2, 2.1).\nNot visited by the default run.'),
        (.52, (.2, .5, .95), 'Blue: Workstation B', 'Default finish at (4.5, 0).'),
    ]
    for y, color, title, body in entries:
        key.add_patch(Rectangle((.03, y-.018), .065, .047, facecolor=color,
                                edgecolor='#334155', transform=key.transAxes))
        key.text(.13, y+.015, title, fontsize=11, weight='bold', va='center', color='#17283c')
        key.text(.13, y-.035, body, fontsize=10, va='top', color='#475569', linespacing=1.5)
    key.text(.03, .35, 'SOLID OBSTACLES', fontsize=14, weight='bold', color='#17283c')
    for y, color, text in [(.28, (.12, .36, .58), 'Large dark-blue blocks: racks'),
                           (.21, (.9, .58, .16), 'Orange boxes: workbenches'),
                           (.14, (.55, .31, .14), 'Brown box: pallets'),
                           (.07, (.30, .36, .42), 'Grey boundary: walls')]:
        key.add_patch(Rectangle((.03, y-.012), .055, .028, facecolor=color))
        key.text(.12, y, text, fontsize=10, va='center', color='#334155')

    fig.text(.055, .95, 'Normal factory — labelled map', fontsize=23,
             weight='bold', color='#17283c')
    fig.text(.055, .905, 'Default demo: Loading station → Workstation B',
             fontsize=14, color='#475569')
    fig.text(.055, .065, 'Station pads are flat markers. The robot stops at the goal you send; colours alone do not trigger stops.',
             fontsize=11, color='#334155')
    fig.text(.055, .035, 'Geometry and colours: factory_scene.json. Dashed arrow shows the intended goal direction, not a recorded trajectory.',
             fontsize=9, color='#64748b')
    output = ROOT / 'media/normal_factory_map.png'
    fig.savefig(output, dpi=170, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(output)


if __name__ == '__main__':
    main()

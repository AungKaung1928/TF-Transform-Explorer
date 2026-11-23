#!/usr/bin/env python3
"""plot_tf_tree.py [out.svg]: draw the TF tree this package sets up, read from its own files.

map -> odom: config/slam_params.yaml (slam_toolbox). odom -> base_footprint: the diff-drive plugin
in description/turtlebot3_burger.urdf.xacro (Gazebo). Robot links: the URDF joints
(robot_state_publisher).
base_link -> sensor_mount -> rotating_sensor: src/frame_broadcaster_node.cpp.
Shaded: the frames src/tf_anomaly_detector.cpp checks.
The configured tree, not a recorded `view_frames`. Standard library only.
"""
import re
import sys
import xml.etree.ElementTree as ET

out = sys.argv[1] if len(sys.argv) > 1 else 'docs/tf_tree.svg'
edges = []  # (parent, child, publisher, dynamic)

slam = open('config/slam_params.yaml').read()
edges.append((re.search(r'map_frame:\s*(\S+)', slam).group(1),
              re.search(r'odom_frame:\s*(\S+)', slam).group(1), 'slam_toolbox', True))

urdf = ET.parse('description/turtlebot3_burger.urdf.xacro').getroot()
for plugin in urdf.iter('plugin'):
    if plugin.find('frame_id') is not None and plugin.find('child_frame_id') is not None:
        edges.append((plugin.find('frame_id').text, plugin.find('child_frame_id').text,
                      'gz diff-drive', True))
for joint in urdf.iter('joint'):
    if joint.find('parent') is not None:
        edges.append((joint.find('parent').get('link'), joint.find('child').get('link'),
                      'robot_state_publisher', joint.get('type') != 'fixed'))

cpp = open('src/frame_broadcaster_node.cpp').read()
pairs = re.findall(r'frame_id = "([^"]+)";\s*\n\s*t\.child_frame_id = "([^"]+)";', cpp)
static_end = cpp.index('void FrameBroadcasterNode::timer_callback')
for m, (p, c) in zip(re.finditer(r'header\.frame_id = "', cpp), pairs):
    edges.append((p, c, 'frame_broadcaster_node', m.start() > static_end))

children = {}
for p, c, _, _ in edges:
    children.setdefault(p, []).append(c)
root = next(p for p, _, _, _ in edges if all(p != c for _, c, _, _ in edges))

# Layout: depth = row, leaves left to right, a parent centred over its children.
pos, rows, col = {}, {}, [0]


def place(f, d):
    kids = children.get(f, [])
    for k in kids:
        place(k, d + 1)
    x = (pos[kids[0]][0] + pos[kids[-1]][0]) / 2 if kids else col[0]
    if not kids:
        col[0] += 1
    pos[f] = (x, d)


place(root, 0)
BW, BH, DX, DY = 128, 30, 140, 74
W = int(col[0] * DX) + 20
H = int((max(d for _, d in pos.values()) + 1) * DY) + 104


def cx(f):
    return 10 + pos[f][0] * DX + DX / 2


def cy(f):
    return 12 + pos[f][1] * DY + BH / 2


det = open('src/tf_anomaly_detector.cpp').read()
pat = r'check_single_transform\("([^"]+)", "([^"]+)"\)'
checked = {f for pair in re.findall(pat, det) for f in pair}
pubs = list(dict.fromkeys(pub for _, _, pub, _ in edges))
colour = dict(zip(pubs, ['#2563eb', '#d97706', '#059669', '#db2777', '#6b7785']))
el = [f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
for p, c, pub, dyn in edges:
    dash = '' if dyn else ' stroke-dasharray="5 4"'
    el.append(f'<line x1="{cx(p):.0f}" y1="{cy(p) + BH / 2:.0f}" x2="{cx(c):.0f}" '
              f'y2="{cy(c) - BH / 2:.0f}" stroke="{colour[pub]}" stroke-width="2"{dash}/>')
for i, pub in enumerate(pubs):
    el.append(f'<line x1="10" y1="{H - 74 + 16 * i}" x2="34" y2="{H - 74 + 16 * i}" '
              f'stroke="{colour[pub]}" stroke-width="2"/><text x="40" y="{H - 70 + 16 * i}" '
              f'fill="#374151" font-size="11">{pub}</text>')
for f in pos:
    fill = '#dbeafe' if f in checked else '#f3f4f6'
    el.append(f'<rect x="{cx(f) - BW / 2:.0f}" y="{cy(f) - BH / 2:.0f}" '
              f'width="{BW}" height="{BH}" rx="6" fill="{fill}" stroke="#374151"/>'
              f'<text x="{cx(f):.0f}" y="{cy(f) + 4:.0f}" text-anchor="middle" fill="#111827" '
              f'font-size="12">{f}</text>')
el.append(f'<text x="10" y="{H - 10}" fill="#6b7785" font-size="11">'
          'solid = moving, dashed = fixed; shaded boxes = frames tf_anomaly_detector checks</text>')
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
       f'font-family="sans-serif">' + ''.join(el) + '</svg>\n')
open(out, 'w').write(svg)
print(f'{len(edges)} edges, root {root} -> {out}')

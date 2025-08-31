"""
Simple ROS topic inspector: finds rospy.Publisher and rospy.Subscriber calls with literal topic names.
Prints a report grouped by topic showing publishers and subscribers (file, line, msg type if literal).

Usage: python3 src/scripts/ros_topic_inspector.py
"""
import re
import os
import sys
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PAT_PUB = re.compile(r"rospy\.Publisher\(\s*([rR]?[\"\'])(.+?)\1\s*,\s*([A-Za-z0-9_\.]+)")
PAT_SUB = re.compile(r"rospy\.Subscriber\(\s*([rR]?[\"\'])(.+?)\1\s*,\s*([A-Za-z0-9_\.]+)")

pubs = defaultdict(list)
subs = defaultdict(list)

for dirpath, dirnames, filenames in os.walk(ROOT):
    # skip typical large dirs
    if 'node_modules' in dirpath or '.git' in dirpath:
        continue
    for fn in filenames:
        if not fn.endswith('.py'):
            continue
        path = os.path.join(dirpath, fn)
        try:
            with open(path, 'r', encoding='utf-8') as f:
                text = f.read()
        except Exception:
            continue
        for m in PAT_PUB.finditer(text):
            topic = m.group(2)
            msg = m.group(3)
            line = text.count('\n', 0, m.start()) + 1
            pubs[topic].append((path, line, msg))
        for m in PAT_SUB.finditer(text):
            topic = m.group(2)
            msg = m.group(3)
            line = text.count('\n', 0, m.start()) + 1
            subs[topic].append((path, line, msg))

all_topics = set(pubs) | set(subs)

print('\nROS topic inspector report (literal topics only)')
print('='*60)
print(f'Total files scanned under: {ROOT}')
print(f'Topics with literal names found: {len(all_topics)}')

for t in sorted(all_topics):
    print('\n' + '-'*60)
    print(f'Topic: {t}')
    if t in pubs:
        print('  Publishers:')
        for p in pubs[t]:
            print(f'    - {p[0]}:{p[1]}  type={p[2]}')
    else:
        print('  Publishers: (none found)')
    if t in subs:
        print('  Subscribers:')
        for s in subs[t]:
            print(f'    - {s[0]}:{s[1]}  type={s[2]}')
    else:
        print('  Subscribers: (none found)')

# quick stats
only_pubs = [t for t in all_topics if t in pubs and t not in subs]
only_subs = [t for t in all_topics if t in subs and t not in pubs]

print('\n' + '='*60)
print(f'Topics published but not subscribed (literal): {len(only_pubs)}')
for t in only_pubs[:50]:
    print('  ' + t)
print(f'Topics subscribed but not published (literal): {len(only_subs)}')
for t in only_subs[:50]:
    print('  ' + t)

print('\nNote: dynamic topic names (rospy.get_param, f-strings, variables) are not detected by this script.')
print('Use this report as a starting point to inspect mismatches and latching/queue_size usage.')

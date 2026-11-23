import json, re, sys, collections
t = open(sys.argv[1], encoding='utf8').read()
nets = {}
w = collections.defaultdict(collections.Counter)
L = collections.defaultdict(float)
for m in re.finditer(r'\(segment\s*\(start ([\d.-]+) ([\d.-]+)\)\s*\(end ([\d.-]+) ([\d.-]+)\)\s*\(width ([\d.]+)\)\s*\(layer "([^"]+)"\)\s*\(net "?([^")]*)"?\)', t):
    x1, y1, x2, y2, wd = map(float, m.groups()[:5])
    n = m.group(7)
    l = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    w[n][wd] += 1
    L[(n, m.group(6))] += l
vias = collections.Counter(re.findall(r'\(via\b[\s\S]*?\(net "?([^")]*)"?\)', t))
print('vias', sum(vias.values()))
for n in sorted(w, key=lambda n: -max(w[n])):
    print(f'{n:14s} widths {dict(w[n])}  F {L[(n,"F.Cu")]:.1f} B {L[(n,"B.Cu")]:.1f} vias {vias.get(n,0)}')

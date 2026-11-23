import re, sys, collections


def stats(fn):
    t = open(fn, encoding='utf8').read()
    L = collections.defaultdict(float)
    for m in re.finditer(r'\(segment\s*\(start ([\d.-]+) ([\d.-]+)\)\s*\(end ([\d.-]+) ([\d.-]+)\)\s*\(width ([\d.]+)\)\s*\(layer "([^"]+)"\)\s*\(net "?([^")]*)"?\)', t):
        x1, y1, x2, y2, wd = map(float, m.groups()[:5])
        L[(m.group(7), m.group(6))] += ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    for m in re.finditer(r'\(arc\s*\(start ([\d.-]+) ([\d.-]+)\)\s*\(mid [\d.-]+ [\d.-]+\)\s*\(end ([\d.-]+) ([\d.-]+)\)\s*\(width ([\d.]+)\)\s*\(layer "([^"]+)"\)\s*\(net "?([^")]*)"?\)', t):
        x1, y1, x2, y2, wd = map(float, m.groups()[:5])
        L[(m.group(7), m.group(6))] += ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    vias = collections.Counter(re.findall(r'\(via\b[\s\S]*?\(net "?([^")]*)"?\)', t))
    return L, vias


a, va = stats(sys.argv[1])
b, vb = stats(sys.argv[2])
nets = sorted(set(n for n, _ in a) | set(n for n, _ in b) | set(va) | set(vb))
rows = []
for n in nets:
    fa, ba, fb, bb = a[(n, 'F.Cu')], a[(n, 'B.Cu')], b[(n, 'F.Cu')], b[(n, 'B.Cu')]
    rows.append((abs(fb + bb - fa - ba) + 5 * abs(vb[n] - va[n]), n, fa, ba, va[n], fb, bb, vb[n]))
rows.sort(reverse=True)
print(f'{"net":14s} | ref F    B  via | new F    B  via')
for r in rows:
    print(f'{r[1]:14s} | {r[2]:5.1f} {r[3]:5.1f} {r[4]:2d} | {r[5]:5.1f} {r[6]:5.1f} {r[7]:2d}')
print('total vias', sum(va.values()), sum(vb.values()), ' length', round(sum(a.values())), round(sum(b.values())))

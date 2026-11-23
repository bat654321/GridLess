import re, sys
t = open(sys.argv[1], encoding='utf8').read()
n = 0; small = 0; arcs = 0; sarc = 0
for m in re.finditer(r'\((segment|arc)\s*\(start ([\d.-]+) ([\d.-]+)\)\s*(?:\(mid [\d.-]+ [\d.-]+\)\s*)?\(end ([\d.-]+) ([\d.-]+)\)', t):
    k, a, b, c, d = m.groups()
    l = ((float(a) - float(c)) ** 2 + (float(b) - float(d)) ** 2) ** .5
    if k == 'segment':
        n += 1; small += l < 0.05
    else:
        arcs += 1; sarc += l < 0.05
print(f'segments {n} (<50um {small}), arcs {arcs} (<50um chord {sarc})')

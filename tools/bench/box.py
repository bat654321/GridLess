import re, sys
t = open(sys.argv[1], encoding='utf8').read()
x0, y0, x1, y1 = map(float, sys.argv[2].split(','))
for m in re.finditer(r'\((segment|arc)\s*\(start ([\d.-]+) ([\d.-]+)\)\s*(?:\(mid [\d.-]+ [\d.-]+\)\s*)?\(end ([\d.-]+) ([\d.-]+)\)\s*\(width ([\d.]+)\)\s*\(layer "([^"]+)"\)\s*\(net "?([^")]*)"?\)', t):
    k, a, b, c, d, w, l, n = m.groups()
    a, b, c, d = map(float, (a, b, c, d))
    if (x0 <= a <= x1 and y0 <= b <= y1) or (x0 <= c <= x1 and y0 <= d <= y1):
        print(f'{n:12s} {l} {k} ({a:.2f},{b:.2f})-({c:.2f},{d:.2f}) w{w}')
for m in re.finditer(r'\(via\s*\(at ([\d.-]+) ([\d.-]+)\)[\s\S]*?\(net "?([^")]*)"?\)', t):
    a, b = float(m[1]), float(m[2])
    if x0 <= a <= x1 and y0 <= b <= y1:
        print(f'{m[3]:12s} VIA ({a:.2f},{b:.2f})')

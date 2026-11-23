import json, re, subprocess, sys, statistics
G = r'C:\Claude_projects\GridLess\target\release\gridless.exe'


def load(fn):
    subprocess.run([G, 'info', fn, '--json', 'tmp_m.json'], capture_output=True)
    return json.load(open('tmp_m.json'))


for fn in sys.argv[1:]:
    b = load(fn)
    nets = [n['name'] for n in b['nets']]
    ds = []
    for v in b['vias']:
        if v['net'] is None or nets[v['net']] == 'GND':
            continue
        q = v['pos']
        best = min(((p['pos']['x'] - q['x']) ** 2 + (p['pos']['y'] - q['y']) ** 2) ** .5 for p in b['pads'] if p['net'] == v['net'])
        ds.append(best)
    ds.sort()
    print(fn, 'signal vias', len(ds), 'median dist to own pad %.2f' % statistics.median(ds), ' <2mm: %d  <4mm: %d  >8mm: %d' % (sum(d < 2 for d in ds), sum(d < 4 for d in ds), sum(d > 8 for d in ds)))

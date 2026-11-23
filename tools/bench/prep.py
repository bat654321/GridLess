"""Prepare StandaloneProgrammer working copies: fixed reference and a bare board."""
import json, re, sys

src = open('ref.kicad_pcb', encoding='utf8').read()


def blocks(t, head):
    """Yield (start, end) of top-level blocks '\n\t(head ...)'."""
    pat = re.compile(r'\n\t\(' + head + r'\b')
    for m in pat.finditer(t):
        i = m.start() + 2
        depth = 0
        j = i
        while True:
            c = t[j]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    break
            elif c == '"':
                j += 1
                while t[j] != '"':
                    if t[j] == '\\':
                        j += 1
                    j += 1
            j += 1
        yield m.start(), j + 1


def drop(t, pred_heads):
    out = []
    last = 0
    spans = []
    for head, pred in pred_heads:
        for s, e in blocks(t, head):
            if pred(t[s:e]):
                spans.append((s, e))
    spans.sort()
    for s, e in spans:
        out.append(t[last:s])
        last = e
    out.append(t[last:])
    return ''.join(out), len(spans)


# 1. outline: drop the thin duplicate Edge.Cuts lines (0.05 mm), keep the 0.254 mm loop
fixed, n = drop(src, [('gr_line', lambda b: '"Edge.Cuts"' in b and '(width 0.05)' in b)])
print('outline duplicates removed:', n)


# 2. zero-length tracks
def zero_len(b):
    m = re.search(r'\(start ([\d.-]+) ([\d.-]+)\)\s*\(end ([\d.-]+) ([\d.-]+)\)', b)
    return m and abs(float(m[1]) - float(m[3])) < 1e-4 and abs(float(m[2]) - float(m[4])) < 1e-4


fixed, n = drop(fixed, [('segment', zero_len)])
print('zero-length segments removed:', n)
fixed = fixed.replace('(allow_soldermask_bridges_in_footprints no)', '(allow_soldermask_bridges_in_footprints yes)')
open('ref_fixed.kicad_pcb', 'w', encoding='utf8').write(fixed)

bare, n = drop(fixed, [('segment', lambda b: True), ('arc', lambda b: True), ('via', lambda b: True)])
print('tracks/vias removed:', n)
open('bare.kicad_pcb', 'w', encoding='utf8').write(bare)

pro = json.load(open('ref.kicad_pro', encoding='utf8'))
ds = pro['board']['design_settings']
ds['rules']['min_resolved_spokes'] = 1
# K1 (0.5 mm pitch) pads are 0.19 mm apart by footprint design
ds['rules']['min_clearance'] = 0.19
ds['allow_soldermask_bridges_in_footprints'] = True
ns = pro['net_settings']
pw = dict(ns['classes'][0])
pw.update({'name': 'Power', 'track_width': 0.4, 'priority': 0})
ns['classes'].append(pw)
ns['netclass_patterns'] = [{'netclass': 'Power', 'pattern': n} for n in ('GND', 'VBUS', '3V3', 'SYS', 'VBAT')]
for name in ('ref_fixed', 'bare'):
    json.dump(pro, open(name + '.kicad_pro', 'w', encoding='utf8'), indent=2)
    open(name + '.kicad_dru', 'w', encoding='utf8').write(
        '(version 1)\n'
        '(rule "K1 fine pitch pads"\n'
        '\t(condition "A.Type == \'Pad\' && B.Type == \'Pad\' && A.memberOfFootprint(\'K1\') && B.memberOfFootprint(\'K1\')")\n'
        '\t(constraint clearance (min 0.19mm)))\n')

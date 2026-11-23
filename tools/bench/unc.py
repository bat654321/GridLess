import json, sys
d = json.load(open(sys.argv[1], encoding='utf8'))
for u in d['unconnected_items']:
    print(' <-> '.join(f"{i['description'][:50]} @({i['pos']['x']:.2f},{i['pos']['y']:.2f})" for i in u['items']))

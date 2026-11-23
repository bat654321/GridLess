import json, sys, collections
d = json.load(open(sys.argv[1], encoding='utf8'))
c = collections.Counter((v['severity'], v['type']) for v in d['violations'])
for k, n in c.most_common():
    print(n, k)
print('unconnected', len(d['unconnected_items']))
if len(sys.argv) > 2:
    for v in d['violations']:
        if v['type'] in sys.argv[2].split(','):
            print(v['type'], v['description'][:100])
            for i in v['items']:
                print('    ', i['description'][:100], i['pos'])

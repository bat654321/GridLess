"""Validate the GridLess interface catalogs.

    python kicad_plugin/check_i18n.py

Checks every ``locales/*.json`` against ``en.json``: the same keys, no empty
values, and an identical set of printf placeholders in every translated string.
Exits with status 1 when a catalog needs attention, so it can run in CI.

Translations are contributed as one JSON file per language; see
``kicad_plugin/common/locales/README.md``.
"""

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LOCALES = os.path.join(HERE, "common", "locales")
PLACEHOLDER = re.compile(r"%(?:%|[0-9]*(?:\.[0-9]+)?[dsf])")


def placeholders(text):
    return sorted(PLACEHOLDER.findall(text))


def load(path):
    with open(path, encoding="utf-8-sig") as f:
        return json.load(f)


def main():
    en_path = os.path.join(LOCALES, "en.json")
    reference = load(en_path)
    keys = set(reference["strings"])
    problems = []

    files = sorted(f for f in os.listdir(LOCALES) if f.endswith(".json"))
    if "en.json" not in files:
        problems.append("en.json is missing")
        files = []

    for name in files:
        code = os.path.splitext(name)[0]
        path = os.path.join(LOCALES, name)
        try:
            data = load(path)
        except ValueError as e:
            problems.append("%s: invalid JSON: %s" % (name, e))
            continue

        if data.get("@code") != code:
            problems.append("%s: @code is %r, expected %r" % (name, data.get("@code"), code))
        if not data.get("@name"):
            problems.append("%s: @name is empty" % name)

        strings = data.get("strings")
        if not isinstance(strings, dict):
            problems.append("%s: no 'strings' object" % name)
            continue

        missing = sorted(keys - set(strings))
        extra = sorted(set(strings) - keys)
        for k in missing:
            problems.append("%s: missing key %r" % (name, k))
        for k in extra:
            problems.append("%s: unknown key %r" % (name, k))

        for key in sorted(keys & set(strings)):
            value = strings[key]
            if not isinstance(value, str) or not value.strip():
                problems.append("%s: empty translation for %r" % (name, key))
                continue
            want = placeholders(key)
            got = placeholders(value)
            if want != got:
                problems.append(
                    "%s: placeholders differ for %r: %s != %s" % (name, key, got, want)
                )

    if problems:
        for p in problems:
            print("ERROR:", p)
        print("\n%d problem(s) in %d catalog(s)" % (len(problems), len(files)))
        return 1

    print("OK: %d interface strings, %d catalogs" % (len(keys), len(files)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Interface localisation for the GridLess KiCad plugins.

KiCad stores the chosen interface language in ``kicad_common.json`` as the
native language label (for example ``"Русский"``), or ``"Default"`` when the
plugin should follow the operating system.  The legacy SWIG plugin can ask
KiCad directly (``pcbnew.GetLanguage()``); the IPC plugin is a separate
process and reads the same setting from the KiCad user configuration.

The catalogs live in ``locales/<code>.json`` and are plain English-to-local
dictionaries, so a translation can be contributed as a single file.  Missing
entries simply fall back to English.

Set ``GRIDLESS_LANG`` to a catalog code (``ru``), a KiCad label
(``Русский``) or ``en`` to override the detection, which is handy for testing.

The catalogs follow the languages shipped with KiCad 10.  Some of the initial
translations (especially Georgian, Tamil, Telugu and Thai) are first drafts and
welcome review by native speakers; see ``locales/README.md``.
"""

import json
import locale
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LOCALES = os.path.join(HERE, "locales")
FALLBACK = "en"

# KiCad's LANGUAGE_DESCR labels (common/pgm_base.cpp).  These are the exact
# strings KiCad writes to system.language; they are deliberately not translated.
KICAD_LABELS = {
    "English": "en",
    "العربية": "ar",
    "فارسی": "fa",
    "Bahasa Indonesia": "id",
    "Български": "bg",
    "Català": "ca",
    "Čeština": "cs",
    "Dansk": "da",
    "Deutsch": "de",
    "Ελληνικά": "el",
    "Eesti": "et",
    "Español": "es",
    "Español (Latinoamericano)": "es_MX",
    "Français": "fr",
    "עברית": "he",
    "हिन्दी": "hi",
    "Hrvatski": "hr",
    "한국어": "ko",
    "Italiano": "it",
    "Latviešu": "lv",
    "Lietuvių": "lt",
    "Magyar": "hu",
    "Nederlands": "nl",
    "Norsk Bokmål": "no",
    "日本語": "ja",
    "ქართული": "ka",
    "ภาษาไทย": "th",
    "Polski": "pl",
    "Português": "pt",
    "Português (Brasil)": "pt_BR",
    "Română": "ro",
    "Русский": "ru",
    "Српски": "sr",
    "Slovenčina": "sk",
    "Slovenščina": "sl",
    "Suomi": "fi",
    "Svenska": "sv",
    "Tiếng Việt": "vi",
    "தமிழ்": "ta",
    "తెలుగు": "te",
    "Türkçe": "tr",
    "Українська": "uk",
    "简体中文": "zh_CN",
    "繁體中文": "zh_TW",
}

# Locale aliases seen in operating-system locale names.
ALIASES = {
    "nb": "no",  # Norwegian Bokmål
    "nn": "no",  # Norwegian Nynorsk
    "zh": "zh_CN",
    "pt": "pt",
    "es": "es",
    "sr_latn": "sr",
}

_hint = None
_lang = None
_catalog = None


def _available():
    try:
        names = os.listdir(LOCALES)
    except OSError:
        return {FALLBACK}
    return {os.path.splitext(n)[0] for n in names if n.endswith(".json")} | {FALLBACK}


def _version_key(name):
    m = re.match(r"^(\d+)\.(\d+)$", name)
    return (int(m.group(1)), int(m.group(2))) if m else (0, 0)


def _config_dirs():
    """Likely KiCad user-configuration folders, newest version first."""
    roots = []
    env = os.environ.get("KICAD_CONFIG_HOME")
    if env:
        roots.append(env)
    home = os.path.expanduser("~")
    if sys.platform == "win32":
        roots.append(os.path.join(os.environ.get("APPDATA") or os.path.join(home, "AppData", "Roaming"), "kicad"))
    elif sys.platform == "darwin":
        roots.append(os.path.join(home, "Library", "Preferences", "kicad"))
    else:
        roots.append(os.path.join(os.environ.get("XDG_CONFIG_HOME") or os.path.join(home, ".config"), "kicad"))

    out = []
    for root in roots:
        out.append(root)
        try:
            versions = [d for d in os.listdir(root) if _version_key(d) != (0, 0)]
        except OSError:
            continue
        versions.sort(key=_version_key, reverse=True)
        out.extend(os.path.join(root, v) for v in versions)
    return out


def _configured_label():
    """The language label stored by KiCad, or None."""
    for d in _config_dirs():
        path = os.path.join(d, "kicad_common.json")
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding="utf-8-sig") as f:
                data = json.load(f)
            value = data.get("system", {}).get("language")
        except (OSError, ValueError):
            continue
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def _system_locale():
    """The user's operating-system UI locale, as a locale name when possible."""
    if sys.platform == "win32":
        try:
            import ctypes

            langid = ctypes.windll.kernel32.GetUserDefaultUILanguage()
            name = locale.windows_locale.get(langid)
            if name:
                return name
        except Exception:  # noqa: BLE001
            pass
    for var in ("LC_ALL", "LC_MESSAGES", "LANG"):
        value = os.environ.get(var)
        if value and value not in ("C", "POSIX") and not value.startswith("C."):
            return value
    try:
        name = locale.getlocale()[0]
    except Exception:  # noqa: BLE001
        name = None
    if not name:
        try:
            name = locale.getdefaultlocale()[0]
        except Exception:  # noqa: BLE001
            name = None
    return name


def normalise(name, available=None):
    """Map a KiCad label or locale name to a catalog code, or None."""
    if not name:
        return None
    name = name.strip()
    if name in KICAD_LABELS:
        return KICAD_LABELS[name]
    available = available or _available()
    code = name.replace("-", "_")
    if code in available:
        return code
    code = code.split(".")[0].split("@")[0]
    low = code.lower()
    if low in ALIASES:
        low = ALIASES[low]
    if low in available:
        return low
    base = low.split("_")[0]
    if base in ALIASES:
        base = ALIASES[base]
    if base in available:
        return base
    return None


def set_language_hint(label):
    """Use KiCad's live language label (SWIG plugin); ignores empty values."""
    global _hint, _lang, _catalog
    if label:
        _hint = label
        _lang = None
        _catalog = None


def current_language():
    """The catalog code in use, e.g. ``ru`` or ``pt_BR``."""
    global _lang
    if _lang is not None:
        return _lang
    available = _available()
    lang = None
    override = os.environ.get("GRIDLESS_LANG") or os.environ.get("GRIDLESS_LANGUAGE")
    if override:
        lang = normalise(override, available)
    if lang is None:
        label = _hint or _configured_label()
        if label and label != "Default":
            lang = normalise(label, available)
        else:
            lang = normalise(_system_locale(), available)
    _lang = lang if lang in available else FALLBACK
    return _lang


def _load(code):
    global _catalog
    if _catalog is not None:
        return _catalog
    path = os.path.join(LOCALES, code + ".json")
    try:
        with open(path, encoding="utf-8-sig") as f:
            data = json.load(f)
        _catalog = data.get("strings", {})
    except (OSError, ValueError):
        _catalog = {}
    return _catalog


def _(text):
    """Translate an English interface string."""
    code = current_language()
    if code == FALLBACK:
        return text
    return _load(code).get(text, text)


def language_name():
    """Native name of the language in use."""
    try:
        with open(os.path.join(LOCALES, current_language() + ".json"), encoding="utf-8-sig") as f:
            return json.load(f).get("@name", current_language())
    except (OSError, ValueError):
        return current_language()

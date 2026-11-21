"""Tests for the GridLess language detection and catalogs.

    python kicad_plugin/test_i18n.py
"""

import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "common"))

import gridless_i18n as i18n  # noqa: E402

_ORIGINAL_SYSTEM_LOCALE = i18n._system_locale


class LanguageTests(unittest.TestCase):
    def setUp(self):
        i18n._hint = None
        i18n._lang = None
        i18n._catalog = None
        i18n._system_locale = _ORIGINAL_SYSTEM_LOCALE
        for key in ("GRIDLESS_LANG", "GRIDLESS_LANGUAGE", "KICAD_CONFIG_HOME"):
            os.environ.pop(key, None)

    def tearDown(self):
        self.setUp()

    def test_all_kicad_labels_map_to_catalogs(self):
        for label, code in i18n.KICAD_LABELS.items():
            self.assertEqual(i18n.normalise(label), code, label)
            self.assertTrue(os.path.isfile(os.path.join(i18n.LOCALES, code + ".json")), code)

    def test_locale_names_and_aliases(self):
        self.assertEqual(i18n.normalise("ru_RU.UTF-8"), "ru")
        self.assertEqual(i18n.normalise("pt-BR"), "pt_BR")
        self.assertEqual(i18n.normalise("nb_NO"), "no")
        self.assertEqual(i18n.normalise("zh_CN"), "zh_CN")
        self.assertIsNone(i18n.normalise("kk_KZ"))

    def test_override_and_fallback(self):
        os.environ["GRIDLESS_LANG"] = "ru"
        self.assertEqual(i18n.current_language(), "ru")
        self.assertEqual(i18n._("Cancel"), "Отмена")

        os.environ["GRIDLESS_LANG"] = "does-not-exist"
        i18n._lang = None
        i18n._catalog = None
        i18n._system_locale = lambda: "en_US"
        self.assertEqual(i18n.current_language(), "en")
        self.assertEqual(i18n._("Cancel"), "Cancel")

    def test_default_uses_system_locale(self):
        i18n._system_locale = lambda: "de_DE"
        self.assertEqual(i18n.current_language(), "de")
        self.assertEqual(i18n._("Close"), "Schließen")

    def test_configured_label_wins(self):
        with tempfile.TemporaryDirectory() as tmp:
            os.environ["KICAD_CONFIG_HOME"] = tmp
            path = os.path.join(tmp, "10.0")
            os.makedirs(path)
            with open(os.path.join(path, "kicad_common.json"), "w", encoding="utf-8") as f:
                json.dump({"system": {"language": "Русский"}}, f, ensure_ascii=False)
            self.assertEqual(i18n._configured_label(), "Русский")
            self.assertEqual(i18n.current_language(), "ru")

    def test_placeholders_are_preserved(self):
        os.environ["GRIDLESS_LANG"] = "ru"
        text = i18n._("DRC round %d of up to %d: %s violation(s)")
        self.assertEqual(text % (1, 8, 0), "Раунд DRC 1 из 8: нарушений — 0")


if __name__ == "__main__":
    unittest.main()

# Interface translations

GridLess follows the language selected in KiCad (**Preferences → Common
Settings → Language**, stored in `kicad_common.json`).  The SWIG plugin asks
KiCad for the live setting; the IPC plugin reads the same file.  If KiCad is set
to *Default*, the operating-system UI language is used.  Set `GRIDLESS_LANG`
(for example `ru`, `pt_BR` or `Русский`) to override the detection when testing.

Each file here is a small JSON catalog named after the KiCad language code.
`en.json` is the source of truth: its keys are the English interface strings and
its values are identical to the keys.  A missing entry falls back to English, but
please keep the catalogs complete.

## Fixing or adding a translation

1. Copy `en.json` to `<code>.json` (use the code from KiCad's translation folder,
   for example `de`, `pt_BR`, `zh_TW`).
2. Set `"@code"` and `"@name"` (the native language name shown in KiCad).
3. Translate every value in `"strings"`.  Do **not** translate the keys.
4. Keep printf placeholders exactly as they are (`%d`, `%s`, `%.1f`, `%.3f`);
   they may move inside the sentence.
5. Run the checker:

   ```bash
   python kicad_plugin/check_i18n.py
   ```

It reports missing or extra keys, empty values and placeholder mismatches.
A translation is a single self-contained file, so it is easy to review.

## Current state

The catalogs cover all 44 languages shipped with KiCad 10.  The translations
were prepared with AI assistance and should be treated as first drafts.  Native
speakers are very welcome to improve them, especially the less common languages
(Georgian, Tamil, Telugu, Thai) and technical terms such as *autorouter*,
*differential pair*, *length matching* and *copper pour*.

The router's live log in the progress window stays in English: it is diagnostic
output from the Rust executable, not part of the plugin interface.  The IPC
action name in the toolbar comes from KiCad's static `plugin.json`; it is the
language-neutral brand name **GridLess**, while every dialog and report follows
the selected language.  The classic SWIG plugin can translate its menu label too,
because it is built at runtime.

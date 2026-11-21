# Publishing GridLess

This is the practical path from "works on my machine" to "a KiCad user can find
and install it". It covers the repository, the PCM package, the official addon
catalogue and the announcements.

## 1. Repository

Create a public repository at `https://github.com/bat654321/GridLess` (the
namespace used by the package metadata). The repository should contain:

- `README.md` (English) and `README.ru.md`;
- `LICENSE` (MIT) — required for the official addon repository;
- `CHANGELOG.md`, `CONTRIBUTING.md`;
- `docs/articles/` with the English and Russian write-ups and their images;
- the Rust sources, both plugins, `kicad_plugin/pcm/` and the CI workflows.

Repository settings:

- description: *Gridless topological autorouter for KiCad 10 — any-angle tracks,
  arcs, differential pairs and length matching*;
- topics: `kicad`, `autorouter`, `pcb`, `routing`, `eda`, `rust`;
- enable Issues and Discussions (or at least Issues — the official repository
  requires a place to report problems);
- add a social preview image (the before/after picture works well);
- protect `main` and require the CI check.

Remove generated data before the first commit: `target/`, `out/`, `testdata/`,
`dist/` and `__pycache__/` are already ignored. Check that no confidential board
or API key is in the history.

## 2. Build the release

The release job in `.github/workflows/release.yml` builds the router for Windows,
Linux and macOS, then creates one PCM archive containing all available binaries.
The archive lists the platforms it supports, so KiCad offers it only where it can
run.

Locally (Windows example):

```bash
cargo build --release
python kicad_plugin/pcm/build_pcm.py \
    --windows target/release/gridless.exe \
    --download-url https://github.com/bat654321/GridLess/releases/download/v0.1.0/gridless-0.1.0-pcm.zip
```

The script writes:

- `dist/gridless-0.1.0-pcm.zip` — upload this as a release asset;
- `dist/metadata.json` — submission metadata with SHA-256, sizes and the URL.

Test the archive locally with **Plugin and Content Manager → Install from File…**
before publishing. At minimum, install it on Windows, enable the API server, open
the Carrier reference board and route it.

Create an annotated tag `v0.1.0` and a GitHub Release. Upload:

- `gridless-0.1.0-pcm.zip`;
- optional command-line archives for users who do not want the plugin
  (`gridless-0.1.0-windows-x86_64.zip`, `...-linux-x86_64.tar.gz`,
  `...-macos-arm64.tar.gz`);
- the before/after image and a screenshot of the plugin dialog.

## 3. Submit to the official PCM repository

The official catalogue is built from the metadata repository, **not** from the
public `kicad/addons/repository` repository.

1. Fork `https://gitlab.com/kicad/addons/metadata`.
2. Add `packages/com.github.bat654321.gridless/metadata.json` — use the
   `dist/metadata.json` produced above. You may also add an icon next to it.
3. Validate it locally against `schema-v2.json` (the repository CI does this too):

   ```bash
   python -m pip install jsonschema
   python -c "import json,jsonschema; \
     jsonschema.validate(json.load(open('dist/metadata.json')), \
     json.load(open('schema-v2.json')))"
   ```

4. Commit, push to your fork and open a merge request against `main`.
5. Watch the CI and answer review comments. The KiCad team checks the licence,
   namespacing, English metadata, a public download URL and a working issue
   tracker.

Official rules that matter here:

- the package must be open source and GPL-compatible (MIT is fine);
- the identifier is namespaced by the host: `com.github.bat654321.gridless`;
- metadata must be in English; package contents may be translated (ours are);
- the download URL must be public and the SHA-256 must match the uploaded archive;
- the package must not be a copy of an existing addon.

If the archive content changes, bump the version, rebuild, upload a new release
asset and update `download_url`, `download_sha256`, `download_size` and
`install_size` in a new merge request.

## 3a. Third-party PCM repository (already live)

The official catalogue can take days or weeks, so the same package is also served
from GitHub Pages:

- branch `gh-pages` in the same repository;
- `repository.json`, `packages.json` and `resources.zip` at its root;
- public URL: `https://bat654321.github.io/GridLess/repository.json`.

Users add that URL in **Plugin and Content Manager → Manage repositories**. The
package then appears and updates through the PCM. This is a legitimate KiCad
feature and does not conflict with the official submission.

To update it after a release:

1. Copy `dist/metadata.json` (or the `pcm-metadata` workflow artifact) into
   `packages.json` as `{"packages": [<metadata>]}`.
2. Recompute the SHA-256 of `packages.json`.
3. Put the new icon under `resources/<identifier>/icon.png`, zip it as
   `resources.zip`, and recompute its SHA-256.
4. Update both hashes, `update_timestamp` and `update_time_utc` in
   `repository.json`.
5. Commit and push to `gh-pages`. GitHub Pages rebuilds automatically.

`kicad_plugin/pcm/` contains the build script; the repository layout is small
enough to maintain by hand.

### GitLab CI and identity verification

GitLab.com requires **identity verification** before it will run CI jobs for an
account. The verification options (phone or credit card) are not available in
every country. If the submitter cannot complete it:

- the merge request is still created and is mergeable if the upstream project does
  not require a successful pipeline;
- post a comment explaining the situation and include the output of the
  repository's own `ci/validate-package.py`;
- ask a KiCad maintainer to run the validation, or add a co-maintainer in a
  supported country who can push the branch and run the pipeline.

Do not try to bypass the verification with a borrowed identity, number or card.
The package itself is unaffected: the validator can be run locally against the
public release asset.

## 4. Announce

Do not spray the same text everywhere. Lead with the differentiator and show the
result. A short demo (30–60 s screen recording of Carrier being routed) is the
single most useful asset; the before/after picture is the fallback.

### Where

1. **KiCad Forum → External Plugins**
   <https://forum.kicad.info/c/external-plugins/12> — the main long-lived thread.
   Use the prepared text in `docs/announce/forum.en.md`, attach two or three
   images and link the repository and the release.
2. **r/KiCad** <https://www.reddit.com/r/KiCad/> — a shorter post, one image,
   link to the repository; answer questions in the comments.
3. **KiCad Discord** — the `#plugins`/show-and-tell channel; a couple of
   sentences and a video are enough.
4. **Mastodon/Fosstodon** with `#KiCad` `#PCB` `#EDA` `#Rust`; attach the
   before/after image.
5. **Hacker News — Show HN** <https://news.ycombinator.com/showhn.html>. Title
   with the concrete result, for example *Show HN: A gridless TopoR-style
   autorouter for KiCad*. Add a first comment explaining what is new, what is
   measured and what is still broken.
6. **Hackaday.io** project page / tip line; **EEVblog** and other PCB forums if
   the thread rules allow it.
7. **Russian-speaking audience**: the Russian article in `docs/articles/` is ready
   for Habr or a Telegram/community post; link the English repository.

Post to the forum and Reddit first, then wait for feedback and fix obvious
problems before pushing to larger channels. The KiCad forum thread is also where
users will report installation problems, so watch it.

### What to say

- What it is: a **gridless, topological** autorouter (TopoR-style), not another
  45° grid router; any-angle tracks and arcs around pads.
- The measured result on Carrier: zero clearance errors, 28 vias vs 30 by hand,
  1286 mm vs 1330 mm, about 11 s with the zone check.
- Honest limits: Beacon still has ~12 violations and 3 unconnected nets around
  the RF part; dense 0.4 mm parts and RF keepouts need work; the SWIG plugin
  routes the saved file, the IPC plugin handles unsaved edits.
- How to install: PCM archive from Releases (enable the KiCad API when asked), or
  `python kicad_plugin/install.py` from the repository.
- Ask for boards: "if you have a board it fails on, please open an issue and
  attach a minimal reproduction".
- Mention the interface languages: it follows KiCad's language setting and ships
  with translations for all 44 languages in KiCad 10.

Avoid claiming "better than Freerouting" or "production-ready". The honest,
measurable story is more convincing and sets the right expectations.

## 5. After the launch

- Reply to issues within a few days; add a minimal-reproduction request template.
- Keep the forum thread updated with each release, not with a new thread.
- Publish translation fixes as small pull requests; thank the contributors in the
  changelog.
- Once the PCM listing is live, update the README and the forum post to point at
  **Plugin and Content Manager** as the default install path.
- Ask two or three users with different boards to test and quote their results
  (with permission) in the README.

## Checklist

- [ ] public repository with MIT `LICENSE`, English README and issue tracker
- [ ] `cargo test --release` and `python kicad_plugin/check_i18n.py` pass
- [ ] PCM archive installs from file and routes Carrier on Windows
- [ ] release `v0.1.0` with archive, checksum and images
- [ ] metadata merge request to `kicad/addons/metadata`
- [ ] forum thread + Reddit post + short video
- [ ] follow-up release with Linux/macOS builds tested by users

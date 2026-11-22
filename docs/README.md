# Documentation contributor guide

Documentation: https://real-world-rl.readthedocs.io/

This is one Sphinx project: Chinese pages live at `docs/index.rst`,
`docs/tutorials/` and `docs/reference/`; English mirrors the same paths under
`docs/en/`. Shared snippets live in `docs/examples/`, with translated copies in
`docs/examples/en/`. Keep commands and values identical across snippet pairs.
Labels use `<page>-<anchor>` in Chinese and `en-<page>-<anchor>` in English,
with English anchor slugs and references targeting their own language.

## Build and check

Run from the repository root (Python 3.12 on Read the Docs):

```bash
pip install -r docs/requirements.txt
sphinx-build -W -b html docs docs/_build/html
python scripts/check_zh_manifest.py --strict
PYTHONPATH=.:rl-garden python -m pytest tests/test_tutorial_commands.py tests/test_site_examples.py tests/test_docs_pairs.py -q
```

The tests require the `learning,test` extras and pinned rl-garden submodule.
They guard CLI parsing, include markers, example schemas, translated example
values, and pair tracking. Use `.. tutorial-test: skip` immediately before a
shell directive only for blocks that are not CLI invocations.

The pair checker requires all RST counterparts and English example counterparts,
plus `README.md` / `README.zh-CN.md`. It warns when exactly one side changed;
`--strict` fails on issues. After reviewing both translations, run
`python scripts/check_zh_manifest.py --update` to record both SHA-256 hashes in
`docs/translation-manifest.json`. Open `docs/_build/html/index.html` to preview.

## Publish on Read the Docs

1. Push the documentation changes to the GitHub repository's default branch.
2. Sign in to [Read the Docs Community](https://app.readthedocs.org/), connect
   GitHub, and add `JaimeParker/real-world-rl`.
3. Open **Builds** and check the `latest` build for `main`, or start it from the
   dashboard. `.readthedocs.yaml` selects `docs/conf.py`, installs the pinned
   documentation requirements and treats Sphinx warnings as failures.
4. Open the published Chinese homepage from the dashboard. Check the cards and
   the **English** language switch. Both languages belong to the same project.

The hosting URL is the one assigned by the RTD project dashboard; this source
change does not publish a project or verify its public address.

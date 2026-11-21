# Build and publish the docs

Run from the repository root with Python 3.12, the version used by Read the Docs.

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -r site/requirements.txt
python -m sphinx -W -n --keep-going -b html site site/_build/html
```

Open `site/_build/html/index.html`. Chinese tutorials live at the root; English
counterparts live under `en/`. English pages remain short placeholders until the
Chinese chapters are final. The sidebar switches between matching pages.
`design/` contains unpublished planning notes. `docs/*.md` remain GitHub reference
pages; Sphinx does not generate or copy them.

## Shared snippets

Keep shell commands and YAML in `site/examples/`. Include shell blocks using
`.. literalinclude:: ../examples/install.sh`, `:language: bash`,
`:start-after: # start NAME` and `:end-before: # end NAME`, as in expnote's
`experiments.sh`. YAML files use the same markers with `:language: yaml`;
`:lines:` is also supported. Do not duplicate commands in translated pages.
Shell files contain separate tutorial steps, not unattended hardware scripts.
Replace every hardware placeholder and review numeric example limits before use.
`mock-doctor.txt` is an abbreviated captured mock output, not a hardware report.

Validate includes, loaders, and CLI syntax without Sphinx or hardware:

```bash
PYTHONPATH=.:rl-garden python -m pytest tests/test_site_examples.py tests/test_cli_registry.py -q
```

Use an environment with the `learning,test` extras and the pinned rl-garden
submodule installed for these tests. Sphinx only needs `site/requirements.txt`.

## Publish on Read the Docs

1. Push the documentation changes to the GitHub repository's default branch.
2. Sign in to [Read the Docs Community](https://app.readthedocs.org/), connect
   GitHub, and add `JaimeParker/real-world-rl`.
3. Open **Builds** and check the `latest` build for `main`, or start it from the
   dashboard. `.readthedocs.yaml` selects `site/conf.py`, installs the pinned
   documentation requirements and treats Sphinx warnings as failures.
4. Open the published Chinese homepage from the dashboard. Check the cards and
   the **English** language switch. Both languages belong to the same project.

The hosting URL is the one assigned by the RTD project dashboard; this source
change does not publish a project or verify its public address.

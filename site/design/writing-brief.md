# Tutorial writing brief (for every chapter run)

Goal: 说人话，且让没用过本仓的用户学会怎么用. The reader already knows robotics (Franka, FCI, TCP pose, joint space, RealSense) and RL (policy, BC, actor/learner, reward, replay buffer). Do not explain those basics. Explain this repo's own concepts in plain words.

Benchmark chapters: site/tutorials/install.rst and site/tutorials/station.rst. Match their structure, depth and tone.

## Format
- reStructuredText, following ~/Projects/expnote/docs. Chinese at the root (site/tutorials/), English placeholders under site/en/tutorials/.
- Every command and every YAML shown lives in a file under site/examples/ and is pulled in with `literalinclude` between `# start <name>` / `# end <name>` markers. Nothing longer than 3 lines goes inline.
- Expected outputs live under site/examples/*.txt. Real outputs (mock, --help) are copied verbatim. Hardware outputs are format examples built from the CLI's format strings and labelled 示例.
- Internal links use :doc:. Link docs/*.md on GitHub only for deep reference, never in place of explaining a step.

## Per section
1. One sentence on what this step does and why.
2. The action: a command, or the YAML to edit.
3. **你应该看到**: the expected output.
4. **如果不对**: the 2–4 most likely causes and their fixes, keyed to real doctor row names and error messages.

## YAML
- Show the snippet first, then a two-column table: 必须改的字段 | 怎么得到这个值 (give the exact command or Desk menu).
- After the table, one sentence saying the other fields keep their defaults, naming them, plus when you would change them.
- Give a worked example whenever a value is read from a command: command output → YAML line.

## Wording
- Short sentences, one idea each. Define each repo term in one plain sentence the first time it appears.
- No defensive sentences against misunderstandings a robotics reader would not have.
- Use `.. warning::` before any step that moves the arm.
- Close each chapter with a single `.. note::` titled 本章哪些内容需要真机确认, then a short 检查清单, then a :doc: link to the next chapter.

## Grounding
- Every field, flag, extra, default, output string and doctor row name must come from code in this repo (real_world_rl/, scripts/, pyproject.toml, examples/). Where docs and code disagree, follow the code. Run `--help` and mock commands to confirm.
- Never invent values. Hardware-specific values get marked placeholders plus instructions for obtaining the real value.
- If you find a code bug, report it; don't fix it.

## Shared names across chapters
- Configs: `local/station.yaml` and `local/task.yaml` from station.rst.
- Use the demo/output paths that the previous chapters introduced; read them before choosing new ones.
- New example files are prefixed with the chapter name (e.g. site/examples/spacemouse.sh), so parallel runs don't collide.

## Verification
- Build: `python -m sphinx -W -n --keep-going -b html site <tmpdir>` with zero warnings, using the venv from the earlier run or a fresh one built from site/requirements.txt.
- Tests: `PYTHONPATH=.:rl-garden ~/miniconda3/envs/rlg/bin/python -m pytest tests/test_site_examples.py tests/test_cli_registry.py -q` must pass.
- Reread the chapter as a first-time user and fix any step that needs a value or command not yet introduced.
- Edit only site/ (your chapters and their examples, plus the en/ placeholder for each). tests/test_site_examples.py only if a new kind of example needs validation. No commits.

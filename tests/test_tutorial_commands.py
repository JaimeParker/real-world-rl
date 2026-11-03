"""Every command of the hardware tutorial (``docs/tutorial/*.md``) parses against the current CLIs.

Each ```bash block of a page is read as a small shell script: backslash-newline continuations are
joined, ``NAME=value`` lines define variables for the later blocks of the same page, and ``$NAME``
expands to them. Each remaining command is parsed, and only parsed, by its program's own parser
(the training registry, the recorder, deploy, the reward-data collector, rl-garden's ACT and
classifier trainers, robot_infra's setup, doctor, daemon and discovery): nothing connects, trains or
opens a device. A flag the tutorial uses that a parser no longer knows fails the test.

A block preceded by ``<!-- tutorial-test: skip -->`` on the line before its fence is not parsed
(installs, ``cp``, ``pytest``, ``tensorboard``, a lone flag). A command whose program this test
does not know fails, so a new kind of command needs a parser here or the skip marker.

``hil.yaml`` (the tutorial's copy of the preset) is parsed as ``examples/presets/hil_single_arm.yaml``;
``ROBOT_PC`` and ``LEARNER_IP`` become ``127.0.0.1``. CPU only, no network, no hardware.
"""
from __future__ import annotations

import argparse
import functools
import re
import shlex
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
TUTORIAL_DIR = REPO_ROOT / "docs" / "tutorial"
DOCS_INDEX = REPO_ROOT / "docs" / "README.md"
PRESET = REPO_ROOT / "examples" / "presets" / "hil_single_arm.yaml"

SKIP_MARKER = "<!-- tutorial-test: skip -->"
PRESET_COPY = "hil.yaml"
# The tutorial's joint_abs copy of the preset: action_mode switched and joint_limits uncommented.
PRESET_ABS_COPY = "hil_abs.yaml"
HOST_PLACEHOLDERS = {"ROBOT_PC": "127.0.0.1", "LEARNER_IP": "127.0.0.1"}
# The tutorial must keep at least this many commands under test (guards the extractor).
MIN_PARSED_COMMANDS = 30

_FENCE_OPEN = "```bash"
_FENCE_CLOSE = "```"
_BLOCKQUOTE_PREFIX = re.compile(r"^\s*(?:>\s?)*")
_VARIABLE = re.compile(r"\$(?:\{(\w+)\}|(\w+))")
_ASSIGNMENT = re.compile(r"^([A-Za-z_]\w*)=(.*)$", re.DOTALL)
_SHELL_OPERATORS = {"|", "||", "&&", ";", "&", ">", ">>", "<", "2>", "2>&1"}
_QUOTE_OR_EXPANSION_CHARS = set("'\"\\$`")
_GLOB_CHARS = set("*?[")
_PYTHONS = ("python", "python3")


@dataclass(frozen=True)
class BashBlock:
    page: str
    line: int
    text: str
    skipped: bool


@dataclass(frozen=True)
class TutorialCommand:
    page: str
    line: int
    argv: tuple[str, ...]

    @property
    def id(self) -> str:
        return f"{self.page}:{self.line}:{' '.join(self.argv[:3])}"


class ParsedOnly(Exception):
    """Raised by the patched ``ArgumentParser.parse_args`` once a ``main`` has parsed its argv."""


# -- extraction ---------------------------------------------------------------------------------


def bash_blocks(page: Path) -> list[BashBlock]:
    """The ```bash blocks of ``page`` (also inside blockquotes), with the line of their fence and
    whether the line before the fence is the skip marker."""
    lines = [_BLOCKQUOTE_PREFIX.sub("", line) for line in page.read_text().splitlines()]
    blocks: list[BashBlock] = []
    index = 0
    while index < len(lines):
        if lines[index].strip() != _FENCE_OPEN:
            index += 1
            continue
        start = index
        skipped = start > 0 and lines[start - 1].strip() == SKIP_MARKER
        body: list[str] = []
        index += 1
        while index < len(lines) and lines[index].strip() != _FENCE_CLOSE:
            body.append(lines[index])
            index += 1
        if index == len(lines):
            raise ValueError(f"{page.name}:{start + 1}: unterminated ```bash block")
        blocks.append(BashBlock(page.name, start + 1, "\n".join(body), skipped))
        index += 1
    return blocks


def logical_lines(text: str) -> list[str]:
    """Shell lines of a block: continuations joined, blank and comment-only lines dropped."""
    joined = re.sub(r"\\\n", " ", text)
    return [line.strip() for line in joined.splitlines() if line.strip() and not line.strip().startswith("#")]


def expand_variables(line: str, variables: dict[str, str]) -> str:
    """``$NAME`` / ``${NAME}`` outside single quotes replaced by the variable's value. The values
    hold no quotes, backslashes or ``$`` (checked on assignment), so the text substitution followed
    by ``shlex`` splitting matches the shell's expansion and word splitting; an unquoted value with
    glob characters is refused, since the shell would glob it."""
    out: list[str] = []
    quote: str | None = None
    index = 0
    while index < len(line):
        char = line[index]
        if char == "\\" and quote != "'" and index + 1 < len(line):
            out.append(line[index : index + 2])
            index += 2
            continue
        if char == "'" and quote != '"':
            quote = None if quote == "'" else "'"
        elif char == '"' and quote != "'":
            quote = None if quote == '"' else '"'
        elif char == "$" and quote != "'":
            match = _VARIABLE.match(line, index)
            if match is not None:
                name = match.group(1) or match.group(2)
                if name not in variables:
                    raise ValueError(f"undefined variable ${name} (define it in an earlier block of the page)")
                value = variables[name]
                if quote is None and _GLOB_CHARS & set(value):
                    raise ValueError(f"${name} holds glob characters; quote it as \"${name}\"")
                out.append(value)
                index = match.end()
                continue
        out.append(char)
        index += 1
    return "".join(out)


@functools.lru_cache(maxsize=1)
def joint_abs_preset() -> Path:
    """``PRESET`` as the tutorial's ``hil_abs.yaml``: ``action_mode: joint_abs`` with the commented
    ``joint_limits`` line enabled. Written once per session into a temporary directory."""
    text = PRESET.read_text()
    assert "action_mode: joint_delta" in text and "\n  # joint_limits:" in text
    text = text.replace("action_mode: joint_delta", "action_mode: joint_abs")
    text = text.replace("\n  # joint_limits:", "\n  joint_limits:")
    path = Path(tempfile.mkdtemp(prefix="tutorial-preset-")) / PRESET_ABS_COPY
    path.write_text(text)
    return path


def substitute_placeholders(argv: Sequence[str]) -> tuple[str, ...]:
    substituted = []
    for token in argv:
        if token == PRESET_COPY:
            token = str(PRESET)
        elif token == PRESET_ABS_COPY:
            token = str(joint_abs_preset())
        for placeholder, value in HOST_PLACEHOLDERS.items():
            token = token.replace(placeholder, value)
        substituted.append(token)
    return tuple(substituted)


def page_commands(page: Path) -> list[TutorialCommand]:
    """The commands of ``page``'s non-skipped ```bash blocks, variables expanded."""
    variables: dict[str, str] = {}
    commands: list[TutorialCommand] = []
    for block in bash_blocks(page):
        if block.skipped:
            continue
        for line in logical_lines(block.text):
            where = f"{page.name}:{block.line}"
            try:
                tokens = shlex.split(expand_variables(line, variables), comments=True)
            except ValueError as error:
                raise ValueError(f"{where}: {error}: {line}") from error
            operators = _SHELL_OPERATORS & set(tokens)
            if operators:
                raise ValueError(f"{where}: shell operators {sorted(operators)} need the skip marker: {line}")
            assignments = 0
            for token in tokens:
                match = _ASSIGNMENT.match(token)
                if match is None:
                    break
                assignments += 1
            if assignments == len(tokens):
                for token in tokens:
                    name, value = _ASSIGNMENT.match(token).groups()
                    if _QUOTE_OR_EXPANSION_CHARS & set(value):
                        raise ValueError(f"{where}: {name}'s value must not contain quotes, '\\', '$' or '`'")
                    variables[name] = value
                continue
            argv = substitute_placeholders(tokens[assignments:])
            commands.append(TutorialCommand(page.name, block.line, argv))
    return commands


def tutorial_pages() -> list[Path]:
    return sorted(TUTORIAL_DIR.glob("*.md"))


def _collect() -> tuple[list[TutorialCommand], list[str]]:
    commands: list[TutorialCommand] = []
    errors: list[str] = []
    for page in tutorial_pages():
        try:
            commands.extend(page_commands(page))
        except ValueError as error:
            errors.append(str(error))
    return commands, errors


COMMANDS, EXTRACTION_ERRORS = _collect()


# -- parsers ------------------------------------------------------------------------------------


def _check_base_provider(spec: str) -> None:
    if spec:
        from real_world_rl.policies.base_provider import parse_base_provider_spec

        parse_base_provider_spec(spec)


def parse_training(argv: Sequence[str]) -> None:
    from real_world_rl.training.registry import registry

    command = registry.parse_command(list(argv))
    _check_base_provider(getattr(command.args, "base_provider", ""))


def parse_recorder(argv: Sequence[str]) -> None:
    from real_world_rl.demos.record import parse_record_args

    parse_record_args(list(argv))


def parse_deploy(argv: Sequence[str]) -> None:
    import tyro

    from real_world_rl.deploy import DeployArgs

    args = tyro.cli(DeployArgs, args=list(argv))
    _check_base_provider(args.base_provider or "")


def parse_collector(argv: Sequence[str]) -> None:
    from real_world_rl.reward.collect_data import Args, selects_real_robot

    if selects_real_robot(argv):
        from real_world_rl.reward.collect_real_robot import parse_collect_args

        parse_collect_args(list(argv))
        return
    import tyro

    tyro.cli(Args, args=list(argv))


def parse_act_trainer(argv: Sequence[str]) -> None:
    import tyro
    from rl_garden.models.act.train_trajectories import Args

    tyro.cli(Args, args=list(argv))


def parse_classifier_trainer(argv: Sequence[str]) -> None:
    from rl_garden.models.reward.success.train import parse_args

    saved = sys.argv
    sys.argv = ["rl_garden.models.reward.success.train", *argv]
    try:
        parse_args()
    finally:
        sys.argv = saved


def parse_setup(argv: Sequence[str]) -> None:
    from robot_infra.setup.__main__ import build_parser

    build_parser().parse_args(list(argv))


def parse_server(argv: Sequence[str]) -> None:
    from robot_infra.server.daemon import parse_args

    parse_args(list(argv))


def _parse_with_main(main: Callable[[list[str]], object], argv: Sequence[str]) -> None:
    """Runs ``main(argv)`` only up to its ``ArgumentParser.parse_args`` call."""
    original = argparse.ArgumentParser.parse_args

    def parse_then_stop(self, args=None, namespace=None):
        original(self, args, namespace)
        raise ParsedOnly

    argparse.ArgumentParser.parse_args = parse_then_stop
    try:
        main(list(argv))
    except ParsedOnly:
        return
    finally:
        argparse.ArgumentParser.parse_args = original
    raise AssertionError("main() returned without parsing its arguments")


def parse_doctor(argv: Sequence[str]) -> None:
    from robot_infra.doctor.cli import main

    _parse_with_main(main, argv)


def parse_discovery(argv: Sequence[str]) -> None:
    from robot_infra.discovery.__main__ import main

    _parse_with_main(main, argv)


SCRIPT_PARSERS: dict[str, Callable[[Sequence[str]], None]] = {
    "examples/train_real_world.py": parse_training,
    "examples/record_real_demos.py": parse_recorder,
    "examples/deploy_real_robot.py": parse_deploy,
}
MODULE_PARSERS: dict[str, Callable[[Sequence[str]], None]] = {
    "real_world_rl.training": parse_training,
    "real_world_rl.demos": parse_recorder,
    "real_world_rl.deploy": parse_deploy,
    "real_world_rl.reward.collect_data": parse_collector,
    "rl_garden.models.act.train_trajectories": parse_act_trainer,
    "rl_garden.models.reward.success.train": parse_classifier_trainer,
    "robot_infra.setup": parse_setup,
    "robot_infra.doctor": parse_doctor,
    "robot_infra.server": parse_server,
    "robot_infra.discovery": parse_discovery,
}
CONSOLE_SCRIPT_PARSERS: dict[str, Callable[[Sequence[str]], None]] = {
    "real-world-rl-train": parse_training,
    "real-world-rl-record": parse_recorder,
    "real-world-rl-deploy": parse_deploy,
    "real-world-rl-collect-reward": parse_collector,
}


def parser_for(argv: Sequence[str]) -> tuple[Callable[[Sequence[str]], None], list[str]]:
    """The parser of ``argv``'s program and the arguments it takes."""
    if argv[0] in CONSOLE_SCRIPT_PARSERS:
        return CONSOLE_SCRIPT_PARSERS[argv[0]], list(argv[1:])
    if argv[0] in _PYTHONS and len(argv) >= 3 and argv[1] == "-m" and argv[2] in MODULE_PARSERS:
        return MODULE_PARSERS[argv[2]], list(argv[3:])
    if argv[0] in _PYTHONS and len(argv) >= 2 and argv[1] in SCRIPT_PARSERS:
        return SCRIPT_PARSERS[argv[1]], list(argv[2:])
    raise LookupError(
        f"no parser for {' '.join(argv[:3])!r}: add one to tests/test_tutorial_commands.py or put "
        f"{SKIP_MARKER} on the line before the block"
    )


# -- tests --------------------------------------------------------------------------------------


def test_the_tutorial_blocks_are_well_formed():
    assert tutorial_pages(), f"no tutorial pages under {TUTORIAL_DIR}"
    assert not EXTRACTION_ERRORS, "\n".join(EXTRACTION_ERRORS)
    assert len(COMMANDS) >= MIN_PARSED_COMMANDS, (
        f"only {len(COMMANDS)} tutorial commands found; the extractor or the skip markers are off"
    )


def test_the_docs_index_links_every_tutorial_page():
    index = DOCS_INDEX.read_text()
    missing = [page.name for page in tutorial_pages() if f"tutorial/{page.name}" not in index]
    assert not missing, f"docs/README.md does not link {missing}"


@pytest.mark.parametrize("command", COMMANDS, ids=[command.id for command in COMMANDS])
def test_tutorial_command_parses(command: TutorialCommand):
    try:
        parse, args = parser_for(command.argv)
    except LookupError as error:
        pytest.fail(f"{command.page}:{command.line}: {error}")
    try:
        parse(args)
    except SystemExit as error:
        pytest.fail(
            f"{command.page}:{command.line}: the command no longer parses (exit {error.code}):\n"
            f"{shlex.join(command.argv)}"
        )


def test_an_unknown_flag_fails_the_parse():
    """The guarantee the tutorial test rests on: a flag the CLI does not know exits the parse."""
    with pytest.raises(SystemExit):
        parse_training(["hil_serl", "--config", str(PRESET), "--no-such-flag", "1"])
    with pytest.raises(SystemExit):
        parse_recorder(["--config", str(PRESET), "--no-such-flag", "1"])
    with pytest.raises(SystemExit):
        parse_doctor(["--no-such-flag"])


def test_variables_expand_like_the_shell():
    variables = {"FLAGS": "--a 1 --b 2", "BASE": "checkpoint:x.pt?algo=bc"}
    line = expand_variables('run $FLAGS --base "$BASE" \'$FLAGS\'', variables)
    assert shlex.split(line) == ["run", "--a", "1", "--b", "2", "--base", "checkpoint:x.pt?algo=bc", "$FLAGS"]
    with pytest.raises(ValueError, match="glob"):
        expand_variables("run --base $BASE", variables)
    with pytest.raises(ValueError, match="undefined"):
        expand_variables("run $MISSING", variables)

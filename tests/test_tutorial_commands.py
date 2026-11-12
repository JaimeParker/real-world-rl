"""Hardware tutorial and testing-page bash blocks parse against the current CLIs.

Pages are collected from ``docs/tutorial/*.md`` and ``docs/testing/*.md``.

Each ```bash block of a page is read as a small shell script: backslash-newline continuations are
joined, ``NAME=value`` lines define variables for the later blocks of the same page, and ``$NAME``
expands to them. Each remaining command is parsed, and only parsed, by its program's own parser
(the training registry, the recorder, deploy, the reward-data collector, rl-garden's ACT and
classifier trainers, rwrl and its device commands): nothing connects, trains or
opens a device. A flag the tutorial uses that a parser no longer knows fails the test.

A block preceded by ``<!-- tutorial-test: skip -->`` on the line before its fence is not parsed
(installs, ``cp``, ``pytest``, ``tensorboard``, a lone flag). A command whose program this test
does not know fails, so a new kind of command needs a parser here or the skip marker.

``hil.yaml`` (the tutorial's copy of the preset) is parsed as ``examples/presets/hil_single_arm.yaml``;
``ROBOT_PC`` and ``LEARNER_IP`` become ``127.0.0.1``. CPU only, no network, no hardware.
"""
from __future__ import annotations

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
TESTING_DIR = REPO_ROOT / "docs" / "testing"
DOCS_INDEX = REPO_ROOT / "docs" / "README.md"
PRESET = REPO_ROOT / "examples" / "presets" / "hil_single_arm.yaml"

SKIP_MARKER = "<!-- tutorial-test: skip -->"
PRESET_COPY = "hil.yaml"
# The tutorial's joint_abs copy of the preset: action_mode switched and joint_limits uncommented.
PRESET_ABS_COPY = "hil_abs.yaml"
# The testing page's commissioned local copy; parse its CLI against the source preset.
BIMANUAL_PRESET_COPY = "local/bimanual.smoke.yaml"
# The tutorial's local copy of the GELLO preset (08-11); parse its CLI against the source preset.
GELLO_PRESET_COPY = "local/gello_cartesian.yaml"
GELLO_PRESET = REPO_ROOT / "examples" / "presets" / "gello_cartesian.yaml"
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
        blocks.append(BashBlock(str(page.relative_to(REPO_ROOT)), start + 1, "\n".join(body), skipped))
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
        elif token == BIMANUAL_PRESET_COPY:
            token = str(REPO_ROOT / "examples" / "presets" / "bimanual_gello_smoke.yaml")
        elif token == GELLO_PRESET_COPY:
            token = str(GELLO_PRESET)
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
            commands.append(TutorialCommand(block.page, block.line, argv))
    return commands


def tutorial_pages() -> list[Path]:
    return sorted(TUTORIAL_DIR.glob("*.md"))


def session_pages() -> list[Path]:
    return sorted(TESTING_DIR.glob("*.md"))


def _collect() -> tuple[list[TutorialCommand], list[str]]:
    commands: list[TutorialCommand] = []
    errors: list[str] = []
    for page in [*tutorial_pages(), *session_pages()]:
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
}
CONSOLE_SCRIPT_PARSERS: dict[str, Callable[[Sequence[str]], None]] = {
    "real-world-rl-train": parse_training,
    "real-world-rl-record": parse_recorder,
    "real-world-rl-deploy": parse_deploy,
    "real-world-rl-collect-reward": parse_collector,
}


def parser_for(argv: Sequence[str]) -> tuple[Callable[[Sequence[str]], None], list[str]]:
    """The parser of ``argv``'s program and the arguments it takes."""
    if argv[0] == "rwrl":
        from real_world_rl.cli import build_parser
        from real_world_rl.cli.cmd_learning import LearningCommand

        parser = build_parser()
        args, remaining = parser.parse_known_args(list(argv[1:]))
        if isinstance(args._command, LearningCommand):
            learning_parsers = {
                "train": parse_training,
                "record": parse_recorder,
                "deploy": parse_deploy,
                "collect-reward": parse_collector,
            }
            # Keep --config in the entrypoint argv: the training registry and
            # recorder/collector parse functions load presets before invoking tyro.
            entrypoint_args = args._command.entrypoint_argv(args, remaining)
            return learning_parsers[args._command.name], list(entrypoint_args)
        if args.command == "supervise":
            # Validate supervisor flags before recursively validating its child.
            parser.parse_args(list(argv[1:]))
            child = args.child_command
            if child[:1] == ["--"]:
                child = child[1:]
            if not child:
                raise LookupError("supervise requires a child command after --")
            return parser_for(child)
        return parser.parse_args, list(argv[1:])
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


def test_testing_pages_include_a_parsed_supervised_recorder():
    """Testing pages cannot skip every expanded supervisor + recorder command."""
    assert session_pages(), f"no testing pages under {TESTING_DIR}"
    supervised = [
        command for command in COMMANDS
        if command.page.startswith("docs/testing/")
        and command.argv[:2] == ("rwrl", "supervise")
    ]
    assert any(parser_for(command.argv)[0] is parse_recorder for command in supervised), (
        "docs/testing/*.md must retain a parsed supervisor + recorder expansion"
    )


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
    from real_world_rl.cli.errors import CliError

    parse, args = parser_for(["rwrl", "doctor", "--no-such-flag"])
    with pytest.raises(CliError):
        parse(args)


def test_variables_expand_like_the_shell():
    variables = {"FLAGS": "--a 1 --b 2", "BASE": "checkpoint:x.pt?algo=bc"}
    line = expand_variables('run $FLAGS --base "$BASE" \'$FLAGS\'', variables)
    assert shlex.split(line) == ["run", "--a", "1", "--b", "2", "--base", "checkpoint:x.pt?algo=bc", "$FLAGS"]
    with pytest.raises(ValueError, match="glob"):
        expand_variables("run --base $BASE", variables)
    with pytest.raises(ValueError, match="undefined"):
        expand_variables("run $MISSING", variables)


@pytest.mark.parametrize('name,parser_name', [
    ('train', 'parse_training'), ('record', 'parse_recorder'),
    ('deploy', 'parse_deploy'), ('collect-reward', 'parse_collector'),
])
@pytest.mark.parametrize("use_preset_alias", [False, True])
def test_rwrl_learning_parser_hook(monkeypatch, name, parser_name, use_preset_alias):
    calls = []
    monkeypatch.setitem(globals(), parser_name, lambda argv: calls.append(argv))
    command = (['rwrl', '--preset', 'P.yaml', name] if use_preset_alias
               else ['rwrl', name, '--config', 'P.yaml'])
    parse, argv = parser_for([*command, 'algo', '--help'])
    parse(argv)
    assert calls == [['--config', 'P.yaml', 'algo', '--help']]


# -- command spelling guards --------------------------------------------------------------------
# Every ```bash block counts here, the skipped ones too: a stale spelling in an install or
# systemd snippet is as wrong as one in a parsed command.

# `python -m real_world_rl.*` stays out of the pages (rwrl is the entry point) except these.
SUPERVISED_CHILD_MODULES = frozenset({
    "real_world_rl.training", "real_world_rl.demos", "real_world_rl.deploy",
    "real_world_rl.reward.collect_data",
})  # allowed only as the child of a supervise command
LEGACY_MODULE = "real_world_rl.legacy.controller.real.franka_bridge"


def _python_modules(tokens: Sequence[str]) -> list[tuple[int, str, str]]:
    """(index of the interpreter, interpreter, module) for each ``python -m module`` in ``tokens``."""
    return [
        (index, token, tokens[index + 2]) for index, token in enumerate(tokens[:-2])
        if token in _PYTHONS and tokens[index + 1] == "-m"
    ]


def retired_device_cli_forms(line: str) -> list[str]:
    """``robot-infra ...`` and ``python -m robot_infra...`` in a shell ``line``: the device layer
    is part of rwrl now."""
    tokens = line.split()
    found = ["robot-infra"] if "robot-infra" in tokens else []
    found += [
        f"{python} -m {module}" for _, python, module in _python_modules(tokens)
        if module.split(".")[0] == "robot_infra"
    ]
    return found


def disallowed_module_forms(line: str) -> list[str]:
    """``python -m real_world_rl...`` in a shell ``line`` unless allowlisted: a supervised child
    or the legacy franka bridge."""
    tokens = line.split()
    found = []
    for index, python, module in _python_modules(tokens):
        if module.split(".")[0] != "real_world_rl" or module.startswith(LEGACY_MODULE):
            continue
        if module in SUPERVISED_CHILD_MODULES and "supervise" in tokens[:index]:
            continue
        found.append(f"{python} -m {module}")
    return found


def _offending_lines(find: Callable[[str], list[str]]) -> list[str]:
    offenders = []
    for page in [*tutorial_pages(), *session_pages()]:
        for block in bash_blocks(page):
            for line in logical_lines(block.text):
                offenders += [f"{block.page}:{block.line}: {form}: {line}" for form in find(line)]
    return offenders


def test_pages_use_no_retired_device_cli():
    """A regression guard for the CLI merge: no ``robot-infra ...`` or ``python -m robot_infra...``."""
    offenders = _offending_lines(retired_device_cli_forms)
    assert not offenders, "\n".join(offenders)


def test_pages_use_python_modules_only_from_the_allowlist():
    offenders = _offending_lines(disallowed_module_forms)
    assert not offenders, "\n".join(offenders)


@pytest.mark.parametrize("line,retired,disallowed", [
    ("robot-infra daemon --config robot.yaml", ["robot-infra"], []),
    ("sudo robot-infra doctor", ["robot-infra"], []),
    ("python -m robot_infra.setup", ["python -m robot_infra.setup"], []),
    ("python3 -m robot_infra", ["python3 -m robot_infra"], []),
    ("python -m real_world_rl.cli doctor", [], ["python -m real_world_rl.cli"]),
    ("python -m real_world_rl.demos --config p.yaml", [], ["python -m real_world_rl.demos"]),
    ("rwrl supervise --log s.log -- python -m real_world_rl.demos --config p.yaml", [], []),
    ("rwrl supervise -- python -m real_world_rl.cli doctor", [], ["python -m real_world_rl.cli"]),
    (f"python -m {LEGACY_MODULE} --port 5000", [], []),
    ("rwrl doctor --station local/station.yaml", [], []),
    ("python -m pip install -e '.[grpc]'", [], []),
    ("python -m rl_garden.models.act.train_trajectories --demo-paths d.pkl", [], []),
    ("export CONFIG=~/.config/robot_infra/station.yaml", [], []),
])
def test_command_spelling_detectors(line, retired, disallowed):
    assert retired_device_cli_forms(line) == retired
    assert disallowed_module_forms(line) == disallowed

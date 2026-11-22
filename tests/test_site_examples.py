"""Tutorial contracts: no hardware, network, command execution, or Sphinx needed."""
from __future__ import annotations

from pathlib import Path
import re
import shlex

import pytest
import yaml

from real_world_rl.cli import build_parser
from real_world_rl.cli.preset_edit import Preset
from real_world_rl.station import load_station


SITE = Path(__file__).resolve().parents[1] / "docs"
EXAMPLES = SITE / "examples"
INCLUDE = re.compile(r"^([ \t]*(?:- )?)\.\. literalinclude::\s+(.+?)\s*$", re.MULTILINE)


def literal_includes():
    """Collect include directives and their indented option blocks."""
    for page in sorted(SITE.rglob("*.rst")):
        if "_build" in page.relative_to(SITE).parts:
            continue
        text = page.read_text(encoding="utf-8")
        for match in INCLUDE.finditer(text):
            options = {}
            for line in text[match.end():].splitlines():
                if not line.strip():
                    continue
                if len(line) - len(line.lstrip()) <= len(match[1]):
                    break
                option = re.match(r"\s+:(\S+):\s*(.*)", line)
                if option:
                    options[option[1]] = option[2]
            yield page, match[2], options


INCLUDES = list(literal_includes())


@pytest.mark.parametrize("page,target,options", INCLUDES,
                         ids=[f"{page.relative_to(SITE)}:{target}:{i}"
                              for i, (page, target, _) in enumerate(INCLUDES)])
def test_literalinclude(page: Path, target: str, options: dict[str, str]) -> None:
    source = SITE / target.lstrip("/") if target.startswith("/") else page.parent / target
    assert source.is_file(), f"{page}: missing {source}"
    text = source.read_text(encoding="utf-8")
    start = options.get("start-after")
    end = options.get("end-before")
    if start:
        assert start in text, f"{source}: missing {start}"
    if end:
        assert end in text, f"{source}: missing {end}"
    if start and end:
        assert text.index(start) < text.index(end), f"{source}: reversed markers"
    if "lines" in options:
        count = len(text.splitlines())
        for interval in options["lines"].split(","):
            for boundary in interval.split("-"):
                if boundary.strip():
                    assert 1 <= int(boundary) <= count


@pytest.mark.parametrize("path", sorted(EXAMPLES.rglob("*.yaml")), ids=lambda p: p.name)
def test_yaml_loaders(path: Path) -> None:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(document, dict)
    # Classify by schema rather than filename, so added examples cannot evade validation.
    if "type" in document or "robots" in document or "robot" in document:
        assert load_station(path).robots
    elif "real_robot" in document:
        preset = Preset.load(str(path))  # home and teleop run use this loader
        from real_world_rl.envs.real_robot.config import RealRobotEnvConfig
        from real_world_rl.demos.record import parse_record_args

        RealRobotEnvConfig(**preset.env)
        # The recorder's typed preset parser checks obs and nested unknown keys.
        # Parsing alone never builds an environment or opens a device.
        args = parse_record_args(["--config", str(path)])
        assert args.real_robot.action_mode == preset.env["action_mode"]
    else:
        pytest.fail(f"Unclassified tutorial YAML: {path}")


def cli_commands():
    for path in sorted(EXAMPLES.glob("*.sh")):
        text = path.read_text(encoding="utf-8").replace("\\\n", " ")
        for number, line in enumerate(text.splitlines(), 1):
            tokens = shlex.split(line, comments=True)
            if tokens and tokens[0] == "rwrl":
                # Shell logging belongs to the shell, not the rwrl argument parser.
                for index, token in enumerate(tokens):
                    if token in {"|", "2>&1"}:
                        tokens = tokens[:index]
                        break
                yield path, number, tokens[1:]


COMMANDS = list(cli_commands())


@pytest.mark.parametrize("path,number,argv", COMMANDS,
                         ids=[f"{path.name}:{number}" for path, number, _ in COMMANDS])
def test_rwrl_commands_parse(path: Path, number: int, argv: list[str]) -> None:
    parser = build_parser()
    from real_world_rl.cli.cmd_learning import COMMANDS as LEARNING_COMMANDS

    if argv[0] in {command.name for command in LEARNING_COMMANDS}:
        args, remaining = parser.parse_known_args(argv)
        delegated = args._command.entrypoint_argv(args, remaining)
        assert delegated == argv[1:]
        if args._command.name == "deploy-openpi":
            import tyro
            from real_world_rl.deploy.openpi.loop import DeployOpenPiArgs, _make_profile

            if "--help" in delegated:
                with pytest.raises(SystemExit) as result:
                    tyro.cli(DeployOpenPiArgs, args=delegated)
                assert result.value.code == 0
            else:
                # Validate typed arguments and the profile without opening a socket.
                _make_profile(tyro.cli(DeployOpenPiArgs, args=delegated))
        return
    if "--help" in argv:
        with pytest.raises(SystemExit) as result:
            parser.parse_args(argv)
        assert result.value.code == 0, f"{path}:{number}"
    else:
        parser.parse_args(argv)


def test_examples_are_discovered() -> None:
    assert INCLUDES
    assert COMMANDS
    assert list(EXAMPLES.glob("*.yaml"))


def test_task_cameras_match_station() -> None:
    station = load_station(EXAMPLES / "station.yaml").select().config
    task = Preset.load(str(EXAMPLES / "task.yaml"))
    cameras = {camera.name: camera for camera in station.cameras}
    assert set(task.document["obs"]["rgb"]) <= cameras.keys()
    assert all(cameras[name].enable_depth for name in task.document["obs"]["depth"])
    assert all(size > 0 for size in task.document["obs"]["image_size"])


def without_hash_comments(text: str) -> list[str]:
    """Remove comments outside quotes, preserving hashes in command/value strings."""
    result = []
    for line in text.splitlines():
        quote = None
        escaped = False
        kept = []
        for char in line:
            if escaped:
                kept.append(char)
                escaped = False
                continue
            if char == '\\' and quote != "'":
                escaped = True
            elif char in {'"', "'"}:
                if quote is None:
                    quote = char
                elif quote == char:
                    quote = None
            elif char == '#' and quote is None:
                break
            kept.append(char)
        value = ''.join(kept).rstrip()
        if value.strip():
            result.append(value)
    return result


def assert_translated_example(chinese: str, english: str, suffix: str) -> None:
    if suffix in {'.sh', '.yaml', '.yml', '.py'}:
        assert without_hash_comments(chinese) == without_hash_comments(english)
    elif suffix == '.txt':
        zh_lines, en_lines = chinese.splitlines(), english.splitlines()
        # Output translations preserve line correspondence. Remove only translated
        # lines at the same position; never discard arbitrary English differences.
        assert len(zh_lines) == len(en_lines)
        for number, (zh, en) in enumerate(zip(zh_lines, en_lines), 1):
            if not re.search(r'[\u3400-\u9fff]', zh):
                assert zh.replace("，", ",") == en, f'output line {number}: {zh!r} != {en!r}'
    else:
        assert chinese == english


@pytest.mark.parametrize('english', sorted(p for p in (EXAMPLES / 'en').rglob('*') if p.is_file()),
                         ids=lambda p: p.name)
def test_translated_examples_preserve_commands_and_values(english: Path) -> None:
    chinese = EXAMPLES / english.relative_to(EXAMPLES / 'en')
    assert chinese.is_file()
    assert_translated_example(chinese.read_text(), english.read_text(), english.suffix)


@pytest.mark.parametrize('suffix,zh,en', [
    ('.sh', 'rwrl doctor # 检查', 'rwrl status # check'),
    ('.yaml', 'port: 50051 # 端口', 'port: 50052 # port'),
    ('.txt', '示例\nsteps=30\n', 'Example\nsteps=31\n'),
])
def test_translated_example_comparison_detects_changes(suffix, zh, en):
    with pytest.raises(AssertionError):
        assert_translated_example(zh, en, suffix)

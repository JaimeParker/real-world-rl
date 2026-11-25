"""``rwrl shell``: profiles, the per-terminal selection, line preparation, daemons and the UI."""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest

from real_world_rl import cli
from real_world_rl.cli import registry
from real_world_rl.cli.cmd_use import current
from real_world_rl.cli.command import Result
from real_world_rl.common import station_paths
from real_world_rl.shell import daemons as daemons_module
from real_world_rl.shell import monitor
from real_world_rl.shell.daemons import DaemonManager, process_start
from real_world_rl.shell.profiles import discover
from real_world_rl.shell.session import Edit, Run, Session, SessionError, command_name, split_line

EXAMPLES = Path(__file__).resolve().parents[1] / "docs" / "examples"
MOCK = (EXAMPLES / "mock-station.yaml").read_text()
COMMANDS = {command.name for command in cli.COMMANDS}


def add_site(root, name, station=MOCK, tasks=()):
    site = root / "local" / "sites" / name
    (site / "tasks").mkdir(parents=True)
    (site / "station.yaml").write_text(station)
    for task in tasks:
        shutil.copy(EXAMPLES / "task.yaml", site / f"{task}.yaml")
    return site


@pytest.fixture
def root(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    for name in ("RWRL_STATION", "RWRL_PRESET", "RWRL_ROBOT", "RWRL_SHELL", "ROBOT_INFRA_CONFIG"):
        monkeypatch.delenv(name, raising=False)
    add_site(tmp_path, "alpha", MOCK + "name: arm_a\n", ["task", "tasks/peg"])
    both = "robots:\n" + "".join(
        f"  {name}:\n    type: franka\n    arm_backend: fake\n    daemon: {{bind: '127.0.0.1:{port}'}}\n"
        for name, port in (("left", 50151), ("right", 50152)))
    add_site(tmp_path, "both", both, ["tasks/cable"])
    add_site(tmp_path, "old", MOCK + "safety_box: [0, 0, 0, 1, 1, 1]\n")
    return tmp_path


def selected(name, root):
    session = Session(root)
    session.select(session.find(name))
    return session


# -- profiles ------------------------------------------------------------------------------


def test_profiles_list_tasks_robots_and_broken_stations(root):
    found = {profile.name: profile for profile in discover(root)}
    assert list(found) == ["alpha", "both", "old"]
    assert list(found["alpha"].tasks) == ["task", "peg"]
    assert found["both"].robots == ["left", "right"]
    assert "limits migrate" in found["old"].error
    with pytest.raises(SessionError, match="does not load"):
        Session(root).select(found["old"])


def test_station_files_outside_sites_are_listed(root):
    (root / "local" / "station.yaml").write_text(MOCK)
    (root / "xdg" / "rwrl").mkdir(parents=True)
    (root / "xdg" / "rwrl" / "station.yaml").write_text(MOCK)
    assert [profile.name for profile in discover(root)][-2:] == ["local", "user"]


# -- selection -----------------------------------------------------------------------------


def test_selection_stays_in_the_session_and_is_preselected_next_time(root):
    session = selected("alpha", root)
    session.set_task("peg")
    assert station_paths.read_selection(root) == {}
    assert Session(root).profile is None  # preselected only, never selected
    again = Session(root)
    assert again.remembered["profile"] == "alpha"
    again.select(again.find("alpha"))
    assert again.task == "peg"


def test_switching_device_switches_its_task_and_robot(root):
    session = selected("alpha", root)
    assert (session.task, session.robot) == ("task", "arm_a")
    session.select(session.find("both"))
    assert (session.task, session.robot) == ("cable", None)
    session.set_robot("right")
    env = session.child_env()
    assert env["RWRL_ROBOT"] == "right"
    assert env["RWRL_PRESET"].endswith("both/tasks/cable.yaml")


def test_default_stores_the_selection_for_plain_commands(root):
    session = selected("alpha", root)
    session.save_default()
    assert station_paths.read_selection(root) == {
        "station": "local/sites/alpha/station.yaml", "preset": "local/sites/alpha/task.yaml"}
    (root / "local/sites/alpha/task.yaml").unlink()
    session.refresh()
    session.save_default()  # no task: the previous device's preset must not stay behind
    assert "preset" not in station_paths.read_selection(root)


@pytest.mark.parametrize("content", ["{bad", "[]", '{"tasks": 3, "profile": []}'])
def test_a_damaged_memory_file_only_loses_the_preselection(root, content):
    selected("alpha", root)
    (root / "local/.rwrl-shell/last.json").write_text(content)
    assert selected("alpha", root).task == "task"


def test_refresh_notices_a_station_that_stopped_loading(root):
    session = selected("alpha", root)
    assert not session.refresh()
    (root / "local/sites/alpha/station.yaml").write_text("robots: [\n")
    assert session.refresh()
    with pytest.raises(SessionError, match="no longer loads"):
        session.prepare("q", COMMANDS)
    assert isinstance(session.prepare("limits migrate local/sites/alpha/station.yaml", COMMANDS), Run)


# -- lines -----------------------------------------------------------------------------------


def test_a_line_runs_the_plain_command_with_the_selection(root, monkeypatch):
    monkeypatch.setenv("RWRL_STATION", "/elsewhere.yaml")
    plan = selected("alpha", root).prepare("rwrl --json home show", COMMANDS)
    assert plan.argv[1:] == ["-m", "real_world_rl.cli", "--json", "home", "show"]
    assert plan.env["RWRL_STATION"] == str(root / "local/sites/alpha/station.yaml")
    assert plan.env["RWRL_PRESET"] == str(root / "local/sites/alpha/task.yaml")
    assert "RWRL_ROBOT" not in plan.env  # a single robot needs no name
    assert "--yes" not in plan.argv


def test_an_explicit_station_gets_none_of_the_selection(root):
    plan = selected("both", root).prepare("--station x.yaml q", COMMANDS)
    assert not {"RWRL_STATION", "RWRL_PRESET", "RWRL_ROBOT"} & set(plan.env)


@pytest.mark.parametrize("line, message", [
    ("nope", "unknown command"), ("use station x.yaml", "every terminal"), ("shell", "already"),
    ("--json", "no command"), ("home 'x", "cannot read")])
def test_refused_lines(root, line, message):
    with pytest.raises(SessionError, match=message):
        selected("alpha", root).prepare(line, COMMANDS)


def test_without_a_device_only_deviceless_commands_run(root):
    session = Session(root)
    for line in ("q", "doctor", "train hil_serl", "limits show"):
        with pytest.raises(SessionError, match="no device"):
            session.prepare(line, COMMANDS)
    for line in ("setup", "dataset list d", "q --help", "limits migrate x.yaml"):
        assert isinstance(session.prepare(line, COMMANDS), Run)


def test_learning_commands_get_an_editable_config(root):
    session = selected("alpha", root)
    plan = session.prepare("record --output-dir 'my demos'", COMMANDS)
    assert plan == Edit("record --output-dir 'my demos' --config local/sites/alpha/task.yaml")
    assert isinstance(session.prepare(plan.text, COMMANDS), Run)
    for line in ("record --help", "--preset p.yaml train x", "train x --config=p.yaml", "deploy --checkpoint c"):
        assert isinstance(session.prepare(line, COMMANDS), Run)
    session.set_task(None)
    with pytest.raises(SessionError, match="needs a task"):
        session.prepare("train hil_serl", COMMANDS)


@pytest.mark.parametrize("remote, bind, warned", [
    ("127.0.0.1:50051", "127.0.0.1:50051", False), ("localhost:50051", "127.0.0.1:50051", False),
    ("10.0.0.7:50051", "0.0.0.0:50051", False), ("10.0.0.7:50052", "0.0.0.0:50051", True),
    ("10.0.0.7:50051", "10.0.0.8:50051", True)])
def test_config_of_another_daemon_warns(root, remote, bind, warned):
    (root / "local/sites/alpha/station.yaml").write_text(MOCK + f"daemon: {{bind: '{bind}'}}\n")
    (root / "t.yaml").write_text(f"real_robot: {{remote: '{remote}'}}\n")
    plan = selected("alpha", root).prepare("train x --config t.yaml", COMMANDS)
    assert bool(plan.warnings) is warned


def test_line_helpers():
    assert split_line("rwrl home show") == ["home", "show"]
    assert command_name(["--robot", "left", "--json", "q"]) == "q"
    assert command_name(["--robot=left", "q"]) == "q"


# -- what the plain commands do with the environment ----------------------------------------------


class Probe:
    help = ""

    def __init__(self, name="probe", uses_selection=True):
        self.name, self.uses_selection = name, uses_selection

    def register(self, parser):
        pass

    def run(self, args, ctx):
        self.seen = (args.station, args.robot, args.preset)
        return Result({}, "")


def seen(*argv, **options):
    command = Probe(**options)
    assert registry.main([*argv, command.name], commands=[command], prog="rwrl", description="") == 0
    return command.seen


def test_plain_commands_read_the_preset_and_robot_environment(root, monkeypatch):
    preset = str(root / "local/sites/alpha/task.yaml")
    assert seen() == (None, None, None)
    station_paths.write_selection({"preset": "other.yaml"})
    monkeypatch.setenv("RWRL_PRESET", preset)
    monkeypatch.setenv("RWRL_ROBOT", "left")
    assert seen() == (None, "left", preset)
    assert seen("--robot", "right", "--preset", "p") == (None, "right", "p")
    assert seen("--station", "s") == ("s", None, preset)
    assert seen(uses_selection=False) == (None, None, None)
    assert current()["preset"] == {"path": preset, "source": "$RWRL_PRESET", "overrides": "other.yaml"}
    monkeypatch.setenv("RWRL_PRESET", "gone.yaml")
    assert registry.main(["probe"], commands=[Probe()], prog="rwrl", description="") == 2


# -- the command itself ------------------------------------------------------------------------------


def test_shell_refuses_pipes_other_options_and_nesting(root, monkeypatch, capsys):
    assert cli.main(["shell"]) == 2
    assert "terminal" in capsys.readouterr().err
    assert cli.main(["--station", "x", "shell"]) == 2
    assert cli.main(["shell", "--task", "peg"]) == 2
    assert cli.main(["--json", "shell"]) == 2
    monkeypatch.setenv("RWRL_SHELL", "1")
    capsys.readouterr()
    assert cli.main(["shell"]) == 2
    assert "already inside" in capsys.readouterr().err


@pytest.mark.parametrize("missing", ["prompt_toolkit", "rich"])
def test_a_missing_extra_names_the_install_command(root, monkeypatch, capsys, missing):
    monkeypatch.setattr(sys, "stdin", type("T", (), {"isatty": lambda self: True})())
    monkeypatch.setattr(sys.stdout, "isatty", lambda: True, raising=False)
    monkeypatch.setitem(sys.modules, missing, None)
    assert cli.main(["shell"]) == 2
    assert ".[shell]" in capsys.readouterr().err


# -- daemons ---------------------------------------------------------------------------------------------


@pytest.fixture
def manager(root, monkeypatch):
    """A manager whose 'daemon' is a sleeping process."""
    real = subprocess.Popen
    monkeypatch.setattr(daemons_module.subprocess, "Popen",
                        lambda argv, **kw: real([sys.executable, "-c", "import time; time.sleep(60)"], **kw))
    monkeypatch.setattr("real_world_rl.cli.connect.probe_daemon", lambda address, timeout_s=.5: False)
    manager = DaemonManager(selected("both", root))
    yield manager
    for entry in manager.running():
        os.kill(entry["pid"], signal.SIGKILL)


def test_a_started_daemon_is_recorded_detached_and_stopped(manager, root):
    entry = manager.start()
    assert os.getsid(entry["pid"]) == entry["pid"]
    other = DaemonManager(selected("both", root))  # a later shell in this directory
    assert [item["pid"] for item in other.of_profile()] == [entry["pid"]]
    with pytest.raises(SessionError, match="already starting"):
        manager.start()
    assert manager.stop(entry)
    assert manager.running() == [] and other.running() == []


def test_a_reused_pid_is_never_signalled(manager, monkeypatch):
    entry = manager.start()
    monkeypatch.setattr(daemons_module.os, "kill", lambda *args: pytest.fail("signalled"))
    assert manager.stop({**entry, "start_ticks": entry["start_ticks"] + 1})
    assert manager.stop({**entry, "boot_id": "another boot"})
    monkeypatch.undo()


def test_start_refuses_a_used_address_and_a_station_without_daemon(manager, root, monkeypatch):
    monkeypatch.setattr("real_world_rl.cli.connect.probe_daemon", lambda address, timeout_s=.5: True)
    with pytest.raises(SessionError, match="already listens"):
        manager.start()
    with pytest.raises(SessionError, match="declares no daemon"):
        DaemonManager(selected("alpha", root)).start()


def test_wait_ready_ends_when_the_process_does(manager):
    entry = manager.start()
    os.kill(entry["pid"], signal.SIGKILL)
    begin = time.monotonic()
    assert not manager.wait_ready(entry, timeout_s=10)
    assert time.monotonic() - begin < 5


def test_process_start_of_a_missing_process():
    assert process_start(os.getpid()) is not None
    assert process_start(2**22 + 1) is None


# -- status ------------------------------------------------------------------------------------------------


def test_status_queries_identity_once_per_selection_and_per_restart(root):
    session = selected("both", root)
    up, calls = {"127.0.0.1:50151": True}, []
    answers = {"left": {"mismatches": []}, "right": {"mismatches": ["name: x"]}}

    def query(station, robot, env, cwd):
        calls.append(robot)
        return answers[robot]

    watch = monitor.StatusMonitor(session, lambda: None, query=query, probe=lambda address: up.get(address, False))
    watch.update()
    watch.update()
    assert calls == ["left"]
    assert watch.states["left"][0] == monitor.MATCH and watch.states["right"][0] == monitor.DOWN
    up["127.0.0.1:50152"] = True
    watch.update()
    assert calls == ["left", "right"] and watch.states["right"][0] == monitor.MISMATCH
    watch.recheck()
    watch.update()
    assert calls == ["left", "right"] * 2
    session.select(session.find("alpha"))
    watch.update()
    assert watch.states == {"arm_a": (monitor.DIRECT, "")}


def test_status_of_the_previous_device_is_dropped(root):
    session = selected("both", root)

    def query(*args):
        session.select(session.find("alpha"))  # switched while the query ran
        return {"mismatches": []}

    watch = monitor.StatusMonitor(session, lambda: None, query=query, probe=lambda address: True)
    watch.update()
    assert watch.states == {}


# -- UI ----------------------------------------------------------------------------------------------------------


@pytest.fixture
def ui(root):
    pytest.importorskip("prompt_toolkit")
    pytest.importorskip("rich")
    from real_world_rl.shell.app import Shell
    ran, answers, picks = [], [], []
    shell = Shell(selected("alpha", root), ask=lambda question: answers.pop(0),
                  choose=lambda title, items, **options: picks.pop(0),
                  run=lambda argv, env, cwd: ran.append(argv[3:]) or 0)
    shell.ran, shell.answers, shell.picks = ran, answers, picks
    return shell


def test_lines_reach_commands_or_builtins(ui, capsys):
    assert ui.handle("home show") and ui.handle("")
    assert ui.ran == [["home", "show"]]
    ui.handle("task peg")
    assert ui.session.task == "peg"
    ui.handle("bogus")
    assert "unknown command" in capsys.readouterr().out
    ui.handle("help home")
    assert ui.ran[-1] == ["home", "--help"]
    assert ui.handle("exit") is False


def test_switch_with_several_robots_asks_for_one(ui):
    ui.picks[:] = ["both", "right"]
    ui.handle("switch")
    assert (ui.session.profile.name, ui.session.robot) == ("both", "right")
    ui.handle("robot all")
    assert ui.session.robot is None


def test_cancelling_the_list_keeps_the_device(ui):
    ui.picks[:] = [None]
    ui.handle("switch")
    assert ui.session.profile.name == "alpha"


def test_menu_prefills_and_never_runs(ui):
    ui.picks[:] = ["home go"]
    ui.handle("menu")
    assert ui._prefill == "home go" and ui.ran == []


def test_config_is_prefilled_then_runs(ui):
    ui.handle("train hil_serl")
    assert ui.ran == []
    ui.handle(ui._prefill)
    assert ui.ran == [["train", "hil_serl", "--config", "local/sites/alpha/task.yaml"]]


def test_default_needs_a_yes(ui, root):
    ui.answers[:] = ["n"]
    ui.handle("default")
    assert station_paths.read_selection(root) == {}
    ui.answers[:] = ["y"]
    ui.handle("default")
    assert station_paths.read_selection(root)["preset"].endswith("alpha/task.yaml")


def test_config_of_another_daemon_needs_a_yes(ui, root):
    (root / "t.yaml").write_text("real_robot: {remote: '10.0.0.7:1'}\n")
    ui.answers[:] = [""]
    ui.handle("train x --config t.yaml")
    assert ui.ran == []


@pytest.mark.parametrize("answer, runs", [({"mismatches": ["name: x"]}, False), ({"mismatches": []}, True),
                                          ({"error": "down"}, True)])
def test_learning_commands_refuse_a_daemon_of_another_robot(ui, root, answer, runs, capsys):
    ui.session.select(ui.session.find("both"))
    (root / "t.yaml").write_text("real_robot: {remote: '127.0.0.1:50152'}\n")
    asked = []
    ui._query = lambda station, robot, env, cwd: asked.append(robot) or answer
    ui.handle("train x --config t.yaml")
    assert asked == ["right"]
    assert bool(ui.ran) is runs
    assert ("refused" in capsys.readouterr().out) is not runs


def test_daemon_run_is_a_foreground_command_and_stop_only_touches_its_own(ui, capsys):
    ui.handle("daemon run --no-preflight")
    assert ui.ran == [["daemon", "--no-preflight"]]
    ui.handle("daemon stop")
    ui.handle("daemon --bind x")
    out = capsys.readouterr().out
    assert "was started from the shell" in out and "options go after" in out


def test_toolbar_and_prompt_mark_a_wrong_robot(ui):
    ui.monitor.states = {"arm_a": (monitor.MISMATCH, "127.0.0.1:1: name: x")}
    assert any(style == "class:bad" and "WRONG ROBOT 127.0.0.1:1: name: x" in text for style, text in ui.toolbar())
    assert ui.prompt_text()[0][0] == "class:bad"


def test_messages_are_never_read_as_markup_and_paths_stay_whole(ui, capsys):
    from real_world_rl.shell import view
    path = "/very/long/" + "x" * 200 + "/[bold]station[/].yaml"
    view.error(path)
    view.log(path, "PASS  [red]row[/red]")
    out = capsys.readouterr().out
    assert out.count(path) == 2 and "[red]row[/red]" in out
    assert "\x1b" not in out  # no colour outside a terminal


def test_symbols_fall_back_to_ascii(monkeypatch):
    pytest.importorskip("prompt_toolkit")
    pytest.importorskip("rich")
    from real_world_rl.shell import view
    monkeypatch.setattr(view.sys, "stdout", type("Out", (), {"encoding": "ascii"})())
    assert {view.symbol(name) for name in view.SYMBOLS} == {plain for _, plain in view.SYMBOLS.values()}


def test_info_shows_the_selection(ui, capsys):
    ui.handle("info")
    out = capsys.readouterr().out
    for text in ("alpha", "local/sites/alpha/station.yaml", "task", "arm_a", "direct", "this terminal only"):
        assert text in out


def test_every_list_of_the_shell_renders(ui):
    from real_world_rl.shell.select import Selector
    drawn = []

    def draw(title, items, *, preselect=None, style=None):
        drawn.append("".join(text for _, text in Selector(title, items, preselect=preselect).fragments()))

    ui._choose = draw
    for line in ("switch", "task", "robot", "menu"):
        ui.handle(line)
    assert len(drawn) == 4
    assert all(ui.commands.values())  # every command has a help text
    assert "old" in drawn[0] and "does not load" in drawn[0]


def test_list_filters_and_skips_disabled():
    pytest.importorskip("prompt_toolkit")
    pytest.importorskip("rich")
    from real_world_rl.shell.select import Item, Selector
    selector = Selector("t", [Item(1, "old", disabled=True), Item(2, "zizhu"), Item(3, "lab")])
    assert selector.current().value == 2
    selector.move(1)
    selector.move(1)
    assert selector.current().value == 2
    selector.set_filter("lab")
    assert selector.current().value == 3
    selector.set_filter("old")
    assert selector.current() is None
    selector.set_filter("nothing")
    assert selector.current() is None
    selector.move(1)
    assert "no match" in str(selector.fragments())


def test_completion(ui):
    pytest.importorskip("prompt_toolkit")
    pytest.importorskip("rich")
    from prompt_toolkit.document import Document
    from real_world_rl.shell.completion import ShellCompleter
    completer = ShellCompleter(ui.session, ui.parser)

    def words(text):
        return [item.text for item in completer.get_completions(Document(text), None)]

    assert words("sw") == ["switch"]
    assert set(words("home ")) == {"show", "set", "go"}
    assert words("rwrl gripper cl") == ["close"]
    assert words("switch b") == ["both"]
    assert words("task p") == ["peg"]
    assert "--duration-s" in words("teleop read --")
    assert "--config" in words("train --c")
    assert words("") and "use" not in words("")


def test_every_menu_command_exists(ui):
    from real_world_rl.shell.app import MENU
    from real_world_rl.shell.completion import BUILTINS
    for _, line, _ in MENU:
        assert line.split()[0] in COMMANDS | set(BUILTINS)


def test_ctrl_c_ends_the_command_and_not_the_shell(root):
    pytest.importorskip("prompt_toolkit")
    pytest.importorskip("rich")
    script = ("import sys, os, signal\nfrom real_world_rl.shell.app import run_foreground\n"
              "code = run_foreground([sys.executable, '-c', 'import time; print(\"up\", flush=True); time.sleep(30)'],"
              " dict(os.environ), '.')\nprint('shell lives', code, flush=True)\n")
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)}
    process = subprocess.Popen([sys.executable, "-c", script], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                               stdin=subprocess.DEVNULL, text=True, start_new_session=True, env=env)
    assert process.stdout.readline().strip() == "up"
    os.killpg(process.pid, signal.SIGINT)
    assert "shell lives" in process.communicate(timeout=10)[0]
    assert process.returncode == 0


def test_errors_are_coloured_only_on_a_terminal(root, monkeypatch, capsys):
    from real_world_rl.cli.errors import CliError
    registry._report_error(CliError("broken", hint="fix it"), False)
    assert capsys.readouterr().err == "error: broken (hint: fix it)\n"
    monkeypatch.delenv("NO_COLOR", raising=False)
    monkeypatch.setenv("TERM", "xterm")
    monkeypatch.setattr(sys.stderr, "isatty", lambda: True, raising=False)
    registry._report_error(CliError("broken"), False)
    assert "\x1b[1;31merror:" in capsys.readouterr().err

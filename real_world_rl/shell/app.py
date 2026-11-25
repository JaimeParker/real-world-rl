"""The prompt_toolkit front end of ``rwrl shell``."""
from __future__ import annotations

from contextlib import contextmanager
import os
from pathlib import Path
import signal
import subprocess
import sys
import termios

from prompt_toolkit import PromptSession
from prompt_toolkit.history import FileHistory

from . import monitor as status, view
from .completion import BUILTINS, DAEMON_ACTIONS, ShellCompleter, command_help
from .daemons import DaemonManager, tail
from .select import Item, choose
from .session import Edit, Session, SessionError, has_option, split_line, state_dir

# Category, line put on the prompt, what it does. Choosing one never runs it directly.
MENU = (
    ("look", "status", "connection, arm health, devices"),
    ("look", "q", "joint positions"),
    ("look", "tcp", "TCP pose"),
    ("look", "gripper show", "gripper feedback"),
    ("look", "camera show", "camera rate and health"),
    ("look", "home show", "home and distance to it"),
    ("look", "limits show", "joint limits and TCP box"),
    ("move", "home go", "move to home (asks first)"),
    ("move", "gripper open", "open the gripper (asks first)"),
    ("move", "gripper close", "close the gripper (asks first)"),
    ("move", "stop", "stop the arm"),
    ("move", "recover", "recover the arm from an error"),
    ("teleop", "teleop read", "live teleop samples, no motion"),
    ("teleop", "teleop run --record ", "supervised teleop session"),
    ("teleop", "teleop calibrate", "calibrate the teleop device"),
    ("data", "record --output-dir ", "record demonstrations"),
    ("data", "dataset list ", "episodes of a dataset"),
    ("data", "dataset view ", "review a dataset"),
    ("learn", "train ", "train a policy"),
    ("learn", "deploy ", "deploy a trained policy"),
    ("setup", "doctor", "check host, station and devices"),
    ("setup", "home set", "save the current joints as the task home"),
    ("setup", "discover", "visible hardware"),
    ("setup", "setup", "configure a station"),
    ("setup", "daemon start", "start the local daemon"),
    ("setup", "daemon stop", "stop the daemon started here"),
    ("shell", "switch", "change device"),
    ("shell", "task", "change task"),
    ("shell", "info", "selected files"),
)


@contextmanager
def child_owns_terminal():
    """Ctrl-C goes to the foreground child only; the terminal mode is restored afterwards. A
    handler (not SIG_IGN, which a child would inherit) keeps the shell alive."""
    previous = signal.signal(signal.SIGINT, lambda signum, frame: None)
    fd = sys.stdin.fileno() if os.isatty(0) else None
    saved = termios.tcgetattr(fd) if fd is not None else None
    try:
        yield
    finally:
        if saved is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        signal.signal(signal.SIGINT, previous)


def run_foreground(argv, env, cwd) -> int:
    with child_owns_terminal():
        child = subprocess.Popen(argv, env=env, cwd=cwd)
        try:
            return child.wait()
        except BaseException:
            child.terminate()
            child.wait()
            raise


class Shell:
    def __init__(self, session: Session, *, prompt=None, ask=None, choose=choose, run=run_foreground,
                 query=status.query_identity) -> None:
        from real_world_rl.cli import build_parser
        self.session = session
        self.daemons = DaemonManager(session)
        self.parser = build_parser()
        self.commands = command_help(self.parser)
        self._choose = choose
        self._run = run
        self._query = query
        self._prompt_session = prompt
        self._ask = ask or view.ask
        self.monitor = status.StatusMonitor(session, self._invalidate)
        self._prefill = ""

    # -- display -----------------------------------------------------------------------

    def _invalidate(self) -> None:
        if self._prompt_session is not None:
            self._prompt_session.app.invalidate()  # thread safe

    def prompt_text(self):
        session = self.session
        arrow = f" {view.symbol('arrow')} "
        if session.profile is None:
            return [("class:warn", "no device"), ("class:arrow", arrow)]
        wrong = any(state == status.MISMATCH for state, _ in self.monitor.states.values())
        return [("class:bad" if wrong else "class:device", session.profile.name),
                ("class:task", f"/{session.task}" if session.task else ""),
                ("class:robot", f" [{session.robot or 'all robots'}]"),
                ("class:bad" if wrong else "class:arrow", arrow)]

    def toolbar(self):
        profile = self.session.profile
        if profile is None:
            return [("class:warn", " no device selected: type `switch` ")]
        if profile.error is not None:
            return [("class:bad", f" {profile.name} does not load: {profile.error} ")]
        separator = ("class:bar.key", f"  {view.symbol('sep')}  ")
        parts = [("class:bar.device", f" {profile.name}"), separator,
                 ("class:bar.key", "task "), ("", self.session.task or "none")]
        dot, cross = view.symbol("dot") + " ", view.symbol("cross") + " "
        for name in self.session.active_robots():
            state, detail = self.monitor.states.get(name, (status.CHECKING, ""))
            style, text = {
                status.DIRECT: ("class:bar.key", dot + "direct, no daemon"),
                status.DOWN: ("class:warn", f"{dot}daemon {detail} down"),
                status.CHECKING: ("class:bar.key", f"{dot}daemon {detail} checking"),
                status.MATCH: ("class:good", f"{dot}daemon {detail} ok"),
                status.MISMATCH: ("class:bad", f"{cross}WRONG ROBOT {detail}"),
                status.UNKNOWN: ("class:warn", f"{dot}daemon not identified {detail}"),
            }[state]
            parts += [separator, ("bold", name + " "), (style, text)]
        return parts

    def confirmed(self, question: str) -> bool:
        try:
            return self._ask(question).strip().lower() == "y"
        except (EOFError, KeyboardInterrupt):
            print()
            return False

    # -- built-in commands ---------------------------------------------------------------

    def switch(self, name: str | None = None) -> None:
        if name is None:
            profiles = self.session.profiles()
            if not profiles:
                raise SessionError(f"no device profile below {self.session.cwd}/local/sites; "
                                   "run `setup`, then move its station.yaml to local/sites/<name>/")
            remembered = self.session.remembered.get("profile")
            current = self.session.profile.name if self.session.profile else remembered
            title = "Select a device" + (f" (last used: {remembered})" if remembered else "")
            name = self._choose(title, [Item(item.name, item.name, item.summary(), disabled=item.error is not None)
                                        for item in profiles], preselect=current, style=view.STYLE)
            if name is None:
                broken = [item for item in profiles if item.error is not None]
                if len(broken) == len(profiles):
                    for item in broken:
                        view.error(f"{item.name}: {item.error}")
                return
        self.session.select(self.session.find(name))
        if self.session.robot is None:
            self.robot()
        if len(self.session.profile.tasks) > 1:
            self.task()
        self.info()

    def task(self, name: str | None = None) -> None:
        profile = self.session.require_profile()
        if name is None:
            if not profile.tasks:
                raise SessionError(f"{profile.name} has no task: add {self.session.shown(profile.station.parent)}"
                                   "/task.yaml or tasks/<name>.yaml")
            name = self._choose("Select a task", [Item(key, key, self.session.shown(path))
                                                  for key, path in profile.tasks.items()],
                                preselect=self.session.task, style=view.STYLE)
            if name is None:
                return
        self.session.set_task(name)

    def robot(self, name: str | None = None) -> None:
        profile = self.session.require_profile()
        if name is None:
            items = [Item(key, key) for key in profile.robots]
            if len(items) > 1:
                items.append(Item("all", "all", "no single robot; commands that need one ask for --robot"))
            name = self._choose("Select a robot", items, preselect=self.session.robot or "all", style=view.STYLE)
            if name is None:
                return
        self.session.set_robot(None if name == "all" and "all" not in profile.robots else name)

    def info(self) -> None:
        view.info(self.session.info_rows())

    def menu(self) -> None:
        self._prefill = self._choose("Pick a command", [Item(line, line.strip(), text, group=group)
                                                        for group, line, text in MENU], style=view.STYLE) or ""

    def default(self) -> None:
        self.session.require_profile()
        if self.confirmed("Use this device and task for every plain `rwrl` command in this directory "
                          "(other terminals and agents too)?"):
            view.success(f"stored in {self.session.shown(self.session.save_default())}")

    def help(self) -> None:
        view.command_list([("shell", BUILTINS), ("robot commands", {
            name: text for name, text in self.commands.items() if name not in (*BUILTINS, "use", "shell")})])
        view.say("`help <command>` shows its options; Tab completes; the rwrl prefix is not needed.", "bright_black")

    def daemon(self, words: list[str]) -> None:
        action = words[0] if words else "status"
        if action.startswith("-") or action not in DAEMON_ACTIONS:
            raise SessionError("daemon " + "|".join(DAEMON_ACTIONS) + "; options go after start or run")
        if len(words) > 1 and action not in ("start", "run"):
            raise SessionError(f"daemon {action} takes no options")
        if action == "run":
            self.execute(["daemon", *words[1:]])
        elif action == "start":
            self._daemon_start(words[1:])
        elif action == "stop":
            entries = self.daemons.of_profile()
            if not entries:
                raise SessionError("no running daemon of this device was started from the shell; "
                                   "others must be stopped where they were started")
            for entry in entries:
                self._daemon_stop(entry)
        elif action == "log":
            log = self.daemons.last_log()
            if log is None:
                raise SessionError("no daemon of this device was started from the shell")
            view.log(self.session.shown(log), tail(log))
        else:
            self._daemon_status()

    def _daemon_status(self) -> None:
        addresses = self.daemons.addresses()
        if not addresses:
            view.say("no daemon declared: commands connect directly", "bright_black")
        for name, address in addresses.items():
            answer = self._query(str(self.session.profile.station), name, self.session.child_env(),
                                 self.session.cwd)
            if "error" in answer:
                text, state = answer["error"], "warn"
            elif answer["mismatches"]:
                text, state = "WRONG ROBOT: " + "; ".join(answer["mismatches"]), "bad"
            else:
                text, state = "serving this robot", "good"
            view.daemon_row(name, address, text, state)
        for entry in self.daemons.of_profile():
            view.say(f"started from the shell: pid {entry['pid']}, "
                     f"log {self.session.shown(Path(entry['log']))}", "bright_black")

    def _daemon_start(self, extra: list[str]) -> None:
        if any(has_option([word], "--station", "--bind") for word in extra):
            raise SessionError("daemon start serves the selected station at its own address; "
                               "use `daemon run` for other options")
        entry = self.daemons.start(extra)
        view.say(f"log {self.session.shown(Path(entry['log']))}; Ctrl-C stops waiting, not the daemon", "bright_black")
        try:
            with view.console.status(f"starting daemon (pid {entry['pid']})"):
                ready = self.daemons.wait_ready(entry)
        except KeyboardInterrupt:
            view.warning("still starting; see `daemon log`")
            return
        if ready:
            view.success("daemon is listening; it keeps running after you leave the shell")
        elif self.daemons.alive(entry):
            view.warning("not listening yet; see `daemon log`")
        else:
            view.error("daemon ended while starting")
            view.log(entry["log"], tail(Path(entry["log"])))

    def _daemon_stop(self, entry: dict) -> None:
        if not self.confirmed(f"Stop the daemon of {entry['profile']} (pid {entry['pid']})? "
                              "Anything using that robot loses its connection."):
            return
        with view.console.status("stopping daemon"):
            stopped = self.daemons.stop(entry)
        if stopped:
            view.success("daemon stopped")
        else:
            view.warning(f"daemon has not ended yet (pid {entry['pid']}); see {entry['log']}")

    # -- loop ----------------------------------------------------------------------------

    def execute(self, words: list[str]) -> None:
        import shlex
        plan = self.session.prepare(shlex.join(words), self.commands)
        if isinstance(plan, Edit):
            self._prefill = plan.text
            view.say("task config filled in; edit if needed, Enter runs it", "bright_black")
            return
        if plan.warnings:
            for text in plan.warnings:
                view.warning(text)
            if not self.confirmed("Run it anyway?"):
                return
        for name in plan.daemon_robots:
            answer = self._query(str(self.session.profile.station), name, plan.env, self.session.cwd)
            # An unreachable daemon is the command's own error.
            if answer.get("mismatches"):
                raise SessionError(f"refused: the daemon of {name} serves another robot "
                                   f"({'; '.join(answer['mismatches'])})")
        code = self._run(plan.argv, plan.env, self.session.cwd)
        if code:
            view.ended(f"exit {code}" if code > 0 else f"ended by {signal.Signals(-code).name}")
        if self.session.refresh() and self.session.profile.error:
            view.warning(f"{self.session.profile.name} no longer loads: {self.session.profile.error}")

    def handle(self, line: str) -> bool:
        """Run one line; False leaves the shell."""
        try:
            words = split_line(line)
            if not words:
                return True
            head, rest = words[0], words[1:]
            self.session.refresh()
            if head in ("exit", "quit"):
                return not self.leave()
            if head in ("switch", "task", "robot"):
                if len(rest) > 1:
                    raise SessionError(f"{head} takes one name")
                getattr(self, head)(*rest)
            elif head in ("menu", "default", "info") and not rest:
                getattr(self, head)()
            elif head == "use" and not rest:
                self.info()
            elif head == "help":
                self.help() if not rest else self.execute([*rest, "--help"])
            elif head == "daemon" and not has_option(rest, "--help", "-h"):
                self.daemon(rest)
            else:
                self.execute(words)
        except SessionError as error:
            view.error(str(error))
        except KeyboardInterrupt:
            print()
        except Exception as error:  # one failed line must not end the session
            if os.environ.get("RWRL_DEBUG") == "1":
                raise
            view.error(f"{type(error).__name__}: {error}")
        self.monitor.recheck()
        return True

    def leave(self) -> bool:
        for entry in self.daemons.running():
            view.warning(f"the daemon of {entry['profile']} (pid {entry['pid']}) keeps running unless you stop it")
            self._daemon_stop(entry)
        return True

    def loop(self) -> None:
        self.monitor.start()
        try:
            while True:
                prefill, self._prefill = self._prefill, ""
                try:
                    line = self._prompt_session.prompt(self.prompt_text, default=prefill,
                                                       bottom_toolbar=self.toolbar)
                except KeyboardInterrupt:
                    continue
                except EOFError:
                    line = "exit"
                if not self.handle(line):
                    return
                if line.strip():
                    view.gap()
        finally:
            self.monitor.stop()


def history(session: Session) -> FileHistory:
    directory = state_dir(session.cwd)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "history"
    path.touch(mode=0o600)
    return FileHistory(str(path))


def run(profile: str | None = None, task: str | None = None) -> int:
    from real_world_rl.cli.errors import CliError, SUCCESS_EXIT_CODE, USAGE_EXIT_CODE
    session = Session()
    shell = Shell(session)
    shell._prompt_session = PromptSession(
        history=history(session), completer=ShellCompleter(session, shell.parser), complete_in_thread=True,
        style=view.STYLE, refresh_interval=None)
    inherited = session.env.get("RWRL_STATION") or session.env.get("ROBOT_INFRA_CONFIG")
    if inherited:
        view.warning(f"the station in the environment ({inherited}) is ignored here; the shell uses your selection")
    try:
        if profile is None:
            shell.switch()
        else:
            session.select(session.find(profile))
            if task is not None:
                session.set_task(task)
            shell.info()
    except SessionError as error:
        if profile is not None:
            raise CliError(str(error), code=USAGE_EXIT_CODE) from error
        view.error(str(error))
    view.say("`menu` picks a command, `help` lists them, `switch` changes device, Ctrl-D leaves.", "bright_black")
    view.gap()
    shell.loop()
    return SUCCESS_EXIT_CODE

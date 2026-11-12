# 1. Install

**Goal:** a project environment in which `rwrl --help` lists the commands and
`rwrl doctor --mock` ends with `Result: no blocking findings`. No robot, camera or teleop
device is needed on this page.

**Before you start:** the NUC (or any Linux machine you will run `rwrl` on) has Python 3.10
with venv support, `git`, and network access for `pip`. No Python environment is active.

## 1. Clone the repository

On the NUC, in the parent directory where the checkout should live, no environment active:

<!-- tutorial-test: skip -->
```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
```

`--recurse-submodules` also fetches `rl-garden/`, the learning library this project installs
from source.

<a id="4-install-the-project-environment"></a>

## 2. Run the installer

On the NUC, repository root, no environment active:

<!-- tutorial-test: skip -->
```bash
bash scripts/install.sh
source .venv/bin/activate
```

The installer creates (or reuses) `.venv` with `python3.10`, clears an inherited `PYTHONPATH`,
and installs rl-garden and this project in editable mode with the `learning` and `franka-gello`
extras (and, with `learning`, the `test` extra). This `.venv` is the **project environment**
that every later page activates. At the end it runs `rwrl doctor --mock` against a throwaway
station whose arm and gripper are fake: a pass checks the software, not the hardware.

| Option | Use it for |
|---|---|
| `--extras LIST` | other extras, comma separated (default `learning,franka-gello`); for example `learning,franka-gello,spacemouse` or add `realsense` |
| `--daemon-only` | a robot PC that only serves the daemon: extras `franky,grpc`, no `rl-garden/`, no torch |
| `--venv PATH` | another venv location (default `.venv`) |
| `--python PROGRAM` | another interpreter for a new venv (default `python3.10`) |
| `--torch-index URL` | install torch from this index first, for example `https://download.pytorch.org/whl/cpu` |
| `--station YAML` | run the final doctor against a real station instead of the mock one |
| `--no-doctor` | skip the final doctor |

`bash scripts/install.sh --help` prints the same list. Extras for later pages are installed
there: RealSense on [04](04-first-motion.md#3-cameras), SpaceMouse on
[06](06-teleop-spacemouse.md).

<details>
<summary>What the installer does, by hand</summary>

On the NUC, repository root, in any activated venv or non-base conda environment:

<!-- tutorial-test: skip -->
```bash
unset PYTHONPATH
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e ./rl-garden -e '.[learning,franka-gello,test]'
```

</details>

## 3. Check the command

On the NUC, repository root, project environment active:

<!-- tutorial-test: skip -->
```bash
rwrl --help
```

*You should see:* the usage line and the commands, among them `status`, `q`, `tcp`, `stop`,
`recover`, `home`, `gripper`, `camera`, `doctor`, `setup`, `daemon`, `supervise`, `discover`,
`teleop`, `workspace`, `archive`, `train`, `record`, `deploy` and `collect-reward`. Global
options such as `--station` and `--yes` go **before** the command; `rwrl <command> --help`
shows one command's flags.

## 4. Run the doctor without hardware

On the NUC, repository root, project environment active:

```bash
rwrl doctor --mock
```

*You should see* (your platform, interpreter path and report directory differ):

<!-- doctor-example: mock -->
```text
PASS           host.platform: Linux ubuntu 22.04 x86_64; container=False; wsl=False
PASS           host.python: /home/robot/real-world-rl/.venv/bin/python
SKIPPED        station: no station configured; arm handshake not performed
    hint: run rwrl setup when ready; software/USB success is not robot acceptance
Report: /home/robot/real-world-rl/local/doctor/20261112T101500-3f2a9c1d
Result: no blocking findings; review SKIPPED/WARN coverage
```

Each line is `STATUS id: message`; a line that is not `PASS` is followed by its `hint:`.
`host.python` must be the `.venv` interpreter: a `WARN` there means the project environment is
not active. `station` is `SKIPPED` because there is no `local/station.yaml` yet; page 02 writes
it. `Report:` is the directory holding this run's `report.json`. The last line is green when
nothing failed. The status words are explained in [00](00-overview.md#status-words).

Without `--mock`, the doctor also inspects this host: diagnostic tools, USB devices, serial
and input permissions, realtime limits and the Python dependencies of each extra. On the NUC,
repository root, project environment active:

```bash
rwrl doctor
```

The rows depend on the machine. Before page 02, expect `station` and the `coverage.*` rows to
be `SKIPPED`; nothing is hardware-validated yet.

## If it fails

- **Installer stage error:** the installer prints `ERROR: <stage> failed` and the exact command
  to retry from the repository root. Fix the cause it names and rerun the installer.
- **`rl-garden/` is empty** (a checkout made without `--recurse-submodules`): on the NUC,
  repository root, no environment needed:

  <!-- tutorial-test: skip -->
  ```bash
  git submodule update --init
  ```

  then rerun the installer. `rl_garden==0.0.1` is installed from this submodule, not from a
  public index.
- **`Unknown env backend 'real_robot'`:** this package is not installed in the interpreter that
  runs the command; activate the project environment.
- **`franky` import errors:** the package is `franky-control` (it brings libfranka; no ROS);
  the name is not `franka`.
- **A red doctor row:** read its `hint:` line, then the matching section of
  [rwrl doctor: red lights](../doctor.md#red-lights-what-to-do), starting with
  [Python environment and extras](../doctor.md#python-environment-and-extras).

Next: [02 Set up the station](02-setup.md).

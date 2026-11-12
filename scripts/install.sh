#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'HELP'
Usage: bash scripts/install.sh [options]
Run from the repository root. Installs all three local projects into a venv.

  --extras LIST      Comma-separated root extras (default: franka-gello).
                     The test extra is always included; an empty LIST selects only test.
  --venv PATH        Create or reuse this venv (default: .venv).
  --python PROGRAM   Interpreter for a new venv (default: python3.10).
  --torch-index URL  Install torch from this index first (default: pip's index).
                     CPU example: https://download.pytorch.org/whl/cpu
  --station YAML     Run the existing doctor against this station (default: mock).
  --no-doctor        Skip the final doctor call (default: run doctor).
  -h, --help         Show this help and exit.

No station: doctor --mock checks software only, against a throwaway franka station
whose arm and gripper are fake, then prints the hardware next step.
With a station: doctor checks dependencies and hardware; no duplicate preflight here.
This script never uses sudo or updates the git submodule automatically.
HELP
}

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

require_value() {
  if [[ $# -lt 2 || $2 == --* || ( $1 != --extras && -z $2 ) ]]; then
    fail "argument parsing: $1 needs a value; retry with --help for usage."
  fi
}

run_step() {
  local label=$1
  shift
  printf '\n==> %s\n' "$label"
  if "$@"; then
    return 0
  else
    local result=$?
    printf 'ERROR: %s failed (exit %s).\nRetry from the repository root:\n  ' "$label" "$result" >&2
    printf '%q ' "$@" >&2
    printf '\n' >&2
    exit "$result"
  fi
}

extras=franka-gello
venv_dir=.venv
python_bin=python3.10
torch_index=
station=
run_doctor=true

while [[ $# -gt 0 ]]; do
  case $1 in
    --extras) require_value "$@"; extras=$2; shift 2 ;;
    --venv) require_value "$@"; venv_dir=$2; shift 2 ;;
    --python) require_value "$@"; python_bin=$2; shift 2 ;;
    --torch-index) require_value "$@"; torch_index=$2; shift 2 ;;
    --station) require_value "$@"; station=$2; shift 2 ;;
    --no-doctor) run_doctor=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "argument parsing: unknown option '$1'; retry with --help." ;;
  esac
done

[[ -n $venv_dir && -n $python_bin ]] || fail 'argument parsing: --venv and --python must not be empty; retry with valid values.'
if [[ -n $extras && ! $extras =~ ^[a-zA-Z0-9_-]+(,[a-zA-Z0-9_-]+)*$ ]]; then
  fail 'argument parsing: --extras must be a comma-separated list such as franka-gello,spacemouse; retry without spaces.'
fi
[[ -f pyproject.toml && -d robot_infra ]] || fail 'checkout check: run bash scripts/install.sh from the real-world-rl repository root.'
[[ -f rl-garden/pyproject.toml ]] || fail 'checkout check: rl-garden is not initialized; run git submodule update --init from the repository root, then retry.'

unset PYTHONPATH
# Absolute paths keep retries unambiguous, including venv paths containing spaces.
if [[ $venv_dir != /* ]]; then
  venv_dir="$PWD/$venv_dir"
fi
venv_python="$venv_dir/bin/python"
if [[ -e $venv_dir ]]; then
  [[ -f $venv_dir/pyvenv.cfg && -x $venv_python ]] || fail "venv check: '$venv_dir' is not a usable venv; retry with --venv pointing to a new directory."
  printf 'Reusing venv: %s\n' "$venv_dir"
else
  run_step 'Create venv (interpreter needs venv support)' "$python_bin" -m venv "$venv_dir"
fi

run_step 'Upgrade pip' "$venv_python" -m pip install -U pip
if [[ -n $torch_index ]]; then
  run_step 'Install torch from the requested index' "$venv_python" -m pip install torch --index-url "$torch_index"
fi
install_extras="${extras:+$extras,}test"
run_step 'Install local projects and extras' "$venv_python" -m pip install \
  -e ./rl-garden -e ./robot_infra -e ".[${install_extras}]"
run_step 'Import installed projects' "$venv_python" -c \
  'import rl_garden, robot_infra, real_world_rl; print("Imports OK:", rl_garden.__file__, robot_infra.__file__, real_world_rl.__file__)'

if $run_doctor; then
  if [[ -n $station ]]; then
    run_step 'Doctor (station dependencies and hardware)' "$venv_python" -m robot_infra.doctor --config "$station"
  else
    printf 'No --station supplied: mock doctor does not verify hardware.\n'
    # The device doctor refuses to run without a station file, and none exists before the
    # tutorial's station step; --mock swaps every backend for its fake, so this one is enough.
    mock_station=$(mktemp "${TMPDIR:-/tmp}/rwrl-mock-station.XXXXXX")
    trap 'rm -f "$mock_station"' EXIT
    printf 'type: franka\narm_backend: fake\ngripper:\n  backend: fake\n' > "$mock_station"
    run_step 'Doctor (mock)' "$venv_python" -m robot_infra.doctor --mock --station "$mock_station"
    rm -f "$mock_station"
    printf 'Next, configure local/station.yaml, then run from this repository root:\n  '
    printf '%q doctor --station local/station.yaml\n' "$venv_dir/bin/rwrl"
  fi
fi
printf '\nInstallation complete. Activate this venv:\n  source %q\n' "$venv_dir/bin/activate"

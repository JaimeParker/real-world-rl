#!/usr/bin/env bash
# Build and install, on the robot PC itself (plain Ubuntu, no Docker), the joint_controller
# package that `joint_controller: trajectory_impedance` needs, into the ACTIVE Python environment,
# against the franky that is installed there:
#
#   franky     v2.0.0, unmodified, pip-installed from source against a system libfranka. If the
#              environment does not have that franky yet (or has the PyPI wheel), it is built first
#              by native/serl_controller/scripts/install-on-ubuntu.sh -- the one franky build path
#              of this repository -- which also (re)builds serl_controller against it, so every
#              native package in the environment belongs to the same franky build;
#   joint_controller  this package, pip-installed against that franky (scikit-build-core);
#
# then runs the C++ tests (incl. the franky motion test), the Python binding tests, a smoke
# import and the pairing check (build_info.json vs the installed libfranky.so).
#
# Usage, from the repository root, with the project venv / conda env activated:
#
#   LIBFRANKA_VERSION=0.17.0 bash native/joint_controller/scripts/install-on-ubuntu.sh
#
# Environment (all optional except LIBFRANKA_VERSION):
#   WORK=~/.cache/rwrl-native-build   sources, build trees and logs ($WORK/logs); the same
#                                     default as serl_controller's script (shared franky checkout)
#   REBUILD_FRANKY=1                  always run the franky (+ serl_controller) build first
#   SKIP_TESTS=1                      skip the C++ and Python tests (not recommended)
#   PYTHON=python                     the interpreter of the target environment
#   Everything else (SKIP_APT, SKIP_DEPS, REPLACE_LIBFRANKA, RETRIES, PIP_INDEX_URL, ...) is passed
#   through to serl_controller's script when it runs.
set -euo pipefail

JC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${JC_DIR}/../.." && pwd)"
SERL_SCRIPT="${REPO_ROOT}/native/serl_controller/scripts/install-on-ubuntu.sh"
export WORK="${WORK:-${HOME}/.cache/rwrl-native-build}"
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-}"
FRANKY_TAG="${FRANKY_TAG:-v2.0.0}"
FRANKY_COMMIT="${FRANKY_COMMIT:-e7e63c7c7118f427ad33211500b33b81b4011bb4}"
PYBIND11_VERSION_PINNED="3.0.4"   # franky v2.0.0's (docker/common/dependency_versions)
PY="${PYTHON:-python}"
command -v "${PY}" >/dev/null 2>&1 || PY=python3

log() { printf '\n== %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

[ -n "${LIBFRANKA_VERSION}" ] || die "set LIBFRANKA_VERSION (e.g. LIBFRANKA_VERSION=0.17.0); pick it from \
the robot server version with Franka's compatibility matrix, see https://real-world-rl.readthedocs.io/zh-cn/latest/en/reference/serl-controller.html"
[[ "${LIBFRANKA_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || die "LIBFRANKA_VERSION must look like 0.17.0"
"${PY}" - <<'PY' || die "activate the project venv / conda env first (never build into the system Python)"
import os, sys
in_env = sys.prefix != sys.base_prefix or bool(os.environ.get("CONDA_PREFIX"))
sys.exit(0 if in_env else 1)
PY
mkdir -p "${WORK}/build" "${WORK}/logs"
BUILD_ID="rwrl-native-$(date -u +%Y%m%dT%H%M%SZ)"
FRANKY_SRC="${WORK}/src/franky"

# -- 1. franky: reuse the source build in this environment, or build it (serl_controller's script) --
franky_reusable() {
  # franky_control 2.0.0+libfranka.<LIBFRANKA_VERSION> built from source (no bundled
  # franky_control.libs), its checkout at the pinned commit and unmodified.
  [ -d "${FRANKY_SRC}/.git" ] || return 1
  [ "$(git -C "${FRANKY_SRC}" rev-parse HEAD 2>/dev/null)" = "${FRANKY_COMMIT}" ] || return 1
  [ -z "$(git -C "${FRANKY_SRC}" status --porcelain)" ] || return 1
  "${PY}" - "${LIBFRANKA_VERSION}" <<'PY'
import importlib.metadata as md, sys, sysconfig
from pathlib import Path
try:
    version = md.version("franky-control")
except md.PackageNotFoundError:
    sys.exit(1)
site = Path(sysconfig.get_paths()["platlib"])
ok = (version == "2.0.0+libfranka." + sys.argv[1] and (site / "franky" / "libfranky.so").is_file()
      and not (site / "franky_control.libs").exists())
sys.exit(0 if ok else 1)
PY
}
if [ "${REBUILD_FRANKY:-0}" = 1 ] || ! franky_reusable; then
  log "1. franky ${FRANKY_TAG} (+ serl_controller) via ${SERL_SCRIPT#"${REPO_ROOT}"/}"
  [ -f "${SERL_SCRIPT}" ] || die "${SERL_SCRIPT} not found"
  bash "${SERL_SCRIPT}"
  franky_reusable || die "franky is still not a source build of ${FRANKY_TAG} against libfranka ${LIBFRANKA_VERSION}"
else
  log "1. franky: reusing the source build in this environment"
fi
SITE="$("${PY}" -c 'import sysconfig; print(sysconfig.get_paths()["platlib"])')"
FRANKY_LIB="${SITE}/franky/libfranky.so"
FRANKY_DIST_VERSION="$("${PY}" -c 'import importlib.metadata as m; print(m.version("franky-control"))')"
echo "franky_control ${FRANKY_DIST_VERSION}: ${FRANKY_LIB}"
VENV_BIN="$(dirname "$("${PY}" -c 'import sys; print(sys.executable)')")"
export PATH="${VENV_BIN}:${PATH}"
hash -r
"${PY}" -m pip install -q "cmake>=3.24,<4" "scikit-build-core>=0.10" "pybind11==${PYBIND11_VERSION_PINNED}" numpy pytest

# -- 2. joint_controller against that franky -----------------------------------------------------
log "2. joint_controller (scikit-build-core, no build isolation)"
"${PY}" -m pip uninstall -y joint-controller >/dev/null 2>&1 || true
JC_DEFINES=(
  -C cmake.define.JOINT_CONTROLLER_WITH_FRANKY=ON
  -C cmake.define.FRANKY_INCLUDE_DIR="${FRANKY_SRC}/include"
  -C cmake.define.FRANKY_LIBRARY="${FRANKY_LIB}"
  -C cmake.define.JOINT_CONTROLLER_FRANKY_DIST_VERSION="${FRANKY_DIST_VERSION}"
  -C cmake.define.JOINT_CONTROLLER_FRANKY_COMMIT="${FRANKY_COMMIT}"
  -C cmake.define.JOINT_CONTROLLER_BUILD_ID="${BUILD_ID}"
)
rm -rf "${WORK}/build/jc-skbuild"
"${PY}" -m pip install -v --no-build-isolation --no-deps "${JC_DIR}" "${JC_DEFINES[@]}" \
  -C build-dir="${WORK}/build/jc-skbuild" > "${WORK}/logs/jc_build.log" 2>&1 \
  || { tail -80 "${WORK}/logs/jc_build.log"; die "joint_controller build failed"; }
grep -E "joint_controller: |pybind11 [0-9]" "${WORK}/logs/jc_build.log" | sort -u || true
grep -q "pybind11 ${PYBIND11_VERSION_PINNED}" "${WORK}/logs/jc_build.log" \
  || die "joint_controller was not built with pybind11 ${PYBIND11_VERSION_PINNED} (franky's); see ${WORK}/logs/jc_build.log"
EXT="$(ls "${SITE}"/joint_controller/_joint_controller*.so)"
readelf -d "${EXT}" | grep -E "NEEDED|RPATH|RUNPATH" | tee "${WORK}/logs/jc_ext_dynamic.txt"

# -- 3. tests -------------------------------------------------------------------------------------
if [ "${SKIP_TESTS:-0}" != 1 ]; then
  log "3a. C++ tests (planner, law, params, soak, franky motion), same compiler and franky"
  T="${WORK}/build/jc-tests"
  rm -rf "${T}"
  cmake -S "${JC_DIR}" -B "${T}" -DCMAKE_BUILD_TYPE=Release \
    -DJOINT_CONTROLLER_WITH_FRANKY=ON -DJOINT_CONTROLLER_BUILD_PYTHON=OFF -DJOINT_CONTROLLER_BUILD_TESTS=ON \
    -DFRANKY_INCLUDE_DIR="${FRANKY_SRC}/include" -DFRANKY_LIBRARY="${FRANKY_LIB}" \
    > "${WORK}/logs/jc_ctest_configure.log" 2>&1 || { tail -40 "${WORK}/logs/jc_ctest_configure.log"; die "test configure failed"; }
  cmake --build "${T}" -j "$(nproc)" > "${WORK}/logs/jc_ctest_build.log" 2>&1 \
    || { tail -60 "${WORK}/logs/jc_ctest_build.log"; die "test build failed"; }
  ctest --test-dir "${T}" --output-on-failure 2>&1 | tee "${WORK}/logs/jc_ctest.log"
  test "${PIPESTATUS[0]}" -eq 0

  log "3b. Python binding tests"
  (cd "${JC_DIR}" && "${PY}" -m pytest -q -p no:cacheprovider tests/python) 2>&1 | tee "${WORK}/logs/jc_pytest.log"
  test "${PIPESTATUS[0]}" -eq 0
fi

# -- 4. smoke import and pairing check ------------------------------------------------------------
log "4. smoke import and pairing check"
(cd /tmp && "${PY}" - <<'PY'
import hashlib, sysconfig
from pathlib import Path
import franky, joint_controller as jc
m = jc.TrajectoryImpedanceMotion(jc.default_params("panda"))
assert isinstance(m, franky.BaseTorqueMotion), type(m).__mro__
info = jc.build_info()
lib = Path(sysconfig.get_paths()["platlib"]) / "franky" / "libfranky.so"
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
assert info["libfranky_sha256"] == sha, ("joint_controller was built against another libfranky.so", info, sha)
print("joint_controller", jc.__version__, "derives from franky.BaseTorqueMotion and matches libfranky.so: ok")
print("build_info:", info)
PY
)

{
  echo "built: $(date -u +%Y-%m-%dT%H:%M:%SZ) (${BUILD_ID})"
  echo "python: $("${PY}" -c 'import sys; print(sys.version.split()[0], sys.executable)')"
  echo "compiler: $(c++ --version | head -1)"
  echo "franky: ${FRANKY_TAG} ${FRANKY_COMMIT} (unmodified, from source), installed as franky_control ${FRANKY_DIST_VERSION}"
  echo "joint_controller: $("${PY}" -c 'import importlib.metadata as m; print(m.version("joint-controller"))')"
} | tee "${WORK}/BUILD_INFO-joint_controller.txt"
echo "done. Next: set joint_controller: trajectory_impedance on the station arm and run rwrl doctor."

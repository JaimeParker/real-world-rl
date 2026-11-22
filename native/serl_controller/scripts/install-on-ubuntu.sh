#!/usr/bin/env bash
# Build and install, on the robot PC itself (plain Ubuntu, no Docker), the pair that
# `cartesian_controller: serl` needs, into the ACTIVE Python environment:
#
#   libfranka  LIBFRANKA_VERSION, from source (franky's own, unmodified dependency scripts),
#              or the one already installed if it has that version;
#   franky     v2.0.0, unmodified, pip-installed from source against that libfranka;
#   serl_controller  this package, pip-installed against that franky (scikit-build-core).
#
# then runs the C++ tests (incl. the golden SERL comparisons), the Python binding tests, a
# smoke import and the `rwrl doctor` pairing check (sdk:serl_controller).
#
# Usage, from the repository root, with the project venv / conda env activated:
#
#   LIBFRANKA_VERSION=0.17.0 bash native/serl_controller/scripts/install-on-ubuntu.sh
#
# LIBFRANKA_VERSION comes from the robot's server version (Franka's compatibility matrix,
# https://frankarobotics.github.io/docs/compatibility.html); see https://real-world-rl.readthedocs.io/zh-cn/latest/en/reference/serl-controller.html.
#
# Environment (all optional except LIBFRANKA_VERSION):
#   WORK=~/.cache/rwrl-native-build   sources, build trees and logs ($WORK/logs)
#   SKIP_APT=1                        do not apt-get install the build packages
#   SKIP_DEPS=1                       do not build eigen / pybind11 / ruckig / libfranka
#                                     (they must already be installed, libfranka at LIBFRANKA_VERSION)
#   REPLACE_LIBFRANKA=1               allow replacing a different libfranka found under /usr/local
#   SKIP_TESTS=1                      skip the C++ and Python tests (not recommended)
#   RETRIES=3                         attempts per network-dependent step (git clones, pip)
#   PYTHON=python                     the interpreter of the target environment
#   PIP_INDEX_URL, PIP_DEFAULT_TIMEOUT, ...   honoured by every pip call (e.g. a PyPI mirror)
set -euo pipefail

SERL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${SERL_DIR}/../.." && pwd)"
WORK="${WORK:-${HOME}/.cache/rwrl-native-build}"
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-}"
FRANKY_REPO="${FRANKY_REPO:-https://github.com/TimSchneider42/franky.git}"
FRANKY_TAG="${FRANKY_TAG:-v2.0.0}"
FRANKY_COMMIT="${FRANKY_COMMIT:-e7e63c7c7118f427ad33211500b33b81b4011bb4}"
PYBIND11_VERSION_PINNED="3.0.4"   # franky v2.0.0's (docker/common/dependency_versions)
RETRIES="${RETRIES:-3}"
PY="${PYTHON:-python}"
command -v "${PY}" >/dev/null 2>&1 || PY=python3

log() { printf '\n== %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
retry() {
  local attempt=1
  until "$@"; do
    if [ "${attempt}" -ge "${RETRIES}" ]; then
      echo "failed after ${attempt} attempts: $*" >&2
      return 1
    fi
    echo "attempt ${attempt}/${RETRIES} failed (network?), retrying: $*" >&2
    attempt=$((attempt + 1))
    sleep $((10 * attempt))
  done
}

clone() {  # clone <url> <dir>: a failed attempt leaves no partial checkout behind
  rm -rf "$2"
  git clone --quiet "$1" "$2"
}
# git over HTTP/1.1 for this script and franky's dependency scripts only (no global git config
# change): flaky links to GitHub often fail HTTP/2 with "Error in the HTTP2 framing layer".
export GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=http.version GIT_CONFIG_VALUE_0=HTTP/1.1

# -- 0. checks -------------------------------------------------------------------------------
[ -n "${LIBFRANKA_VERSION}" ] || die "set LIBFRANKA_VERSION (e.g. LIBFRANKA_VERSION=0.17.0); pick it from \
the robot server version with Franka's compatibility matrix, see https://real-world-rl.readthedocs.io/zh-cn/latest/en/reference/serl-controller.html"
[[ "${LIBFRANKA_VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || die "LIBFRANKA_VERSION must look like 0.17.0"
[ -r /etc/os-release ] && . /etc/os-release
[ "${ID:-}" = "ubuntu" ] || echo "warning: tested on Ubuntu 20.04/22.04/24.04, this is ${PRETTY_NAME:-unknown}" >&2
"${PY}" - <<'PY' || die "activate the project venv / conda env first (never build into the system Python)"
import os, sys
in_env = sys.prefix != sys.base_prefix or bool(os.environ.get("CONDA_PREFIX"))
sys.exit(0 if in_env else 1)
PY
PY_VERSION="$("${PY}" -c 'import sys; print("%d.%d.%d" % sys.version_info[:3])')"
if [ "$(id -u)" -eq 0 ]; then SUDO=(); else SUDO=(sudo); fi
mkdir -p "${WORK}/src" "${WORK}/build" "${WORK}/logs"
BUILD_ID="rwrl-native-$(date -u +%Y%m%dT%H%M%SZ)"
echo "python ${PY_VERSION} ($("${PY}" -c 'import sys; print(sys.executable)')), $(uname -m), ${PRETTY_NAME:-?}"
echo "libfranka ${LIBFRANKA_VERSION}, franky ${FRANKY_TAG} (${FRANKY_COMMIT:0:7}), work dir ${WORK}, build id ${BUILD_ID}"

# -- 1. build packages -----------------------------------------------------------------------
# The list of franky's own Ubuntu image (franky/docker/run/Dockerfile) plus venv/pip.
APT_PACKAGES=(build-essential cmake git ca-certificates curl pkg-config python3-dev python3-venv
              python3-pip libboost-all-dev libfmt-dev libssl-dev libmysqlclient-dev libpq-dev)
if [ "${SKIP_APT:-0}" != 1 ]; then
  log "1. apt packages"
  "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive apt-get update
  "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${APT_PACKAGES[@]}"
fi

# -- 2. Python build tools in the target environment ----------------------------------------
log "2. build tools in the active environment"
# CMake >= 3.24 for serl_controller (Ubuntu 22.04 ships 3.22); < 4 so that older dependencies
# with cmake_minimum_required < 3.5 still configure.
retry "${PY}" -m pip install --upgrade "pip" "setuptools" "wheel" "cmake>=3.24,<4" \
  "scikit-build-core>=0.10" "pybind11==${PYBIND11_VERSION_PINNED}" numpy pytest
VENV_BIN="$(dirname "$("${PY}" -c 'import sys; print(sys.executable)')")"
export PATH="${VENV_BIN}:${PATH}"
hash -r
cmake --version | head -1

# -- 3. franky source, unmodified -------------------------------------------------------------
log "3. franky ${FRANKY_TAG}"
FRANKY_SRC="${WORK}/src/franky"
if [ ! -d "${FRANKY_SRC}/.git" ]; then
  retry clone "${FRANKY_REPO}" "${FRANKY_SRC}"
fi
retry git -C "${FRANKY_SRC}" fetch --quiet --tags origin
git -C "${FRANKY_SRC}" checkout --quiet --detach "${FRANKY_TAG}"
actual="$(git -C "${FRANKY_SRC}" rev-parse HEAD)"
[ "${actual}" = "${FRANKY_COMMIT}" ] || die "franky ${FRANKY_TAG} is ${actual}, expected ${FRANKY_COMMIT}"
[ -z "$(git -C "${FRANKY_SRC}" status --porcelain)" ] || die "franky checkout ${FRANKY_SRC} is modified; refusing to build"

# -- 4. C++ dependencies (franky's own scripts) ------------------------------------------------
installed_libfranka_version() {
  local prefix file
  for prefix in ${CMAKE_PREFIX_PATH:+${CMAKE_PREFIX_PATH//:/ }} /usr/local /usr /opt/openrobots; do
    for file in "${prefix}"/lib/cmake/Franka/FrankaConfigVersion.cmake \
                "${prefix}"/lib/*/cmake/Franka/FrankaConfigVersion.cmake \
                "${prefix}"/share/Franka/cmake/FrankaConfigVersion.cmake; do
      if [ -f "${file}" ]; then
        sed -n 's/^ *set(PACKAGE_VERSION "\([0-9.]*\)").*/\1/p' "${file}" | head -1
        return 0
      fi
    done
  done
}
# Version of an installed CMake package under /usr/local (its <Name>ConfigVersion.cmake), or empty.
installed_cmake_version() {
  local file
  file="$(find /usr/local/lib /usr/local/share -maxdepth 4 -iname "$1" 2>/dev/null | head -1)"
  [ -n "${file}" ] && sed -n 's/^ *set(PACKAGE_VERSION "\([0-9.]*\)").*/\1/p' "${file}" | head -1
  return 0
}
# install_dep <name> <ConfigVersion file name> <wanted version>: franky's install-<name>, retried,
# skipped when that version is already installed (so a rerun after a network failure is quick).
install_dep() {
  local have
  have="$(installed_cmake_version "$2")"
  if [ -n "$3" ] && [ "${have}" = "$3" ]; then
    echo "$1 $3: already installed"
    return 0
  fi
  retry bash "${RUN_COMMON}/install-$1" >> "${WORK}/logs/install-$1.log" 2>&1 \
    || { tail -40 "${WORK}/logs/install-$1.log"; die "install-$1 failed (log: ${WORK}/logs/install-$1.log)"; }
  echo "$1: ok"
}
if [ "${SKIP_DEPS:-0}" != 1 ]; then
  log "4. eigen, pybind11, ruckig, poco, pinocchio, libfranka (franky/docker/common, unmodified)"
  COMMON="${FRANKY_SRC}/docker/common"
  set -a
  # shellcheck disable=SC1091
  . "${COMMON}/dependency_versions"
  set +a
  export LIBFRANKA_VERSION
  # Run franky's scripts from a copy: install-libfranka itself starts by (re)building poco and,
  # from 0.14, pinocchio; the copy has those two calls replaced by ':' because they run as their own
  # (retried, skippable) steps below. Nothing else changes; the checkout stays pristine.
  RUN_COMMON="${WORK}/build/franky-docker-common"
  rm -rf "${RUN_COMMON}"
  cp -a "${COMMON}" "${RUN_COMMON}"
  sed -i -E 's#^( *)"\$\{SCRIPT_DIR\}/install-(poco|pinocchio)"$#\1: \# install-\2: run separately by install-on-ubuntu.sh#' \
    "${RUN_COMMON}/install-libfranka"
  [ "$(diff "${COMMON}/install-libfranka" "${RUN_COMMON}/install-libfranka" | grep -c '^>')" -eq 2 ] \
    || die "franky's install-libfranka changed shape; cannot split out poco/pinocchio"
  install_dep eigen 'Eigen3ConfigVersion.cmake' "${EIGEN_VERSION}"
  install_dep pybind11 'pybind11ConfigVersion.cmake' "${PYBIND11_VERSION}"
  install_dep ruckig 'ruckig-config-version.cmake' "${RUCKIG_VERSION}"
  have="$(installed_libfranka_version || true)"
  if [ "${have}" = "${LIBFRANKA_VERSION}" ]; then
    echo "libfranka ${have} already installed; not rebuilding it"
  else
    if [ -n "${have}" ] && [ "${REPLACE_LIBFRANKA:-0}" != 1 ]; then
      die "libfranka ${have} is installed; installing ${LIBFRANKA_VERSION} replaces it for every program \
on this PC. Rerun with REPLACE_LIBFRANKA=1 if that is intended"
    fi
    poco_version="${POCO_VERSION#poco-}"
    install_dep poco 'PocoConfigVersion.cmake' "${poco_version%%[a-z]*}"
    if [ "$(printf '%s\n%s\n' 0.14.0 "${LIBFRANKA_VERSION}" | sort -V | head -1)" = 0.14.0 ]; then
      echo "pinocchio ${PINOCCHIO_VERSION} (libfranka >= 0.14 needs it; the longest build step)"
      install_dep pinocchio 'pinocchio-config-version.cmake' "${PINOCCHIO_VERSION#v}"
    fi
    echo "building libfranka ${LIBFRANKA_VERSION}"
    retry bash "${RUN_COMMON}/install-libfranka" >> "${WORK}/logs/install-libfranka.log" 2>&1 \
      || { tail -60 "${WORK}/logs/install-libfranka.log"; die "install-libfranka failed"; }
  fi
fi
have="$(installed_libfranka_version || true)"
[ "${have}" = "${LIBFRANKA_VERSION}" ] || die "libfranka ${LIBFRANKA_VERSION} not found (found: ${have:-none})"
echo "libfranka: ${have}"

# -- 5. franky from source into the environment ------------------------------------------------
log "5. franky ${FRANKY_TAG} from source (pip, against libfranka ${LIBFRANKA_VERSION})"
"${PY}" -m pip uninstall -y franky-control serl-controller >/dev/null 2>&1 || true
rm -rf "${WORK}/build/franky"
cp -a "${FRANKY_SRC}" "${WORK}/build/franky"   # pip builds in-tree; keep the checkout pristine
# The version label makes the installed version 2.0.0+libfranka.X, like franky's CI wheels.
retry env FRANKY_LIBFRANKA_VERSION_LABEL="${LIBFRANKA_VERSION}" \
  "${PY}" -m pip install -v --no-build-isolation "${WORK}/build/franky" > "${WORK}/logs/franky_build.log" 2>&1 \
  || { tail -60 "${WORK}/logs/franky_build.log"; die "franky build failed"; }
SITE="$("${PY}" -c 'import sysconfig; print(sysconfig.get_paths()["platlib"])')"
FRANKY_LIB="${SITE}/franky/libfranky.so"
[ -f "${FRANKY_LIB}" ] || die "franky installed without ${FRANKY_LIB}"
FRANKY_DIST_VERSION="$("${PY}" -c 'import importlib.metadata as m; print(m.version("franky-control"))')"
echo "franky_control ${FRANKY_DIST_VERSION}: ${FRANKY_LIB}"
(cd /tmp && "${PY}" -c 'import franky; print("import franky: ok")')

# -- 6. serl_controller against that franky ----------------------------------------------------
log "6. serl_controller (scikit-build-core, no build isolation)"
SERL_DEFINES=(
  -C cmake.define.SERL_CONTROLLER_WITH_FRANKY=ON
  -C cmake.define.FRANKY_INCLUDE_DIR="${FRANKY_SRC}/include"
  -C cmake.define.FRANKY_LIBRARY="${FRANKY_LIB}"
  -C cmake.define.SERL_CONTROLLER_FRANKY_DIST_VERSION="${FRANKY_DIST_VERSION}"
  -C cmake.define.SERL_CONTROLLER_FRANKY_COMMIT="${FRANKY_COMMIT}"
  -C cmake.define.SERL_CONTROLLER_BUILD_ID="${BUILD_ID}"
)
rm -rf "${WORK}/build/serl-skbuild"
"${PY}" -m pip install -v --no-build-isolation --no-deps "${SERL_DIR}" "${SERL_DEFINES[@]}" \
  -C build-dir="${WORK}/build/serl-skbuild" > "${WORK}/logs/serl_build.log" 2>&1 \
  || { tail -80 "${WORK}/logs/serl_build.log"; die "serl_controller build failed"; }
grep -E "serl_controller: |pybind11 [0-9]" "${WORK}/logs/serl_build.log" | sort -u || true
grep -q "pybind11 ${PYBIND11_VERSION_PINNED}" "${WORK}/logs/serl_build.log" \
  || die "serl_controller was not built with pybind11 ${PYBIND11_VERSION_PINNED} (franky's); see ${WORK}/logs/serl_build.log"
EXT="$(ls "${SITE}"/serl_controller/_serl_controller*.so)"
readelf -d "${EXT}" | grep -E "NEEDED|RPATH|RUNPATH" | tee "${WORK}/logs/serl_ext_dynamic.txt"

# -- 7. tests -----------------------------------------------------------------------------------
if [ "${SKIP_TESTS:-0}" != 1 ]; then
  log "7a. C++ tests (golden SERL comparisons + franky motions), same compiler and franky"
  T="${WORK}/build/serl-tests"
  rm -rf "${T}"
  retry cmake -S "${SERL_DIR}" -B "${T}" -DCMAKE_BUILD_TYPE=Release \
    -DSERL_CONTROLLER_WITH_FRANKY=ON -DSERL_CONTROLLER_BUILD_PYTHON=OFF -DSERL_CONTROLLER_BUILD_TESTS=ON \
    -DFRANKY_INCLUDE_DIR="${FRANKY_SRC}/include" -DFRANKY_LIBRARY="${FRANKY_LIB}" \
    > "${WORK}/logs/ctest_configure.log" 2>&1 || { tail -40 "${WORK}/logs/ctest_configure.log"; die "test configure failed"; }
  cmake --build "${T}" -j "$(nproc)" > "${WORK}/logs/ctest_build.log" 2>&1 \
    || { tail -60 "${WORK}/logs/ctest_build.log"; die "test build failed"; }
  ctest --test-dir "${T}" --output-on-failure 2>&1 | tee "${WORK}/logs/ctest.log"
  test "${PIPESTATUS[0]}" -eq 0
  "${T}/tests/cpp/test_golden_equivalence" > "${WORK}/logs/golden_equivalence.txt"
  "${T}/tests/cpp/test_joint_reset_golden" > "${WORK}/logs/joint_reset_golden.txt"
  grep -hE "max (abs|rel)|scenario|ticks" "${WORK}/logs/golden_equivalence.txt" "${WORK}/logs/joint_reset_golden.txt" | tail -12 || true

  log "7b. Python binding tests"
  (cd "${SERL_DIR}" && "${PY}" -m pytest -q -p no:cacheprovider tests/python) 2>&1 | tee "${WORK}/logs/pytest.log"
  test "${PIPESTATUS[0]}" -eq 0
fi

# -- 8. smoke import and the doctor's pairing check --------------------------------------------
log "8. smoke import and pairing check"
(cd /tmp && "${PY}" - <<'PY'
import franky, serl_controller as sc
m = sc.SerlCartesianImpedanceMotion()
assert isinstance(m, franky.BaseTorqueMotion), type(m).__mro__
j = sc.SerlJointPositionMotion([0, 0, 0, -1.9, 0, 2, 0])
assert isinstance(j, franky.BaseJointPositionMotion), type(j).__mro__
print("serl_controller", sc.__version__, "derives from franky's motions: ok")
print("build_info:", sc.build_info())
PY
)
# The same check as `rwrl doctor` (sdk:serl_controller), without a station or a robot.
PAIRING_PY='
import sys
try:
    from real_world_rl.devices.backends.arms.franky import serl_build_pairing
except ImportError:
    sys.path.insert(0, sys.argv[1])
    from real_world_rl.devices.backends.arms.franky import serl_build_pairing
ok, message, hint, _ = serl_build_pairing()
print(("PASS" if ok else "FAIL") + "  sdk:serl_controller: " + message)
if not ok:
    print("hint: " + hint)
sys.exit(0 if ok else 1)
'
(cd /tmp && "${PY}" -c "${PAIRING_PY}" "${REPO_ROOT}")

{
  echo "built: $(date -u +%Y-%m-%dT%H:%M:%SZ) (${BUILD_ID})"
  echo "os: ${PRETTY_NAME:-?} $(uname -r) $(uname -m)"
  echo "python: ${PY_VERSION} ($("${PY}" -c 'import sys; print(sys.executable)'))"
  echo "compiler: $(c++ --version | head -1)"
  echo "libfranka: ${LIBFRANKA_VERSION}"
  echo "franky: ${FRANKY_TAG} ${FRANKY_COMMIT} (unmodified, from source), installed as franky_control ${FRANKY_DIST_VERSION}"
  echo "serl_controller: $("${PY}" -c 'import importlib.metadata as m; print(m.version("serl-controller"))')"
} | tee "${WORK}/BUILD_INFO.txt"
echo "done. Next: set cartesian_controller: serl in the station and run rwrl doctor (see https://real-world-rl.readthedocs.io/zh-cn/latest/en/reference/serl-controller.html)."

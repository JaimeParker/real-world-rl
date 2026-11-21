#!/usr/bin/env bash
# Runs INSIDE franky's build image (see build-on-host.sh) with a franky wheel from the same
# image: configures serl_controller with franky + Python + tests and a compile database, builds
# it (so every translation unit compiles with the real franky / libfranka / pybind11 headers),
# then runs tools/lint.sh (clang-tidy + clang-format, pinned versions from PyPI).
#
#   docker run --rm -v <franky src>:/src/franky:ro -v <serl_controller>:/src/serl_controller:ro \
#     -v <dir with franky_control-*.whl>:/wheels:ro --entrypoint /bin/bash <image> \
#     /src/serl_controller/docker/lint-in-image.sh
#
# PIP_INDEX_URL etc. are honoured. Exit status: that of tools/lint.sh.
set -euo pipefail

PYBIN="${PYBIN:-cp310-cp310}"
PY="/opt/python/${PYBIN}/bin/python"
CLANG_TOOLS_VERSION="${CLANG_TOOLS_VERSION:-18.1.8}"
PYBIND11_VERSION="${PYBIND11_VERSION:-3.0.4}"  # franky's pin (ABI coupling rule)
TOOLSET_ENABLE=$(ls /opt/rh/gcc-toolset-*/enable 2>/dev/null | sort -V | tail -1)
# shellcheck disable=SC1090
[ -n "${TOOLSET_ENABLE}" ] && source "${TOOLSET_ENABLE}"
export LD_LIBRARY_PATH="/usr/local/lib64:/usr/local/lib"

WORK=/work
mkdir -p "${WORK}"
cp -a /src/serl_controller "${WORK}/serl_controller"
rm -rf "${WORK}/serl_controller/build"

"${PY}" -m venv "${WORK}/venv"
"${WORK}/venv/bin/pip" install -q "clang-tidy==${CLANG_TOOLS_VERSION}" "clang-format==${CLANG_TOOLS_VERSION}" \
  "pybind11==${PYBIND11_VERSION}" /wheels/franky_control-*-"${PYBIN}"-*.whl
SITE=$("${WORK}/venv/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["platlib"])')
FRANKY_LIB="${SITE}/franky/libfranky.so"
FRANKA_LIB=$(ls "${SITE}"/franky_control.libs/libfranka-*.so*)

cd "${WORK}/serl_controller"
cmake -S . -B build/lint -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DSERL_CONTROLLER_WITH_FRANKY=ON -DSERL_CONTROLLER_BUILD_PYTHON=ON -DSERL_CONTROLLER_BUILD_TESTS=ON \
  -DPython_EXECUTABLE="${WORK}/venv/bin/python" -Dpybind11_DIR="$("${WORK}/venv/bin/python" -m pybind11 --cmakedir)" \
  -DFRANKY_INCLUDE_DIR=/src/franky/include -DFRANKY_LIBRARY="${FRANKY_LIB}" -DFRANKY_LINK_LIBRARIES="${FRANKA_LIB}"
cmake --build build/lint -j "$(nproc)"
# clang-tidy runs the clang frontend on gcc-toolset's libstdc++ (the compiler of the build).
export CLANG_TIDY="${WORK}/venv/bin/clang-tidy"
export CLANG_TIDY_EXTRA_ARGS="--extra-arg=--gcc-install-dir=$(dirname "$(g++ -print-libgcc-file-name)")"
export CLANG_FORMAT="${WORK}/venv/bin/clang-format"
tools/lint.sh build/lint

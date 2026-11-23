#!/usr/bin/env bash
# Runs INSIDE franky's build image (see build-on-host.sh). Builds, from the same toolchain,
# pybind11 and libfranka:
#   1. the unmodified franky wheel, with franky's own manylinux-build-wheels;
#   2. the joint_controller wheel, against that wheel's libfranky.so / libfranka;
#   3. (WITH_SERL=1, /src/serl_controller mounted) the serl_controller wheel, the same way;
# then runs the C++ (CTest) and Python (pytest) tests and copies the wheels to /out/wheels.
# Sources are mounted read-only and copied to /work first.
set -euo pipefail

PYBIN="${PYBIN:-cp310-cp310}"
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-0.17.0}"
WITH_SERL="${WITH_SERL:-1}"
[ -d /src/serl_controller ] || WITH_SERL=0
PY="/opt/python/${PYBIN}/bin/python"
WORK=/work
OUT=/out
mkdir -p "${WORK}" "${OUT}/wheels" "${OUT}/logs"
rm -f "${OUT}"/wheels/*.whl

# Same compiler as franky's manylinux-build-wheels (highest gcc-toolset).
TOOLSET_ENABLE=$(ls /opt/rh/gcc-toolset-*/enable 2>/dev/null | sort -V | tail -1)
# shellcheck disable=SC1090
[ -n "${TOOLSET_ENABLE}" ] && source "${TOOLSET_ENABLE}"
export LD_LIBRARY_PATH="/usr/local/lib64:/usr/local/lib"

cp -a /src/franky "${WORK}/franky"
cp -a /src/joint_controller "${WORK}/joint_controller"
rm -rf "${WORK}/joint_controller/build"
if [ "${WITH_SERL}" = 1 ]; then
  cp -a /src/serl_controller "${WORK}/serl_controller"
  rm -rf "${WORK}/serl_controller/build"
fi

echo "== 1. franky wheel (unmodified franky, franky's own build script)"
mkdir -p "${WORK}/franky-build"
(cd "${WORK}/franky-build" && \
  FRANKY_LIBFRANKA_VERSION_LABEL="${LIBFRANKA_VERSION}" PYBIN_VERSIONS="${PYBIN}" \
  manylinux-build-wheels "${WORK}/franky")
FRANKY_WHL=$(ls "${WORK}"/franky-build/dist/franky_control-*-"${PYBIN}"-*.whl)
echo "franky wheel: ${FRANKY_WHL}"

echo "== 2. build venv with the franky wheel installed"
"${PY}" -m venv "${WORK}/venv-build"
"${WORK}/venv-build/bin/pip" install -q --upgrade pip
"${WORK}/venv-build/bin/pip" install -q "${FRANKY_WHL}" numpy pytest
SITE=$("${WORK}/venv-build/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["platlib"])')
FRANKY_LIB="${SITE}/franky/libfranky.so"
FRANKA_LIB=$(ls "${SITE}"/franky_control.libs/libfranka-*.so*)
echo "libfranky: ${FRANKY_LIB}"; echo "libfranka: ${FRANKA_LIB}"
FRANKY_DIST_VERSION=$("${WORK}/venv-build/bin/python" -c 'import importlib.metadata as m; print(m.version("franky-control"))')
BUILD_ID="${BUILD_ID:-rwrl-franky-build-$(date -u +%Y%m%dT%H%M%SZ)}"
echo "franky dist version: ${FRANKY_DIST_VERSION}; build id: ${BUILD_ID}"

# build_wheel <package dir name> <CMake prefix (JOINT_CONTROLLER | SERL_CONTROLLER)>: scikit-build-core
# against the franky wheel, then auditwheel with nothing from franky bundled (libfranky.so and
# libfranka-<hash>.so come from the franky wheel next to it: RPATH $ORIGIN/../franky:$ORIGIN/../franky_control.libs).
build_wheel() {
  local name="$1" prefix="$2"
  "${WORK}/venv-build/bin/pip" wheel -v --no-deps "${WORK}/${name}" -w "${WORK}/${name}-raw" \
    -C cmake.define."${prefix}"_WITH_FRANKY=ON \
    -C cmake.define.FRANKY_INCLUDE_DIR="${WORK}/franky/include" \
    -C cmake.define.FRANKY_LIBRARY="${FRANKY_LIB}" \
    -C cmake.define.FRANKY_LINK_LIBRARIES="${FRANKA_LIB}" \
    -C cmake.define."${prefix}"_FRANKY_DIST_VERSION="${FRANKY_DIST_VERSION}" \
    -C cmake.define."${prefix}"_FRANKY_COMMIT="${FRANKY_COMMIT:-}" \
    -C cmake.define."${prefix}"_BUILD_ID="${BUILD_ID}" \
    -C build-dir="${WORK}/${name}-skbuild" > "${OUT}/logs/${name}_wheel_build.log" 2>&1 \
    || { tail -80 "${OUT}/logs/${name}_wheel_build.log"; exit 1; }
  grep -E "pybind11 |libfranka |${name}:" "${OUT}/logs/${name}_wheel_build.log" || true
  auditwheel repair --plat "${AUDITWHEEL_PLAT:-manylinux_2_34_x86_64}" \
    --exclude libfranky.so --exclude "$(basename "${FRANKA_LIB}")" \
    -w "${WORK}/${name}-dist" "$(ls "${WORK}/${name}-raw/${name}"-*.whl)"
  auditwheel show "$(ls "${WORK}/${name}-dist/${name}"-*.whl)"
}

# ctest_package <package dir name> <CMake prefix>: CTest with the same toolchain against the wheel's libraries.
ctest_package() {
  local name="$1" prefix="$2"
  pushd "${WORK}/${name}" >/dev/null
  cmake --preset docker \
    -DFRANKY_INCLUDE_DIR="${WORK}/franky/include" -DFRANKY_LIBRARY="${FRANKY_LIB}" \
    -DFRANKY_LINK_LIBRARIES="${FRANKA_LIB}" -D"${prefix}"_FRANKY_DIST_VERSION="${FRANKY_DIST_VERSION}" \
    -D"${prefix}"_FRANKY_COMMIT="${FRANKY_COMMIT:-}" -D"${prefix}"_BUILD_ID="${BUILD_ID}"
  cmake --build --preset docker -j "$(nproc)"
  ctest --preset docker 2>&1 | tee "${OUT}/logs/${name}_ctest.log"
  test "${PIPESTATUS[0]}" -eq 0
  popd >/dev/null
}

echo "== 3. joint_controller wheel"
build_wheel joint_controller JOINT_CONTROLLER
JC_WHL=$(ls "${WORK}"/joint_controller-dist/joint_controller-*.whl)
WHEELS=("${FRANKY_WHL}" "${JC_WHL}")
if [ "${WITH_SERL}" = 1 ]; then
  echo "== 3b. serl_controller wheel (same franky build)"
  build_wheel serl_controller SERL_CONTROLLER
  WHEELS+=("$(ls "${WORK}"/serl_controller-dist/serl_controller-*.whl)")
fi

echo "== 4. C++ tests (CTest), same toolchain, linked against the wheel's libraries"
ctest_package joint_controller JOINT_CONTROLLER
if [ "${WITH_SERL}" = 1 ]; then ctest_package serl_controller SERL_CONTROLLER; fi

echo "== 5. Python tests in a fresh venv with only the built wheels"
"${PY}" -m venv "${WORK}/venv-test"
"${WORK}/venv-test/bin/pip" install -q "${WHEELS[@]}" pytest
(cd /tmp && "${WORK}/venv-test/bin/python" - <<'PY'
import franky, joint_controller as jc, joint_controller._joint_controller as ext
print("franky", franky.__file__)
print("joint_controller", jc.__version__, "GELLO", jc.GELLO_COMMIT, ext.__file__)
m = jc.TrajectoryImpedanceMotion(jc.default_params("panda"))
assert isinstance(m, franky.BaseTorqueMotion)
print("MRO:", [c.__module__ + "." + c.__name__ for c in type(m).__mro__])
print("build_info:", jc.build_info())
PY
)
(cd "${WORK}/joint_controller" && "${WORK}/venv-test/bin/python" -m pytest -v tests/python) 2>&1 | tee "${OUT}/logs/joint_controller_pytest.log"
test "${PIPESTATUS[0]}" -eq 0
if [ "${WITH_SERL}" = 1 ]; then
  (cd "${WORK}/serl_controller" && "${WORK}/venv-test/bin/python" -m pytest -v tests/python) 2>&1 | tee "${OUT}/logs/serl_controller_pytest.log"
  test "${PIPESTATUS[0]}" -eq 0
fi
EXT=$(ls "${WORK}"/venv-test/lib/python*/site-packages/joint_controller/_joint_controller*.so)
readelf -d "${EXT}" | grep -E "NEEDED|RPATH|RUNPATH" | tee "${OUT}/logs/joint_controller_ext_dynamic.txt"

echo "== 6. outputs"
cp "${WHEELS[@]}" "${OUT}/wheels/"
{
  echo "built: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "image: ${IMAGE:-?}"
  echo "build id: ${BUILD_ID}"
  echo "franky: ${FRANKY_TAG:-?} ${FRANKY_COMMIT:-?} (unmodified), wheel version ${FRANKY_DIST_VERSION}"
  echo "libfranka: ${LIBFRANKA_VERSION}"
  echo "python: $("${PY}" --version 2>&1) (${PYBIN})"
  echo "compiler: $(g++ --version | head -1)"
  echo "pybind11 (joint_controller build): $(grep -m1 -oE 'pybind11 [0-9.]+' "${OUT}/logs/joint_controller_wheel_build.log" || echo '?')"
  echo "pybind11 (franky build, container): $(grep PYBIND11_VERSION /usr/bin/dependency_versions)"
  echo "wheels: $(cd "${OUT}/wheels" && ls ./*.whl | tr '\n' ' ')"
} > "${OUT}/wheels/BUILD_INFO.txt"
(cd "${OUT}/wheels" && sha256sum ./*.whl > SHA256SUMS)
cat "${OUT}/wheels/BUILD_INFO.txt" "${OUT}/wheels/SHA256SUMS"

#!/usr/bin/env bash
# Runs INSIDE franky's build image (see build-on-host.sh). Builds, from the same
# toolchain, pybind11 and libfranka:
#   1. the unmodified franky wheel, with franky's own manylinux-build-wheels;
#   2. the serl_controller wheel, against that wheel's libfranky.so / libfranka;
# then runs the C++ (CTest) and Python (pytest) tests and copies both wheels to
# /out/wheels. Sources are mounted read-only and copied to /work first.
set -euo pipefail

PYBIN="${PYBIN:-cp310-cp310}"
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-0.17.0}"
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
cp -a /src/serl_controller "${WORK}/serl_controller"
rm -rf "${WORK}/serl_controller/build"

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
# Written into serl_controller/build_info.json: `rwrl doctor` compares it with the installed franky wheel.
BUILD_DEFINES=(
  -DSERL_CONTROLLER_FRANKY_DIST_VERSION="${FRANKY_DIST_VERSION}"
  -DSERL_CONTROLLER_FRANKY_COMMIT="${FRANKY_COMMIT:-}"
  -DSERL_CONTROLLER_BUILD_ID="${BUILD_ID}"
)

echo "== 3. serl_controller wheel"
"${WORK}/venv-build/bin/pip" wheel -v --no-deps "${WORK}/serl_controller" -w "${WORK}/serl-raw" \
  -C cmake.define.SERL_CONTROLLER_WITH_FRANKY=ON \
  -C cmake.define.FRANKY_INCLUDE_DIR="${WORK}/franky/include" \
  -C cmake.define.FRANKY_LIBRARY="${FRANKY_LIB}" \
  -C cmake.define.FRANKY_LINK_LIBRARIES="${FRANKA_LIB}" \
  -C cmake.define.SERL_CONTROLLER_FRANKY_DIST_VERSION="${FRANKY_DIST_VERSION}" \
  -C cmake.define.SERL_CONTROLLER_FRANKY_COMMIT="${FRANKY_COMMIT:-}" \
  -C cmake.define.SERL_CONTROLLER_BUILD_ID="${BUILD_ID}" \
  -C build-dir="${WORK}/serl-skbuild" > "${OUT}/logs/serl_wheel_build.log" 2>&1 \
  || { tail -80 "${OUT}/logs/serl_wheel_build.log"; exit 1; }
grep -E "pybind11 |libfranka |serl_controller:" "${OUT}/logs/serl_wheel_build.log" || true
RAW_WHL=$(ls "${WORK}"/serl-raw/serl_controller-*.whl)
# Bundle nothing from franky: libfranky.so and libfranka-<hash>.so are provided by
# the franky wheel installed next to it (RPATH $ORIGIN/../franky:$ORIGIN/../franky_control.libs).
auditwheel repair --plat "${AUDITWHEEL_PLAT:-manylinux_2_34_x86_64}" \
  --exclude libfranky.so --exclude "$(basename "${FRANKA_LIB}")" \
  -w "${WORK}/serl-dist" "${RAW_WHL}"
SERL_WHL=$(ls "${WORK}"/serl-dist/serl_controller-*.whl)
auditwheel show "${SERL_WHL}"

echo "== 4. C++ tests (CTest), same toolchain, linked against the wheel's libraries"
pushd "${WORK}/serl_controller" >/dev/null
cmake --preset docker \
  -DFRANKY_INCLUDE_DIR="${WORK}/franky/include" -DFRANKY_LIBRARY="${FRANKY_LIB}" \
  -DFRANKY_LINK_LIBRARIES="${FRANKA_LIB}" "${BUILD_DEFINES[@]}"
cmake --build --preset docker -j "$(nproc)"
ctest --preset docker 2>&1 | tee "${OUT}/logs/ctest.log"
test "${PIPESTATUS[0]}" -eq 0
popd >/dev/null
"${WORK}/serl_controller/build/docker/tests/cpp/test_golden_equivalence" > "${OUT}/logs/golden_equivalence.txt"
"${WORK}/serl_controller/build/docker/tests/cpp/test_joint_reset_golden" > "${OUT}/logs/joint_reset_golden.txt"

echo "== 5. Python tests in a fresh venv with only the two wheels"
"${PY}" -m venv "${WORK}/venv-test"
"${WORK}/venv-test/bin/pip" install -q "${FRANKY_WHL}" "${SERL_WHL}" pytest
(cd /tmp && "${WORK}/venv-test/bin/python" - <<'PY'
import franky, serl_controller, serl_controller._serl_controller as ext
print("franky", franky.__file__)
print("serl_controller", serl_controller.__version__, "SERL", serl_controller.SERL_COMMIT, ext.__file__)
m = serl_controller.SerlCartesianImpedanceMotion()
assert isinstance(m, franky.BaseTorqueMotion)
print("MRO:", [c.__module__ + "." + c.__name__ for c in type(m).__mro__])
j = serl_controller.SerlJointPositionMotion([0, 0, 0, -1.9, 0, 2, 0])
assert isinstance(j, franky.BaseJointPositionMotion)
print("MRO:", [c.__module__ + "." + c.__name__ for c in type(j).__mro__])
print("build_info:", serl_controller.build_info())
PY
)
(cd "${WORK}/serl_controller" && "${WORK}/venv-test/bin/python" -m pytest -v tests/python) 2>&1 | tee "${OUT}/logs/pytest.log"
test "${PIPESTATUS[0]}" -eq 0
EXT=$(ls "${WORK}"/venv-test/lib/python*/site-packages/serl_controller/_serl_controller*.so)
readelf -d "${EXT}" | grep -E "NEEDED|RPATH|RUNPATH" | tee "${OUT}/logs/serl_ext_dynamic.txt"

echo "== 6. outputs"
cp "${FRANKY_WHL}" "${SERL_WHL}" "${OUT}/wheels/"
cp "${OUT}/logs/golden_equivalence.txt" "${OUT}/logs/joint_reset_golden.txt" "${OUT}/wheels/"
{
  echo "built: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "image: ${IMAGE:-?}"
  echo "build id: ${BUILD_ID}"
  echo "franky: ${FRANKY_TAG:-?} ${FRANKY_COMMIT:-?} (unmodified), wheel version ${FRANKY_DIST_VERSION}"
  echo "libfranka: ${LIBFRANKA_VERSION}"
  echo "python: $("${PY}" --version 2>&1) (${PYBIN})"
  echo "compiler: $(g++ --version | head -1)"
  echo "pybind11 (serl build): $(grep -m1 -oE 'pybind11 [0-9.]+' "${OUT}/logs/serl_wheel_build.log" || echo '?')"
  echo "pybind11 (franky build, container): $(grep PYBIND11_VERSION /usr/bin/dependency_versions)"
} > "${OUT}/wheels/BUILD_INFO.txt"
(cd "${OUT}/wheels" && sha256sum ./*.whl > SHA256SUMS)
cat "${OUT}/wheels/BUILD_INFO.txt" "${OUT}/wheels/SHA256SUMS"

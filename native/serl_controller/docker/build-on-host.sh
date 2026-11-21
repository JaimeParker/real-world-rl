#!/usr/bin/env bash
# Build UNMODIFIED franky (v2.0.0) + serl_controller wheels together, for
# CPython 3.10 / manylinux_2_34_x86_64 / libfranka 0.17.0 (the Zizhu robot PC).
#
# Usage (on the build host, from anywhere):
#   native/serl_controller/docker/build-on-host.sh            # WORK defaults below
#   WORK=/path/to/workdir LIBFRANKA_VERSION=0.17.0 .../build-on-host.sh
#
# Steps: clone franky at the pinned tag into $WORK/src/franky and verify the
# commit; build franky's OWN build image (franky/docker/build/Dockerfile,
# unmodified) with LIBFRANKA_VERSION; run docker/build-wheels.sh inside it in a
# container named $CONTAINER. Results: $WORK/out/wheels/*.whl, $WORK/out/logs/.
# No GPU is used or requested.
set -euo pipefail

SERL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="${WORK:-${HOME}/data0/rwrl-franky-build}"
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-0.17.0}"
MANYLINUX_VERSION="${MANYLINUX_VERSION:-manylinux_2_34_x86_64}"
PYBIN="${PYBIN:-cp310-cp310}"
FRANKY_REPO="${FRANKY_REPO:-https://github.com/TimSchneider42/franky.git}"
FRANKY_TAG="${FRANKY_TAG:-v2.0.0}"
FRANKY_COMMIT="${FRANKY_COMMIT:-e7e63c7c7118f427ad33211500b33b81b4011bb4}"
IMAGE="${IMAGE:-rwrl-franky-build:franky-${FRANKY_TAG}-libfranka-${LIBFRANKA_VERSION}-${MANYLINUX_VERSION}}"
CONTAINER="${CONTAINER:-rwrl-franky-build}"

mkdir -p "${WORK}/src" "${WORK}/out"
FRANKY_SRC="${WORK}/src/franky"
if [ ! -d "${FRANKY_SRC}/.git" ]; then
  git clone --quiet "${FRANKY_REPO}" "${FRANKY_SRC}"
fi
git -C "${FRANKY_SRC}" fetch --quiet --tags origin
git -C "${FRANKY_SRC}" checkout --quiet --detach "${FRANKY_TAG}"
actual="$(git -C "${FRANKY_SRC}" rev-parse HEAD)"
if [ "${actual}" != "${FRANKY_COMMIT}" ]; then
  echo "franky ${FRANKY_TAG} is ${actual}, expected ${FRANKY_COMMIT}" >&2
  exit 1
fi
if [ -n "$(git -C "${FRANKY_SRC}" status --porcelain)" ]; then
  echo "franky checkout ${FRANKY_SRC} is modified; refusing to build" >&2
  exit 1
fi

echo "== building image ${IMAGE} from franky's own docker/build/Dockerfile"
docker build \
  --build-arg "LIBFRANKA_VERSION=${LIBFRANKA_VERSION}" \
  --build-arg "MANYLINUX_VERSION=${MANYLINUX_VERSION}" \
  -f "${FRANKY_SRC}/docker/build/Dockerfile" -t "${IMAGE}" "${FRANKY_SRC}"

echo "== running ${CONTAINER}"
docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
# Optional pip settings (e.g. a PyPI mirror when pypi.org is slow from the build host).
pip_env=()
for var in PIP_INDEX_URL PIP_EXTRA_INDEX_URL PIP_TRUSTED_HOST PIP_DEFAULT_TIMEOUT PIP_RETRIES; do
  if [ -n "${!var:-}" ]; then pip_env+=(-e "${var}=${!var}"); fi
done
docker run --name "${CONTAINER}" "${pip_env[@]}" \
  -e "PYBIN=${PYBIN}" -e "LIBFRANKA_VERSION=${LIBFRANKA_VERSION}" \
  -e "FRANKY_TAG=${FRANKY_TAG}" -e "FRANKY_COMMIT=${FRANKY_COMMIT}" -e "IMAGE=${IMAGE}" \
  -v "${FRANKY_SRC}:/src/franky:ro" \
  -v "${SERL_DIR}:/src/serl_controller:ro" \
  -v "${WORK}/out:/out" \
  --entrypoint /bin/bash "${IMAGE}" /src/serl_controller/docker/build-wheels.sh
echo "== wheels in ${WORK}/out/wheels"
ls -l "${WORK}/out/wheels"

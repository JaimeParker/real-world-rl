#!/usr/bin/env bash
# clang-tidy (.clang-tidy) + clang-format (.clang-format) over joint_controller's OWN C++ code.
#
#   tools/lint.sh <build-dir-with-compile_commands.json> [--fix-format]
#
# Linted: include/joint_controller/*.hpp, src/*.cpp, python/bindings.cpp (if it is in the
# compile database, i.e. a franky + Python build), tests/cpp/*.{cpp,hpp}.
# Translation units missing from the compile database (e.g. motion.cpp in the franky-free `dev`
# preset) are skipped with a note. CLANG_TIDY / CLANG_FORMAT override the tool names,
# CLANG_TIDY_EXTRA_ARGS adds clang-tidy arguments (e.g. --extra-arg=--gcc-install-dir=...).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${1:?usage: tools/lint.sh <build-dir> [--fix-format]}"
BUILD="$(cd "${BUILD}" && pwd)"
FIX_FORMAT="${2:-}"
CLANG_TIDY="${CLANG_TIDY:-clang-tidy}"
CLANG_FORMAT="${CLANG_FORMAT:-clang-format}"
test -f "${BUILD}/compile_commands.json" || { echo "no ${BUILD}/compile_commands.json" >&2; exit 2; }

cd "${ROOT}"
mapfile -t TUS < <(ls src/*.cpp python/bindings.cpp tests/cpp/*.cpp)
mapfile -t FORMAT_FILES < <(ls include/joint_controller/*.hpp src/*.cpp python/bindings.cpp tests/cpp/*.cpp tests/cpp/*.hpp)

status=0
echo "== clang-format ($("${CLANG_FORMAT}" --version))"
if [ "${FIX_FORMAT}" = "--fix-format" ]; then
  "${CLANG_FORMAT}" -i "${FORMAT_FILES[@]}"
else
  "${CLANG_FORMAT}" --dry-run -Werror "${FORMAT_FILES[@]}" || status=1
fi

echo "== clang-tidy ($("${CLANG_TIDY}" --version | grep -m1 -i version))"
for tu in "${TUS[@]}"; do
  if ! grep -q "\"file\": \"[^\"]*/${tu}\"" "${BUILD}/compile_commands.json"; then
    echo "-- skip ${tu} (not in this compile database)"
    continue
  fi
  echo "-- ${tu}"
  # shellcheck disable=SC2086
  "${CLANG_TIDY}" ${CLANG_TIDY_EXTRA_ARGS:-} -p "${BUILD}" --quiet "${tu}" || status=1
done
exit "${status}"

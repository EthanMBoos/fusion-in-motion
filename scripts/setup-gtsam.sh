#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="$repo_root/third_party/gtsam"
build_dir="$source_dir/build"
install_dir="$source_dir/install"

if [[ ! -d "$source_dir/.git" ]]; then
  mkdir -p "$(dirname "$source_dir")"
  git clone --depth 1 --branch 4.2.2 https://github.com/borglab/gtsam.git "$source_dir"
fi

cmake -S "$source_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$install_dir" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_DOCS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_WITH_TBB=OFF
cmake --build "$build_dir" --target install --parallel 2

echo "GTSAM 4.2.2 installed in $install_dir"

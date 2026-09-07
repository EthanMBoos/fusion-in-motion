#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run="${1:?usage: ./scripts/run-stone-soup.sh RUN_DIRECTORY}"
output="$run/reports/stone-soup/tracks.csv"
export UV_PROJECT_ENVIRONMENT="$repo_root/third_party/stone-soup-venv"

if ! command -v uv >/dev/null; then
  echo "uv is required: https://docs.astral.sh/uv/getting-started/installation/" >&2
  exit 1
fi

if ! command -v fusion >/dev/null; then
  echo "fusion is not installed; see docs/INSTALL.md" >&2
  exit 1
fi

uv run --locked \
  --project "$repo_root/integrations/stone_soup" \
  "$repo_root/integrations/stone_soup/stone_soup_tracker.py" \
  "$run" "$output"

fusion score tracks "$run" "$output" --id stone-soup --ego-source truth

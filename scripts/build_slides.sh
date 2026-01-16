#!/usr/bin/env bash
set -euo pipefail

# Build slides using marp or reveal-md (optional)
if command -v marp >/dev/null 2>&1; then
  marp docs/ppt/slides.md -o docs/ppt/slides.html
  marp docs/ppt/slides.md -o docs/ppt/slides.pdf
  echo "Slides generated via marp."
elif command -v reveal-md >/dev/null 2>&1; then
  reveal-md docs/ppt/slides.md --static docs/ppt
  echo "Slides generated via reveal-md."
else
  echo "No slide renderer found. Install marp or reveal-md."
  exit 1
fi

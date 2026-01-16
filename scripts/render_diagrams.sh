#!/usr/bin/env bash
set -euo pipefail

# Render Mermaid diagrams if mermaid-cli is installed
if ! command -v mmdc >/dev/null 2>&1; then
  echo "mmdc not found. Install with: npm install -g @mermaid-js/mermaid-cli"
  exit 1
fi

mkdir -p docs/architecture

mmdc -i docs/architecture/system_architecture.mmd -o docs/architecture/system_architecture.png
mmdc -i docs/architecture/state_machine.mmd -o docs/architecture/state_machine.png

echo "Rendered diagrams to docs/architecture/*.png"

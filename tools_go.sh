#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sed -i 's/\r$//' "$ROOT"/tools_*.sh "$ROOT"/tools_test_control_once.py 2>/dev/null || true
bash "$ROOT/tools_run_control_test.sh"

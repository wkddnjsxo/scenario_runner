#!/usr/bin/env bash
set -euo pipefail

GRPC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$GRPC_ROOT/src/grpc_example_new2.py" "$@"

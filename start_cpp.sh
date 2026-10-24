#!/usr/bin/env bash
# Compatibility entry point. Both launchers now use the C++ pipeline.
set -e
PANDA_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$PANDA_WORKSPACE/start.sh" "$@"

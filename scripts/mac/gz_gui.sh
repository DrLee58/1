#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/mac/gz_env.sh"

echo "GZ_IP=${GZ_IP}"
echo "GZ_PARTITION=${GZ_PARTITION}"
echo "Starting Gazebo Harmonic GUI"

exec gz sim -g

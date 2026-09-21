#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/mac/gz_env.sh"

WORLD="${ROOT}/src/physical_ai_amr/worlds/physical_ai_world.sdf"

echo "GZ_IP=${GZ_IP}"
echo "GZ_PARTITION=${GZ_PARTITION}"
echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "Starting Gazebo Harmonic server: ${WORLD}"

exec gz sim -s -r -v 4 "${WORLD}"

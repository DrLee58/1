#!/usr/bin/env bash
# Gazebo Transport on macOS often fails multicast when Docker Desktop
# and utun interfaces are present. Bind discovery to loopback instead.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_PARTITION="${GZ_PARTITION:-physical_ai}"
export GZ_SIM_RESOURCE_PATH="${ROOT}/src/physical_ai_amr/models:${GZ_SIM_RESOURCE_PATH:-}"

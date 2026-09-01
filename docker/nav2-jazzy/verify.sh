#!/usr/bin/env bash
set -euo pipefail

readonly SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
readonly PACKAGE_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
readonly IMAGE_NAME=${BAC_NAV2_IMAGE:-bac-nav2-jazzy-verification}

docker build \
  --tag "${IMAGE_NAME}" \
  --file "${SCRIPT_DIR}/Dockerfile" \
  "${PACKAGE_ROOT}"

docker run --rm \
  --mount "type=bind,src=${PACKAGE_ROOT},dst=/source,readonly" \
  "${IMAGE_NAME}" \
  bash /source/docker/nav2-jazzy/test_package.sh

#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

readonly PACKAGE_NAME=bilateral_arc_clearance_controller
readonly SOURCE_DIR=${BAC_SOURCE_DIR:-/source}
readonly WORK_DIR=${BAC_WORK_DIR:-/tmp/bac-nav2-jazzy-ws}

if [[ ! -f "${SOURCE_DIR}/package.xml" ]]; then
  echo "Package source was not found at ${SOURCE_DIR}" >&2
  exit 2
fi

mkdir -p "${WORK_DIR}"
cd "${SOURCE_DIR}"

colcon --log-base "${WORK_DIR}/log-build" build \
  --base-paths "${SOURCE_DIR}" \
  --packages-select "${PACKAGE_NAME}" \
  --build-base "${WORK_DIR}/build" \
  --install-base "${WORK_DIR}/install" \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON \
    -DBAC_BUILD_NAV2_PLUGIN=ON

set +u
source "${WORK_DIR}/install/setup.bash"
set -u

colcon --log-base "${WORK_DIR}/log-test" test \
  --base-paths "${SOURCE_DIR}" \
  --packages-select "${PACKAGE_NAME}" \
  --build-base "${WORK_DIR}/build" \
  --install-base "${WORK_DIR}/install" \
  --event-handlers console_direct+

colcon --log-base "${WORK_DIR}/log-result" test-result \
  --test-result-base "${WORK_DIR}/build" \
  --verbose

PACKAGE_PREFIX=$(ros2 pkg prefix "${PACKAGE_NAME}")
readonly PACKAGE_PREFIX
readonly PLUGIN_DESCRIPTION="${PACKAGE_PREFIX}/share/${PACKAGE_NAME}/bilateral_arc_clearance_controller_plugin.xml"
if [[ ! -f "${PLUGIN_DESCRIPTION}" ]]; then
  echo "Installed Nav2 plugin description was not found: ${PLUGIN_DESCRIPTION}" >&2
  exit 3
fi

echo "ROS 2 Jazzy/Nav2 plugin build and tests passed."

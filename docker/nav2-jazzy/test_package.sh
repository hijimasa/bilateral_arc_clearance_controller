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

# --- Ackermann motion model -------------------------------------------------
# The suite above passes even if the Ackermann tests were never registered, so
# require the labelled subset to exist and run.
readonly ACKERMANN_LOG="${WORK_DIR}/ackermann-tests.log"
if ! ctest --test-dir "${WORK_DIR}/build/${PACKAGE_NAME}" \
    --label-regex ackermann --output-on-failure >"${ACKERMANN_LOG}" 2>&1; then
  echo "Ackermann labelled tests failed:" >&2
  cat "${ACKERMANN_LOG}" >&2
  exit 4
fi
ACKERMANN_TEST_COUNT=$(sed -n 's/^.*tests passed, .* out of \([0-9][0-9]*\).*$/\1/p' \
    "${ACKERMANN_LOG}" | tail -n 1)
readonly ACKERMANN_TEST_COUNT
if [[ -z "${ACKERMANN_TEST_COUNT}" || "${ACKERMANN_TEST_COUNT}" -lt 2 ]]; then
  echo "Expected at least 2 tests labelled 'ackermann', found '${ACKERMANN_TEST_COUNT:-0}'" >&2
  cat "${ACKERMANN_LOG}" >&2
  exit 4
fi
echo "Ackermann labelled tests: ${ACKERMANN_TEST_COUNT} passed."

readonly ACKERMANN_CONFIG="${PACKAGE_PREFIX}/share/${PACKAGE_NAME}/config/bac_controller_ackermann.yaml"
if [[ ! -f "${ACKERMANN_CONFIG}" ]]; then
  echo "Installed Ackermann configuration was not found: ${ACKERMANN_CONFIG}" >&2
  exit 5
fi
if ! grep -q 'motion_model.type: ackermann' "${ACKERMANN_CONFIG}"; then
  echo "Installed Ackermann configuration does not select the Ackermann model" >&2
  exit 5
fi

# The parameter path only proves itself in an installed node: a valid Ackermann
# configuration must come up, and an unsupported model must be rejected rather
# than silently falling back to differential drive.
run_filter_node() {
  local log=$1
  shift
  timeout 15s ros2 run "${PACKAGE_NAME}" bac_filter_node --ros-args "$@" >"${log}" 2>&1
  return $?
}

readonly VALID_LOG="${WORK_DIR}/ackermann-node-valid.log"
VALID_STATUS=0
run_filter_node "${VALID_LOG}" \
  -p motion_model.type:=ackermann -p turn_radius_min:=1.0 || VALID_STATUS=$?
readonly VALID_STATUS
# 124 is the timeout expiring on a node that stayed up, which is the pass here.
if [[ "${VALID_STATUS}" -ne 124 ]]; then
  echo "Ackermann filter node exited with ${VALID_STATUS} instead of running:" >&2
  cat "${VALID_LOG}" >&2
  exit 6
fi

readonly REJECT_LOG="${WORK_DIR}/ackermann-node-rejected.log"
if run_filter_node "${REJECT_LOG}" -p motion_model.type:=tricycle; then
  echo "An unsupported motion_model.type was accepted:" >&2
  cat "${REJECT_LOG}" >&2
  exit 6
fi
if ! grep -q "motion_model.type" "${REJECT_LOG}"; then
  echo "An unsupported motion_model.type was not reported clearly:" >&2
  cat "${REJECT_LOG}" >&2
  exit 6
fi

readonly RADIUS_LOG="${WORK_DIR}/ackermann-node-radius.log"
if run_filter_node "${RADIUS_LOG}" \
    -p motion_model.type:=ackermann -p turn_radius_min:=0.0; then
  echo "A non-positive Ackermann turn_radius_min was accepted:" >&2
  cat "${RADIUS_LOG}" >&2
  exit 6
fi
echo "Ackermann installed-configuration checks passed."

readonly OMNI_LOG="${WORK_DIR}/omni-tests.log"
if ! ctest --test-dir "${WORK_DIR}/build/${PACKAGE_NAME}" \
    --label-regex omni --output-on-failure >"${OMNI_LOG}" 2>&1; then
  echo "Holonomic labelled tests failed:" >&2
  cat "${OMNI_LOG}" >&2
  exit 4
fi
OMNI_TEST_COUNT=$(sed -n 's/^.*tests passed, .* out of \([0-9][0-9]*\).*$/\1/p' \
    "${OMNI_LOG}" | tail -n 1)
readonly OMNI_TEST_COUNT
if [[ -z "${OMNI_TEST_COUNT}" || "${OMNI_TEST_COUNT}" -lt 2 ]]; then
  echo "Expected at least 2 tests labelled 'omni', found '${OMNI_TEST_COUNT:-0}'" >&2
  cat "${OMNI_LOG}" >&2
  exit 4
fi
echo "Holonomic labelled tests: ${OMNI_TEST_COUNT} passed."

readonly OMNI_CONFIG="${PACKAGE_PREFIX}/share/${PACKAGE_NAME}/config/bac_controller_omni.yaml"
if [[ ! -f "${OMNI_CONFIG}" ]]; then
  echo "Installed holonomic configuration was not found: ${OMNI_CONFIG}" >&2
  exit 5
fi
if ! grep -q 'motion_model.type: omni' "${OMNI_CONFIG}"; then
  echo "Installed holonomic configuration does not select the holonomic model" >&2
  exit 5
fi

readonly OMNI_VALID_LOG="${WORK_DIR}/omni-node-valid.log"
OMNI_VALID_STATUS=0
run_filter_node "${OMNI_VALID_LOG}" \
  -p motion_model.type:=omni -p limits.vy_max:=0.3 || OMNI_VALID_STATUS=$?
readonly OMNI_VALID_STATUS
if [[ "${OMNI_VALID_STATUS}" -ne 124 ]]; then
  echo "Holonomic filter node exited with ${OMNI_VALID_STATUS} instead of running:" >&2
  cat "${OMNI_VALID_LOG}" >&2
  exit 6
fi

# Selecting the holonomic model without lateral authority would silently
# degrade it to a drive that cannot steer, so it must be refused.
readonly OMNI_VY_LOG="${WORK_DIR}/omni-node-vy.log"
if run_filter_node "${OMNI_VY_LOG}" \
    -p motion_model.type:=omni -p limits.vy_max:=0.0; then
  echo "A holonomic configuration with zero limits.vy_max was accepted:" >&2
  cat "${OMNI_VY_LOG}" >&2
  exit 6
fi
echo "Holonomic installed-configuration checks passed."

echo "ROS 2 Jazzy/Nav2 plugin build and tests passed."

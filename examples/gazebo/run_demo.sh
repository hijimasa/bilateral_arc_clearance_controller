#!/usr/bin/env bash
set -euo pipefail

DEMO_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PACKAGE_ROOT=$(cd "${DEMO_DIR}/../.." && pwd)
OUTPUT_DIR=${BAC_DEMO_OUTPUT_DIR:-${PACKAGE_ROOT}/docs/media}
VIDEO_NAME=bac_gazebo_appearing_obstacle
WORK_DIR=$(mktemp -d /tmp/bac-gazebo-demo.XXXXXX)
CAPTURE_DIR=${WORK_DIR}/capture
WS_DIR=${WORK_DIR}/ws
PIDS=()

cleanup()
{
  local pid
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

mkdir -p "${CAPTURE_DIR}" "${WS_DIR}/src" "${OUTPUT_DIR}"
ln -s "${PACKAGE_ROOT}" "${WS_DIR}/src/bilateral_arc_clearance_controller"

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${WS_DIR}"
colcon build --packages-select bilateral_arc_clearance_controller \
  --cmake-args -DBUILD_TESTING=OFF -DBAC_BUILD_NAV2_PLUGIN=OFF \
  --event-handlers console_direct+
set +u
source "${WS_DIR}/install/setup.bash"
set -u

export LIBGL_ALWAYS_SOFTWARE=1
export RCUTILS_COLORIZED_OUTPUT=1
export BAC_DEMO_OUTPUT="${CAPTURE_DIR}"
export BAC_DEMO_DURATION=${BAC_DEMO_DURATION:-28}
export BAC_GAZEBO_VERSION
# Gazebo Classic 11 reports its version but exits 255 in some headless builds.
BAC_GAZEBO_VERSION=$(gazebo --version 2>&1 || true)
BAC_GAZEBO_VERSION=${BAC_GAZEBO_VERSION%%$'\n'*}

if [[ -z "${DISPLAY:-}" ]]; then
  Xvfb :99 -screen 0 1280x720x24 -nolisten tcp \
    >"${CAPTURE_DIR}/xvfb.log" 2>&1 &
  PIDS+=("$!")
  export DISPLAY=:99
  sleep 1
fi

ros2 launch gazebo_ros gazebo.launch.py \
  world:="${DEMO_DIR}/worlds/appearing_obstacle.world" gui:=false verbose:=false \
  extra_gazebo_args:="--seed 42" \
  >"${CAPTURE_DIR}/gazebo.log" 2>&1 &
PIDS+=("$!")

for _ in $(seq 1 60); do
  if ros2 service type /spawn_entity >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
if ! ros2 service type /spawn_entity >/dev/null 2>&1; then
  echo "Gazebo spawn service did not become ready" >&2
  exit 1
fi

ros2 run gazebo_ros spawn_entity.py -entity bac_demo_robot \
  -file "${DEMO_DIR}/models/robot.urdf" -x 0.0 -y 0.0 -z 0.02 \
  >"${CAPTURE_DIR}/spawn_robot.log" 2>&1

ros2 run bilateral_arc_clearance_controller bac_filter_node --ros-args \
  --params-file "${DEMO_DIR}/bac_demo.yaml" \
  -r cmd_vel_in:=/nav_cmd_vel -r cmd_vel_out:=/cmd_vel \
  -r scan:=/scan -r odom:=/odom -r avoid_status:=/avoid_status \
  >"${CAPTURE_DIR}/bac_filter.log" 2>&1 &
PIDS+=("$!")

python3 "${DEMO_DIR}/scripts/demo_driver.py" --ros-args -p use_sim_time:=true \
  >"${CAPTURE_DIR}/driver.log" 2>&1 &
PIDS+=("$!")

python3 "${DEMO_DIR}/scripts/record_demo.py" --ros-args -p use_sim_time:=true \
  >"${CAPTURE_DIR}/recorder.log" 2>&1 &
RECORDER_PID=$!
PIDS+=("${RECORDER_PID}")

# Trigger on robot position rather than host time so CPU load cannot change the
# initial condition of the appearing-obstacle event.
python3 "${DEMO_DIR}/scripts/spawn_obstacle.py" \
  "${DEMO_DIR}/models/appearing_obstacle.sdf" --ros-args -p use_sim_time:=true \
  >"${CAPTURE_DIR}/spawn_obstacle.log" 2>&1

wait "${RECORDER_PID}"

ffmpeg -hide_banner -loglevel error -y -framerate 12 \
  -i "${CAPTURE_DIR}/frames/frame_%05d.jpg" \
  -c:v libx264 -preset medium -crf 25 -pix_fmt yuv420p -movflags +faststart \
  -metadata title="BAC Gazebo appearing-obstacle evidence" \
  "${CAPTURE_DIR}/${VIDEO_NAME}.mp4"
ffmpeg -hide_banner -loglevel error -y \
  -ss 9 -i "${CAPTURE_DIR}/${VIDEO_NAME}.mp4" -frames:v 1 \
  "${CAPTURE_DIR}/${VIDEO_NAME}_thumbnail.jpg"

set +e
python3 "${DEMO_DIR}/scripts/evaluate_demo.py" "${PACKAGE_ROOT}" "${CAPTURE_DIR}"
EVALUATION_STATUS=$?
set -e
cp "${CAPTURE_DIR}/${VIDEO_NAME}.mp4" "${OUTPUT_DIR}/${VIDEO_NAME}.mp4"
cp "${CAPTURE_DIR}/${VIDEO_NAME}_thumbnail.jpg" "${OUTPUT_DIR}/${VIDEO_NAME}_thumbnail.jpg"
cp "${CAPTURE_DIR}/telemetry.csv" "${OUTPUT_DIR}/${VIDEO_NAME}_telemetry.csv"
cp "${CAPTURE_DIR}/evidence.json" "${OUTPUT_DIR}/${VIDEO_NAME}_evidence.json"
if (( EVALUATION_STATUS != 0 )); then
  echo "Gazebo evidence checks failed; diagnostic artifacts were retained" >&2
  exit "${EVALUATION_STATUS}"
fi

ffprobe -v error -show_entries format=duration,size \
  -of default=noprint_wrappers=1 "${CAPTURE_DIR}/${VIDEO_NAME}.mp4"
echo "Gazebo evidence written to ${OUTPUT_DIR}"

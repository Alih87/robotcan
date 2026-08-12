#!/usr/bin/env bash

WS="/ws/isaac_ros-dev"

NAVPVT_TOPIC="/robot_gps_node/navpvt"
HEADING_TOPIC="/dual_gnss/heading_enu_rad"
DRIVE_SERVICE="/send_drive_cmd"

FORWARD_WAYPOINT_FILE="/ws/isaac_ros-dev/gps_waypoints.csv"
BACK_WAYPOINT_FILE="/ws/isaac_ros-dev/gps_waypoints_back.csv"

LOG_DIR="/ws/isaac_ros-dev/field_test_logs"
FORWARD_LOG="/ws/isaac_ros-dev/field_test_logs/gps_waypoint_log_forward_field.csv"
BACK_LOG="/ws/isaac_ros-dev/field_test_logs/gps_waypoint_log_back_field.csv"

FORWARD_TARGET_LAT="35.829614400"
FORWARD_TARGET_LON="126.686568622"

RETURN_TARGET_LAT="35.829283136"
RETURN_TARGET_LON="126.687334732"

FORWARD_NODE_PID=""
BACK_NODE_PID=""

cleanup()
{
  echo ""
  echo "Cleanup requested."

  if [ "${FORWARD_NODE_PID}" != "" ]; then
    kill -INT "${FORWARD_NODE_PID}" 2>/dev/null
    sleep 1
    kill -TERM "${FORWARD_NODE_PID}" 2>/dev/null
  fi

  if [ "${BACK_NODE_PID}" != "" ]; then
    kill -INT "${BACK_NODE_PID}" 2>/dev/null
    sleep 1
    kill -TERM "${BACK_NODE_PID}" 2>/dev/null
  fi
}

trap cleanup INT TERM

source /opt/ros/humble/setup.bash
source /ws/isaac_ros-dev/install/setup.bash

mkdir -p "${LOG_DIR}"

cat > "${FORWARD_WAYPOINT_FILE}" <<WAYPOINTS
lat_deg,lon_deg
${FORWARD_TARGET_LAT},${FORWARD_TARGET_LON}
WAYPOINTS

cat > "${BACK_WAYPOINT_FILE}" <<WAYPOINTS
lat_deg,lon_deg
${RETURN_TARGET_LAT},${RETURN_TARGET_LON}
WAYPOINTS

rm -f "${FORWARD_LOG}"
rm -f "${BACK_LOG}"

echo "Forward waypoint:"
cat "${FORWARD_WAYPOINT_FILE}"

echo ""
echo "Reverse waypoint:"
cat "${BACK_WAYPOINT_FILE}"

echo ""
echo "Starting FORWARD..."

ros2 run robotcan gps_waypoint_follower_v2 --ros-args \
  -p navpvt_topic:="${NAVPVT_TOPIC}" \
  -p heading_topic:="${HEADING_TOPIC}" \
  -p drive_service:="${DRIVE_SERVICE}" \
  -p waypoint_file:="${FORWARD_WAYPOINT_FILE}" \
  -p log_file:="${FORWARD_LOG}" \
  -p drive_mode:=W \
  -p drive_direction:=F \
  -p speed:=1 \
  -p waypoint_radius_m:=0.8 \
  -p steering_kp:=0.4 \
  -p max_steering_deg:=4.0 \
  -p heading_deadband_deg:=4.0 &

FORWARD_NODE_PID=$!

echo "Forward PID: ${FORWARD_NODE_PID}"

while true
do
  if ! kill -0 "${FORWARD_NODE_PID}" 2>/dev/null; then
    echo "Forward node stopped."
    break
  fi

  if [ -f "${FORWARD_LOG}" ]; then
    if grep -q "mission_complete_final_waypoint_reached" "${FORWARD_LOG}"; then
      echo "Forward mission complete."
      break
    fi
  fi

  sleep 1
done

echo "Stopping forward node..."
kill -INT "${FORWARD_NODE_PID}" 2>/dev/null
sleep 2
kill -TERM "${FORWARD_NODE_PID}" 2>/dev/null
wait "${FORWARD_NODE_PID}" 2>/dev/null
FORWARD_NODE_PID=""

echo ""
echo "Starting REVERSE..."

ros2 run robotcan gps_waypoint_follower_v2 --ros-args \
  -p navpvt_topic:="${NAVPVT_TOPIC}" \
  -p heading_topic:="${HEADING_TOPIC}" \
  -p drive_service:="${DRIVE_SERVICE}" \
  -p waypoint_file:="${BACK_WAYPOINT_FILE}" \
  -p log_file:="${BACK_LOG}" \
  -p drive_mode:=W \
  -p drive_direction:=R \
  -p speed:=1 \
  -p waypoint_radius_m:=0.8 \
  -p steering_kp:=0.25 \
  -p max_steering_deg:=4.0 \
  -p heading_deadband_deg:=5.0 &

BACK_NODE_PID=$!

echo "Reverse PID: ${BACK_NODE_PID}"

while true
do
  if ! kill -0 "${BACK_NODE_PID}" 2>/dev/null; then
    echo "Reverse node stopped."
    break
  fi

  if [ -f "${BACK_LOG}" ]; then
    if grep -q "mission_complete_final_waypoint_reached" "${BACK_LOG}"; then
      echo "Reverse mission complete."
      break
    fi
  fi

  sleep 1
done

echo "Stopping reverse node..."
kill -INT "${BACK_NODE_PID}" 2>/dev/null
sleep 2
kill -TERM "${BACK_NODE_PID}" 2>/dev/null
wait "${BACK_NODE_PID}" 2>/dev/null
BACK_NODE_PID=""

trap - INT TERM

echo ""
echo "DONE"
echo "Forward log: ${FORWARD_LOG}"
echo "Back log: ${BACK_LOG}"

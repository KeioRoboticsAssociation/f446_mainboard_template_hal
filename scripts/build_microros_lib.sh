#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ENV_FILE_DEFAULT="${ROOT_DIR}/.microros.env"
ENV_FILE_FALLBACK="${ROOT_DIR}/scripts/microros.env"
ENV_FILE="${MICROROS_ENV_FILE:-${ENV_FILE_DEFAULT}}"
if [ -f "${ENV_FILE}" ]; then
  echo "[microros] Loading env: ${ENV_FILE}"
  set +u
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set -u
elif [ -f "${ENV_FILE_FALLBACK}" ]; then
  echo "[microros] Loading env: ${ENV_FILE_FALLBACK}"
  set +u
  # shellcheck disable=SC1090
  source "${ENV_FILE_FALLBACK}"
  set -u
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -z "${MICROROS_WS:-}" ]; then
  set +e
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  if ROS_PREFIX="$(ros2 pkg prefix micro_ros_setup 2>/dev/null)"; then
    MICROROS_WS="$(cd "$(dirname "$(dirname "${ROS_PREFIX}")")" && pwd)"
  elif [ -d "${HOME}/ros2_ws" ]; then
    MICROROS_WS="${HOME}/ros2_ws"
  else
    echo "[microros] MICROROS_WS is not set and micro_ros_setup is not found in the current ROS environment." >&2
    echo "[microros] Please set MICROROS_WS to your ROS2 workspace root (the directory containing src/ and install/)." >&2
    exit 1
  fi
  set -u
  set -e
fi
OUT_DIR="${OUT_DIR:-${ROOT_DIR}/microros}"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/.microros_build}"
CLEAN="${CLEAN:-0}"

TOOLCHAIN_FILE="${TOOLCHAIN_FILE:-${ROOT_DIR}/cmake/microros_toolchain_stm32f446.cmake}"
COLCON_META="${COLCON_META:-${ROOT_DIR}/cmake/microros_colcon.meta}"

echo "[microros] ROS_DISTRO=${ROS_DISTRO}"
echo "[microros] MICROROS_WS=${MICROROS_WS}"
echo "[microros] BUILD_DIR=${BUILD_DIR}"
echo "[microros] OUT_DIR=${OUT_DIR}"

mkdir -p "${BUILD_DIR}"

if [ ! -f "${MICROROS_WS}/install/local_setup.bash" ]; then
  echo "[microros] micro_ros_setup workspace not found at ${MICROROS_WS}, creating..."
  mkdir -p "${MICROROS_WS}/src"
  if [ ! -d "${MICROROS_WS}/src/micro_ros_setup/.git" ]; then
    git clone -b "${ROS_DISTRO}" https://github.com/micro-ROS/micro_ros_setup.git "${MICROROS_WS}/src/micro_ros_setup"
  fi

  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  (cd "${MICROROS_WS}" && colcon build --symlink-install --packages-select micro_ros_setup)
fi

if [ ! -f "${TOOLCHAIN_FILE}" ]; then
  echo "[microros] TOOLCHAIN_FILE not found: ${TOOLCHAIN_FILE}" >&2
  exit 1
fi

if [ ! -f "${COLCON_META}" ]; then
  echo "[microros] COLCON_META not found: ${COLCON_META}" >&2
  exit 1
fi

mkdir -p "${BUILD_DIR}/bin"
cat > "${BUILD_DIR}/bin/rosdep" <<'EOF'
#!/usr/bin/env bash
echo "[microros] rosdep skipped (no root in this environment)" >&2
exit 0
EOF
chmod +x "${BUILD_DIR}/bin/rosdep"

pushd "${BUILD_DIR}" >/dev/null

if [ -d firmware ]; then
  if [ "${CLEAN}" = "1" ]; then
    echo "[microros] Removing existing firmware workspace: ${BUILD_DIR}/firmware"
    rm -rf firmware
  fi
fi

export PATH="${BUILD_DIR}/bin:${PATH}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${MICROROS_WS}/install/local_setup.bash"
set -u

if [ ! -d firmware ]; then
  ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
fi
ros2 run micro_ros_setup build_firmware.sh "${TOOLCHAIN_FILE}" "${COLCON_META}"

mkdir -p "${OUT_DIR}"
rm -rf "${OUT_DIR}/include" "${OUT_DIR}/libmicroros.a"
cp -R firmware/build/include "${OUT_DIR}/include"
cp firmware/build/libmicroros.a "${OUT_DIR}/libmicroros.a"

echo
echo "[microros] Built:"
echo "  ${OUT_DIR}/libmicroros.a"
echo "  ${OUT_DIR}/include"
echo
echo "[microros] Configure this project with:"
echo "  cmake --preset Debug -DMICROROS_INCLUDE_DIR=\"${OUT_DIR}/include\" -DMICROROS_LIB=\"${OUT_DIR}/libmicroros.a\""

popd >/dev/null

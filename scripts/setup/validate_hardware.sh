#!/usr/bin/env bash
set -Eeuo pipefail

ROAM_E_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROAM_E_PROJECT_ROOT="$(cd -- "${ROAM_E_SCRIPT_DIR}/../.." && pwd)"
ROAM_E_ROS_DISTRO="jazzy"
ROAM_E_RUNTIME_CHECKS=false
ROAM_E_FAILURES=0
ROAM_E_WARNINGS=0

usage() {
    cat <<'EOF'
Usage: scripts/setup/validate_hardware.sh [--runtime]

Validate a Roam-E installation without starting nodes or sending motor
commands. --runtime additionally checks for one message from each important
topic; use it only while a normal Roam-E launch is already running.
EOF
}

while (($# > 0)); do
    case "$1" in
        --runtime)
            ROAM_E_RUNTIME_CHECKS=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown argument: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

pass() {
    printf '[PASS] %s\n' "$*"
}

warn() {
    printf '[WARN] %s\n' "$*"
    ROAM_E_WARNINGS=$((ROAM_E_WARNINGS + 1))
}

fail() {
    printf '[FAIL] %s\n' "$*"
    ROAM_E_FAILURES=$((ROAM_E_FAILURES + 1))
}

check_path() {
    local ROAM_E_PATH_TO_CHECK="$1"
    local ROAM_E_DESCRIPTION="$2"
    if [[ -e "${ROAM_E_PATH_TO_CHECK}" ]]; then
        pass "${ROAM_E_DESCRIPTION}: ${ROAM_E_PATH_TO_CHECK}"
    else
        fail "${ROAM_E_DESCRIPTION} missing: ${ROAM_E_PATH_TO_CHECK}"
    fi
}

printf 'Roam-E hardware validation (read-only; motors remain untouched)\n\n'

ROAM_E_ARCH="$(dpkg --print-architecture 2>/dev/null || uname -m)"
if [[ "${ROAM_E_ARCH}" == arm64 || "${ROAM_E_ARCH}" == aarch64 ]]; then
    pass "64-bit ARM operating system"
else
    fail "expected 64-bit ARM, found ${ROAM_E_ARCH}"
fi

ROAM_E_MEMORY_KIB="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)"
if ((ROAM_E_MEMORY_KIB >= 7000000)); then
    pass "memory: $((ROAM_E_MEMORY_KIB / 1024)) MiB (Pi 5 8 GB class)"
else
    warn "memory: $((ROAM_E_MEMORY_KIB / 1024)) MiB; avoid full Nav2 on this host"
fi

if command -v vcgencmd >/dev/null 2>&1; then
    ROAM_E_THROTTLED="$(vcgencmd get_throttled 2>/dev/null || true)"
    if [[ "${ROAM_E_THROTTLED}" == 'throttled=0x0' ]]; then
        pass "no current or historical Pi throttling flags"
    elif [[ -n "${ROAM_E_THROTTLED}" ]]; then
        warn "Pi power/thermal flags: ${ROAM_E_THROTTLED}"
    fi
fi

check_path /dev/i2c-3 'BNO055 I2C bus'
check_path \
    /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
    'Arduino Nano serial link'
check_path \
    /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    'RPLidar serial link'
check_path \
    "${ROAM_E_PROJECT_ROOT}/config/bno055_calibration.json" \
    'BNO055 calibration file'
check_path \
    "${ROAM_E_PROJECT_ROOT}/maps/office_02.yaml" \
    'office_02 map metadata'
check_path \
    "${ROAM_E_PROJECT_ROOT}/maps/office_02.pgm" \
    'office_02 occupancy image'

if id -nG | tr ' ' '\n' | grep -qx dialout; then
    pass 'user belongs to dialout'
else
    fail 'user is not in dialout; log out after bootstrap or add the group'
fi
if id -nG | tr ' ' '\n' | grep -qx i2c; then
    pass 'user belongs to i2c'
else
    fail 'user is not in i2c; log out after bootstrap or add the group'
fi

if systemctl is-enabled --quiet roam-e-bno055-restore.service 2>/dev/null; then
    pass 'BNO055 calibration restore service is enabled'
else
    fail 'BNO055 calibration restore service is not enabled'
fi
if systemctl is-active --quiet roam-e-bno055-restore.service 2>/dev/null; then
    pass 'BNO055 calibration restore completed this boot'
else
    warn 'BNO055 calibration restore is not active; inspect its journal'
fi

if /usr/bin/python3 -c 'import serial, smbus2, rplidar' 2>/dev/null; then
    pass 'Python serial, SMBus, and RPLidar imports'
else
    fail 'one or more Python hardware libraries cannot be imported'
fi

if [[ -r "/opt/ros/${ROAM_E_ROS_DISTRO}/setup.bash" ]]; then
    pass "ROS 2 ${ROAM_E_ROS_DISTRO} installation"
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROAM_E_ROS_DISTRO}/setup.bash"
    set -u
else
    fail "ROS 2 ${ROAM_E_ROS_DISTRO} is not installed"
fi

if [[ -r "${ROAM_E_PROJECT_ROOT}/install/setup.bash" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${ROAM_E_PROJECT_ROOT}/install/setup.bash"
    set -u
    if ros2 pkg prefix roam_e >/dev/null 2>&1 && \
       ros2 pkg prefix roam_e_description >/dev/null 2>&1; then
        pass 'Roam-E ROS workspace overlay'
    else
        fail 'Roam-E packages are missing from the workspace overlay'
    fi
else
    fail 'workspace has not been built (install/setup.bash missing)'
fi

ROAM_E_PYTHON_SOURCE_OK=true
while IFS= read -r -d '' ROAM_E_PYTHON_FILE; do
    if ! python3 -c \
        'import ast, pathlib, sys; path = pathlib.Path(sys.argv[1]); ast.parse(path.read_text(encoding="utf-8"), filename=str(path))' \
        "${ROAM_E_PYTHON_FILE}"; then
        ROAM_E_PYTHON_SOURCE_OK=false
    fi
done < <(
    find \
        "${ROAM_E_PROJECT_ROOT}/src/roam_e" \
        "${ROAM_E_PROJECT_ROOT}/scripts/bno055" \
        -type f -name '*.py' -print0
)
if [[ "${ROAM_E_PYTHON_SOURCE_OK}" == true ]]; then
    pass 'Python source compilation'
else
    fail 'Python source compilation'
fi

if [[ "${ROAM_E_RUNTIME_CHECKS}" == true ]]; then
    printf '\nRuntime topic checks (no publishers are started):\n'
    ROAM_E_TOPICS=(
        /scan
        /imu/data
        /wheel/odometry
        /odometry/filtered
        /diagnostics
    )
    for ROAM_E_TOPIC in "${ROAM_E_TOPICS[@]}"; do
        if timeout 8 ros2 topic echo "${ROAM_E_TOPIC}" --once \
                >/dev/null 2>&1; then
            pass "message received on ${ROAM_E_TOPIC}"
        else
            fail "no message received on ${ROAM_E_TOPIC} within 8 seconds"
        fi
    done

    ROAM_E_TF_OUTPUT="$(
        timeout 8 ros2 run tf2_ros tf2_echo \
            base_footprint laser_frame 2>&1 || true
    )"
    if grep -q 'Translation:' <<< "${ROAM_E_TF_OUTPUT}"; then
        pass 'base_footprint -> laser_frame transform'
    else
        fail 'base_footprint -> laser_frame transform unavailable'
    fi
fi

printf '\nValidation summary: %d failure(s), %d warning(s).\n' \
    "${ROAM_E_FAILURES}" "${ROAM_E_WARNINGS}"
if ((ROAM_E_FAILURES > 0)); then
    exit 1
fi

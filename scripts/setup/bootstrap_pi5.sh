#!/usr/bin/env bash
set -Eeuo pipefail

ROAM_E_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROAM_E_PROJECT_ROOT="$(cd -- "${ROAM_E_SCRIPT_DIR}/../.." && pwd)"
ROAM_E_ROS_DISTRO="jazzy"
ROAM_E_EXPECTED_CODENAME="noble"
ROAM_E_UPGRADE_OS=false
ROAM_E_SKIP_ROSDEP_UPDATE=false

usage() {
    cat <<'EOF'
Usage: scripts/setup/bootstrap_pi5.sh [options]

Prepare a fresh 64-bit Ubuntu 24.04 Raspberry Pi 5 for Roam-E.

Options:
  --upgrade-os          Apply all available OS package upgrades.
  --skip-rosdep-update  Reuse the existing rosdep index (offline reruns).
  -h, --help            Show this help.

Run this script as the normal robot user from the cloned Roam-E repository.
It uses sudo for system changes, leaves motor output disabled, and is safe to
rerun. A reboot is required after the first run.
EOF
}

while (($# > 0)); do
    case "$1" in
        --upgrade-os)
            ROAM_E_UPGRADE_OS=true
            ;;
        --skip-rosdep-update)
            ROAM_E_SKIP_ROSDEP_UPDATE=true
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

if [[ "${EUID}" -eq 0 ]]; then
    printf 'Run this script as the normal robot user, not as root.\n' >&2
    exit 1
fi

if [[ ! -r /etc/os-release ]]; then
    printf 'Cannot identify the operating system.\n' >&2
    exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
ROAM_E_OS_ID="${ID:-unknown}"
ROAM_E_OS_CODENAME="${VERSION_CODENAME:-unknown}"
ROAM_E_ARCH="$(dpkg --print-architecture)"

if [[ "${ROAM_E_OS_ID}" != ubuntu || \
      "${ROAM_E_OS_CODENAME}" != "${ROAM_E_EXPECTED_CODENAME}" ]]; then
    printf 'Expected Ubuntu 24.04 (%s), found %s (%s).\n' \
        "${ROAM_E_EXPECTED_CODENAME}" "${ROAM_E_OS_ID}" \
        "${ROAM_E_OS_CODENAME}" >&2
    exit 1
fi

if [[ "${ROAM_E_ARCH}" != arm64 ]]; then
    printf 'Expected the 64-bit arm64 image, found %s.\n' \
        "${ROAM_E_ARCH}" >&2
    exit 1
fi

if [[ ! -f "${ROAM_E_PROJECT_ROOT}/src/roam_e/package.xml" ]]; then
    printf 'Run this script from inside a complete Roam-E checkout.\n' >&2
    exit 1
fi

printf 'Bootstrapping %s on %s (%s).\n' \
    "${ROAM_E_PROJECT_ROOT}" "${ROAM_E_OS_CODENAME}" "${ROAM_E_ARCH}"
sudo -v

# Some Raspberry Pi Ubuntu Server images ship with the Noble release and
# security pockets enabled, but omit noble-updates. Mixing base development
# packages with already-updated runtime libraries then makes ROS dependencies
# such as liblz4-dev and libzstd-dev impossible to install. Enable the updates
# pocket before the first package operation.
ROAM_E_UBUNTU_SOURCES=/etc/apt/sources.list.d/ubuntu.sources
noble_updates_configured() {
    grep -RhEq \
        '(^Suites:.*noble-updates)|(^[[:space:]]*deb .* noble-updates([[:space:]]|$))' \
        /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null
}

if ! noble_updates_configured; then
    if [[ ! -f "${ROAM_E_UBUNTU_SOURCES}" ]]; then
        printf 'Ubuntu package sources do not enable noble-updates, and %s is missing.\n' \
            "${ROAM_E_UBUNTU_SOURCES}" >&2
        exit 1
    fi

    printf 'Enabling the Ubuntu noble-updates package pocket.\n'
    if [[ ! -f "${ROAM_E_UBUNTU_SOURCES}.roam-e-before-bootstrap" ]]; then
        sudo cp --preserve=mode,timestamps \
            "${ROAM_E_UBUNTU_SOURCES}" \
            "${ROAM_E_UBUNTU_SOURCES}.roam-e-before-bootstrap"
    fi
    sudo sed -i -E \
        '/^Suites:[[:space:]]+noble([[:space:]]|$)/ { /noble-updates/! s/$/ noble-updates/; }' \
        "${ROAM_E_UBUNTU_SOURCES}"

    if ! noble_updates_configured; then
        printf 'Could not enable noble-updates in %s.\n' \
            "${ROAM_E_UBUNTU_SOURCES}" >&2
        exit 1
    fi
fi

sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    locales \
    software-properties-common \
    curl \
    ca-certificates \
    git \
    build-essential \
    python3-pip \
    python3-venv \
    python3-serial \
    python3-smbus2 \
    i2c-tools

sudo add-apt-repository -y universe
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

if [[ "${ROAM_E_UPGRADE_OS}" == true ]]; then
    sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
fi

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null \
        | grep -q 'ok installed'; then
    printf 'Installing the official ROS 2 apt source configuration.\n'
    ROAM_E_APT_SOURCE_VERSION="$(
        curl -fsSL \
            https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
            | sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p' \
            | head -n 1
    )"

    if [[ -z "${ROAM_E_APT_SOURCE_VERSION}" ]]; then
        printf 'Could not determine the latest ros-apt-source release.\n' >&2
        exit 1
    fi

    ROAM_E_APT_SOURCE_DEB="$(mktemp /tmp/ros2-apt-source.XXXXXX.deb)"
    cleanup_apt_source() {
        rm -f -- "${ROAM_E_APT_SOURCE_DEB}"
    }
    trap cleanup_apt_source EXIT

    curl -fL \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROAM_E_APT_SOURCE_VERSION}/ros2-apt-source_${ROAM_E_APT_SOURCE_VERSION}.${ROAM_E_OS_CODENAME}_all.deb" \
        -o "${ROAM_E_APT_SOURCE_DEB}"
    sudo dpkg -i "${ROAM_E_APT_SOURCE_DEB}"
    cleanup_apt_source
    trap - EXIT
fi

sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    "ros-${ROAM_E_ROS_DISTRO}-ros-base" \
    ros-dev-tools

# ROS-generated setup files read optional variables which may be unset.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROAM_E_ROS_DISTRO}/setup.bash"
set -u

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
if [[ "${ROAM_E_SKIP_ROSDEP_UPDATE}" == false ]]; then
    rosdep update --rosdistro "${ROAM_E_ROS_DISTRO}"
fi

rosdep install \
    --from-paths "${ROAM_E_PROJECT_ROOT}/src" \
    --ignore-src \
    --rosdistro "${ROAM_E_ROS_DISTRO}" \
    -r -y

# The RPLIDAR driver is not packaged for ROS or Ubuntu. ROS Python executables
# use /usr/bin/python3 and therefore see this user's site directory; Ubuntu
# 24.04 requires the explicit PEP 668 override.
python3 -m pip install \
    --user \
    --break-system-packages \
    'rplidar==0.9.2'

# The calibration utilities intentionally use an isolated, tiny environment.
if [[ ! -x "${ROAM_E_PROJECT_ROOT}/.venv/bin/python" ]]; then
    python3 -m venv "${ROAM_E_PROJECT_ROOT}/.venv"
fi
"${ROAM_E_PROJECT_ROOT}/.venv/bin/python" -m pip install \
    --disable-pip-version-check \
    'pyserial==3.5' \
    'smbus2==0.6.1'

for ROAM_E_GROUP in dialout i2c gpio video; do
    if getent group "${ROAM_E_GROUP}" >/dev/null; then
        sudo usermod -aG "${ROAM_E_GROUP}" "${USER}"
    fi
done

# The Astra Pro depth interface is vendor-specific rather than video4linux.
# Restrict access to members of the video group instead of making the USB
# device writable by every local user.
ROAM_E_ORBBEC_UDEV_RULE=/etc/udev/rules.d/56-orbbec.rules
ROAM_E_ORBBEC_UDEV_ENTRY='SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0403", MODE:="0660", GROUP:="video"'
if [[ ! -f "${ROAM_E_ORBBEC_UDEV_RULE}" ]] || \
        ! grep -qxF "${ROAM_E_ORBBEC_UDEV_ENTRY}" \
            "${ROAM_E_ORBBEC_UDEV_RULE}"; then
    printf '%s\n' "${ROAM_E_ORBBEC_UDEV_ENTRY}" \
        | sudo tee "${ROAM_E_ORBBEC_UDEV_RULE}" >/dev/null
    sudo udevadm control --reload-rules
fi

# Direct libuvc RGB capture needs access to the distinct 0502 USB interface.
ROAM_E_RGB_UDEV_RULE=/etc/udev/rules.d/57-roam-astra-rgb.rules
ROAM_E_RGB_UDEV_ENTRY='SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0502", MODE:="0660", GROUP:="video"'
if [[ ! -f "${ROAM_E_RGB_UDEV_RULE}" ]]; then
    printf '%s\n' "${ROAM_E_RGB_UDEV_ENTRY}" \
        | sudo tee "${ROAM_E_RGB_UDEV_RULE}" >/dev/null
    sudo udevadm control --reload-rules
elif ! grep -qxF "${ROAM_E_RGB_UDEV_ENTRY}" "${ROAM_E_RGB_UDEV_RULE}"; then
    printf 'Existing RGB USB rule differs; review %s manually.\n' "${ROAM_E_RGB_UDEV_RULE}"
fi

# Orbbec's legacy ARM64 SDK does not provide the SONAME symlink expected by
# the ROS openni2 wrapper. Add it when the separately supplied SDK is present.
ROAM_E_OPENNI2_SDK_DIR="${HOME}/OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin"
if [[ -f "${ROAM_E_OPENNI2_SDK_DIR}/libOpenNI2.so" ]]; then
    ln -sfn libOpenNI2.so "${ROAM_E_OPENNI2_SDK_DIR}/libOpenNI2.so.0"
else
    printf 'Astra depth SDK not found at %s; camera setup was skipped.\n' \
        "${ROAM_E_OPENNI2_SDK_DIR}"
fi

ROAM_E_BOOT_CONFIG=/boot/firmware/config.txt
if [[ ! -f "${ROAM_E_BOOT_CONFIG}" ]]; then
    printf 'Expected Raspberry Pi boot configuration at %s.\n' \
        "${ROAM_E_BOOT_CONFIG}" >&2
    exit 1
fi

if [[ ! -f "${ROAM_E_BOOT_CONFIG}.roam-e-before-bootstrap" ]]; then
    sudo cp --preserve=mode,timestamps \
        "${ROAM_E_BOOT_CONFIG}" \
        "${ROAM_E_BOOT_CONFIG}.roam-e-before-bootstrap"
fi

ROAM_E_BOOT_SETTINGS=(
    'dtparam=i2c_arm=on'
    'enable_uart=1'
    'dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=17,i2c_gpio_scl=27,i2c_gpio_delay_us=2'
)
for ROAM_E_BOOT_SETTING in "${ROAM_E_BOOT_SETTINGS[@]}"; do
    if ! grep -qxF "${ROAM_E_BOOT_SETTING}" "${ROAM_E_BOOT_CONFIG}"; then
        printf '%s\n' "${ROAM_E_BOOT_SETTING}" \
            | sudo tee -a "${ROAM_E_BOOT_CONFIG}" >/dev/null
    fi
done

cd "${ROAM_E_PROJECT_ROOT}"
colcon build --symlink-install

"${ROAM_E_PROJECT_ROOT}/scripts/setup/install_bno055_restore_service.sh"

printf '\nBootstrap completed without arming or commanding the motors.\n'
printf 'Log out and back in (for group membership), then reboot.\n'
printf 'After reboot run: scripts/setup/validate_hardware.sh\n'

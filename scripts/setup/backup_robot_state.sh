#!/usr/bin/env bash
set -Eeuo pipefail

ROAM_E_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROAM_E_PROJECT_ROOT="$(cd -- "${ROAM_E_SCRIPT_DIR}/../.." && pwd)"
ROAM_E_BACKUP_PARENT="${HOME}/roam-e-backups"

usage() {
    cat <<'EOF'
Usage: scripts/setup/backup_robot_state.sh [destination-directory]

Create a migration archive containing maps, calibration, firmware, systemd
and Raspberry Pi hardware configuration, plus a software inventory. Network
credentials, SSH private keys, build output, and ROS logs are never included.
EOF
}

if (($# > 1)); then
    usage >&2
    exit 2
fi
if (($# == 1)); then
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        *)
            ROAM_E_BACKUP_PARENT="$1"
            ;;
    esac
fi

if [[ ! -d "${ROAM_E_PROJECT_ROOT}/.git" ]]; then
    printf 'Not running from a Roam-E Git checkout: %s\n' \
        "${ROAM_E_PROJECT_ROOT}" >&2
    exit 1
fi

mkdir -p "${ROAM_E_BACKUP_PARENT}"
ROAM_E_TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ROAM_E_HOSTNAME="$(hostname -s | tr -cd '[:alnum:]_.-')"
ROAM_E_ARCHIVE_BASENAME="roam-e-state-${ROAM_E_HOSTNAME}-${ROAM_E_TIMESTAMP}"
ROAM_E_STAGING_DIR="$(mktemp -d /tmp/roam-e-backup.XXXXXX)"
ROAM_E_ARCHIVE_PATH="${ROAM_E_BACKUP_PARENT}/${ROAM_E_ARCHIVE_BASENAME}.tar.gz"

cleanup() {
    if [[ "${ROAM_E_STAGING_DIR}" == /tmp/roam-e-backup.* && \
          -d "${ROAM_E_STAGING_DIR}" ]]; then
        rm -rf -- "${ROAM_E_STAGING_DIR}"
    fi
}
trap cleanup EXIT

mkdir -p \
    "${ROAM_E_STAGING_DIR}/project" \
    "${ROAM_E_STAGING_DIR}/system"

cp -a \
    "${ROAM_E_PROJECT_ROOT}/config" \
    "${ROAM_E_PROJECT_ROOT}/maps" \
    "${ROAM_E_PROJECT_ROOT}/firmware" \
    "${ROAM_E_STAGING_DIR}/project/"

git -C "${ROAM_E_PROJECT_ROOT}" rev-parse HEAD \
    > "${ROAM_E_STAGING_DIR}/project/git-commit.txt"
git -C "${ROAM_E_PROJECT_ROOT}" status --short \
    > "${ROAM_E_STAGING_DIR}/project/git-status.txt"
git -C "${ROAM_E_PROJECT_ROOT}" remote get-url origin \
    > "${ROAM_E_STAGING_DIR}/project/git-origin.txt" 2>/dev/null || true

cp -a /etc/os-release "${ROAM_E_STAGING_DIR}/system/"
if [[ -r /boot/firmware/config.txt ]]; then
    cp -a /boot/firmware/config.txt \
        "${ROAM_E_STAGING_DIR}/system/boot-config.txt"
fi
if [[ -r /etc/systemd/system/roam-e-bno055-restore.service ]]; then
    cp -a /etc/systemd/system/roam-e-bno055-restore.service \
        "${ROAM_E_STAGING_DIR}/system/"
fi
if [[ -d /etc/udev/rules.d ]]; then
    cp -a /etc/udev/rules.d "${ROAM_E_STAGING_DIR}/system/"
fi

dpkg-query -W -f='${binary:Package}\t${Version}\n' \
    > "${ROAM_E_STAGING_DIR}/system/dpkg-packages.tsv"
apt-mark showmanual \
    > "${ROAM_E_STAGING_DIR}/system/apt-manual.txt"
python3 -m pip freeze --user \
    > "${ROAM_E_STAGING_DIR}/system/python-user-packages.txt" 2>/dev/null \
    || true
id > "${ROAM_E_STAGING_DIR}/system/user-groups.txt"
uname -a > "${ROAM_E_STAGING_DIR}/system/uname.txt"
ls -l /dev/serial/by-id \
    > "${ROAM_E_STAGING_DIR}/system/serial-by-id.txt" 2>&1 || true
ls -l /dev/i2c-* \
    > "${ROAM_E_STAGING_DIR}/system/i2c-devices.txt" 2>&1 || true

(
    cd "${ROAM_E_STAGING_DIR}"
    find . -type f ! -name SHA256SUMS -print0 \
        | sort -z \
        | xargs -0 sha256sum \
        > SHA256SUMS
)

tar -C "${ROAM_E_STAGING_DIR}" -czf "${ROAM_E_ARCHIVE_PATH}" .
sha256sum "${ROAM_E_ARCHIVE_PATH}" \
    > "${ROAM_E_ARCHIVE_PATH}.sha256"

printf 'Migration backup created:\n  %s\n  %s.sha256\n' \
    "${ROAM_E_ARCHIVE_PATH}" "${ROAM_E_ARCHIVE_PATH}"

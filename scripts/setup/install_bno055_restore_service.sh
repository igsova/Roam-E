#!/usr/bin/env bash
set -Eeuo pipefail

ROAM_E_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROAM_E_PROJECT_ROOT="$(cd -- "${ROAM_E_SCRIPT_DIR}/../.." && pwd)"
ROAM_E_TEMPLATE="${ROAM_E_PROJECT_ROOT}/config/systemd/roam-e-bno055-restore.service.in"
ROAM_E_SERVICE_NAME="roam-e-bno055-restore.service"
ROAM_E_SERVICE_PATH="/etc/systemd/system/${ROAM_E_SERVICE_NAME}"
ROAM_E_START_SERVICE=false

usage() {
    cat <<'EOF'
Usage: scripts/setup/install_bno055_restore_service.sh [--start]

Install and enable the BNO055 calibration restore service using the absolute
path of this Roam-E checkout. --start also runs the service immediately;
without it, the service first runs at the next boot.
EOF
}

while (($# > 0)); do
    case "$1" in
        --start)
            ROAM_E_START_SERVICE=true
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

if [[ "${ROAM_E_PROJECT_ROOT}" =~ [[:space:]] ]]; then
    printf 'The project path cannot contain whitespace for this systemd unit: %s\n' \
        "${ROAM_E_PROJECT_ROOT}" >&2
    exit 1
fi

for ROAM_E_REQUIRED_PATH in \
    "${ROAM_E_TEMPLATE}" \
    "${ROAM_E_PROJECT_ROOT}/.venv/bin/python" \
    "${ROAM_E_PROJECT_ROOT}/scripts/bno055/restore.py" \
    "${ROAM_E_PROJECT_ROOT}/config/bno055_calibration.json"
do
    if [[ ! -e "${ROAM_E_REQUIRED_PATH}" ]]; then
        printf 'Required path is missing: %s\n' "${ROAM_E_REQUIRED_PATH}" >&2
        exit 1
    fi
done

ROAM_E_TEMP_UNIT="$(mktemp /tmp/roam-e-bno055-restore.XXXXXX)"
cleanup() {
    rm -f -- "${ROAM_E_TEMP_UNIT}"
}
trap cleanup EXIT

sed "s|@PROJECT_ROOT@|${ROAM_E_PROJECT_ROOT}|g" \
    "${ROAM_E_TEMPLATE}" > "${ROAM_E_TEMP_UNIT}"

sudo install -o root -g root -m 0644 \
    "${ROAM_E_TEMP_UNIT}" "${ROAM_E_SERVICE_PATH}"
sudo systemctl daemon-reload
sudo systemctl enable "${ROAM_E_SERVICE_NAME}"

if [[ "${ROAM_E_START_SERVICE}" == true ]]; then
    sudo systemctl restart "${ROAM_E_SERVICE_NAME}"
    sudo systemctl --no-pager --full status "${ROAM_E_SERVICE_NAME}"
fi

printf 'Installed and enabled %s\n' "${ROAM_E_SERVICE_PATH}"

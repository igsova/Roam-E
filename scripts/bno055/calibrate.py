#!/usr/bin/env python3
"""Calibrate the BNO055 interactively and save its calibration profile.

Puts the sensor in NDOF fusion mode, prints the gyro, accelerometer and
magnetometer calibration status while you move the robot, and writes the 22
offset/radius registers plus the axis remap to config/bno055_calibration.json.
restore.py loads that file at boot. Run it with sudo on the Pi (I2C bus 3).
"""

import datetime
import hashlib
import json
import os
import time
from pathlib import Path

from smbus2 import SMBus

BUS = 3
ADDRESS = 0x29

CHIP_ID = 0x00
PAGE_ID = 0x07
CALIB_STAT = 0x35
SYS_STATUS = 0x39
SYS_ERROR = 0x3A
UNIT_SEL = 0x3B
OPR_MODE = 0x3D
PWR_MODE = 0x3E
SYS_TRIGGER = 0x3F
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN = 0x42

CALIBRATION_START = 0x55
CALIBRATION_LENGTH = 22

CONFIG_MODE = 0x00
NDOF_MODE = 0x0C

OUTPUT_PATH = (
    Path(__file__).resolve().parents[2] / "config" / "bno055_calibration.json"
)

bus = None
configuration_active = False

def read_byte(register, attempts=5):
    last_error = None

    for _ in range(attempts):
        try:
            return bus.read_byte_data(ADDRESS, register)
        except OSError as error:
            last_error = error
            time.sleep(0.02)

    raise last_error

def write_byte(register, value, attempts=5):
    last_error = None

    for _ in range(attempts):
        try:
            bus.write_byte_data(ADDRESS, register, value)
            return
        except OSError as error:
            last_error = error
            time.sleep(0.02)

    raise last_error

def read_block(register, length, attempts=5):
    last_error = None

    for _ in range(attempts):
        try:
            return bus.read_i2c_block_data(
                ADDRESS,
                register,
                length
            )
        except OSError as error:
            last_error = error
            time.sleep(0.02)

    raise last_error

def decode_calibration(value):
    return {
        "sys": (value >> 6) & 3,
        "gyro": (value >> 4) & 3,
        "accel": (value >> 2) & 3,
        "mag": value & 3,
    }


def main():
    global bus, configuration_active
    bus = SMBus(BUS)
    try:
        detected_id = read_byte(CHIP_ID)

        if detected_id != 0xA0:
            raise RuntimeError(
                f"Unexpected chip ID: 0x{detected_id:02X}"
            )

        print("Resetting previous calibration...")

        write_byte(OPR_MODE, CONFIG_MODE)
        configuration_active = True
        time.sleep(0.03)

        try:
            bus.write_byte_data(
                ADDRESS,
                SYS_TRIGGER,
                0x20
            )
        except OSError:
            pass

        time.sleep(0.8)

        deadline = time.monotonic() + 3.0

        while True:
            try:
                detected_id = bus.read_byte_data(
                    ADDRESS,
                    CHIP_ID
                )

                if detected_id == 0xA0:
                    break
            except OSError:
                pass

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "BNO055 did not return after reset"
                )

            time.sleep(0.05)

        write_byte(PAGE_ID, 0x00)
        write_byte(PWR_MODE, 0x00)
        time.sleep(0.02)

        write_byte(UNIT_SEL, 0x00)
        write_byte(SYS_TRIGGER, 0x00)
        time.sleep(0.02)

        write_byte(OPR_MODE, NDOF_MODE)
        configuration_active = False
        time.sleep(1.0)

        print("Fresh NDOF calibration started.")

        stable_samples = 0
        calibration = None

        while stable_samples < 5:
            calibration = decode_calibration(
                read_byte(CALIB_STAT)
            )

            status = read_byte(SYS_STATUS)
            system_error = read_byte(SYS_ERROR)

            physical_ready = (
                calibration["gyro"] == 3
                and calibration["accel"] == 3
                and calibration["mag"] == 3
            )

            healthy = (
                status == 5
                and system_error == 0
            )

            if physical_ready and healthy:
                stable_samples += 1
            else:
                stable_samples = 0

            print("\033[2J\033[H", end="")
            print("BNO055 RECALIBRATION")
            print("Press Ctrl+C to cancel without saving")
            print()
            print(
                f"SYS={calibration['sys']}  "
                f"GYR={calibration['gyro']}  "
                f"ACC={calibration['accel']}  "
                f"MAG={calibration['mag']}"
            )
            print(
                f"Status={status}  "
                f"Error={system_error}  "
                f"Stable={stable_samples}/5"
            )
            print()
            print("GYR: Keep the robot completely still.")
            print(
                "ACC: Hold level, left, right, nose-up, "
                "nose-down and inverted."
            )
            print(
                "MAG: Slowly rotate through yaw, pitch and roll."
            )
            print()
            print(
                "Pause for several seconds in each orientation. "
                "Keep motors stopped."
            )

            if not healthy:
                raise RuntimeError(
                    f"Fusion unhealthy: "
                    f"status={status}, error={system_error}"
                )

            time.sleep(0.25)

        print("\nCalibration complete. Reading offsets...")

        write_byte(OPR_MODE, CONFIG_MODE)
        configuration_active = True
        time.sleep(0.04)

        write_byte(PAGE_ID, 0x00)

        unit_selection = read_byte(UNIT_SEL)
        axis_map_config = read_byte(AXIS_MAP_CONFIG)
        axis_map_sign = read_byte(AXIS_MAP_SIGN)

        calibration_data = None

        for _ in range(5):
            first = read_block(
                CALIBRATION_START,
                CALIBRATION_LENGTH
            )

            time.sleep(0.05)

            second = read_block(
                CALIBRATION_START,
                CALIBRATION_LENGTH
            )

            if first == second:
                calibration_data = first
                break

        if calibration_data is None:
            raise RuntimeError(
                "Calibration offset reads did not match"
            )

        write_byte(OPR_MODE, NDOF_MODE)
        configuration_active = False
        time.sleep(0.05)

        raw_bytes = bytes(calibration_data)

        document = {
            "schema_version": 1,
            "sensor": "Bosch BNO055",
            "saved_at_utc": (
                datetime.datetime.now(datetime.timezone.utc)
                .isoformat()
            ),
            "i2c_bus": BUS,
            "i2c_address": f"0x{ADDRESS:02X}",
            "chip_id": f"0x{detected_id:02X}",
            "operation_mode": f"0x{NDOF_MODE:02X}",
            "unit_selection": f"0x{unit_selection:02X}",
            "axis_map_config": f"0x{axis_map_config:02X}",
            "axis_map_sign": f"0x{axis_map_sign:02X}",
            "calibration_status_when_saved": calibration,
            "acceptance_rule": (
                "GYR=3, ACC=3 and MAG=3 for five samples; "
                "fusion status=5 and error=0"
            ),
            "register_start": f"0x{CALIBRATION_START:02X}",
            "register_length": CALIBRATION_LENGTH,
            "data": calibration_data,
            "data_hex": raw_bytes.hex(),
            "sha256": hashlib.sha256(raw_bytes).hexdigest(),
        }

        temporary_path = OUTPUT_PATH.with_suffix(".json.tmp")

        with temporary_path.open("w", encoding="utf-8") as file:
            json.dump(document, file, indent=2)
            file.write("\n")

        os.replace(temporary_path, OUTPUT_PATH)

        sudo_uid = os.environ.get("SUDO_UID")
        sudo_gid = os.environ.get("SUDO_GID")

        if sudo_uid is not None and sudo_gid is not None:
            os.chown(
                OUTPUT_PATH,
                int(sudo_uid),
                int(sudo_gid)
            )

        os.chmod(OUTPUT_PATH, 0o600)

        print()
        print("NEW CALIBRATION SAVED SUCCESSFULLY")
        print(f"File: {OUTPUT_PATH}")
        print(f"Data: {raw_bytes.hex()}")
        print(
            "SHA-256: "
            f"{document['sha256']}"
        )

    except KeyboardInterrupt:
        print("\nCalibration cancelled; nothing was saved.")

    finally:
        if configuration_active:
            try:
                write_byte(OPR_MODE, NDOF_MODE)
                time.sleep(0.05)
            except Exception:
                pass

        bus.close()


if __name__ == '__main__':
    main()

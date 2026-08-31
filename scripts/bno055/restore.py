#!/home/robot/Roam-E/.venv/bin/python

import hashlib
import json
import time
from pathlib import Path

from smbus2 import SMBus


BUS_NUMBER = 3
ADDRESS = 0x29

CALIBRATION_FILE = Path(
    "/home/robot/Roam-E/config/bno055_calibration.json"
)

CHIP_ID = 0x00
EXPECTED_CHIP_ID = 0xA0

PAGE_ID = 0x07
EULER_HEADING_LSB = 0x1A
CALIB_STAT = 0x35
UNIT_SEL = 0x3B
OPR_MODE = 0x3D
PWR_MODE = 0x3E
SYS_TRIGGER = 0x3F
SYS_STATUS = 0x39
SYS_ERROR = 0x3A
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN = 0x42

CALIBRATION_START = 0x55
CALIBRATION_LENGTH = 22

CONFIG_MODE = 0x00
NDOF_MODE = 0x0C


def parse_number(value, default):
    if value is None:
        return default

    if isinstance(value, str):
        return int(value, 0)

    return int(value)


def retry(operation, description, attempts=5, delay=0.03):
    last_error = None

    for attempt in range(attempts):
        try:
            return operation()
        except OSError as error:
            last_error = error

            if attempt + 1 < attempts:
                time.sleep(delay)

    raise RuntimeError(
        f"{description} failed after {attempts} attempts: {last_error}"
    )


def read_register(bus, register):
    return retry(
        lambda: bus.read_byte_data(ADDRESS, register),
        f"reading register 0x{register:02X}",
    )


def write_register(bus, register, value):
    retry(
        lambda: bus.write_byte_data(ADDRESS, register, value),
        f"writing register 0x{register:02X}",
    )


def read_block(bus, register, length):
    return bytes(
        retry(
            lambda: bus.read_i2c_block_data(
                ADDRESS,
                register,
                length,
            ),
            f"reading {length} bytes from register 0x{register:02X}",
        )
    )


def write_block(bus, register, data):
    values = list(data)

    retry(
        lambda: bus.write_i2c_block_data(
            ADDRESS,
            register,
            values,
        ),
        f"writing {len(values)} bytes to register 0x{register:02X}",
    )


def unpack_calibration(calibration):
    system = (calibration >> 6) & 0x03
    gyroscope = (calibration >> 4) & 0x03
    accelerometer = (calibration >> 2) & 0x03
    magnetometer = calibration & 0x03

    return system, gyroscope, accelerometer, magnetometer


def signed_16(low, high):
    value = low | (high << 8)

    if value & 0x8000:
        value -= 0x10000

    return value


def read_orientation(bus):
    data = read_block(bus, EULER_HEADING_LSB, 6)

    heading_raw = data[0] | (data[1] << 8)
    roll_raw = signed_16(data[2], data[3])
    pitch_raw = signed_16(data[4], data[5])

    return (
        heading_raw / 16.0,
        roll_raw / 16.0,
        pitch_raw / 16.0,
    )


def load_profile():
    if not CALIBRATION_FILE.is_file():
        raise RuntimeError(
            f"Calibration file does not exist: {CALIBRATION_FILE}"
        )

    with CALIBRATION_FILE.open("r", encoding="utf-8") as file:
        profile = json.load(file)

    data_value = profile.get("data_hex")

    if data_value is None:
        data_value = profile.get("data")

    if isinstance(data_value, list):
        calibration_data = bytes(data_value)
    elif isinstance(data_value, str):
        cleaned = "".join(data_value.split())

        if cleaned.startswith("0x"):
            cleaned = cleaned[2:]

        try:
            calibration_data = bytes.fromhex(cleaned)
        except ValueError as error:
            raise RuntimeError(
                "Calibration data is not valid hexadecimal"
            ) from error
    else:
        raise RuntimeError(
            "Calibration file has no usable data_hex or data field"
        )

    if len(calibration_data) != CALIBRATION_LENGTH:
        raise RuntimeError(
            "Calibration data has the wrong length: "
            f"expected {CALIBRATION_LENGTH}, "
            f"received {len(calibration_data)}"
        )

    calculated_sha = hashlib.sha256(calibration_data).hexdigest()
    saved_sha = profile.get("sha256")

    if not isinstance(saved_sha, str):
        raise RuntimeError(
            "Calibration file does not contain a SHA-256 checksum"
        )

    if calculated_sha.lower() != saved_sha.lower():
        raise RuntimeError(
            "Calibration checksum mismatch. "
            "The file may be damaged or incomplete."
        )

    saved_bus = parse_number(
        profile.get("bus", profile.get("i2c_bus")),
        BUS_NUMBER,
    )
    saved_address = parse_number(
        profile.get("address"),
        ADDRESS,
    )

    if saved_bus != BUS_NUMBER:
        raise RuntimeError(
            f"Profile expects I2C bus {saved_bus}, "
            f"but this script uses bus {BUS_NUMBER}"
        )

    if saved_address != ADDRESS:
        raise RuntimeError(
            f"Profile expects address 0x{saved_address:02X}, "
            f"but this script uses 0x{ADDRESS:02X}"
        )

    axis_config = parse_number(
        profile.get("axis_map_config"),
        0x24,
    )
    axis_sign = parse_number(
        profile.get("axis_map_sign"),
        0x00,
    )

    return (
        calibration_data,
        calculated_sha,
        axis_config,
        axis_sign,
    )


def wait_for_sensor(bus, timeout=4.0):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        try:
            if bus.read_byte_data(ADDRESS, CHIP_ID) == EXPECTED_CHIP_ID:
                return
        except OSError:
            pass

        time.sleep(0.05)

    raise RuntimeError(
        "BNO055 did not return after its software reset"
    )


def main():
    (
        calibration_data,
        saved_sha,
        axis_config,
        axis_sign,
    ) = load_profile()

    print("Calibration file validated.")
    print(f"File: {CALIBRATION_FILE}")
    print(f"SHA-256: {saved_sha}")
    print()

    bus = SMBus(BUS_NUMBER)
    configuration_active = False

    try:
        chip_id = read_register(bus, CHIP_ID)

        if chip_id != EXPECTED_CHIP_ID:
            raise RuntimeError(
                f"Unexpected BNO055 chip ID: 0x{chip_id:02X}"
            )

        print("Soft-resetting BNO055...")

        try:
            bus.write_byte_data(ADDRESS, SYS_TRIGGER, 0x20)
        except OSError:
            # The sensor may stop acknowledging immediately during reset.
            pass

        time.sleep(0.8)
        wait_for_sensor(bus)

        print("BNO055 returned after reset.")

        write_register(bus, PAGE_ID, 0x00)
        write_register(bus, OPR_MODE, CONFIG_MODE)
        configuration_active = True
        time.sleep(0.03)

        write_register(bus, PWR_MODE, 0x00)
        time.sleep(0.02)

        write_register(bus, PAGE_ID, 0x00)
        write_register(bus, UNIT_SEL, 0x00)
        write_register(bus, AXIS_MAP_CONFIG, axis_config)
        write_register(bus, AXIS_MAP_SIGN, axis_sign)

        # Use the internal clock.
        write_register(bus, SYS_TRIGGER, 0x00)
        time.sleep(0.03)

        print("Writing saved calibration offsets...")

        write_block(
            bus,
            CALIBRATION_START,
            calibration_data,
        )
        time.sleep(0.05)

        readback = read_block(
            bus,
            CALIBRATION_START,
            CALIBRATION_LENGTH,
        )

        if readback != calibration_data:
            raise RuntimeError(
                "Calibration register readback does not match "
                "the saved data"
            )

        readback_sha = hashlib.sha256(readback).hexdigest()

        if readback_sha != saved_sha:
            raise RuntimeError(
                "Calibration register checksum verification failed"
            )

        print("Calibration register readback verified.")

        write_register(bus, OPR_MODE, NDOF_MODE)
        configuration_active = False
        time.sleep(1.0)

        mode = read_register(bus, OPR_MODE)
        status = read_register(bus, SYS_STATUS)
        error = read_register(bus, SYS_ERROR)

        if mode != NDOF_MODE:
            raise RuntimeError(
                f"BNO055 did not enter NDOF mode: 0x{mode:02X}"
            )

        if status != 0x05:
            raise RuntimeError(
                f"Unexpected fusion status: 0x{status:02X}"
            )

        if error != 0x00:
            raise RuntimeError(
                f"BNO055 reported system error: 0x{error:02X}"
            )

        print()
        print("Waiting for restored calibration to become active...")

        ready = False
        deadline = time.monotonic() + 5.0
        last_calibration = 0

        while time.monotonic() < deadline:
            last_calibration = read_register(bus, CALIB_STAT)
            system, gyro, accel, mag = unpack_calibration(
                last_calibration
            )

            print(
                f"SYS={system} GYR={gyro} "
                f"ACC={accel} MAG={mag}"
            )

            if gyro == 3 and accel == 3 and mag == 3:
                ready = True
                break

            time.sleep(0.25)

        if not ready:
            raise RuntimeError(
                "Offsets were written correctly, but the physical "
                "sensor calibration did not reach GYR=3 ACC=3 MAG=3"
            )

        print()
        print("CALIBRATION RESTORE VERIFIED")
        print(f"Mode:   0x{mode:02X}")
        print(f"Status: 0x{status:02X}")
        print(f"Error:  0x{error:02X}")
        print(f"SHA-256: {readback_sha}")
        print()
        print("Five orientation samples:")

        for sample in range(1, 6):
            heading, roll, pitch = read_orientation(bus)
            calibration = read_register(bus, CALIB_STAT)
            system, gyro, accel, mag = unpack_calibration(
                calibration
            )

            print(
                f"{sample:02d}: "
                f"heading={heading:8.2f}° "
                f"roll={roll:8.2f}° "
                f"pitch={pitch:8.2f}° | "
                f"SYS={system} GYR={gyro} "
                f"ACC={accel} MAG={mag}"
            )

            time.sleep(0.2)

    finally:
        if configuration_active:
            try:
                bus.write_byte_data(
                    ADDRESS,
                    OPR_MODE,
                    NDOF_MODE,
                )
            except OSError:
                pass

        bus.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled.")
        raise SystemExit(130)
    except Exception as error:
        print()
        print(f"RESTORE FAILED: {error}")
        raise SystemExit(1)


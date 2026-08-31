#!/home/robot/Roam-E/.venv/bin/python

import math
import time

import serial
from smbus2 import SMBus


I2C_BUS = 3
BNO_ADDRESS = 0x29

SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
SERIAL_BAUD = 115200

PWM = 80

BASELINE_SECONDS = 5.0
MOTOR_SECONDS = 5.0
RECOVERY_SECONDS = 4.0

COMMAND_INTERVAL = 0.08
SAMPLE_INTERVAL = 0.10

CHIP_ID = 0x00
MAG_DATA = 0x0E
EULER_DATA = 0x1A
CALIB_STAT = 0x35
OPR_MODE = 0x3D
SYS_STATUS = 0x39
SYS_ERROR = 0x3A


def signed_16(low, high):
    value = low | (high << 8)

    if value & 0x8000:
        value -= 0x10000

    return value


def retry(operation, description, attempts=5):
    last_error = None

    for attempt in range(attempts):
        try:
            return operation()
        except OSError as error:
            last_error = error

            if attempt + 1 < attempts:
                time.sleep(0.02)

    raise RuntimeError(
        f"{description} failed after {attempts} attempts: {last_error}"
    )


def read_register(bus, register):
    return retry(
        lambda: bus.read_byte_data(BNO_ADDRESS, register),
        f"reading register 0x{register:02X}",
    )


def read_block(bus, register, length):
    return retry(
        lambda: bus.read_i2c_block_data(
            BNO_ADDRESS,
            register,
            length,
        ),
        f"reading register block 0x{register:02X}",
    )


def calibration_values(value):
    return (
        (value >> 6) & 0x03,
        (value >> 4) & 0x03,
        (value >> 2) & 0x03,
        value & 0x03,
    )


def read_sensor(bus):
    euler = read_block(bus, EULER_DATA, 6)
    magnetic = read_block(bus, MAG_DATA, 6)

    heading = (euler[0] | (euler[1] << 8)) / 16.0
    roll = signed_16(euler[2], euler[3]) / 16.0
    pitch = signed_16(euler[4], euler[5]) / 16.0

    # BNO055 magnetometer scale: 16 LSB per microtesla.
    mag_x = signed_16(magnetic[0], magnetic[1]) / 16.0
    mag_y = signed_16(magnetic[2], magnetic[3]) / 16.0
    mag_z = signed_16(magnetic[4], magnetic[5]) / 16.0

    mag_strength = math.sqrt(
        mag_x * mag_x +
        mag_y * mag_y +
        mag_z * mag_z
    )

    calibration = read_register(bus, CALIB_STAT)
    system, gyro, accel, mag = calibration_values(calibration)

    return {
        "heading": heading,
        "roll": roll,
        "pitch": pitch,
        "mag_x": mag_x,
        "mag_y": mag_y,
        "mag_z": mag_z,
        "mag_strength": mag_strength,
        "system": system,
        "gyro": gyro,
        "accel": accel,
        "mag": mag,
    }


def circular_difference(first, second):
    return (first - second + 180.0) % 360.0 - 180.0


def circular_mean(values):
    sine = sum(math.sin(math.radians(value)) for value in values)
    cosine = sum(math.cos(math.radians(value)) for value in values)

    angle = math.degrees(math.atan2(sine, cosine))

    return angle % 360.0


def send_stop(arduino):
    arduino.write(b"STOP\n")
    arduino.flush()

    deadline = time.monotonic() + 1.0

    while time.monotonic() < deadline:
        line = arduino.readline().decode(
            errors="replace"
        ).strip()

        if line == "OK,STOP":
            return True

    return False


def run_phase(arduino, bus, label, duration, pwm=None):
    print()
    print(f"=== {label} ===")

    if pwm is None:
        if not send_stop(arduino):
            raise RuntimeError(
                f"STOP was not confirmed before {label}"
            )

        command = None
    else:
        command = f"PWM,{pwm},{pwm}\n".encode()

    samples = []
    start = time.monotonic()
    next_command = start
    next_sample = start
    next_report = start

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start

            if elapsed >= duration:
                break

            if command is not None and now >= next_command:
                arduino.write(command)
                arduino.flush()
                next_command += COMMAND_INTERVAL

            if now >= next_sample:
                sample = read_sensor(bus)
                samples.append(sample)
                next_sample += SAMPLE_INTERVAL

                if now >= next_report:
                    print(
                        f"{elapsed:4.1f}s: "
                        f"heading={sample['heading']:7.2f}° "
                        f"field={sample['mag_strength']:6.2f} µT "
                        f"cal="
                        f"{sample['system']}/"
                        f"{sample['gyro']}/"
                        f"{sample['accel']}/"
                        f"{sample['mag']}"
                    )

                    next_report += 1.0

            # Discard telemetry so the serial buffer cannot fill.
            waiting = arduino.in_waiting

            if waiting:
                arduino.read(waiting)

            time.sleep(0.005)

    finally:
        if command is not None:
            if not send_stop(arduino):
                raise RuntimeError(
                    f"STOP was not confirmed after {label}"
                )

    return samples


def summarize(label, samples, baseline_heading, baseline_field):
    headings = [sample["heading"] for sample in samples]
    fields = [sample["mag_strength"] for sample in samples]

    mean_heading = circular_mean(headings)
    mean_field = sum(fields) / len(fields)

    mean_heading_shift = abs(
        circular_difference(mean_heading, baseline_heading)
    )

    maximum_heading_shift = max(
        abs(circular_difference(value, baseline_heading))
        for value in headings
    )

    field_change_percent = (
        100.0 * abs(mean_field - baseline_field) / baseline_field
        if baseline_field
        else 0.0
    )

    print()
    print(label)
    print(f"  Mean heading:          {mean_heading:.2f}°")
    print(f"  Mean heading shift:    {mean_heading_shift:.2f}°")
    print(f"  Maximum heading shift: {maximum_heading_shift:.2f}°")
    print(f"  Mean magnetic field:   {mean_field:.2f} µT")
    print(f"  Magnetic-field change: {field_change_percent:.1f}%")

    if (
        mean_heading_shift <= 2.0
        and maximum_heading_shift <= 3.0
        and field_change_percent <= 10.0
    ):
        result = "PASS"
    elif (
        mean_heading_shift <= 5.0
        and maximum_heading_shift <= 8.0
        and field_change_percent <= 20.0
    ):
        result = "CAUTION"
    else:
        result = "INTERFERENCE DETECTED"

    print(f"  Result: {result}")

    return result


def main():
    bus = SMBus(I2C_BUS)

    try:
        chip_id = read_register(bus, CHIP_ID)
        mode = read_register(bus, OPR_MODE)
        status = read_register(bus, SYS_STATUS)
        error = read_register(bus, SYS_ERROR)
        calibration = read_register(bus, CALIB_STAT)

        system, gyro, accel, mag = calibration_values(calibration)

        print("BNO055 motor-interference test")
        print(f"Chip ID: 0x{chip_id:02X}")
        print(f"Mode:    0x{mode:02X}")
        print(f"Status:  0x{status:02X}")
        print(f"Error:   0x{error:02X}")
        print(
            f"Calibration: SYS={system} GYR={gyro} "
            f"ACC={accel} MAG={mag}"
        )

        if chip_id != 0xA0:
            raise RuntimeError("BNO055 chip ID is incorrect")

        if mode != 0x0C or status != 0x05 or error != 0x00:
            raise RuntimeError(
                "BNO055 is not ready. Run restore.py first."
            )

        if gyro != 3 or accel != 3 or mag != 3:
            raise RuntimeError(
                "Physical sensors are not calibrated. "
                "Run restore.py first."
            )

        with serial.Serial(
            SERIAL_PORT,
            SERIAL_BAUD,
            timeout=0.02,
        ) as arduino:
            print()
            print("Arduino resetting...")
            time.sleep(2.5)
            arduino.reset_input_buffer()

            if not send_stop(arduino):
                raise RuntimeError(
                    "Motor-controller firmware was not detected"
                )

            try:
                baseline = run_phase(
                    arduino,
                    bus,
                    "MOTORS STOPPED — BASELINE",
                    BASELINE_SECONDS,
                )

                forward = run_phase(
                    arduino,
                    bus,
                    "BOTH MOTORS FORWARD",
                    MOTOR_SECONDS,
                    PWM,
                )

                recovery = run_phase(
                    arduino,
                    bus,
                    "MOTORS STOPPED — RECOVERY",
                    RECOVERY_SECONDS,
                )

                reverse = run_phase(
                    arduino,
                    bus,
                    "BOTH MOTORS REVERSE",
                    MOTOR_SECONDS,
                    -PWM,
                )

                final_recovery = run_phase(
                    arduino,
                    bus,
                    "MOTORS STOPPED — FINAL RECOVERY",
                    RECOVERY_SECONDS,
                )

            finally:
                arduino.write(b"STOP\n")
                arduino.flush()

        baseline_heading = circular_mean(
            [sample["heading"] for sample in baseline]
        )
        baseline_field = sum(
            sample["mag_strength"] for sample in baseline
        ) / len(baseline)

        print()
        print("=== RESULTS ===")
        print(f"Baseline heading:       {baseline_heading:.2f}°")
        print(f"Baseline magnetic field:{baseline_field:7.2f} µT")

        forward_result = summarize(
            "Forward motors",
            forward,
            baseline_heading,
            baseline_field,
        )

        recovery_result = summarize(
            "Recovery after forward",
            recovery,
            baseline_heading,
            baseline_field,
        )

        reverse_result = summarize(
            "Reverse motors",
            reverse,
            baseline_heading,
            baseline_field,
        )

        final_result = summarize(
            "Final stopped recovery",
            final_recovery,
            baseline_heading,
            baseline_field,
        )

        print()
        print("=== SUMMARY ===")
        print("Forward:", forward_result)
        print("Reverse:", reverse_result)
        print("Recovery after forward:", recovery_result)
        print("Final recovery:", final_result)

    finally:
        bus.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled; motors stopped.")
        raise SystemExit(130)
    except Exception as error:
        print()
        print(f"TEST FAILED: {error}")
        raise SystemExit(1)


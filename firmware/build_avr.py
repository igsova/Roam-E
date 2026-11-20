#!/usr/bin/env python3
"""Compile the Nano sketch using an extracted Ubuntu AVR toolchain; never upload.

Expected layout below --toolchain-root: usr/bin/avr-* and
usr/share/arduino/hardware/arduino/avr from gcc-avr, binutils-avr, avr-libc and
arduino-core-avr packages. No system packages, USB devices or repository files
are modified. Output contains generated build artifacts only.

Target is a classic ATmega328P Nano at 16 MHz with the eightanaloginputs variant.
This build does not detect a physical board, its MCU or its bootloader baud rate.
"""
import argparse
from pathlib import Path
import subprocess


def run(arguments):
    subprocess.run([str(value) for value in arguments], check=True, timeout=60)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolchain-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--sketch', type=Path, default=(
        Path(__file__).resolve().parents[1] /
        'firmware/roam_e_motor_controller/roam_e_motor_controller.ino'
    ))
    args = parser.parse_args()
    toolchain = args.toolchain_root.resolve(strict=True)
    sketch = args.sketch.resolve(strict=True)
    output = args.output_dir.resolve()
    # Refuse reuse: do not overwrite an existing artifact directory.
    output.mkdir(parents=True, exist_ok=False)
    binaries = toolchain / 'usr/bin'
    core_base = toolchain / 'usr/share/arduino/hardware/arduino/avr'
    core = core_base / 'cores/arduino'
    variant = core_base / 'variants/eightanaloginputs'
    common = [
        '-mmcu=atmega328p', '-DF_CPU=16000000L', '-DARDUINO=10819',
        '-DARDUINO_AVR_NANO', '-DARDUINO_ARCH_AVR', '-Os', '-g', '-flto',
        '-ffunction-sections', '-fdata-sections',
        '-I' + str(core), '-I' + str(variant),
        '-I' + str(toolchain / 'usr/avr/include'),
    ]
    cpp_flags = ['-std=gnu++11', '-fno-exceptions', '-fno-threadsafe-statics']
    objects = []
    for source in sorted(core.iterdir()):
        if source.suffix not in ('.c', '.cpp', '.S'):
            continue
        compiler = binaries / ('avr-g++' if source.suffix == '.cpp' else 'avr-gcc')
        flags = cpp_flags if source.suffix == '.cpp' else (
            ['-std=gnu11'] if source.suffix == '.c' else ['-x', 'assembler-with-cpp']
        )
        target = output / (source.name + '.o')
        run([compiler, *common, *flags, '-c', source, '-o', target])
        objects.append(target)
    archive = output / 'core.a'
    run([binaries / 'avr-gcc-ar', 'rcs', archive, *objects])
    sketch_object = output / 'roam_e_motor_controller.o'
    run([
        binaries / 'avr-g++', *common, *cpp_flags, '-Wall', '-Wextra',
        '-x', 'c++', '-c', sketch, '-o', sketch_object,
    ])
    elf = output / 'roam_e_motor_controller.elf'
    run([
        binaries / 'avr-gcc', '-mmcu=atmega328p', '-Os', '-flto',
        '-Wl,--gc-sections', '-o', elf, sketch_object, archive, '-lm',
    ])
    run([
        binaries / 'avr-objcopy', '-O', 'ihex', '-R', '.eeprom', elf,
        output / 'roam_e_motor_controller.hex',
    ])
    run([binaries / 'avr-size', '-C', '--mcu=atmega328p', elf])
    print('Built for ATmega328P / 16 MHz; no controller access or upload.')
    print('Build directory: ' + str(output))


if __name__ == '__main__':
    main()

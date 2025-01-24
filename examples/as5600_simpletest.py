# SPDX-FileCopyrightText: Copyright (c) 2025 Noel Anderson
#
# SPDX-License-Identifier: Unlicense

import time

import board
import busio

from as5600 import AS5600

i2c = busio.I2C(board.SCL, board.SDA)
sensor = AS5600(i2c)

# is the magnet detected?
print(f"Magnet Detected: {sensor.is_magnet_detected}")

# Set the hysteresis to 3 LSB to reduce the chance of flickering when magnet is still
sensor.hysteresis = AS5600.HYSTERESIS_3LSB

while True:
    print(f"Angle: {sensor.angle}")
    time.sleep(1)

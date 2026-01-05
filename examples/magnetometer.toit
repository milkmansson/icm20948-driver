// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

/**
A simple example of how to use the ICM20948 driver.
*/

import gpio
import i2c
import icm20948

main:
  bus := i2c.Bus
    --sda=gpio.Pin 8
    --scl=gpio.Pin 9
    --frequency=400_000

  bus-device-count := bus.scan.size
  if not (bus.test icm20948.Driver.AK09916-I2C-ADDRESS):
    print "Bus scan has $bus-device-count devices"

  device := bus.device icm20948.I2C-ADDRESS-ALT
  sensor := icm20948.Driver device
  sensor.on
  sensor.configure-accel
  sensor.configure-gyro

  sensor.

  while true:
    print "Acceleration: $sensor.read-accel"
    print "Gyroscope: $sensor.read-gyro"
    sleep --ms=1000
  sensor.off

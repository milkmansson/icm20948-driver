// Copyright (C) 2025 Toitware Contributors. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

/**
Uses I2C bypass mode to allow access to the AK09916 directly on the host
  I2C bus.  It requires the additional ak09916 driver package.

This is for testing purposes only.  The bypass configuration will remain until
  disabled. It also automatically resets on the next power on/reset of the
  ICM20948.
*/

import gpio
import i2c
import icm20948
import ak0991x

main:
  print

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

  // Enable bypass:
  sensor.enable-i2c-bypass
  if not (bus.test icm20948.Driver.AK09916-I2C-ADDRESS):
    print "bus missing the device. stopping..."
    return

  ak-device := bus.device ak0991x.Ak0991x.I2C-ADDRESS
  ak-sensor := ak0991x.Ak0991x ak-device

  print "Bus contains AK09916."
  print "Hardware ID: 0x$(%02x ak-sensor.get-hardware-id)"
  ak-sensor.set-operating-mode ak0991x.Ak0991x.OPMODE-CONT-MODE1-10HZ
  sleep --ms=250
  print "Data Ready: $(ak-sensor.is-data-ready)"
  print "Magnetic Field: $ak-sensor.read-magnetic-field"
  print "Bearing (no compensation)  : $(%0.3f ak-sensor.read-bearing)"
  print "Current Accel: $sensor.read-accel"
  print "Current Accel: $sensor.read-gyro"
  print "Bearing (with compensation): $(%0.3f ak-sensor.read-bearing-fused --accel=sensor.read-accel --gyro=sensor.read-gyro)"
  print
  print "Repating bearing, with tilt compensation from ICM20948.  Get ready..."
  4.repeat:
    sleep --ms=1500
    print "$it \t bearing: $(%0.3f ak-sensor.read-bearing) \t fused: $(%0.3f ak-sensor.read-bearing-fused --accel=sensor.read-accel --gyro=sensor.read-gyro)"
    print "    raw mag: $ak-sensor.read-magnetic-field"
    print "    raw accel: $sensor.read-accel"
    print "    raw gyro: $sensor.read-gyro"

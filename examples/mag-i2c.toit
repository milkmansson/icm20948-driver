// Copyright (C) 2026 Toitware Contributors. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

/**
A simple example of how to use the megnetometer via the ICM20948 driver.
*/

import gpio
import i2c
import icm20948

main:
  print
  print
  bus := i2c.Bus
    --sda=gpio.Pin 19
    --scl=gpio.Pin 20
    --frequency=400_000

  bus-device-count := bus.scan.size
  if not (bus.test icm20948.I2C-ADDRESS-ALT):
    print "Bus scan does not contain ICM20948 [0x$(%02x icm20948.I2C-ADDRESS-ALT)]"
    return

  print "Bus contains ICM20948 [0x$(%02x icm20948.I2C-ADDRESS-ALT)]"
  device := bus.device icm20948.I2C-ADDRESS-ALT
  sensor := icm20948.Driver device
  sensor.on

  print " configuring accelerometer.."
  sensor.configure-accel
  print " configuring gyroscope.."
  sensor.configure-gyro
  print " configuring magnetometer.."
  sensor.configure-mag
  print
  print "Starting 10 reads:"
  print   " Temperature:  $(%0.2f sensor.read-temp) c"
  10.repeat:
    sleep --ms=1000
    print " -------------------------------------------"
    print " Acceleration: $sensor.read-accel"
    print " Gyroscope:    $sensor.read-gyro"
    print " Magnetometer: $sensor.read-mag"
  print " -------------------------------------------"
  // For troubleshooting the 10 bytes taken from the magnetometer
  //print "$(sensor.dump-bytes 10 --reg=0x3b --bank=0)"

  sensor.off

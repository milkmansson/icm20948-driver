// Copyright (C) 2026 Toitware Contributors. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

/**
A simple example of how to use the ICM20948 driver.
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
  print " starting fifo.."
  sensor.fifo-start --accel=true --gyro=true --mag=true --temp=true
  print " starting runner for 10 seconds.."
  sleep --ms=1000
  sensor.run (::
    //print " - $it"
    out := []
    out.add ("accel: $(sensor.read-accel it[0..6])".pad --left 40 ' ')
    out.add ("gyro: $(sensor.read-gyro it[6..12])".pad --left 40 ' ')
    out.add ("mag: $(sensor.read-mag it[14..20])".pad --left 40 ' ')
    out.add ("temp: $(%0.3f sensor.read-temp it[20..22])".pad --left 14 ' ')
    print out
    )
  sleep --ms=10_000

  print " stopping runner.."
  sensor.run-stop
  sleep --ms=1_000
  print " stopping fifo.."
  sensor.fifo-stop
  print " stopping sensor.."
  sensor.off

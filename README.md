# A toit driver for the Ivensense/TDK ICM20948 9-axis MEMS motion sensor.
The ICM-20948 is a 9-axis devices, comprising of a native
gyroscope and acceleromoeter, with a built in instance of an AK09916
magnetometer.  It has many features.

## Usage
For complete examples, see [examples folder](./examples/)

## Features

### Standard Functions: 9-axis
The basic accelerometer and gyroscope functions are exposed using `.read-accel`
and `.read-gyro`.  These will return Point3f's of the x/y/z vector.  Before
running these functions, each function must be configured using `configure-gyro`
and `configure-accel`.

For the final three axes -, the ICM20948 hardware implementation of the compass/
magnetometer component is quite different, however this is handled in this
driver with similar functions.  To use the magenetometer, use `.read-mag`.
This requires setup first, using `.configure-mag`.

### FIFO:
Basic FIFO handling is implemented in this driver.  (The driver assumes that the
magnetometer is the onboard AK09916, and no other devices are connected to the
AUX I2C bus.)  The usage patterns are in the examples. Essentially, after
starting and configuring the driver normally, start the FIFO, and assign a
lambda to be run each time a frame is returned:
```Toit
// Start the fifo specifying which functions should be in the result frame.
sensor.fifo-start --accel=true --gyro=true --mag=true --temp=true

// The sensor starts the specified lambda for each frame received from the FIFO.
sensor.run (::
  out := []
  out.add ("accel: $(sensor.read-accel it[0..6])".pad --left 40 ' ')
  out.add ("gyro: $(sensor.read-gyro it[6..12])".pad --left 40 ' ')
  out.add ("mag: $(sensor.read-mag it[14..20])".pad --left 40 ' ')
  out.add ("temp: $(%0.3f sensor.read-temp it[20..22])".pad --left 14 ' ')
  print out)

// This is run as a separate task, and therefore we wait 5 seconds to see some
// results come to the screen.
sleep --ms=5_000

// This stops the background sensor.run task.
sensor.run-stop
```
The Output Data Rate can be configured using `.set-sample-rate-hz xxx`, where
xxx is the number of samples per second.  The slowest the device can go is
approximately 4.4Hz, supplying numbers lower than 1 has no further effect.

### Auxiliary Bus:
Internally, the device accesses the magnetometer on an auxiliary I2C bus.  The
device has the capability of exposing direct wired I2C access to the AX09916.
This is bypass mode - it essentially sits on the I2C bus alongside the ICM20948,
and is discoverable and can be interacted with directly.  To do this, use
`enable-i2c-bypass`.  (Reverse it by using `disable-i2c-bypass`.)  As with any
other device, a separate driver is required to access/use it, such as this
[AK0991x](https://github.com/milkmansson/toit-ak0991x) driver.

Note that in this mode, synchronisation of measurements isn't possible, and
onboard features using the magnetometer, (like DMP etc) will not be available
whilst in this mode.  (An example of how to use this mode is given in
[this example](./examples/mag-i2c-bypass.toit).)

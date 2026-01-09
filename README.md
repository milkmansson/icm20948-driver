# A Toit driver for the Ivensense/TDK ICM20948 9-axis MEMS motion sensor.
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
```
The sensor starts the specified lambda for each frame received from the FIFO.
The frame is `it` in each execution of the lambda.  Each line here shows a
'slice' of the frame being passed it to the driver functions which convert the
bytes to their proper formats using the driver native functions:
```
sensor.run (::
  out := []
  out.add "accel: $(sensor.read-accel it[0..6])"
  out.add "gyro: $(sensor.read-gyro it[6..12])"
  out.add "mag: $(sensor.read-mag it[14..20])"
  out.add "temp: $(sensor.read-temp it[20..22])"
  print out)

// This is run as a separate task, and therefore we wait 5 seconds to see some
// results come to the screen.
sleep --ms=5_000

// This stops the background sensor.run task.
sensor.run-stop
```
Enabling each feature adds bytes to the resulting frame.  Note that the die
temperature is not valid for FIFO output when specified on its own.  When all are enabled, each frame is 24 bytes. The outputs are added in the following order:
| Precedence | Data | Bytes | Description |
| - | - | - | - |
| 1 | Accelerometer | 6 bytes | x/y/z (int16's in big endian format). |
| 2 | Gyroscope | 6 bytes | x/y/z (int16's in big endian format). |
| 3 | Magnetometer | 10 bytes | Of these, the 6 bytes at [2..6] are the x/y/z (int16's in big endian format) |
| 4 | Die/chip temperature | final 2 bytes | Two bytes, an int16 in big-endian format. |

For conversion from raw counts, see the source code, or the
[datasheet](https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf).

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

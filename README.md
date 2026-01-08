# icm20948
Driver for ICM-20948 9-axis devices.  This device comprises of a native
gyroscope and acceleromoeter, with a built in instance of an AK09916
magnetometer.  It has many features.

## Usage
For complete examples, see [examples folder](./examples/)

## Features

### Standard Functions: 6-axis:
The basic accelerometer and gyroscope functions are exposed using `.read-accel`
and `.read-gyro`.  These will return Point3f's of the x/y/z vector.  Before
running these functions, they must be configured using `configure-gyro` and
`configure-accel`.
The hardware implementation of the compass/magnetometer component is quite
different, however these are implemented similarly.  To use the magenetometer,
use `.read-mag`.  This requires setup first, using `.configure-mag`.

### FIFO:
Basic FIFO handling is implemented in this driver.  (The driver assumes that the
magnetometer is the onboard AK09916, and no other devices are connected to the
AUX I2C bus.)  The usage patterns are in the examples, however essentially:
```

```

### Auxiliary Bus:
If the use case requires direct I2C access to the AX09916, the device has the
capability of exposing it on the I2C bus alongside the ICM20948.  To do this,
use `enable-i2c-bypass`.  (Reverse it by using `disable-i2c-bypass`.)
Synchronisation of measurements won't be possible, and onboard features like DMP
etc will not be available whilst in this mode.  (An example use of this )

// Copyright (C) 2026 Toitware Contributors. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import io
import serial.device show Device
import serial.registers show Registers
import math
import log

I2C-ADDRESS     ::= 0b1101000
I2C-ADDRESS-ALT ::= 0b1101001

ACCEL-SCALE-2G  ::= 0
ACCEL-SCALE-4G  ::= 1
ACCEL-SCALE-8G  ::= 2
ACCEL-SCALE-16G ::= 3

GYRO-SCALE-250DPS   ::= 0
GYRO-SCALE-500DPS   ::= 1
GYRO-SCALE-1000DPS  ::= 2
GYRO-SCALE-2000DPS  ::= 3

class Driver:

  static WHO-AM-I_ ::= 0xEA

  static ACCEL-SENSITIVITY_ ::= [16384.0, 8192.0, 4096.0, 2048.0]
  static GYRO-SENSITIVITY_ ::= [131.0, 65.5, 32.8, 16.4]

  static REGISTER-REG-BANK-SEL_   ::= 0x7F

  // Bank 0
  static REGISTER-WHO-AM-I_       ::= 0x00
  static REGISTER-USER-CTRL_      ::= 0x03
  static REGISTER-LP-CONFIG_      ::= 0x05
  static REGISTER-PWR-MGMT-1_     ::= 0x06
  static REGISTER-PWR-MGMT-2_     ::= 0x07
  static REGISTER-INT-PIN-CFG_    ::= 0x0F
  static REGISTER-INT-ENABLE_     ::= 0x10
  static REGISTER-INT-ENABLE-1_   ::= 0x11
  static REGISTER-INT-ENABLE-2_   ::= 0x12
  static REGISTER-INT-ENABLE-3_   ::= 0x13
  static REGISTER-I2C-MST-STATUS_ ::= 0x17
  static REGISTER-INT-STATUS_     ::= 0x19
  static REGISTER-INT-STATUS-1_   ::= 0x1A
  static REGISTER-INT-STATUS-2_   ::= 0x1B
  static REGISTER-INT-STATUS-3_   ::= 0x1C
  static REGISTER-ACCEL-XOUT-H_   ::= 0x2D
  static REGISTER-ACCEL-XOUT-L_   ::= 0x2E
  static REGISTER-ACCEL-YOUT-H_   ::= 0x2F
  static REGISTER-ACCEL-YOUT-L_   ::= 0x30
  static REGISTER-ACCEL-ZOUT-H_   ::= 0x31
  static REGISTER-ACCEL-ZOUT-L_   ::= 0x32
  static REGISTER-GYRO-XOUT-H_    ::= 0x33
  static REGISTER-GYRO-XOUT-L_    ::= 0x34
  static REGISTER-GYRO-YOUT-H_    ::= 0x35
  static REGISTER-GYRO-YOUT-L_    ::= 0x36
  static REGISTER-GYRO-ZOUT-H_    ::= 0x37
  static REGISTER-GYRO-ZOUT-L_    ::= 0x38
  static REGISTER-TEMP-OUT-H_     ::= 0x39
  static REGISTER-TEMP-OUT-L_     ::= 0x3a

  static REGISTER-EXT-SLV-DATA-00_ ::= 0x3b
  static REGISTER-EXT-SLV-DATA-01_ ::= 0x3c
  static REGISTER-EXT-SLV-DATA-02_ ::= 0x3d
  static REGISTER-EXT-SLV-DATA-03_ ::= 0x3e
  static REGISTER-EXT-SLV-DATA-04_ ::= 0x3f
  static REGISTER-EXT-SLV-DATA-05_ ::= 0x40
  static REGISTER-EXT-SLV-DATA-06_ ::= 0x41
  static REGISTER-EXT-SLV-DATA-07_ ::= 0x42
  static REGISTER-EXT-SLV-DATA-08_ ::= 0x43
  static REGISTER-EXT-SLV-DATA-09_ ::= 0x44
  //...
  static REGISTER-EXT-SLV-DATA-23_ ::= 0x52

  static REGISTER-FIFO-EN-1_        ::= 0x66
  static REGISTER-FIFO-EN-2_        ::= 0x67
  static REGISTER-FIFO-RST_         ::= 0x68
  static REGISTER-FIFO-MODE_        ::= 0x69
  static REGISTER-FIFO-COUNT_       ::= 0x70
  static REGISTER-FIFO-R-W_         ::= 0x72
  static REGISTER-DATA-RDY-STATUS_  ::= 0x74
  static REGISTER-FIFO-CFG_         ::= 0x76


  // Masks: $REGISTER-USER-CTRL_
  static USER-CTRL-DMP-EN_      ::= 0b10000000
  static USER-CTRL-FIFO-EN_     ::= 0b01000000
  static USER-CTRL-I2C-MST-EN_  ::= 0b00100000
  static USER-CTRL-I2C-IF-DIS_  ::= 0b00010000  // Reset I2C Slave module and put the serial interface in SPI mode only.
  static USER-CTRL-DMP-RST_     ::= 0b00001000  // Reset DMP. Asynchronous. Takes 1 clock cycle of 20 Mhz clock.
  static USER-CTRL-SRAM-RST_    ::= 0b00000100  // Reset SRAM. Asynchronous. Takes 1 clock cycle of 20 Mhz clock.
  static USER-CTRL-I2C-MST-RST_ ::= 0b00000010  // Reset I2C. Asynchronous. Takes 1 clock cycle of 20 Mhz clock. Could cause I2C slave to hang. See datasheet.

  // Masks: REGISTER-LP-CONFIG_
  static LP-CONFIG-I2C-MST-CYCLE_    ::= 0b01000000
  static LP-CONFIG-I2C-ACCEL-CYCLE_  ::= 0b00100000
  static LP-CONFIG-I2C-GYRO-CYCLE_   ::= 0b00010000

  // Masks: REGISTER-I2C-MST-STATUS_
  static I2C-MST-STATUS-I2C-SLV4-DONE_ ::= 0b01000000
  static I2C-MST-STATUS-I2C-SLV4-NAK_  ::= 0b00010000
  static I2C-MST-STATUS-PASS-THROUGH_  ::= 0b10000000
  static I2C-MST-STATUS-I2C-LOST-ARB_  ::= 0b00100000
  static I2C-MST-STATUS-I2C-SLV3-NAK_  ::= 0b00001000
  static I2C-MST-STATUS-I2C-SLV2-NAK_  ::= 0b00000100
  static I2C-MST-STATUS-I2C-SLV1-NAK_  ::= 0b00000010
  static I2C-MST-STATUS-I2C-SLV0-NAK_  ::= 0b00000001

  // Bank 2
  static REGISTER-GYRO-SMPLRT-DIV_  ::= 0x0
  static REGISTER-GYRO-CONFIG-1_    ::= 0x1
  static REGISTER-GYRO-CONFIG-2_    ::= 0x2
  static REGISTER-ODR-ALIGN-EN_     ::= 0x9
  static REGISTER-ACCEL-CONFIG_     ::= 0x14
  static REGISTER-ACCEL-CONFIG-2_   ::= 0x15

  // Bank 3 - I2C Master Engine.
  static REGISTER-I2C-MST-ODR-CONFIG_ ::= 0x00
  static REGISTER-I2C-MST-CTRL_       ::= 0x01
  static REGISTER-I2C-MST-DELAY-CTRL_ ::= 0x02

  // Slave read/write engines:
  // SLV0–SLV3: continuous / automatic reads - Repeatedly read data from the
  //   external sensors and deposit it into Bank 0 $REGISTER-EXT-SLV-DATA-00_
  //   through to REGISTER-EXT-SLV-DATA-23.  Where several slaves are configured
  //   result bytes are packed together one after the other, starting at
  //   $REGISTER-EXT-SLV-DATA-00_.  Max allowed = 24 result bytes.
  // SLV4: one-shot command channel - Perform single, blocking I²C transactions
  //   (writes or reads).  Result goes into $REGISTER-I2C-SLV4-DI_.
  //   Must wait for 'DONE' from I2C-MST-STATUS.
  static REGISTER-I2C-SLV0-ADDR_      ::= 0x03  // R/W and PHY address of I2C Slave x.
  static REGISTER-I2C-SLV0-REG_       ::= 0x04  // I2C slave x register address from where to begin data transfer.
  static REGISTER-I2C-SLV0-CTRL_      ::= 0x05  //
  static REGISTER-I2C-SLV0-DO_        ::= 0x06  // Data out when slave x is set to write.

  static REGISTER-I2C-SLV1-ADDR_      ::= 0x07
  static REGISTER-I2C-SLV1-REG_       ::= 0x08
  static REGISTER-I2C-SLV1-CTRL_      ::= 0x09
  static REGISTER-I2C-SLV1-DO_        ::= 0x0A

  static REGISTER-I2C-SLV2-ADDR_      ::= 0x0B
  static REGISTER-I2C-SLV2-REG_       ::= 0x0C
  static REGISTER-I2C-SLV2-CTRL_      ::= 0x0D
  static REGISTER-I2C-SLV2-DO_        ::= 0x0E

  static REGISTER-I2C-SLV3-ADDR_      ::= 0x0F
  static REGISTER-I2C-SLV3-REG_       ::= 0x10
  static REGISTER-I2C-SLV3-CTRL_      ::= 0x11
  static REGISTER-I2C-SLV3-DO_        ::= 0x12

  static REGISTER-I2C-SLV4-ADDR_      ::= 0x13
  static REGISTER-I2C-SLV4-REG_       ::= 0x14
  static REGISTER-I2C-SLV4-CTRL_      ::= 0x15
  static REGISTER-I2C-SLV4-DO_        ::= 0x16  // Data OUT when slave 4 is set to write.
  static REGISTER-I2C-SLV4-DI_        ::= 0x17  // Data IN when slave 4.


  // Masks: $REGISTER-INT-PIN-CFG_
  static INT-PIN-CFG-INT1-ACTL_             ::= 0b10000000
  static INT-PIN-CFG-INT1-OPEN_             ::= 0b01000000
  static INT-PIN-CFG-INT1-LATCH-EN_         ::= 0b00100000
  static INT-PIN-CFG-INT-INT-ANYRD-2CLEAR_  ::= 0b00010000
  static INT-PIN-CFG-ACTL-FSYNC_            ::= 0b00001000
  static INT-PIN-CFG-FSYNC-INT-MODE-EN_     ::= 0b00000100
  static INT-PIN-CFG-BYPASS-EN_             ::= 0b00000010

  // Masks: $REGISTER-I2C-MST-CTRL_
  static I2C-MULT-MST-EN_ ::= 0b10000000
  static I2C-MST-P-NSR_   ::= 0b00010000
  static I2C-MST-CLK_     ::= 0b00001111 // To use 400 kHz, MAX, it is recommended to set I2C-MST-CLK_ to 7.

  // Masks: REGISTER-I2C-SLVx-ADDR_ [x=0..4]
  static I2C-SLVx-ADDR-R_      ::= 0b10000000  // 1 = transfer is R for slave x
  static I2C-SLVx-ADDR-W_      ::= 0b00000000  // 0 = transfer is W for slave x
  static I2C-SLVx-ADDR-I2C-ID_ ::= 0b01111111  // PHY address of I2C slave x

  // Masks: REGISTER-I2C-SLVx-CTRL_ [x=0..4]
  static I2C-SLVx-CTRL-EN_      ::= 0b10000000  // Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA register, which is always EXT_SENS_DATA_00 for I 2C slave 0
  static I2C-SLVx-CTRL-BYTE-SW_ ::= 0b01000000  // 1 – Swap bytes when reading both the low and high byte of a word.
  static I2C-SLVx-CTRL-REG-DIS_ ::= 0b00100000  // Disables writing the register value - when set it will only read/write data.
  static I2C-SLVx-CTRL-GRP_     ::= 0b00010000  // Whether 16 bit byte reads are 00..01 or 01..02.
  static I2C-SLVx-CTRL-LENG_    ::= 0b00001111  // Number of bytes to be read from I2C slave X.

  // Masks: REGISTER-I2C-MST-DELAY-CTRL_
  static I2C-MST-DELAY-ES-SHADOW_ ::= 0b10000000
  static I2C-MST-DELAY-SLV4-EN_   ::= 0b00010000
  static I2C-MST-DELAY-SLV3-EN_   ::= 0b00001000
  static I2C-MST-DELAY-SLV2-EN_   ::= 0b00000100
  static I2C-MST-DELAY-SLV1-EN_   ::= 0b00000010
  static I2C-MST-DELAY-SLV0-EN_   ::= 0b00000001

  // Register Map for AK09916
  static REG-AK09916-MAN-ID_    ::= 0x00  // R 1 Manufacturer ID.
  static REG-AK09916-DEV-ID_    ::= 0x01  // R 1 Device ID.
  static REG-AK09916-RSV-1_     ::= 0x02  // R 1 Reserved.
  static REG-AK09916-RSV-2_     ::= 0x03  // R 1 Reserved.
  static REG-AK09916-STATUS-1_  ::= 0x10  // R 1 Data status.
  static REG-AK09916-X-AXIS_    ::= 0x11  // R 2 X Axis LSB (MSB 0x12).  Signed int.
  static REG-AK09916-Y-AXIS_    ::= 0x13  // R 2 Y Axis LSB (MSB 0x14).  Signed int.
  static REG-AK09916-Z-AXIS_    ::= 0x15  // R 2 Y Axis LSB (MSB 0x16).  Signed int.
  static REG-AK09916-STATUS-2_  ::= 0x18  // R 1 Data status.
  static REG-AK09916-CONTROL-1_ ::= 0x30  // R 1 Control Settings.
  static REG-AK09916-CONTROL-2_ ::= 0x31  // R 1 Control Settings.
  static REG-AK09916-CONTROL-3_ ::= 0x32  // R 1 Control Settings.

  static AK09916-DEV-ID_          ::= 0b00001001 // Device ID should always be this.
  static AK09916-STATUS-1-DOR_    ::= 0b00000010 // Data Overrun.
  static AK09916-STATUS-1-DRDY_   ::= 0b00000001 // New data is ready.
  static AK09916-STATUS-2-HOFL_   ::= 0b00001000 // Hardware Overflow.
  static AK09916-STATUS-2-RSV28_  ::= 0b00010000 // Reserved for AKM.
  static AK09916-STATUS-2-RSV29_  ::= 0b00100000 // Reserved for AKM.
  static AK09916-STATUS-2-RSV30_  ::= 0b01000000 // Reserved for AKM.
  static AK09916-CONTROL-2-STEST_ ::= 0b00010000 // Self Test Mode.
  static AK09916-CONTROL-2-MODE4_ ::= 0b00001000 // Continuous Mode 4.
  static AK09916-CONTROL-2-MODE3_ ::= 0b00000110 // Continuous Mode 3.
  static AK09916-CONTROL-2-MODE2_ ::= 0b00000100 // Continuous Mode 2.
  static AK09916-CONTROL-2-MODE1_ ::= 0b00000010 // Continuous Mode 1.
  static AK09916-CONTROL-2-MODE0_ ::= 0b00000001 // Single Measurement Mode.
  static AK09916-CONTROL-2-OFF_   ::= 0b00000000 // Power down mode.
  static AK09916-CONTROL-3-SRST_  ::= 0b00000001 // Software Reset.

  static AK09916-I2C-ADDRESS ::= 0x0C  // Magnetometer I2C address when configured for bus bridging.

  // $write-register_ statics for bit width.  All 16 bit read/writes are LE.
  static WIDTH-8_ ::= 8
  static WIDTH-16_ ::= 16
  static DEFAULT-REGISTER-WIDTH_ ::= WIDTH-8_

  // I2C Slave Write Timeout
  static COMMAND-TIMEOUT-MS_ ::= 1000

  accel-sensitivity_/float := 0.0
  gyro-sensitivity_/float := 0.0

  bank_/int := -1
  reg_/Registers := ?
  logger_/log.Logger := ?

  constructor dev/Device --logger/log.Logger=log.default:
    reg_ = dev.registers
    logger_ = logger.with-name "icm20948"
    set-bank_ 0

  on:
    tries := 5
    set-bank_ 0
    while (reg_.read-u8 REGISTER-WHO-AM-I_) != WHO-AM-I_:
      tries--
      if tries == 0: throw "INVALID_CHIP"
      sleep --ms=1

    reset_

    // Enable ACCEL and GYRO.
    set-bank_ 0
    // print ((reg_.read_u8 REGISTER_PWR_MGMT_1_).stringify 16)
    reg_.write-u8 REGISTER-PWR-MGMT-1_ 0b00000001
    reg_.write-u8 REGISTER-PWR-MGMT-2_ 0b00000000

    // Configure to defaults for immediate function (avoid divide by zero)
    configure-accel --scale=ACCEL-SCALE-2G
    configure-gyro  --scale=GYRO-SCALE-250DPS
    configure-mag   // No user selectable scale applies.

  configure-accel --scale/int=ACCEL-SCALE-2G:
    r := reg_.read-u8 REGISTER-LP-CONFIG_
    reg_.write-u8 REGISTER-LP-CONFIG_ r & ~0b100000

    // Configure accel.
    cfg := 0b00111_00_1
    cfg |= scale << 1
    set-bank_ 2
    reg_.write-u8 REGISTER-ACCEL-CONFIG_ cfg
    set-bank_ 0

    accel-sensitivity_ = ACCEL-SENSITIVITY_[scale]

    sleep --ms=10

  configure-gyro --scale/int=GYRO-SCALE-250DPS:
    r := reg_.read-u8 REGISTER-LP-CONFIG_
    reg_.write-u8 REGISTER-LP-CONFIG_ r & ~0b10000

    set-bank_ 2

    //reg_.write_u8 REGISTER_GYRO_SMPLRT_DIV_ 0

    reg_.write-u8 REGISTER-GYRO-CONFIG-1_ 0b111_00_1 | scale << 1
    //reg_.write_u8 REGISTER_GYRO_CONFIG_2_ 0b1111

    set-bank_ 0

    gyro-sensitivity_ = GYRO-SENSITIVITY_[scale]

    sleep --ms=10

  off:

  read-point_ reg/int sensitivity/float --le/bool=false -> math.Point3f:
    bytes := reg_.read-bytes reg 6
    if le:
      return math.Point3f
        (io.LITTLE-ENDIAN.int16 bytes 0) / sensitivity
        (io.LITTLE-ENDIAN.int16 bytes 2) / sensitivity
        (io.LITTLE-ENDIAN.int16 bytes 4) / sensitivity
    else:
      return math.Point3f
        (io.BIG-ENDIAN.int16 bytes 0) / sensitivity
        (io.BIG-ENDIAN.int16 bytes 2) / sensitivity
        (io.BIG-ENDIAN.int16 bytes 4) / sensitivity

  read-accel -> math.Point3f?:
    if accel-sensitivity_ == 0.0:
      throw "ACCEL NOT CONFIGURED"

    set-bank_ 0
    // Wait for ready, but with a timeout.
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
          while ((reg_.read-u8 REGISTER-INT-STATUS-1_) & 0x01) == 0:
            sleep --ms=1

    if exception:
      logger_.error "read-accel timed out" --tags={"timeout-ms":COMMAND-TIMEOUT-MS_}
      return null

    return read-point_ REGISTER-ACCEL-XOUT-H_ accel-sensitivity_

  read-gyro -> math.Point3f?:
    if gyro-sensitivity_ == 0.0:
      throw "GYRO NOT CONFIGURED"

    set-bank_ 0
    // Wait for ready, but with a timeout.
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
          while ((reg_.read-u8 REGISTER-INT-STATUS-1_) & 0x01) == 0:
            sleep --ms=1

    if exception:
      logger_.error "read-gyro timed out" --tags={"timeout-ms":COMMAND-TIMEOUT-MS_}
      return null

    return read-point_ REGISTER-GYRO-XOUT-H_ gyro-sensitivity_

  reset_:
    set-bank_ 0
    reg_.write-u8 REGISTER-PWR-MGMT-1_ 0b10000001
    bank_ = -1
    sleep --ms=100
    set-bank_ 0
    if (reg_.read-u8 REGISTER-WHO-AM-I_) != WHO-AM-I_:
      logger_.error "bus did not recover from reset"
      throw "Bus did not recover from reset."

  /**
  Sets the current bank for register reads and writes.

  Tracks current bank in $bank_ and sets only when necessary, in order to
    reduce traffic on the I2C bus.
  */
  set-bank_ bank/int:
    assert: 0 <= bank <= 3
    if bank_ == bank:
      //logger_.debug "bank already set" --tags={"bank":bank}
      return
    reg_.write-u8 REGISTER-REG-BANK-SEL_ bank << 4
    //logger_.debug "set bank" --tags={"bank":bank}
    bank_ = bank

  /**
  Enables I2C bypass such that AK09916 appears and is reachable on external I2C bus.

  Requires the ICM20948 device to be turned $on.

  This is largely for direct access to the AK09916, for testing/debugging
    purposes.  It is not used for any other reasons.  It could be suggested that
    Software oriented fusion could still be done, however, as the MCU must
    manage timing there is no synchronization with accel/gyro.  The bus will
    conflict if it is misconfigured.  The mode prevents DMP fusion.  This mode
    also prevents access to AK09916 data over SPI.
  */
  enable-i2c-bypass -> none:
    set-i2c-master_ false
    set-i2c-bypass-mux_ true

  /** Reverses $enable-i2c-bypass. */
  disable-i2c-bypass -> none:
    set-i2c-bypass-mux_ false
    set-i2c-master_ true

  /** Read on-die Thermomenter. */
  read-die-temp -> float:
    set-bank_ 0
    raw := reg_.read-bytes REGISTER-TEMP-OUT-H_ 2
    return ((io.BIG-ENDIAN.int16 raw 0).to-float / 333.87) + 21.0

  /**
  Set ODR Alignment.

  Enables ODR start-time alignment when any of the following registers is
    written (with the same value or with different values): GYRO_SMPLRT_DIV,
    ACCEL_SMPLRT_DIV_1, ACCEL_SMPLRT_DIV_2, I2C_MST_ODR_CONFIG
  */
  set-align-odr_ aligned/bool -> none:
    set-bank_ 2
    value := 0
    if aligned: value = 1
    write-register_ REGISTER-ODR-ALIGN-EN_ value
    sleep --ms=5

  reset-i2c-master_ -> none:
    set-bank_ 0
    write-register_ REGISTER-USER-CTRL_ 1 --mask=USER-CTRL-I2C-MST-RST_
    sleep --ms=5

  set-i2c-master_ enabled/bool -> none:
    set-bank_ 0
    value := 0
    if enabled: value = 1
    write-register_ REGISTER-USER-CTRL_ value --mask=USER-CTRL-I2C-MST-EN_

  set-i2c-bypass-mux_ enabled/bool -> none:
    set-bank_ 0
    value := 0
    if enabled: value = 1
    write-register_ REGISTER-INT-PIN-CFG_ value --mask=INT-PIN-CFG-BYPASS-EN_

  set-i2c-bus-speed_ speed -> none:
    assert: 0 <= speed <= 15
    set-bank_ 3
    write-register_ REGISTER-I2C-MST-CTRL_ speed --mask=I2C-MST-CLK_

  set-i2c-master-duty-cycle-mode_ on/bool -> none:
    set-bank_ 0
    value := 0
    if on: value = 1
    write-register_ REGISTER-LP-CONFIG_ value --mask=LP-CONFIG-I2C-MST-CYCLE_

  /**
  Set ODR for I2C Master.

  ODR configuration for external sensor when gyroscope and accelerometer are
    disabled. ODR is computed: 1.1 kHz/(2^((odr_config[3:0])) ) When gyroscope
    is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR. If
    gyroscope is disabled, then all sensors (including I2C_MASTER) use the
    accelerometer ODR.
  */
  set-i2c-master-duty-odr_ odr/int -> none:
    assert: 0b0000 <= odr <= 0b1111
    set-bank_ 3
    write-register_ REGISTER-I2C-MST-ODR-CONFIG_ odr

  /**
  Enables Master Delay for slave $slave.

  When enabled, slave 0 will only be accessed 1/(1+I2C_SLC4_DLY) samples as
    determined by I2C_MST_ODR_CONFIG.
  */
  set-i2c-master-delay_ --slave/int enable/bool -> none:
    assert: 0 <= slave <= 4
    set-bank_ 3
    mask := 1 << slave
    value := 0
    if enable: value = 1
    write-register_ REGISTER-I2C-MST-DELAY-CTRL_ value --mask=mask

  /**
  Delays shadowing of external sensor data until all data is received.
  */
  shadow-sensor-only-when-complete_ enable/bool -> none:
    set-bank_ 3
    value := 0
    if enable: value = 1
    write-register_ REGISTER-I2C-MST-DELAY-CTRL_ value --mask=I2C-MST-DELAY-ES-SHADOW_

  /**
  Enables magnetometer on slave I2C bus.

  Configures the ICM20948 for communication with AK09916. This includes
    resetting the AK09916, configuring it for 100Hz sampling, and then
    configuring the ICM20948 to
  */
  configure-mag -> none:
    // Set as master again, in case it was bypassed in the previous run.
    set-i2c-bypass-mux_ false
    // Disconnect I2C master whilst its being reset.
    set-i2c-master_ false
    // Keep ODR in step with the actual device reads.
    set-align-odr_ true
    // Reset I2C master.
    reset-i2c-master_
    // Reconnect I2C master after reset.
    set-i2c-master_ true
    // Set bus speed to 400Khz
    set-i2c-bus-speed_ 0x07
    set-i2c-master-duty-cycle-mode_ true
    // Set ODR to 137Hz.
    set-i2c-master-duty-odr_ 0x03
    // Enable master delay for this slave.
    set-i2c-master-delay_ --slave=0 true
    // Reset Magnetometer Slave.
    write-slave_ REG-AK09916-CONTROL-3_ AK09916-CONTROL-3-SRST_
    // Put Magnetometer Slave into Continuous Mode 4 (100Hz).
    write-slave_ REG-AK09916-CONTROL-2_ AK09916-CONTROL-2-MODE4_

    /*
    Common method is to set SLV0: to get 9 bytes starting at
      REG-AK09916-STATUS-1_ via ODR.  Sparkfun engineering advises that reading
      10 bytes starting at REG-AK09916-RSV-2_ (undocumented/reserved) allows
      getting the bytes in pairs as the DMP engine expects them to be.

    The remainder of  this driver is tuned for this - first byte (in
      REGISTER-EXT-SLV-DATA-00_) will be original reg REG-AK09916-RSV-2_ (0x03)
      (unused), second byte will be original reg REG-AK09916-STATUS-1_ and
      read sequentially the next 10 bytes from from there (up to and including
      REG-AK09916-STATUS-2_, which triggers the next data refresh on the
      AK09916. This way the driver will read and use the device prepared in the
      same way the DMP engine needs.
    */
    set-slave 0 REG-AK09916-RSV-2_ --size=10

  read-mag -> math.Point3f?:
    set-bank_ 0
    // Wait for ready, but with a timeout.
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
        while (((read-register_ REGISTER-EXT-SLV-DATA-01_ --width=8) & AK09916-STATUS-1-DRDY_) == 0):
          sleep --ms=10   // Was 1ms.

    if exception:
      logger_.error "read-mag timed out" --tags={"timeout-ms":COMMAND-TIMEOUT-MS_}
      return null

    ak09916-dor := read-register_ REGISTER-EXT-SLV-DATA-01_ --mask=AK09916-STATUS-1-DOR_
    if ak09916-dor == 1:
      logger_.error "mag reports data overflow" --tags={"status-1":ak09916-dor}
    ak09916-hofl := read-register_ REGISTER-EXT-SLV-DATA-09_ --mask=AK09916-STATUS-2-HOFL_
    if ak09916-hofl == 1:
      logger_.error "mag reports hardware overflow" --tags={"status-2":ak09916-hofl}

    return read-point_ REGISTER-EXT-SLV-DATA-02_ (1.0 / 0.15) --le

  /**
  Reads back the WHOAMI register from the AK09916, for troubleshooting purposes.
  */
  read-mag-whoami -> int:
    raw := read-slave_ REG-AK09916-DEV-ID_
    if raw == null:
      logger_.error "read-mag-whoami null returned"
      return 0
    return raw

  /**
  Dump $length bytes from $reg address in $bank bank, for troubleshooting.

  $reg Defaults to REGISTER-EXT-SLV-DATA-00_, which is the start of the register
    Where results from the AK09916 are stored.  $bank defaults to bank 0.
  */
  dump-bytes length/int --reg/int=REGISTER-EXT-SLV-DATA-00_ --bank/int=0 -> ByteArray:
    assert: 0 <= bank <= 3
    set-bank_ bank
    return (reg_.read-bytes reg length)

  /**
  Sets up one of the slaves (SLV0..SLV3) to query from an I2C slave device.

  If more than one slave is used, the bytes configured for the $size for each
    will be packed together starting at $REGISTER-EXT-SLV-DATA-00_.
  */
  /* Note: Those result registers are in Bank 0, not 3 where the setup is. */
  set-slave -> none
      slave/int
      i2c-reg/int
      value/int?=null
      --i2c-addr/int=AK09916-I2C-ADDRESS
      --i2c-ctrl/int=0
      --size/int:
    assert: 0 <= size <= 0b1111

    set-bank_ 3
    i2c-slave-addr := REGISTER-I2C-SLV0-ADDR_ + (4 * slave)    // R/W and PHY address of I2C Slave x.
    i2c-slave-reg  := REGISTER-I2C-SLV0-REG_ + (4 * slave)     // I2C slave x register address from where to begin data transfer.
    i2c-slave-ctrl := REGISTER-I2C-SLV0-CTRL_ + (4 * slave)    // BITMASK
    i2c-slave-data := REGISTER-I2C-SLV0-DO_ + (4 * slave)      // Data out when slave x is set to write.

    // Set SLVx to READ the $AK09916-I2C-ADDRESS address.
    if value != null:
      // This is meant to be a command send (eg send content of DO to device).
      write-register_ i2c-slave-addr (i2c-addr | I2C-SLVx-ADDR-W_)
    else:
      // This is meant to be a read (simply read data at register).
      write-register_ i2c-slave-addr (i2c-addr | I2C-SLVx-ADDR-R_)
    // Give SLVx the register to be read.
    write-register_ i2c-slave-reg i2c-reg
    // Give SLVx outgoing data (if a specific command send is needed).
    if value != null:
      write-register_ i2c-slave-data value
    // Write control bitmask witn EN to execute the transaction.
    ctrl := ((i2c-ctrl | I2C-SLVx-CTRL-EN_) | size)
    write-register_ i2c-slave-ctrl ctrl
    //logger_.debug "set slave" --tags={"slave":slave,"slave-addr":i2c-addr,"slave-reg":i2c-reg,"ctrl":"$(bits-grouped_ ctrl)"}

  /**
  Performs a one-shot read of the I2C slave at address $i2c-addr.

  Function uses SLV4. (SLV4 is for ONE SHOT reads and writes to slave device.)
  */
  read-slave_ -> int?
      i2c-reg/int
      --i2c-addr/int=AK09916-I2C-ADDRESS
      --i2c-ctrl/int=0
      --width/int=DEFAULT-REGISTER-WIDTH_:

    set-bank_ 3
    // Set SLV4 to READ the $AK09916-I2C-ADDRESS address.
    write-register_ REGISTER-I2C-SLV4-ADDR_ (i2c-addr | I2C-SLVx-ADDR-R_)
    // Give SLV4 the register to be read.
    write-register_ REGISTER-I2C-SLV4-REG_ i2c-reg

    // Wait for DONE or NACK (or timeout).  Also record display execution time.
    start := Time.monotonic-us
    finished := false
    status-mask := 0x0
    duration := Duration.ZERO
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
        duration = Duration.of:
          // Write SLV4 control values (+EN bit) to execute.
          write-register_ REGISTER-I2C-SLV4-CTRL_ (i2c-ctrl | I2C-SLVx-CTRL-EN_)
          // Wait for DONE or NAK.
          set-bank_ 0
          while not finished:
            status-mask = read-register_ REGISTER-I2C-MST-STATUS_
            if (status-mask & I2C-MST-STATUS-I2C-SLV4-DONE_) != 0: finished = true
            if (status-mask & I2C-MST-STATUS-I2C-SLV4-NAK_) != 0: finished = true
            sleep --ms=10

    // Return null on failure.
    result/int? := null
    if exception:
      logger_.error "read-slave_ timed out" --tags={
        "status-mask":"$(bits-grouped_ status-mask)",
        "ms":duration.in-ms}
      return result
    if (status-mask & I2C-MST-STATUS-I2C-SLV4-NAK_) != 0:
      logger_.error "write-slave_ NAK" --tags={
        "status-mask":"$(bits-grouped_ status-mask)",
        "ms":duration.in-ms}
      return result

    set-bank_ 3
    result = read-register_ REGISTER-I2C-SLV4-DI_ --width=width
    //logger_.debug "read-slave_ succeeded" --tags={"result":"0x$(%02x result)", "status-mask":"$(bits-grouped_ status-mask)", "ms":duration.in-ms}
    return result

  /**
  Performs a one-shot write to the I2C slave at address $i2c-addr.

  Function uses SLV4. (SLV4 used for ONE SHOT reads and writes to slave device.)
  */
  // To-Do: complete the function with '--mask' writes etc like $read-register_.
  write-slave_ -> none
      --i2c-addr/int=AK09916-I2C-ADDRESS
      i2c-reg/int
      value/int
      --i2c-ctrl/int=0
      --width/int=DEFAULT-REGISTER-WIDTH_:
    set-bank_ 3

    // SLV4 in WRITE mode (Force 7 bit address without the READ flag).
    write-register_ REGISTER-I2C-SLV4-ADDR_ ((i2c-addr & 0x7F) | I2C-SLVx-ADDR-W_)
    // Give SLV4 the register to be read.
    write-register_ REGISTER-I2C-SLV4-REG_ i2c-reg
    // Data out (the byte to write).
    write-register_ REGISTER-I2C-SLV4-DO_ value --width=width

    // Wait for DONE or NACK (or timeout), same as read-slave_.
    finished := false
    status-mask := 0x0
    duration := Duration.ZERO
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
        duration = Duration.of:
          // Execute the transaction by writing control value (+ EN bit).
          write-register_ REGISTER-I2C-SLV4-CTRL_ (i2c-ctrl | I2C-SLVx-CTRL-EN_)
          // If slave==4, wait for 'DONE' or 'NAK'
          set-bank_ 0
          while not finished:
            status-mask = read-register_ REGISTER-I2C-MST-STATUS_
            if (status-mask & I2C-MST-STATUS-I2C-SLV4-DONE_) != 0: finished = true
            if (status-mask & I2C-MST-STATUS-I2C-SLV4-NAK_) != 0: finished = true
            sleep --ms=25

    if exception:
      logger_.error "write-slave_ timed out" --tags={
        "status-mask":"$(bits-grouped_ status-mask)",
        "ms":COMMAND-TIMEOUT-MS_}
      throw "write-slave_ timed out"
    if (status-mask & I2C-MST-STATUS-I2C-SLV4-NAK_) != 0:
      logger_.error "write-slave_ NAK" --tags={
        "status-mask":"$(bits-grouped_ status-mask)",
        "ms":duration.in-ms}
      throw "write-slave_ NAK"
    //logger_.debug "write-slave_ succeeded" --tags={
    //  "reg":"0x$(%02x i2c-reg)",
    //  "value":"0x$(%02x value)",
    //  "status-mask":"$(bits-grouped_ status-mask)",
    //  "ms":duration.in-ms}


  /**
  Reads and optionally masks/parses register data. (Little-endian.)
  */
  read-register_
      register/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> any:
    assert: (width == 8) or (width == 16)
    raw/ByteArray := #[]

    if mask == null:
      if width == 8: mask = 0xFF
      else: mask = 0xFFFF
    if offset == null:
      offset = mask.count-trailing-zeros

    register-value/int? := null
    if width == 8:
      if signed:
        register-value = reg_.read-i8 register
      else:
        register-value = reg_.read-u8 register
    else:
      if signed:
        register-value = reg_.read-i16-le register
      else:
        register-value = reg_.read-u16-le register

    if register-value == null:
      logger_.error "read-register_ failed" --tags={"register":register}
      throw "read-register_ failed."

    if ((mask == 0xFFFF) or (mask == 0xFF)) and (offset == 0):
      return register-value
    else:
      masked-value := (register-value & mask) >> offset
      return masked-value

  /**
  Writes register data - either masked or full register writes. (Little-endian.)
  */
  write-register_
      register/int
      value/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> none:
    assert: (width == 8) or (width == 16)
    if mask == null:
      if width == 8: mask = 0xFF
      else: mask = 0xFFFF
    if offset == null:
      offset = mask.count-trailing-zeros

    field-mask/int := (mask >> offset)
    assert: ((value & ~field-mask) == 0)  // fit check

    // Full-width direct write
    if ((width == 8)  and (mask == 0xFF)  and (offset == 0)) or
      ((width == 16) and (mask == 0xFFFF) and (offset == 0)):
      if width == 8:
        signed ? reg_.write-i8 register (value & 0xFF) : reg_.write-u8 register (value & 0xFF)
      else:
        signed ? reg_.write-i16-le register (value & 0xFFFF) : reg_.write-u16-le register (value & 0xFFFF)
      return

    // Read Reg for modification
    old-value/int? := null
    if width == 8:
      if signed :
        old-value = reg_.read-i8 register
      else:
        old-value = reg_.read-u8 register
    else:
      if signed :
        old-value = reg_.read-i16-le register
      else:
        old-value = reg_.read-u16-le register

    if old-value == null:
      logger_.error "write-register_ read existing value (for modification) failed" --tags={"register":register}
      throw "write-register_ read failed"

    new-value/int := (old-value & ~mask) | ((value & field-mask) << offset)
    if width == 8:
      signed ? reg_.write-i8 register new-value : reg_.write-u8 register new-value
      return
    else:
      signed ? reg_.write-i16-le register new-value : reg_.write-u16-le register new-value
      return
    throw "write-register_: Unhandled Circumstance."

  /**
  Provides strings to display bitmasks nicely when testing.
  */
  bits-grouped_ x/int
      --min-display-bits/int=0
      --group-size/int=8
      --sep/string="."
      -> string:

    assert: x >= 0
    assert: group-size > 0

    // raw binary
    bin := "$(%b x)"

    // choose target width: at least min-display-bits, then round up to a full group
    groups := 0
    leftover := 0
    width := bin.size
    if min-display-bits > width:
      width = min-display-bits
    if group-size > width:
      width = group-size
    leftover = width % group-size
    if leftover > 0:
      width = width + (group-size - leftover)

    // left-pad to target width
    bin = bin.pad --left width '0'

    // group left->right
    out := ""
    i := 0
    while i < bin.size:
      if i > 0: out = "$(out)$(sep)"
      j := i + group-size
      if j > bin.size: j = bin.size
      out = "$(out)$(bin[i..j])"
      i = j

    return out

// Copyright (C) 2026 Toitware Contributors. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import io
import serial
import math
import log
import monitor

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

  // Masks: REGISTER-PWR-MGMT-1_
  static PWR-MGMT-1-DEVICE-RESET_ ::= 0b10000000
  static PWR-MGMT-1-SLEEP_        ::= 0b01000000
  static PWR-MGMT-1-LOW-POWER_    ::= 0b00100000
  static PWR-MGMT-1-TEMP-DIS_     ::= 0b00010000
  static PWR-MGMT-1-CLKSEL_       ::= 0b00000111

  // Masks: REGISTER-FIFO-EN-2_
  static FIFO-EN-2-ACCEL-EN_  ::= 0b00010000
  static FIFO-EN-2-GYRO-Z-EN_ ::= 0b00001000
  static FIFO-EN-2-GYRO-Y-EN_ ::= 0b00000100
  static FIFO-EN-2-GYRO-X-EN_ ::= 0b00000010
  static FIFO-EN-2-TEMP-EN_   ::= 0b00000001

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
  static REGISTER-GYRO-SMPLRT-DIV_    ::= 0x0
  static REGISTER-GYRO-CONFIG-1_      ::= 0x1
  static REGISTER-GYRO-CONFIG-2_      ::= 0x2
  static REGISTER-ODR-ALIGN-EN_       ::= 0x9
  static REGISTER-ACCEL-SMPLRT-DIV-1_ ::= 0x10
  static REGISTER-ACCEL-SMPLRT-DIV-2_ ::= 0x11
  static REGISTER-ACCEL-CONFIG_       ::= 0x14
  static REGISTER-ACCEL-CONFIG-2_     ::= 0x15

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

  // Sampling Rate Constants.
  static BASE-RATE-HZ_/int ::= 1125

  // $write-register_ statics for bit width.  All 16 bit read/writes are LE.
  static WIDTH-8_ ::= 8
  static WIDTH-16_ ::= 16
  static DEFAULT-REGISTER-WIDTH_ ::= WIDTH-8_

  // When FIFO buffer in ICM20948 grows past this, we are headingn towards
  // overflow and frame desync.
  static MAX-BUFFER-RESET-SIZE_ ::= 256 // bytes (hardware is 512).

  // I2C Slave Write Timeout
  static COMMAND-TIMEOUT-MS_ ::= 1000

  accel-sensitivity_/float := 0.0
  gyro-sensitivity_/float := 0.0

  bank_/int := -1
  dev_/serial.Device := ?
  reg_/serial.Registers := ?
  logger_/log.Logger := ?
  runner_/Task? := null
  fifo-frame-size_/int := 0
  bank-mutex_ := monitor.Mutex


  constructor device/serial.Device --logger/log.Logger=log.default:
    dev_ = device
    reg_ = device.registers
    logger_ = logger.with-name "icm20948"

    // Set current bank in sync with local var
    set-bank-p_ 0

  on:
    tries := 5
    while (read-register_ 0 REGISTER-WHO-AM-I_) != WHO-AM-I_:
      tries--
      if tries == 0: throw "INVALID_CHIP"
      sleep --ms=1

    reset_
    logger_.debug "device switched on"

    // Enable ACCEL and GYRO.
    // print ((reg_.read_u8 REGISTER_PWR_MGMT_1_).stringify 16)
    write-register_ 0 REGISTER-PWR-MGMT-1_ 0b00000001
    write-register_ 0 REGISTER-PWR-MGMT-2_ 0b00000000

    // Configure to defaults for immediate function (avoid divide by zero)
    //configure-accel --scale=ACCEL-SCALE-2G
    //configure-gyro  --scale=GYRO-SCALE-250DPS
    //configure-mag   // No user selectable scale applies.


  configure-accel --scale/int=ACCEL-SCALE-2G:
    set-accel-duty-cycle-mode_ true

    // Configure accel.
    cfg := 0b00111_00_1
    cfg |= scale << 1
    write-register_ 2 REGISTER-ACCEL-CONFIG_ cfg

    accel-sensitivity_ = ACCEL-SENSITIVITY_[scale]
    sleep --ms=10
    logger_.debug "accelerometer configured"

  configure-gyro --scale/int=GYRO-SCALE-250DPS:
    set-gyro-duty-cycle-mode_ true

    //write-register_ 2 REGISTER_GYRO_SMPLRT_DIV_ 0
    write-register_ 2 REGISTER-GYRO-CONFIG-1_ (0b111_00_1 | scale << 1)
    //write-register_ 2 REGISTER_GYRO_CONFIG_2_ 0b1111

    gyro-sensitivity_ = GYRO-SENSITIVITY_[scale]
    sleep --ms=10
    logger_.debug "gyroscope configured"

  off:
    run-stop
    fifo-stop
    reset-i2c-master_

  read-point_ reg/int? sensitivity/float --le/bool=false --bytes/ByteArray?=null -> math.Point3f:
    raw-bytes := #[]
    if bytes == null:
      bank-mutex_.do:
        set-bank-p_ 0
        raw-bytes = reg_.read-bytes reg 6
    else:
      assert: reg == null
      assert: bytes.size == 6
      raw-bytes = bytes

    if le:
      return math.Point3f
        (io.LITTLE-ENDIAN.int16 raw-bytes 0) / sensitivity
        (io.LITTLE-ENDIAN.int16 raw-bytes 2) / sensitivity
        (io.LITTLE-ENDIAN.int16 raw-bytes 4) / sensitivity
    else:
      return math.Point3f
        (io.BIG-ENDIAN.int16 raw-bytes 0) / sensitivity
        (io.BIG-ENDIAN.int16 raw-bytes 2) / sensitivity
        (io.BIG-ENDIAN.int16 raw-bytes 4) / sensitivity
  /**
  Read Accelerometer data.

  If $bytes are passed, these are used as source data for the function. (Useful
    when decoding direct reads, for example, from FIFO output.)
  */
  read-accel bytes/ByteArray?=null -> math.Point3f?:
    if accel-sensitivity_ == 0.0:
      throw "ACCEL NOT CONFIGURED"

    // If bytes are supplied do them instead of getting from a reg.
    if bytes != null:
      assert: bytes.size == 6
      return read-point_ null accel-sensitivity_ --bytes=bytes

    // Wait for ready, but with a timeout.
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
          while ((read-register_ 0 REGISTER-INT-STATUS-1_) & 0x01) == 0:
            sleep --ms=1

    if exception:
      logger_.error "read-accel timed out" --tags={"timeout-ms":COMMAND-TIMEOUT-MS_}
      return null

    return read-point_ REGISTER-ACCEL-XOUT-H_ accel-sensitivity_

  /**
  Read gyro.

  If 6 $bytes are passed, these are used as source data for the function. (Useful
    when decoding direct reads, for example, from FIFO output.)
  */
  read-gyro bytes/ByteArray=null -> math.Point3f?:
    if gyro-sensitivity_ == 0.0:
      throw "GYRO NOT CONFIGURED"

    // If bytes are supplied use them instead of getting from a reg.
    if bytes != null:
      assert: bytes.size == 6
      return read-point_ null gyro-sensitivity_ --bytes=bytes

    // Wait for ready, but with a timeout.
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
          while ((read-register_ 0 REGISTER-INT-STATUS-1_) & 0x01) == 0:
            sleep --ms=1

    if exception:
      logger_.error "read-gyro timed out" --tags={"timeout-ms":COMMAND-TIMEOUT-MS_}
      return null

    return read-point_ REGISTER-GYRO-XOUT-H_ gyro-sensitivity_

  reset_:
    bank-mutex_.do:
      set-bank-p_ 0
      reg_.write-u8 REGISTER-PWR-MGMT-1_ PWR-MGMT-1-DEVICE-RESET_
      bank_ = -1
      sleep --ms=100
      set-bank-p_ 0
      if (reg_.read-u8 REGISTER-WHO-AM-I_) != WHO-AM-I_:
        logger_.error "bus did not recover from reset after 100ms"
        throw "Bus did not recover from reset after 100ms"
    write-register_ 0 REGISTER-PWR-MGMT-1_ 1 --mask=PWR-MGMT-1-CLKSEL_


  // Set Sampling Rate
  set-sample-rate-hz rate-hz/int -> none:
    assert: 0 < rate-hz <= BASE-RATE-HZ_

    divider := (BASE-RATE-HZ_ / rate-hz) - 1
    if divider < 0: divider = 0
    if divider > 0xFFFF: divider = 0xFFFF

    set-gyro-sample-rate-divider_ divider
    set-accel-sample-rate-divider_ divider

  set-gyro-sample-rate-divider_ divider/int -> none:
    // Gyro divider is 8-bit
    write-register_ 2 REGISTER-GYRO-SMPLRT-DIV_ (divider & 0xFF)

  set-accel-sample-rate-divider_ divider/int -> none:
    // Accel divider is 16-bit
    write-register_ 2 REGISTER-ACCEL-SMPLRT-DIV-1_ divider --width=16

  /**
  Sets the current bank for register reads and writes.

  Tracks current bank in $bank_ and sets only when necessary, in order to
    reduce traffic on the I2C bus.
  */
  set-bank-p_ bank/int:
    assert: 0 <= bank <= 3
    if bank_ == bank:
      //logger_.debug "bank already set" --tags={"bank":bank}
      return
    reg_.write-u8 REGISTER-REG-BANK-SEL_ bank << 4
    //logger_.debug "set bank" --tags={"bank":bank}
    bank_ = bank

  set-sleep enable/bool -> none:
    value := 0
    if enable: value = 1
    write-register_ 0 REGISTER-PWR-MGMT-1_ value --mask=PWR-MGMT-1-SLEEP_

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

  /**
  Read temperature from on-die Thermomenter.

  If $bytes are passed, these are used as source data for the function. (Useful
    when decoding direct reads, for example, from FIFO output.)
  */
  read-temp bytes/ByteArray?=null -> float:
    // If bytes are supplied do them instead of getting from a reg.
    raw := #[]
    if bytes != null:
      assert: bytes.size == 2
      raw = bytes
    else:
      raw = read-register_ 0 REGISTER-TEMP-OUT-H_ --width=16
    return ((io.BIG-ENDIAN.int16 raw 0).to-float / 333.87) + 21.0

  /**
  Set ODR Alignment.

  Enables ODR start-time alignment when any of the following registers is
    written (with the same value or with different values): GYRO_SMPLRT_DIV,
    ACCEL_SMPLRT_DIV_1, ACCEL_SMPLRT_DIV_2, I2C_MST_ODR_CONFIG
  */
  set-align-odr_ aligned/bool -> none:
    value := 0
    if aligned: value = 1
    write-register_ 2 REGISTER-ODR-ALIGN-EN_ value
    sleep --ms=5

  reset-i2c-master_ -> none:
    was-running := is-i2c-master-set_
    write-register_ 0 REGISTER-USER-CTRL_ 1 --mask=USER-CTRL-I2C-MST-RST_
    sleep --ms=5
    if was-running: set-i2c-master_ true

  set-i2c-master_ enabled/bool -> none:
    value := 0
    if enabled: value = 1
    write-register_ 0 REGISTER-USER-CTRL_ value --mask=USER-CTRL-I2C-MST-EN_

  is-i2c-master-set_ -> bool:
    return (read-register_ 0 REGISTER-USER-CTRL_ --mask=USER-CTRL-I2C-MST-EN_) != 0

  set-i2c-bypass-mux_ enabled/bool -> none:
    value := 0
    if enabled: value = 1
    write-register_ 0 REGISTER-INT-PIN-CFG_ value --mask=INT-PIN-CFG-BYPASS-EN_

  set-i2c-bus-speed_ speed -> none:
    assert: 0 <= speed <= 15
    write-register_ 3 REGISTER-I2C-MST-CTRL_ speed --mask=I2C-MST-CLK_

  set-i2c-master-duty-cycle-mode_ enabled/bool -> none:
    value := 0
    if enabled: value = 1
    write-register_ 0 REGISTER-LP-CONFIG_ value --mask=LP-CONFIG-I2C-MST-CYCLE_

  set-accel-duty-cycle-mode_ on/bool -> none:
    value := 0
    if on: value = 1
    write-register_ 0 REGISTER-LP-CONFIG_ value --mask=LP-CONFIG-I2C-ACCEL-CYCLE_

  set-gyro-duty-cycle-mode_ on/bool -> none:
    value := 0
    if on: value = 1
    write-register_ 0 REGISTER-LP-CONFIG_ value --mask=LP-CONFIG-I2C-GYRO-CYCLE_

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
    write-register_ 3 REGISTER-I2C-MST-ODR-CONFIG_ odr

  /**
  Enables Master Delay for slave $slave.

  When enabled, slave x will only be accessed 1/(1+I2C_SLC4_DLY) samples as
    determined by I2C_MST_ODR_CONFIG.
  */
  set-i2c-master-delay_ --slave/int enable/bool -> none:
    assert: 0 <= slave <= 3
    mask := 1 << slave
    value := 0
    if enable: value = 1
    write-register_ 3 REGISTER-I2C-MST-DELAY-CTRL_ value --mask=mask

  /**
  Delays shadowing of external sensor data until all data is received.
  */
  set-i2c-shadow-only-when-complete_ enable/bool -> none:
    value := 0
    if enable: value = 1
    write-register_ 3 REGISTER-I2C-MST-DELAY-CTRL_ value --mask=I2C-MST-DELAY-ES-SHADOW_

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
    // Meant for duty-cycled behavior (low-power patterns). Off for now.
    set-i2c-master-duty-cycle-mode_ true
    // Set ODR to 137Hz.
    set-i2c-master-duty-odr_ 0x03
    // Enable master delay for this slave.
    set-i2c-master-delay_ --slave=0 true

    set-i2c-shadow-only-when-complete_ true

    // See if I2C Slave is up:
    logger_.info "Slave responds" --tags={"SLV WHO-AM-I":"0x$(%02x read-slave_ REG-AK09916-DEV-ID_)"}

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

    $I2C-SLVx-CTRL-BYTE-SW_ is also used in that the device is natively little
      endian, but the ICM20948 (and assuming its reader tool) is big endian.
    */
    set-slave 0 REG-AK09916-RSV-2_ --size=10 --i2c-ctrl=I2C-SLVx-CTRL-BYTE-SW_ | I2C-SLVx-CTRL-GRP_

  read-mag bytes/ByteArray?=null -> math.Point3f?:
    // If bytes are supplied do them instead.
    if bytes != null:
      assert: bytes.size == 6
      return read-point_ null (1.0 / 0.15) --bytes=bytes

    // Wait for ready, but with a timeout.
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
        while (((read-register_ 0 REGISTER-EXT-SLV-DATA-00_ --width=8) & AK09916-STATUS-1-DRDY_) == 0):
          sleep --ms=10   // Was 1ms.

    if exception:
      logger_.error "read-mag timed out" --tags={"timeout-ms":COMMAND-TIMEOUT-MS_}
      return null

    ak09916-dor := read-register_ 0 REGISTER-EXT-SLV-DATA-01_ --mask=AK09916-STATUS-1-DOR_
    if ak09916-dor == 1:
      logger_.error "mag reports data overflow" --tags={"status-1":ak09916-dor}
    ak09916-hofl := read-register_ 0 REGISTER-EXT-SLV-DATA-09_ --mask=AK09916-STATUS-2-HOFL_
    if ak09916-hofl == 1:
      logger_.error "mag reports hardware overflow" --tags={"status-2":ak09916-hofl}

     // using $I2C-SLVx-CTRL-BYTE-SW_ makes --le redundant on the following $read-point_
    return read-point_ REGISTER-EXT-SLV-DATA-02_ (1.0 / 0.15)

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
    out/ByteArray := #[]
    bank-mutex_.do:
      set-bank-p_ bank
      out = reg_.read-bytes reg length
    return out

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

    i2c-slave-addr := REGISTER-I2C-SLV0-ADDR_ + (4 * slave)    // R/W and PHY address of I2C Slave x.
    i2c-slave-reg  := REGISTER-I2C-SLV0-REG_ + (4 * slave)     // I2C slave x register address from where to begin data transfer.
    i2c-slave-ctrl := REGISTER-I2C-SLV0-CTRL_ + (4 * slave)    // BITMASK
    i2c-slave-data := REGISTER-I2C-SLV0-DO_ + (4 * slave)      // Data out when slave x is set to write.

    // Set SLVx to READ the $AK09916-I2C-ADDRESS address.
    if value != null:
      // This is meant to be a command send (eg send content of DO to device).
      write-register_ 3 i2c-slave-addr (i2c-addr | I2C-SLVx-ADDR-W_)
    else:
      // This is meant to be a read (simply read data at register).
      write-register_ 3 i2c-slave-addr (i2c-addr | I2C-SLVx-ADDR-R_)
    // Give SLVx the register to be read.
    write-register_ 3 i2c-slave-reg i2c-reg
    // Give SLVx outgoing data (if a specific command send is needed).
    if value != null:
      write-register_ 3 i2c-slave-data value
    // Write control bitmask witn EN to execute the transaction.
    ctrl := ((i2c-ctrl | I2C-SLVx-CTRL-EN_) | size)
    write-register_ 3 i2c-slave-ctrl ctrl
    logger_.debug "set slave" --tags={"slave":slave,"slave-addr":"$(%02x i2c-addr)","slave-reg":"$(%02x i2c-reg)","ctrl":"$(bits-grouped_ ctrl)"}

  /**
  Performs a one-shot read of the I2C slave at address $i2c-addr.

  Function uses SLV4. (SLV4 is for ONE SHOT reads and writes to slave device.)
  */
  // To-Do: complete the function with '--mask' writes etc like $read-register_.
  read-slave_ -> int?
      i2c-reg/int
      --i2c-addr/int=AK09916-I2C-ADDRESS
      --i2c-ctrl/int=0
      --width/int=DEFAULT-REGISTER-WIDTH_:

    // Set SLV4 to READ the $AK09916-I2C-ADDRESS address.
    write-register_ 3 REGISTER-I2C-SLV4-ADDR_ (i2c-addr | I2C-SLVx-ADDR-R_)
    // Give SLV4 the register to be read.
    write-register_ 3 REGISTER-I2C-SLV4-REG_ i2c-reg

    // Wait for DONE or NACK (or timeout).  Also record display execution time.
    start := Time.monotonic-us
    finished := false
    status-mask := 0x0
    duration := Duration.ZERO
    exception := catch:
      with-timeout --ms=COMMAND-TIMEOUT-MS_:
        duration = Duration.of:
          // Write SLV4 control values (+EN bit) to execute.
          write-register_ 3 REGISTER-I2C-SLV4-CTRL_ (i2c-ctrl | I2C-SLVx-CTRL-EN_)
          // Wait for DONE or NAK.

          while not finished:
            status-mask = read-register_ 0 REGISTER-I2C-MST-STATUS_
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
      logger_.error "read-slave_ NAK" --tags={
        "status-mask":"$(bits-grouped_ status-mask)",
        "ms":duration.in-ms}
      return result

    result = read-register_ 3 REGISTER-I2C-SLV4-DI_ --width=width
    //logger_.debug "read-slave_ succeeded" --tags={"result":"0x$(%02x result)", "status-mask":"$(bits-grouped_ status-mask)", "ms":duration.in-ms}
    return result

  /**
  Performs a one-shot write to the I2C slave at address $i2c-addr.

  Function uses SLV4. (SLV4 used for ONE SHOT reads and writes to slave device.)
  */
  // To-Do: complete the function with '--mask' writes etc like $write-register_.
  write-slave_ -> none
      --i2c-addr/int=AK09916-I2C-ADDRESS
      i2c-reg/int
      value/int
      --i2c-ctrl/int=0
      --width/int=DEFAULT-REGISTER-WIDTH_:

    exception := null
    duration := Duration.ZERO
    status-mask := 0
    bank-mutex_.do:
      set-bank-p_ 3
      // SLV4 in WRITE mode (Force 7 bit address without the READ flag).
      reg_.write-u8 REGISTER-I2C-SLV4-ADDR_ ((i2c-addr & 0x7F) | I2C-SLVx-ADDR-W_)
      // Give SLV4 the register to be read.
      reg_.write-u8 REGISTER-I2C-SLV4-REG_ i2c-reg
      // Data out (the byte to write).
      reg_.write-u8 REGISTER-I2C-SLV4-DO_ value

      // Wait for DONE or NACK (or timeout), same as read-slave_.
      finished := false
      duration = Duration.ZERO
      exception = catch:
        with-timeout --ms=COMMAND-TIMEOUT-MS_:
          duration = Duration.of:
            // Execute the transaction by writing control value (+ EN bit).
            reg_.write-u8 REGISTER-I2C-SLV4-CTRL_ (i2c-ctrl | I2C-SLVx-CTRL-EN_)
            // If slave==4, wait for 'DONE' or 'NAK'
            set-bank-p_ 0
            while not finished:
              status-mask = reg_.read-u8 REGISTER-I2C-MST-STATUS_
              if (status-mask & I2C-MST-STATUS-I2C-SLV4-DONE_) != 0: finished = true
              if (status-mask & I2C-MST-STATUS-I2C-SLV4-NAK_) != 0: finished = true
              sleep --ms=25

    if exception:
      logger_.error "write-slave_ timed out" --tags={
        "status-mask":"$(bits-grouped_ status-mask)",
        "ms":COMMAND-TIMEOUT-MS_,
        "e":exception}
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
  Reads and optionally masks/parses register data.

  This implementation is Big Endian, and enforces a mutex to prevent banks being
    changed mid-write.
  */
  read-register_
      bank/int
      register/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> any:
    assert: 0 <= bank <= 3
    assert: (width == 8) or (width == 16)
    raw/ByteArray := #[]

    if mask == null:
      if width == 8: mask = 0xFF
      else: mask = 0xFFFF
    if offset == null:
      offset = mask.count-trailing-zeros

    register-value/int? := null
    bank-mutex_.do:
      set-bank-p_ bank
      if width == 8:
        if signed:
          register-value = reg_.read-i8 register
        else:
          register-value = reg_.read-u8 register
      else:
        if signed:
          register-value = reg_.read-i16-be register
        else:
          register-value = reg_.read-u16-be register

    if register-value == null:
      logger_.error "read-register_ failed" --tags={"register":register}
      throw "read-register_ failed."

    if ((mask == 0xFFFF) or (mask == 0xFF)) and (offset == 0):
      return register-value
    else:
      masked-value := (register-value & mask) >> offset
      return masked-value

  /**
  Writes register data - either masked or full register writes.

  This implementation is Big Endian, and enforces a mutex to prevent banks being
    changed mid-read.
  */
  write-register_
      bank/int
      register/int
      value/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> none:
    assert: 0 <= bank <= 3
    assert: (width == 8) or (width == 16)
    if mask == null:
      if width == 8: mask = 0xFF
      else: mask = 0xFFFF
    if offset == null:
      offset = mask.count-trailing-zeros

    field-mask/int := (mask >> offset)
    assert: ((value & ~field-mask) == 0)  // fit check

    // Entire lot must be run under one mutex to ensure no other process
    // can interfere before the write is complete.
    bank-mutex_.do:
      // Set Bank
      set-bank-p_ bank

      // Short path for a full-width write.
      if ((width == 8)  and (mask == 0xFF)  and (offset == 0)) or
        ((width == 16) and (mask == 0xFFFF) and (offset == 0)):
        if width == 8:
          signed ? reg_.write-i8 register (value & 0xFF) : reg_.write-u8 register (value & 0xFF)
        else:
          signed ? reg_.write-i16-be register (value & 0xFFFF) : reg_.write-u16-be register (value & 0xFFFF)
        return

      // Register modification path.
      old-value/int? := null
      if width == 8:
        if signed :
          old-value = reg_.read-i8 register
        else:
          old-value = reg_.read-u8 register
      else:
        if signed :
          old-value = reg_.read-i16-be register
        else:
          old-value = reg_.read-u16-be register

      if old-value == null:
        logger_.error "write-register_ read existing value (for modification) failed" --tags={"register":register}
        throw "write-register_ read failed"

      new-value/int := (old-value & ~mask) | ((value & field-mask) << offset)

      if width == 8:
        signed ? reg_.write-i8 register new-value : reg_.write-u8 register new-value
        return
      else:
        signed ? reg_.write-i16-be register new-value : reg_.write-u16-be register new-value
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

  fifo-start
      --mag/bool=false
      --accel/bool=false
      --gyro/bool=false
      --temp/bool=false
      --sample-rate-hz/int=1 -> none:
    if not mag and not accel and not gyro and not temp:
      logger_.error "fifo-start without any selected sensor, doing nothing..."
      return
    //adapter_ = FifoAdapter_ this --logger=logger_
    set-sleep false
    fifo-reset
    set-fifo-mode false
    set-sample-rate-hz sample-rate-hz
    if mag: configure-mag
    fifo-set-mag-data_ mag
    if accel: configure-accel
    fifo-set-accel-data_ accel
    if gyro: configure-gyro
    fifo-set-gyro-data_ gyro
    fifo-set-temp-data_ temp
    fifo-enable_ true

  fifo-stop -> none:
    //if adapter_ == null:
    //  logger_.error "fifo not started"
    //  return
    run-stop
    fifo-enable_ false

    // Zero enabled FIFO options.
    fifo-frame-size_ = 0
    write-register_ 0 REGISTER-FIFO-EN-1_ 0x0
    write-register_ 0 REGISTER-FIFO-EN-2_ 0x0

  fifo-enable_ enable/bool -> none:
    value := 0
    if enable: value = 1
    write-register_ 0 REGISTER-USER-CTRL_ value --mask=USER-CTRL-FIFO-EN_

  run lambda/Lambda -> none:
    assert: runner_ == null
    flush
    runner_ = task::
      while true:
        yield
        frame := next-frame
        lambda.call frame

  run-stop -> none:
    if runner_ != null:
      runner_.cancel
      runner_ = null

  /**
  Sets FIFO mode - stream or snapshot.

  $snapshot = true is Snapshot mode - FIFO stops when full.
  */
  set-fifo-mode snapshot/bool=false -> none:
    value := 0
    if snapshot: value = 1
    write-register_ 0 REGISTER-FIFO-MODE_ value

  /**
  Enables Slave Data in FIFO output.

  When enabled, $slave's data will be included in FIFO output.  Relevant only
    for SLV 0..3.  (SLV4 is single-shot reads/writes with its own data outputs.)
  */
  fifo-set-slave-data_ --slave/int enable/bool -> none:
    assert: 0 <= slave <= 3
    mask := 1 << slave
    value := 0
    if enable: value = 1
    write-register_ 0 REGISTER-FIFO-EN-1_ value --mask=mask
    fifo-update-frame-size_

  fifo-is-slave-data-set_ --slave/int -> bool:
    assert: 0 <= slave <= 3
    mask := 1 << slave
    return (read-register_ 0 REGISTER-FIFO-EN-1_ --mask=mask) == 1

  /**
  Set Accelerometer data to be included in FIFO packets.
  */
  fifo-set-accel-data_ enable/bool -> none:
    value := 0
    if enable: value = 1
    write-register_ 0 REGISTER-FIFO-EN-2_ value --mask=FIFO-EN-2-ACCEL-EN_
    fifo-update-frame-size_

  fifo-is-accel-data-set_ -> bool:
    return (read-register_ 0 REGISTER-FIFO-EN-2_ --mask=FIFO-EN-2-ACCEL-EN_) == 1

  /**
  Set Gyroscope data to be included in FIFO packets.

  Forced to be treated as three (x, y, and z, are all on or all off).
  */
  fifo-set-gyro-data_ enable/bool -> none:
    value := 0
    if enable: value = 1
    write-register_ 0 REGISTER-FIFO-EN-2_ value --mask=FIFO-EN-2-GYRO-Z-EN_
    write-register_ 0 REGISTER-FIFO-EN-2_ value --mask=FIFO-EN-2-GYRO-Y-EN_
    write-register_ 0 REGISTER-FIFO-EN-2_ value --mask=FIFO-EN-2-GYRO-X-EN_
    fifo-update-frame-size_

  fifo-is-gyro-data-set_ -> bool:
    z/bool := (read-register_ 0 REGISTER-FIFO-EN-2_ --mask=FIFO-EN-2-GYRO-Z-EN_) == 1
    y/bool := (read-register_ 0 REGISTER-FIFO-EN-2_ --mask=FIFO-EN-2-GYRO-Y-EN_) == 1
    x/bool := (read-register_ 0 REGISTER-FIFO-EN-2_ --mask=FIFO-EN-2-GYRO-X-EN_) == 1
    if (x != y) and (y != z):
      throw "gyro triplet not in sync"
    return (z and y and z)

  /**
  Set Magnetometer data to be included in FIFO packets.

  A shortcut function $fifo-set-slave-data_ for slave 0.
  */
  fifo-set-mag-data_ enable/bool -> none:
    fifo-set-slave-data_ --slave=0 enable

  fifo-is-mag-data-set_ -> bool:
    return fifo-is-slave-data-set_ --slave=0

  /**
  Set Temperature data to be included in FIFO packets.
  */
  fifo-set-temp-data_ enabled/bool -> none:
    value := 0
    if enabled: value = 1
    write-register_ 0 REGISTER-FIFO-EN-2_ value --mask=FIFO-EN-2-TEMP-EN_
    fifo-update-frame-size_

  fifo-is-temp-data-set_ -> bool:
    return (read-register_ 0 REGISTER-FIFO-EN-2_ --mask=FIFO-EN-2-TEMP-EN_) == 1

  /**
  The number of bytes of data in the FIFO queue for the client to pick up.
  */
  fifo-available-bytes --frame/int?=null-> int:
    value := read-register_ 0 REGISTER-FIFO-COUNT_ --width=16
    if frame != null:
      value = value - (value % frame)
    return value

  fifo-read num-bytes/int --max-buffer-size/int=64 -> ByteArray:
    bytes := #[]
    bank-mutex_.do:
      set-bank-p_ 0
      bytes = reg_.read-bytes
        REGISTER-FIFO-R-W_
        (min max-buffer-size num-bytes)
    return bytes

  /**
  Reset the FIFO mechanism/buffer.
  */
  fifo-reset -> none:
    // Assert and de-assert for a reset (datasheet).
    write-register_ 0 REGISTER-FIFO-RST_ 1
    sleep --ms=5
    write-register_ 0 REGISTER-FIFO-RST_ 0
    sleep --ms=5

  fifo-frame-size -> int:
    return fifo-frame-size_

  fifo-slave-data-size_ --slave/int -> int:
    assert: 0 <= slave <= 3
    slave-reg := REGISTER-I2C-SLV0-CTRL_ + (slave * 4)
    return read-register_ 3 slave-reg --mask=I2C-SLVx-CTRL-LENG_

  fifo-update-frame-size_ -> none:
    output := 0
    output += (fifo-is-accel-data-set_ ? 6 : 0)
    output += (fifo-is-gyro-data-set_ ? 6 : 0)
    output += (fifo-is-temp-data-set_ ? 2 : 0)
    4.repeat:
      output += ((fifo-is-slave-data-set_ --slave=it) ? (fifo-slave-data-size_ --slave=it) : 0)
    fifo-frame-size_ = output
    logger_.debug "framesize updated" --tags={"size":fifo-frame-size_}

/*
class FifoAdapter_:
  static STREAM-DELAY_ ::= Duration --ms=1
  static MAX-BUFFER-RESET-SIZE_ ::= 256 // bytes (hardware is 512)
  static FIFO-CAPACITY ::= 512 // bytes

  logger_/log.Logger
  driver_/Driver

  constructor .driver_/Driver --logger/log.Logger=log.default:
    logger_ = logger.with-name "fifoadapter"
    logger_.info "fifoadapter constructed"
*/
  flush -> none:
    //driver_.fifo-reset
    fifo-reset
  /*
  reset -> none:
    //driver_.fifo-reset
    fifo-reset
    //wait-until-receiver-available_
    sleep --ms=50
    flush
  */

  next-frame -> ByteArray:
    while true:
      yield
      // If the device buffer is too full, dump it, or face desync of frames
      // available := driver_.fifo-available-bytes --frame=driver_.fifo-frame-size
      available := fifo-available-bytes --frame=fifo-frame-size
      if available > MAX-BUFFER-RESET-SIZE_:
        logger_.warn "fifo near-full (resetting)" --tags={"available": available}
        flush

      e := catch:
        if fifo-available-bytes < fifo-frame-size:
          //logger_.debug "waiting for full frame" --tags={"available":driver_.fifo-available-bytes}
          while fifo-available-bytes < fifo-frame-size:
            sleep --ms=10

        frame := fifo-read fifo-frame-size

        return frame
      logger_.warn "error obtaining frame" --tags={"error": e}


  wait-until-receiver-available_:
    // Block until we can read from the device.
    print "Blocking until we can read from the device"
    first ::= fifo-read fifo-available-bytes

    // Consume all data from the device before continuing (without blocking).
    flush

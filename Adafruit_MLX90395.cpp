/******************************************************************************
  This is a library for the MLX90395 magnetometer.

  Designed specifically to work with the MLX90395 breakout from Adafruit:

  ----> https://www.adafruit.com/products/4022

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code, please
  support Adafruit and open-source hardware by purchasing products from
  Adafruit!

  Written by Kevin Townsend/ktown for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 *****************************************************************************/

#include "Adafruit_MLX90395.h"

Adafruit_MLX90395::Adafruit_MLX90395(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_addr
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */

bool Adafruit_MLX90395::begin_I2C(uint8_t i2c_addr, TwoWire *wire) {
  if (!i2c_dev) {
    i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  }
  spi_dev = NULL;

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init();
}

bool Adafruit_MLX90395::_init(void) {
  if (!exitMode())
    return false;

  delay(10);

  if (!reset())
    return false;

  delay(10);

  _gain = getGain();
  if (_gain == 8) { // default high field gain
    _uTLSB = 7.14;
    _gain_default = 8;
  } else {
    _uTLSB = 2.5; // medium field gain
    _gain_default = 9;
  }

  _resolution = getResolution();

  if (!readRegister(0x26, &uniqueID[0]) || !readRegister(0x27, &uniqueID[1]) ||
      !readRegister(0x28, &uniqueID[2])) {
    return false;
  }

  return true;
}

/**
 * Performs a single X/Y/Z conversion and returns the results.
 *
 * @param x     Pointer to where the 'x' value should be stored.
 * @param y     Pointer to where the 'y' value should be stored.
 * @param z     Pointer to where the 'z' value should be stored.
 *
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90395::readData(float *x, float *y, float *z) {
  if (!startSingleMeasurement())
    return false;
  while (!readMeasurement(x, y, z))
    delay(1);

  return true;
}

/**
 * Reads data from data register & returns the results.
 *
 * @param x     Pointer to where the 'x' value should be stored.
 * @param y     Pointer to where the 'y' value should be stored.
 * @param z     Pointer to where the 'z' value should be stored.
 *
 * @return True on command success
 */
bool Adafruit_MLX90395::readMeasurement(float *x, float *y, float *z) {
  uint8_t tx[1] = {0x80}; // Read memory command
  uint8_t rx[12] = {0};   // status, crc, X16, Y16, Z16, T16, V16

  /* Perform the transaction. */
  if (i2c_dev) {
    if (!i2c_dev->write_then_read(tx, 1, rx, 12)) {
      return false;
    }
  }

  // check status
  // Serial.print("Status: "); Serial.println(rx[0], HEX);
  if (rx[0] != MLX90395_STATUS_DRDY) {
    return false;
  }

  int16_t xi, yi, zi;

  // Convert data to uT and float.
  xi = (rx[2] << 8) | rx[3];
  yi = (rx[4] << 8) | rx[5];
  zi = (rx[6] << 8) | rx[7];

  // makes a resolution correction, see DS 18.6. Sensitivity for details
  *x = (float)(xi << _resolution);
  *y = (float)(yi << _resolution);
  *z = (float)(zi << _resolution);

  // multiply by current gain setting & LSB/uT
  *x *= _uTLSB * gainMultipliers[_gain_default] / gainMultipliers[_gain];
  *y *= _uTLSB * gainMultipliers[_gain_default] / gainMultipliers[_gain];
  *z *= _uTLSB * gainMultipliers[_gain_default] / gainMultipliers[_gain];

  return true;
}

/**
 * Perform a soft reset
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90395::reset(void) {
  return (command(MLX90395_REG_RT) == MLX90395_STATUS_RESET);
}

/**
 * Perform a mode exit
 * @return True if the operation succeeded, otherwise false.
 */
bool Adafruit_MLX90395::exitMode(void) {
  // do once and ignore status
  command(MLX90395_REG_EX);

  return command(MLX90395_REG_EX) == 0;
}

/**
 * Begin a single measurement on all axes
 *
 * @return True on command success
 */
bool Adafruit_MLX90395::startSingleMeasurement(void) {
  return (command(MLX90395_REG_SM | 0x0F) == MLX90395_STATUS_SMMODE);
}

/**
 * Get the current oversampling setting
 * @return MLX90395_OSR_1, MLX90395_OSR_2, MLX90395_OSR_4, or MLX90395_OSR_8
 */
mlx90393_osr_t Adafruit_MLX90395::getOSR(void) {
  Adafruit_BusIO_Register reg2 =
      Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_2, 2, MSBFIRST);
  Adafruit_BusIO_RegisterBits osr_bits =
      Adafruit_BusIO_RegisterBits(&reg2, 2, 0);
  return (mlx90393_osr_t)osr_bits.read();
}

/**
 * Set the current oversampling setting
 * @param osrval MLX90395_OSR_1, MLX90395_OSR_2, MLX90395_OSR_4,
 *  or MLX90395_OSR_8
 * @return True on command success
 */
bool Adafruit_MLX90395::setOSR(mlx90393_osr_t osrval) {
  Adafruit_BusIO_Register reg2 =
      Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_2, 2, MSBFIRST);
  Adafruit_BusIO_RegisterBits osr_bits =
      Adafruit_BusIO_RegisterBits(&reg2, 2, 0);
  return osr_bits.write(osrval);
}

/**
 * Get the current gainsel value, see DS 18.6. Sensitivity for values
 * @return 0-15 gain selection offset
 */
uint8_t Adafruit_MLX90395::getGain(void) {
  Adafruit_BusIO_Register reg0 =
      Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_0, 2, MSBFIRST);
  Adafruit_BusIO_RegisterBits gain_bits =
      Adafruit_BusIO_RegisterBits(&reg0, 4, 4);
  return gain_bits.read();
}

/**
 * Get the current gainsel value, see DS 18.6. Sensitivity for values
 * @param gainval 0-15 gain selection offset
 * @return True on command success
 */
bool Adafruit_MLX90395::setGain(uint8_t gainval) {
  gainval &= 0xF;
  _gain = gainval; // cache it

  Adafruit_BusIO_Register reg0 =
      Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_0, 2, MSBFIRST);
  Adafruit_BusIO_RegisterBits gain_bits =
      Adafruit_BusIO_RegisterBits(&reg0, 4, 4);
  return gain_bits.write(gainval);
}

/**
 * Get the resolution, note that its till 16 bits just offset within the 19
 * bit ADC output.
 * @return MLX90395_RES_16, MLX90395_RES_17, MLX90395_RES_18 or
 *  MLX90395_RES_19
 */
mlx90393_res_t Adafruit_MLX90395::getResolution(void) {
  Adafruit_BusIO_Register reg2 =
      Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_2, 2, MSBFIRST);
  Adafruit_BusIO_RegisterBits resX_bits =
      Adafruit_BusIO_RegisterBits(&reg2, 2, 5);
  return (mlx90393_res_t)resX_bits.read();
}

/**
 * Set the resolution, note that its till 16 bits just offset within the 19
 * bit ADC output.
 * @param resval MLX90395_RES_16, MLX90395_RES_17, MLX90395_RES_18 or
 *  MLX90395_RES_19
 * @return True on command success
 */
bool Adafruit_MLX90395::setResolution(mlx90393_res_t resval) {
  _resolution = resval; // cache it

  Adafruit_BusIO_Register reg2 =
      Adafruit_BusIO_Register(i2c_dev, MLX90395_REG_2, 2, MSBFIRST);
  Adafruit_BusIO_RegisterBits resX_bits =
      Adafruit_BusIO_RegisterBits(&reg2, 2, 5);
  resX_bits.write(resval);
  Adafruit_BusIO_RegisterBits resY_bits =
      Adafruit_BusIO_RegisterBits(&reg2, 2, 7);
  resY_bits.write(resval);
  Adafruit_BusIO_RegisterBits resZ_bits =
      Adafruit_BusIO_RegisterBits(&reg2, 2, 9);
  return resZ_bits.write(resval);
}

/****************************************************************/

uint8_t Adafruit_MLX90395::command(uint8_t cmd) {
  uint8_t tx[2] = {0x80, cmd};
  uint8_t status = 0xFF;

  /* Perform the transaction. */
  if (i2c_dev) {
    if (!i2c_dev->write_then_read(tx, 2, &status, 1)) {
      // Serial.println("Failed command");
      return 0xFF;
    }
  }
  return status;
}

bool Adafruit_MLX90395::readRegister(uint8_t reg, uint16_t *data) {
  /* Perform the transaction. */
  if (i2c_dev) {
    Adafruit_BusIO_Register regis =
        Adafruit_BusIO_Register(i2c_dev, reg << 1, 2);
    *data = regis.read();
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t device data, Adafruit Unified Sensor format
    @param  sensor Pointer to an Adafruit Unified sensor_t object that we'll
   fill in
*/
/**************************************************************************/
void Adafruit_MLX90395::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MLX90395", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->min_value = -120000; // -120 gauss in uTesla
  sensor->max_value = 120000;  // +120 gauss in uTesla
  sensor->resolution = _uTLSB; // 240/16-bit uTesla per LSB
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  event Pointer to an Adafruit Unified sensor_event_t object that
   we'll fill in
    @returns True on successful read
*/
/**************************************************************************/
bool Adafruit_MLX90395::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = 1;
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = millis();

  return readData(&event->magnetic.x, &event->magnetic.y, &event->magnetic.z);
}

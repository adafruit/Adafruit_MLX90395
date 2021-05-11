#ifndef ADAFRUIT_MLX90395_H
#define ADAFRUIT_MLX90395_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>

#define MLX90395_STATUS_RESET 0x02
#define MLX90395_STATUS_SMMODE 0x20
#define MLX90395_STATUS_DRDY 0x01
#define MLX90395_REG_0 0x0
#define MLX90395_REG_1 0x2
#define MLX90395_REG_2 0x4

/** Register map. */
enum {
  MLX90395_REG_SM = (0x30), /**> Start single-meas mode. */
  MLX90395_REG_EX = (0x80), /**> Exit mode. */
  MLX90395_REG_RT = (0xF0), /**< Reset. */
};

typedef enum mlx90393_osr {
  MLX90395_OSR_1,
  MLX90395_OSR_2,
  MLX90395_OSR_4,
  MLX90395_OSR_8,
} mlx90393_osr_t;

typedef enum mlx90393_res {
  MLX90395_RES_16,
  MLX90395_RES_17,
  MLX90395_RES_18,
  MLX90395_RES_19,
} mlx90393_res_t;

static const float gainMultipliers[16] = {
    0.2, 0.25,  0.3333, 0.4, 0.5,  0.6, 0.75,  1,
    0.1, 0.125, 0.1667, 0.2, 0.25, 0.3, 0.375, 0.5};

#define MLX90395_DEFAULT_ADDR (0x0C) /* Can also be 0x18, depending on IC */

/** Class for interfacing with MLX90395 magnetometer */
class Adafruit_MLX90395 : public Adafruit_Sensor {
public:
  Adafruit_MLX90395();
  bool begin_I2C(uint8_t i2c_addr = MLX90395_DEFAULT_ADDR,
                 TwoWire *wire = &Wire);
  bool reset(void);
  bool exitMode(void);
  bool startSingleMeasurement(void);
  bool readMeasurement(float *x, float *y, float *z);
  bool readData(float *x, float *y, float *z);

  mlx90393_osr_t getOSR(void);
  bool setOSR(mlx90393_osr_t osrval);
  mlx90393_res_t getResolution(void);
  bool setResolution(mlx90393_res_t resval);
  uint8_t getGain(void);
  bool setGain(uint8_t gainval);

  void getSensor(sensor_t *sensor);
  bool getEvent(sensors_event_t *event);

  uint16_t uniqueID[3]; ///< 48 bits of unique identifier, read during init

private:
  Adafruit_I2CDevice *i2c_dev = NULL;
  Adafruit_SPIDevice *spi_dev = NULL;

  bool _init(void);
  uint8_t command(uint8_t cmd);
  bool readRegister(uint8_t reg, uint16_t *data);

  mlx90393_res_t _resolution = MLX90395_RES_17;
  uint8_t _gain = 0;
  uint8_t _gain_default = 0;
  float _uTLSB = 0;
  int32_t _sensorID = 90395;
};

#endif
